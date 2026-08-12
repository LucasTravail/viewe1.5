#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "calc_engine.h"
#include "esp_log.h"
#include "esp_random.h"
#include "lvgl.h"
#include "qmi8658.h"

#define CYCLE_UNITS 3600 /* 0.1 degree resolution over one full sine cycle */
#define MM_PEAK 150      /* wave amplitude, in mm, in both directions */
/* Values are handled internally in hundredths of a mm (centi-mm) so the
 * captured measurement can be rounded down to 0.01mm instead of whole mm. */
#define MM_PEAK_CMM (MM_PEAK * 100)

#define BATT_CHARGE_TIME_MS 1500 /* how long the 0 -> x% charge-up animation takes */

#define APP_ICON_COUNT 15 /* up to 15 apps in the carousel */

/* App launcher: a horizontal, one-at-a-time carousel (native LVGL scroll
 * snap) instead of a scrolling grid - the current app sits centered and
 * full-size, its neighbours peek in from both edges (dimmed/shrunk, see
 * apps_carousel_refresh_peek), and a swipe left/right moves to the next
 * one. Tapping any card - centered or still peeking - opens it directly. */
#define CAROUSEL_RING_DIAM 148     /* the coloured ring around each app's icon */
#define CAROUSEL_RING_W 6          /* ring stroke width */
#define CAROUSEL_CARD_W 220        /* each card's own width */
#define CAROUSEL_CARD_GAP 40       /* gap between cards - this is what makes neighbours "peek" */
#define CAROUSEL_SLOT_W (CAROUSEL_CARD_W + CAROUSEL_CARD_GAP)
/* Matches the panel's full horizontal resolution (LCD_H_RES in main.c):
 * centers the 1st/last card the same way APP_ROW_PAD used to for the old
 * vertical grid, just along X instead of Y. */
#define CAROUSEL_VIEWPORT_W 471
#define CAROUSEL_SIDE_PAD ((CAROUSEL_VIEWPORT_W - CAROUSEL_CARD_W) / 2)
#define CAROUSEL_PEEK_OPA 90  /* opacity a fully-peeking neighbour is dimmed down to (out of 255) */
#define CAROUSEL_PEEK_ZOOM 195 /* zoom a fully-peeking neighbour is shrunk down to (256 = 100%) */

#define ROUND_STEP_CMM 1            /* value change per encoder detent or drag threshold crossed: 0.01mm */
#define USINAGE_DRAG_PX_PER_STEP 8  /* px of vertical drag needed to move the value by ROUND_STEP_CMM */

#define TARGET_MARKER_TICK_SPAN 6 /* degrees wide, the green "finish line" tick on the gauge ring */

/* --- Calculatrice de cotes layout --- */
#define CALC_OP_BTN_SIZE 66
#define CALC_OP_BTN_GAP 14
#define CALC_MORE_BTN_SIZE 72
#define CALC_MORE_BTN_GAP 14
#define CALC_MORE_COLS 3
#define CALC_KEY_BTN_SIZE 58
#define CALC_KEY_BTN_GAP 10
#define CALC_KEY_COLS 3
#define CALC_KEYPAD_BUF_LEN 24
#define CALC_CAPTURE_FLASH_MS 350 /* how long the CAPTURE confirmation flash lasts */
#define CALC_LIVE_ROUND CALC_ROUND_0_01 /* live reading always shown to 2 decimals, like the gauge */

static const char *USINAGE_TAG = "usinage"; /* phase-transition logging, see idf.py monitor */
static const char *CALC_TAG = "calc";

static lv_obj_t *scr_gauge;   /* mm wave screen (default) */
static lv_obj_t *scr_battery; /* battery screen, reached by swiping right */
static lv_obj_t *scr_apps;    /* app launcher screen, reached by swiping left */
static lv_obj_t *scr_calc;    /* Calculatrice de cotes, opened from the app grid */

static lv_obj_t *arc;
static lv_obj_t *percent_label;
static lv_obj_t *gauge_title_label; /* small caption above the value; shows which app is active */
static lv_obj_t *calib_confirm_btn; /* green check: validates / advances */
static lv_obj_t *calib_cancel_btn;  /* red cross: discards and stops */
static lv_obj_t *target_marker_arc; /* green tick on the ring: where the target value sits */

static lv_obj_t *batt_arc;
static lv_obj_t *batt_label;

static lv_obj_t *apps_carousel; /* the app-launcher's horizontal scroll-snap row */

/* Usinage app state. "Usinage" doesn't get its own screen - it just puts the
 * mm gauge into a caliper-style workflow, driven entirely by the main
 * button (a long touch-press anywhere on the gauge screen) and the
 * confirm/cancel buttons below the value:
 *   LIVE       wave runs freely, nothing set up yet
 *   -> long-press captures it ->
 *   VALUE      round the captured target value (encoder/drag, 0.01mm)
 *   -> confirm ->
 *   TOLERANCE  set the +/- tolerance around that target, same mechanic,
 *              non-negative
 *   -> confirm ->
 *   MEASURE    wave runs again (now reads as the real part being probed);
 *              the green tick on the ring marks the target position
 *   -> long-press captures the actual reading ->
 *   RESULT     shows the gap between actual and target, and the tolerance;
 *              green if within tolerance, red otherwise
 *   -> confirm measures another part (back to MEASURE, same target/tol.)
 *   -> cancel, at any step, aborts straight back to LIVE
 * The main button only does anything in LIVE (start) and MEASURE (capture
 * the actual reading) - elsewhere it's the buttons' job, so a stray press
 * mid-calibration can't restart or skip a step. */
typedef enum
{
    USINAGE_PHASE_LIVE,
    USINAGE_PHASE_VALUE,
    USINAGE_PHASE_TOLERANCE,
    USINAGE_PHASE_MEASURE,
    USINAGE_PHASE_RESULT,
} usinage_phase_t;

static bool usinage_active = false; /* true once the Usinage app has been opened */
static usinage_phase_t usinage_phase = USINAGE_PHASE_LIVE;
static int32_t current_cmm = 0;             /* value being shown/adjusted right now, in centi-mm */
static int32_t confirmed_value_cmm = 0;     /* target, locked in once USINAGE_PHASE_VALUE is confirmed */
static int32_t confirmed_tolerance_cmm = 0; /* +/- tolerance, locked in once USINAGE_PHASE_TOLERANCE is confirmed */
static int32_t usinage_drag_accum_px = 0;   /* sub-threshold vertical drag not yet turned into a step */

/* --- Calculatrice de cotes ---
 * The whole point: the live measurement (the same shared reading the gauge
 * screen animates - see current_cmm) can be dropped into a running
 * calculation with one long-press, mixed freely with manually-typed
 * numbers and with its own history. All the actual arithmetic lives in
 * calc_engine (calc_state below); this screen only ever displays that
 * state and forwards taps into it - see section 22 of the spec this was
 * built from. */
static lv_obj_t *calc_live_value_label; /* top of screen: continuously-updated live reading */
static lv_obj_t *calc_expr_label;       /* small caption: "CAPTURE", or "25.05 +" while an op is pending */
static lv_obj_t *calc_result_label;     /* the big number: current working value, or an error message */

static lv_obj_t *calc_round_overlay; /* ARRONDI: pick a display precision */
static lv_obj_t *calc_more_overlay;  /* "..." : ABS, +/-, C, mm/in, history, memory */
static lv_obj_t *calc_unit_btn_label;

static lv_obj_t *calc_keypad_overlay; /* manual numeric entry */
static lv_obj_t *calc_keypad_preview_label;
static char calc_keypad_buf[CALC_KEYPAD_BUF_LEN] = "";

static lv_obj_t *calc_history_overlay;
static lv_obj_t *calc_history_list; /* rebuilt each time the overlay opens */

static calc_state_t calc_state;

lv_obj_t *btn = NULL;

void btn_cb(lv_event_t *e)
{
    (void)e;
}

/* Placeholder: each grey circle will eventually launch its own app/feature.
 * For now it just proves the icon is clickable. */
static void app_icon_cb(lv_event_t *e)
{
    (void)e;
}

/* Dims and shrinks every card by how far it currently sits from the
 * carousel's own center - the centered card stays at full size/opacity,
 * and anything peeking in from the sides fades/shrinks smoothly the
 * further out it is. Re-run on every scroll tick, so the effect tracks
 * the finger in real time instead of snapping at the end. */
static void apps_carousel_refresh_peek(lv_obj_t *cont)
{
    lv_area_t cont_area;
    lv_obj_get_coords(cont, &cont_area);
    lv_coord_t cont_center_x = (cont_area.x1 + cont_area.x2) / 2;

    uint32_t n = lv_obj_get_child_cnt(cont);
    for (uint32_t i = 0; i < n; i++)
    {
        lv_obj_t *card = lv_obj_get_child(cont, i);
        lv_area_t card_area;
        lv_obj_get_coords(card, &card_area);
        lv_coord_t card_center_x = (card_area.x1 + card_area.x2) / 2;

        /* 0 = dead center, 1 = a full slot-pitch away (fully "peeking") */
        float t = (float)abs((int)(card_center_x - cont_center_x)) / (float)CAROUSEL_SLOT_W;
        if (t > 1.0f)
        {
            t = 1.0f;
        }

        lv_opa_t opa = (lv_opa_t)(255 - t * (255 - CAROUSEL_PEEK_OPA));
        lv_coord_t zoom = (lv_coord_t)(256 - t * (256 - CAROUSEL_PEEK_ZOOM));

        lv_obj_set_style_opa(card, opa, 0);
        lv_obj_set_style_transform_zoom(card, zoom, 0);
    }
}

static void apps_carousel_scroll_cb(lv_event_t *e)
{
    apps_carousel_refresh_peek(lv_event_get_target(e));
}

/* Shared by the live wave, the measurement rounding and the tolerance step:
 * paints a centi-mm value onto the gauge arc/label. In USINAGE_PHASE_TOLERANCE
 * it's always non-negative and shown as "+-0.05mm" in orange; otherwise it's
 * a signed "12.34mm" (blue for positive, red for negative). Always resets
 * the label's own text color to white - usinage_show_result is the only
 * place that overrides it (green/red for the tolerance verdict), and does
 * so *after* calling this. */
static void apply_mm_display(int32_t cmm)
{
    int32_t abs_cmm = abs((int)cmm);
    lv_arc_set_value(arc, abs_cmm);
    lv_obj_set_style_text_color(percent_label, lv_color_white(), 0);

    if (usinage_phase == USINAGE_PHASE_TOLERANCE)
    {
        /* No "\xC2\xB1" (proper unicode +-) here: the bundled fonts only
         * cover ASCII + the LV_SYMBOL_* icons, so that glyph doesn't
         * exist in them and just doesn't draw. Plain ASCII always does. */
        lv_label_set_text_fmt(percent_label, "+-%d.%02dmm", (int)(abs_cmm / 100), (int)(abs_cmm % 100));
        lv_obj_set_style_arc_color(arc, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_INDICATOR);
        return;
    }

    lv_label_set_text_fmt(percent_label, "%s%d.%02dmm", (cmm < 0) ? "-" : "", (int)(abs_cmm / 100), (int)(abs_cmm % 100));
    lv_color_t color = (cmm < 0) ? lv_palette_main(LV_PALETTE_RED) : lv_palette_main(LV_PALETTE_BLUE);
    lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR);
}

/* Points the green ring tick at a target value: same angle math LVGL uses
 * internally for lv_arc_set_value (0 deg = value 0, sweeping the full
 * range to 360 deg), so it lines up exactly with where the main arc's own
 * indicator would sit if it were showing that value. */
static void usinage_place_target_marker(int32_t target_cmm)
{
    int32_t abs_target = abs((int)target_cmm);
    int32_t angle = (int32_t)(((int64_t)abs_target * 360) / MM_PEAK_CMM);
    int32_t start = angle - TARGET_MARKER_TICK_SPAN / 2;
    int32_t end = angle + TARGET_MARKER_TICK_SPAN / 2;
    if (start < 0)
    {
        /* lv_arc_set_angles takes uint16_t: a raw negative value here would
         * wrap around to a huge angle (e.g. -3 -> 65533) instead of wrapping
         * through 0/360 like it should for a target near 0mm. */
        start += 360;
    }
    lv_arc_set_angles(target_marker_arc, (uint16_t)start, (uint16_t)end);
}

/* t runs linearly 0..CYCLE_UNITS forever; cmm = MM_PEAK_CMM*sin(t) gives a
 * perfectly smooth, continuously-differentiable 0->150->0->-150->0 wave
 * (no velocity discontinuity at the turning points, unlike a piecewise
 * linear ramp). Stands in for a live caliper reading until it's captured. */
static void calc_refresh_live_label(void); /* defined below, in the Calculatrice de cotes section */

static void set_arc_value(void *obj, int32_t t)
{
    (void)obj;
    float angle_rad = (2.0f * (float)M_PI) * ((float)t / CYCLE_UNITS);
    current_cmm = (int32_t)lroundf(MM_PEAK_CMM * sinf(angle_rad));
    apply_mm_display(current_cmm);
    calc_refresh_live_label(); /* shared "sensor": keep the calculator's LIVE reading current too */
}

static void start_wave_anim(void)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_exec_cb(&a, set_arc_value);
    lv_anim_set_var(&a, arc);
    lv_anim_set_values(&a, 0, CYCLE_UNITS);
    lv_anim_set_time(&a, 8000);
    lv_anim_set_path_cb(&a, lv_anim_path_linear); /* the sine itself provides the easing */
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
}

static void usinage_show_calibration_buttons(bool show)
{
    if (show)
    {
        lv_obj_clear_flag(calib_confirm_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(calib_cancel_btn, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(calib_confirm_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(calib_cancel_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

/* Only VALUE and TOLERANCE are hand-adjustable (encoder/drag); MEASURE just
 * watches the live wave and RESULT is a frozen readout. */
static bool usinage_is_adjustable(void)
{
    return usinage_phase == USINAGE_PHASE_VALUE || usinage_phase == USINAGE_PHASE_TOLERANCE;
}

static void usinage_show_target_marker(bool show)
{
    if (show)
    {
        lv_obj_clear_flag(target_marker_arc, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(target_marker_arc, LV_OBJ_FLAG_HIDDEN);
    }
}

/* Starts (or restarts, for the next part) the MEASURE step: live wave
 * resumes standing in for the caliper's actual reading, with the green
 * target tick shown on the ring so the operator can see how the needle
 * compares to it as it moves. */
static void usinage_begin_measure(void)
{
    ESP_LOGI(USINAGE_TAG, "-> MEASURE (target=%d cmm, tol=%d cmm)", (int)confirmed_value_cmm, (int)confirmed_tolerance_cmm);
    usinage_phase = USINAGE_PHASE_MEASURE;
    usinage_show_calibration_buttons(false);
    usinage_place_target_marker(confirmed_value_cmm);
    usinage_show_target_marker(true);
    lv_label_set_text_fmt(gauge_title_label, "+-%d.%02dmm", (int)(confirmed_tolerance_cmm / 100),
                           (int)(confirmed_tolerance_cmm % 100));
    start_wave_anim();
}

static void usinage_show_result(void); /* defined below; used by the long-press handler right here */

/* Long-press on the main button: in LIVE, freezes whatever the wave is
 * showing and opens the value calibration step; in MEASURE, freezes the
 * actual reading and shows the result. Anywhere else (VALUE, TOLERANCE,
 * RESULT) it does nothing - only the confirm/cancel buttons progress or
 * end those, so a stray press can't restart or skip a step. */
static void usinage_toggle_capture(void)
{
    if (!usinage_active)
    {
        return;
    }
    ESP_LOGI(USINAGE_TAG, "main-button long-press, phase=%d", (int)usinage_phase);

    if (usinage_phase == USINAGE_PHASE_LIVE)
    {
        usinage_phase = USINAGE_PHASE_VALUE;
        lv_anim_del(arc, set_arc_value); /* freeze on whatever current_cmm already holds */
        usinage_drag_accum_px = 0;
        usinage_show_calibration_buttons(true);
        return;
    }

    if (usinage_phase == USINAGE_PHASE_MEASURE)
    {
        usinage_show_result();
        return;
    }
}

/* Main button, for now: a long touch-press anywhere on the gauge screen.
 * LVGL's own indev already times this (indev_drv.long_press_time in
 * main.c) and fires once per press, so no manual duration tracking needed. */
static void usinage_long_press_cb(lv_event_t *e)
{
    (void)e;
    usinage_toggle_capture();
}

/* Cancels whichever step is active and goes straight back to a
 * live-measuring wave, ready to set up a new target from scratch. */
static void usinage_end_calibration(void)
{
    ESP_LOGI(USINAGE_TAG, "-> LIVE (was phase=%d)", (int)usinage_phase);
    usinage_phase = USINAGE_PHASE_LIVE;
    usinage_show_calibration_buttons(false);
    usinage_show_target_marker(false);
    lv_obj_set_style_text_color(percent_label, lv_color_white(), 0); /* undo RESULT's green/red */
    lv_label_set_text(gauge_title_label, "USINAGE");                 /* in case a later step changed it */
    start_wave_anim();
}

/* Freezes the actual reading, works out the gap to the target, and shows
 * it: big number is the deviation (sign shows over/under target), colored
 * green if within tolerance and red otherwise; title recaps the tolerance;
 * the arc/ring keeps showing the actual reading itself (blue/red, as
 * usual) so it can be compared at a glance against the green target tick. */
static void usinage_show_result(void)
{
    ESP_LOGI(USINAGE_TAG, "-> RESULT (actual=%d cmm)", (int)current_cmm);
    int32_t actual_cmm = current_cmm;
    lv_anim_del(arc, set_arc_value);
    usinage_drag_accum_px = 0;
    usinage_phase = USINAGE_PHASE_RESULT;

    apply_mm_display(actual_cmm);

    int32_t deviation_cmm = actual_cmm - confirmed_value_cmm;
    int32_t abs_deviation = abs((int)deviation_cmm);
    bool in_tolerance = abs_deviation <= confirmed_tolerance_cmm;

    lv_label_set_text_fmt(percent_label, "%s%d.%02dmm", (deviation_cmm < 0) ? "-" : "+", (int)(abs_deviation / 100),
                           (int)(abs_deviation % 100));
    lv_obj_set_style_text_color(percent_label, in_tolerance ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_RED),
                                 0);
    lv_label_set_text_fmt(gauge_title_label, "+-%d.%02dmm %s", (int)(confirmed_tolerance_cmm / 100),
                           (int)(confirmed_tolerance_cmm % 100), in_tolerance ? "OK" : "HORS TOL.");

    usinage_show_calibration_buttons(true); /* confirm or cancel: both go back to MEASURE for the next part */
}

/* Confirming the value locks it in and opens a 2nd calibration step, for
 * the +/- tolerance, starting at 0 - exact same mechanic (encoder/drag,
 * confirm/cancel), just non-negative. Confirming the tolerance starts
 * MEASURE. Confirming a RESULT just measures another part. */
static void usinage_confirm_cb(lv_event_t *e)
{
    (void)e;
    ESP_LOGI(USINAGE_TAG, "confirm pressed, phase=%d", (int)usinage_phase);

    if (usinage_phase == USINAGE_PHASE_VALUE)
    {
        confirmed_value_cmm = current_cmm;
        usinage_phase = USINAGE_PHASE_TOLERANCE;
        usinage_drag_accum_px = 0;
        current_cmm = 0;
        lv_label_set_text_fmt(gauge_title_label, "%s%d.%02dmm", (confirmed_value_cmm < 0) ? "-" : "",
                               (int)(abs((int)confirmed_value_cmm) / 100), (int)(abs((int)confirmed_value_cmm) % 100));
        apply_mm_display(current_cmm);
        return;
    }

    if (usinage_phase == USINAGE_PHASE_TOLERANCE)
    {
        confirmed_tolerance_cmm = current_cmm;
        usinage_begin_measure();
        return;
    }

    if (usinage_phase == USINAGE_PHASE_RESULT)
    {
        usinage_begin_measure(); /* same target/tolerance, ready for the next part */
        return;
    }
}

/* Cancel discards whatever calibration step is in progress and goes back
 * to LIVE (start over) - except from RESULT, where there's nothing left to
 * discard but the reading itself: it goes back to MEASURE instead, so
 * either button gets you to the next part and RESULT is never a dead end
 * that forces redoing the whole target/tolerance setup. */
static void usinage_cancel_cb(lv_event_t *e)
{
    (void)e;
    ESP_LOGI(USINAGE_TAG, "cancel pressed, phase=%d", (int)usinage_phase);

    if (usinage_phase == USINAGE_PHASE_RESULT)
    {
        usinage_begin_measure();
        return;
    }

    usinage_end_calibration();
}

/* Shared by the encoder and the drag: nudges the value being calibrated by
 * delta_cmm centi-mm (positive or negative). Clamped to the gauge's full
 * range for a measurement, or to >=0 for a +/- tolerance. */
static void usinage_adjust(int32_t delta_cmm)
{
    int32_t min_cmm = (usinage_phase == USINAGE_PHASE_TOLERANCE) ? 0 : -MM_PEAK_CMM;

    current_cmm += delta_cmm;
    if (current_cmm > MM_PEAK_CMM)
    {
        current_cmm = MM_PEAK_CMM;
    }
    else if (current_cmm < min_cmm)
    {
        current_cmm = min_cmm;
    }
    apply_mm_display(current_cmm);
}

/* Called from main.c's encoder_read_cb with +1/-1 per detent. Rounds
 * whichever value is being calibrated by 0.01mm per detent; ignored
 * outside VALUE/TOLERANCE. */
void usinage_encoder_tick(int32_t diff)
{
    if (!usinage_active || !usinage_is_adjustable())
    {
        return;
    }
    usinage_adjust(diff * ROUND_STEP_CMM);
}

/* Vertical drag while captured: drag up to increase, down to decrease.
 * Fires continuously while the finger moves (LV_EVENT_PRESSING), so the
 * per-frame delta is accumulated in px and only converted into a 0.01mm
 * step once it crosses USINAGE_DRAG_PX_PER_STEP - keeps the feel
 * smooth/scrubbable instead of jumping the whole gauge range over one
 * drag. Only looks at the vertical component, so a horizontal swipe
 * (screen switching) isn't also read as a value change. */
static void usinage_drag_cb(lv_event_t *e)
{
    (void)e;
    if (!usinage_active || !usinage_is_adjustable())
    {
        return;
    }

    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL)
    {
        return;
    }

    lv_point_t vect;
    lv_indev_get_vect(indev, &vect);
    usinage_drag_accum_px -= vect.y; /* screen y grows downward; dragging up should increase */

    while (usinage_drag_accum_px >= USINAGE_DRAG_PX_PER_STEP)
    {
        usinage_drag_accum_px -= USINAGE_DRAG_PX_PER_STEP;
        usinage_adjust(ROUND_STEP_CMM);
    }
    while (usinage_drag_accum_px <= -USINAGE_DRAG_PX_PER_STEP)
    {
        usinage_drag_accum_px += USINAGE_DRAG_PX_PER_STEP;
        usinage_adjust(-ROUND_STEP_CMM);
    }
}

/* Opens the Usinage app: brings the gauge screen back to the front in a
 * fresh, live-measuring state, with a title so it's clear which app is
 * active - the gauge screen is shared with the plain demo wave otherwise. */
static void usinage_icon_cb(lv_event_t *e)
{
    (void)e;
    usinage_active = true;
    if (usinage_phase != USINAGE_PHASE_LIVE)
    {
        usinage_phase = USINAGE_PHASE_LIVE;
        usinage_show_calibration_buttons(false);
        usinage_show_target_marker(false);
        lv_obj_set_style_text_color(percent_label, lv_color_white(), 0); /* undo RESULT's green/red, if any */
        start_wave_anim();
    }
    lv_label_set_text(gauge_title_label, "USINAGE");
    lv_scr_load_anim(scr_gauge, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
}

/* Keeps the top-of-screen LIVE reading current. Called from set_arc_value
 * on every animation tick (the same shared "sensor" the gauge/Usinage
 * read), regardless of which screen is actually visible - cheap, and it
 * means the value is never stale by the time the operator switches to the
 * calculator. Guarded because it can run before the calculator screen's
 * widgets exist yet (the wave starts animating before scr_calc is built). */
static void calc_refresh_live_label(void)
{
    if (calc_live_value_label == NULL)
    {
        return;
    }
    char buf[32];
    calc_format_value(current_cmm / 100.0, calc_state.unit, CALC_LIVE_ROUND, buf, sizeof(buf));
    lv_label_set_text(calc_live_value_label, buf);
}

static const char *calc_error_message(calc_err_t err)
{
    switch (err)
    {
    case CALC_ERR_DIV_ZERO:
        return "Division impossible";
    case CALC_ERR_OVERFLOW:
        return "Depassement";
    case CALC_ERR_NO_VALUE:
        return "Aucune valeur";
    case CALC_ERR_NO_MEASURE:
        return "Mesure indisponible";
    default:
        return "";
    }
}

/* Redraws the expression caption + big result from calc_state. Called
 * after every button press that can change it - CAPTURE, an operator, a
 * unary op, clear, a keypad/history entry. Never touches the LIVE label
 * (see calc_refresh_live_label) - LIVE and the working value must never
 * be conflated (spec section 16). */
static void calc_refresh_ui(void)
{
    if (calc_state.error != CALC_ERR_NONE)
    {
        lv_obj_set_style_text_color(calc_result_label, lv_palette_main(LV_PALETTE_RED), 0);
        lv_label_set_text(calc_result_label, calc_error_message(calc_state.error));
        lv_label_set_text(calc_expr_label, "");
        return;
    }

    lv_obj_set_style_text_color(calc_result_label, lv_color_white(), 0);

    if (!calc_state.has_value)
    {
        lv_label_set_text(calc_result_label, "--");
        lv_label_set_text(calc_expr_label, "");
        return;
    }

    char buf[32];
    calc_format_value(calc_state.value, calc_state.unit, calc_state.display_round, buf, sizeof(buf));
    lv_label_set_text(calc_result_label, buf);

    if (calc_state.pending_op != CALC_OP_NONE)
    {
        lv_label_set_text_fmt(calc_expr_label, "%s %s ...", buf, calc_op_symbol(calc_state.pending_op));
    }
    else
    {
        lv_label_set_text(calc_expr_label, "CAPTURE");
    }
}

static void calc_flash_revert_cb(lv_timer_t *t)
{
    (void)t;
    lv_obj_set_style_text_color(calc_expr_label, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
}

/* Brief green flash on the caption line to confirm a capture landed,
 * without slowing anything down (spec section 17: no long animation, no
 * confirmation dialog - just visual feedback that resolves itself). */
static void calc_flash_capture(void)
{
    lv_obj_set_style_text_color(calc_expr_label, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_timer_t *t = lv_timer_create(calc_flash_revert_cb, CALC_CAPTURE_FLASH_MS, NULL);
    lv_timer_set_repeat_count(t, 1);
}

/* The physical CAPTURE button, for now: a long touch-press anywhere on the
 * calculator screen (same mechanism as Usinage's main button, bound to its
 * own screen - see the comment on scr_gauge's LONG_PRESSED handler). Grabs
 * whatever current_cmm holds *at this instant*: the live wave itself is
 * never touched, so nothing about a capture can retroactively change once
 * it's in the calculation (spec section 16). */
static void calc_capture_cb(lv_event_t *e)
{
    (void)e;
    double mm = current_cmm / 100.0;
    ESP_LOGI(CALC_TAG, "CAPTURE %.3f mm", mm);
    calc_feed_value(&calc_state, mm);
    calc_flash_capture();
    calc_refresh_ui();
}

static void calc_op_btn_cb(lv_event_t *e)
{
    calc_op_t op = (calc_op_t)(intptr_t)lv_event_get_user_data(e);
    calc_set_op(&calc_state, op);
    calc_refresh_ui();
}

typedef enum
{
    CALC_QUICK_HALF,
    CALC_QUICK_DOUBLE,
} calc_quick_id_t;

/* /2 and x2 live directly on the main screen (not tucked in the "more"
 * menu) because they're explicitly the two most useful shortcuts in
 * mechanical work (spec section 6). */
static void calc_quick_unary_cb(lv_event_t *e)
{
    calc_quick_id_t id = (calc_quick_id_t)(intptr_t)lv_event_get_user_data(e);
    if (id == CALC_QUICK_HALF)
    {
        calc_apply_half(&calc_state);
    }
    else
    {
        calc_apply_double(&calc_state);
    }
    calc_refresh_ui();
}

static void calc_round_choice_cb(lv_event_t *e)
{
    calc_round_t r = (calc_round_t)(intptr_t)lv_event_get_user_data(e);
    calc_state.display_round = r;
    lv_obj_add_flag(calc_round_overlay, LV_OBJ_FLAG_HIDDEN);
    calc_refresh_ui();
}

static void calc_round_open_cb(lv_event_t *e)
{
    (void)e;
    lv_obj_clear_flag(calc_round_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void calc_round_close_cb(lv_event_t *e)
{
    (void)e;
    lv_obj_add_flag(calc_round_overlay, LV_OBJ_FLAG_HIDDEN);
}

/* --- manual numeric entry (spec section 9): only shown on demand, from
 * the "more" menu, so the main screen stays keypad-free (section 21). --- */
static void calc_keypad_refresh_preview(void)
{
    lv_label_set_text(calc_keypad_preview_label, (calc_keypad_buf[0] == '\0') ? "0" : calc_keypad_buf);
}

static void calc_keypad_key_cb(lv_event_t *e)
{
    char ch = (char)(intptr_t)lv_event_get_user_data(e);
    size_t len = strlen(calc_keypad_buf);

    if (ch == '\b')
    {
        if (len > 0)
        {
            calc_keypad_buf[len - 1] = '\0';
        }
    }
    else if (ch == '.')
    {
        if (strchr(calc_keypad_buf, '.') == NULL && len < CALC_KEYPAD_BUF_LEN - 2)
        {
            if (len == 0)
            {
                calc_keypad_buf[0] = '0';
                len = 1;
            }
            calc_keypad_buf[len] = '.';
            calc_keypad_buf[len + 1] = '\0';
        }
    }
    else if (len < (size_t)CALC_KEYPAD_BUF_LEN - 1)
    {
        calc_keypad_buf[len] = ch;
        calc_keypad_buf[len + 1] = '\0';
    }

    calc_keypad_refresh_preview();
}

static void calc_keypad_open_cb(lv_event_t *e)
{
    (void)e;
    calc_keypad_buf[0] = '\0';
    calc_keypad_refresh_preview();
    lv_obj_add_flag(calc_more_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(calc_keypad_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void calc_keypad_ok_cb(lv_event_t *e)
{
    (void)e;
    if (calc_keypad_buf[0] != '\0')
    {
        char *end = NULL;
        double v = strtod(calc_keypad_buf, &end);
        if (end != calc_keypad_buf) /* at least one character parsed as a number */
        {
            calc_feed_value(&calc_state, v);
        }
    }
    lv_obj_add_flag(calc_keypad_overlay, LV_OBJ_FLAG_HIDDEN);
    calc_refresh_ui();
}

static void calc_keypad_cancel_cb(lv_event_t *e)
{
    (void)e;
    lv_obj_add_flag(calc_keypad_overlay, LV_OBJ_FLAG_HIDDEN);
}

/* --- short history (spec section 11): tap an old result to reuse it --- */
static void calc_history_row_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    const calc_history_entry_t *entry = calc_history_get(&calc_state, idx);
    if (entry != NULL)
    {
        calc_feed_value(&calc_state, entry->result);
    }
    lv_obj_add_flag(calc_history_overlay, LV_OBJ_FLAG_HIDDEN);
    calc_refresh_ui();
}

static void calc_history_rebuild(void)
{
    lv_obj_clean(calc_history_list); /* drops every row widget from the last time this was open */

    if (calc_state.history_count == 0)
    {
        lv_obj_t *empty = lv_label_create(calc_history_list);
        lv_obj_set_style_text_font(empty, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(empty, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
        lv_label_set_text(empty, "Historique vide");
        return;
    }

    for (int i = 0; i < calc_state.history_count; i++)
    {
        const calc_history_entry_t *entry = calc_history_get(&calc_state, i);
        char line[40];
        calc_format_history_entry(entry, calc_state.unit, calc_state.display_round, line, sizeof(line));

        lv_obj_t *row = lv_btn_create(calc_history_list);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(row, lv_palette_darken(LV_PALETTE_GREY, 3), 0);
        lv_obj_set_style_shadow_width(row, 0, 0);
        lv_obj_set_style_outline_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 10, 0);
        lv_obj_add_event_cb(row, calc_history_row_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        lv_obj_t *row_label = lv_label_create(row);
        lv_obj_set_style_text_font(row_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(row_label, lv_color_white(), 0);
        lv_label_set_text(row_label, line);
        lv_obj_center(row_label);
    }
}

static void calc_history_open_cb(lv_event_t *e)
{
    (void)e;
    calc_history_rebuild();
    lv_obj_add_flag(calc_more_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(calc_history_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void calc_history_close_cb(lv_event_t *e)
{
    (void)e;
    lv_obj_add_flag(calc_history_overlay, LV_OBJ_FLAG_HIDDEN);
}

/* --- secondary "..." panel: ABS, +/-, C, mm/in, history, keypad, memory -
 * kept off the main screen so it stays uncluttered (spec sections 12, 21). --- */
typedef enum
{
    CALC_MORE_ABS,
    CALC_MORE_NEGATE,
    CALC_MORE_CLEAR,
    CALC_MORE_UNIT,
    CALC_MORE_HISTORY,
    CALC_MORE_KEYPAD,
    CALC_MORE_MPLUS,
    CALC_MORE_MMINUS,
    CALC_MORE_MRECALL,
    CALC_MORE_CANCEL_OP,
    CALC_MORE_CLOSE,
} calc_more_id_t;

static void calc_more_btn_cb(lv_event_t *e)
{
    calc_more_id_t id = (calc_more_id_t)(intptr_t)lv_event_get_user_data(e);

    switch (id)
    {
    case CALC_MORE_ABS:
        calc_apply_abs(&calc_state);
        break;
    case CALC_MORE_NEGATE:
        calc_toggle_sign(&calc_state);
        break;
    case CALC_MORE_CLEAR:
        calc_clear(&calc_state);
        break;
    case CALC_MORE_UNIT:
        calc_state.unit = (calc_state.unit == CALC_UNIT_MM) ? CALC_UNIT_IN : CALC_UNIT_MM;
        lv_label_set_text(calc_unit_btn_label, (calc_state.unit == CALC_UNIT_IN) ? "IN" : "MM");
        break;
    case CALC_MORE_HISTORY:
        calc_history_open_cb(e);
        return;
    case CALC_MORE_KEYPAD:
        calc_keypad_open_cb(e);
        return;
    case CALC_MORE_MPLUS:
        calc_memory_add(&calc_state);
        break;
    case CALC_MORE_MMINUS:
        calc_memory_sub(&calc_state);
        break;
    case CALC_MORE_MRECALL:
        calc_memory_recall(&calc_state);
        break;
    case CALC_MORE_CANCEL_OP:
        calc_cancel_op(&calc_state);
        break;
    case CALC_MORE_CLOSE:
        lv_obj_add_flag(calc_more_overlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    calc_refresh_ui();
}

/* MR doubles as MC on a long-press - keeps the memory row to one button
 * instead of two, without hiding the clear function anywhere obscure. */
static void calc_mr_long_press_cb(lv_event_t *e)
{
    (void)e;
    calc_memory_clear(&calc_state);
    calc_refresh_ui();
}

static void calc_more_open_cb(lv_event_t *e)
{
    (void)e;
    lv_obj_clear_flag(calc_more_overlay, LV_OBJ_FLAG_HIDDEN);
}

/* Opens the Calculatrice de cotes: same reused pattern as Usinage - bring
 * its screen to the front, no per-open state to reset (the calculation
 * naturally survives navigating away and back, which is the whole point
 * of section 10's "successive calculations"). */
static void calc_icon_cb(lv_event_t *e)
{
    (void)e;
    calc_refresh_ui();
    lv_scr_load_anim(scr_calc, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
}

static void set_batt_value(void *obj, int32_t pct)
{
    (void)obj;
    lv_arc_set_value(batt_arc, pct);
    lv_label_set_text_fmt(batt_label, "%d%%", (int)pct);
}

/* Fake battery level, redrawn from scratch every time the screen is
 * entered: pick a new random target and animate the arc/label charging
 * up to it from 0%, like a level filling in. */
static void start_battery_charge_anim(void)
{
    int32_t target_pct = (int32_t)(esp_random() % 101); /* 0..100 inclusive */

    lv_anim_del(batt_arc, set_batt_value);
    set_batt_value(batt_arc, 0);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_exec_cb(&a, set_batt_value);
    lv_anim_set_var(&a, batt_arc);
    lv_anim_set_values(&a, 0, target_pct);
    lv_anim_set_time(&a, BATT_CHARGE_TIME_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out); /* fast start, settles in like a real charge-up */
    lv_anim_start(&a);
}

/* mm gauge is the home screen, flanked on both sides:
 *   apps <--(swipe right)-- [ gauge ] --(swipe right)--> battery
 *   apps --(swipe left)--> [ gauge ] <--(swipe left)-- battery
 * i.e. the screen always slides the same way the finger swiped. */
static void gesture_cb(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL)
    {
        return;
    }
    lv_dir_t dir = lv_indev_get_gesture_dir(indev);
    lv_obj_t *scr_cur = lv_event_get_target(e);

    if (scr_cur == scr_gauge && dir == LV_DIR_RIGHT)
    {
        start_battery_charge_anim();
        lv_scr_load_anim(scr_battery, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    }
    else if (scr_cur == scr_battery && dir == LV_DIR_LEFT)
    {
        lv_scr_load_anim(scr_gauge, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    }
    else if (scr_cur == scr_gauge && dir == LV_DIR_LEFT)
    {
        lv_scr_load_anim(scr_apps, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    }
    else if (scr_cur == scr_apps && dir == LV_DIR_BOTTOM)
    {
        /* Left/right on the apps screen is claimed by the carousel itself
         * (browsing between apps) - swipe down to leave it instead. */
        lv_scr_load_anim(scr_gauge, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    }
    else if (scr_cur == scr_calc && dir == LV_DIR_LEFT)
    {
        /* Same "swipe left = back to the app grid" convention as scr_gauge;
         * scr_calc is only ever reached by tapping its icon there. */
        lv_scr_load_anim(scr_apps, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    }
}

void lvgl_demo_ui(lv_disp_t *disp)
{
    lv_disp_set_rotation(disp, LV_DISP_ROT_180);

    /* --- mm wave screen --- */
    scr_gauge = lv_disp_get_scr_act(disp);
    lv_obj_set_style_bg_color(scr_gauge, lv_color_black(), 0);
    lv_obj_clear_flag(scr_gauge, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr_gauge, gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(scr_gauge, usinage_long_press_cb, LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_event_cb(scr_gauge, usinage_drag_cb, LV_EVENT_PRESSING, NULL);

    arc = lv_arc_create(scr_gauge);
    lv_obj_set_size(arc, 440, 440);
    lv_obj_center(arc);
    lv_arc_set_rotation(arc, 270);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_range(arc, 0, MM_PEAK_CMM);

    lv_obj_set_style_arc_width(arc, 34, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_palette_darken(LV_PALETTE_GREY, 4), LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 34, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_INDICATOR);

    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);

    /* Green "finish line" tick: sits right on top of the same ring (same
     * size/width as arc, same rotation so its angle math lines up), only
     * its indicator is visible, and only during MEASURE/RESULT (see
     * usinage_place_target_marker / usinage_show_target_marker). Created
     * after arc so it draws on top of it. */
    target_marker_arc = lv_arc_create(scr_gauge);
    lv_obj_set_size(target_marker_arc, 440, 440);
    lv_obj_center(target_marker_arc);
    lv_arc_set_rotation(target_marker_arc, 270);
    lv_arc_set_bg_angles(target_marker_arc, 0, 360);
    lv_obj_remove_style(target_marker_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(target_marker_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_opa(target_marker_arc, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_arc_color(target_marker_arc, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(target_marker_arc, 34, LV_PART_INDICATOR);
    lv_arc_set_angles(target_marker_arc, 0, TARGET_MARKER_TICK_SPAN);
    lv_obj_add_flag(target_marker_arc, LV_OBJ_FLAG_HIDDEN);

    percent_label = lv_label_create(scr_gauge);
    lv_obj_set_style_text_font(percent_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(percent_label, lv_color_white(), 0);
    lv_obj_center(percent_label);
    lv_label_set_text(percent_label, "0.00mm");
    /* Non-clickable so a long-press here still hits scr_gauge underneath
     * (arc is already non-clickable) instead of being swallowed here. */
    lv_obj_clear_flag(percent_label, LV_OBJ_FLAG_CLICKABLE);

    gauge_title_label = lv_label_create(scr_gauge);
    lv_obj_set_style_text_font(gauge_title_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(gauge_title_label, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
    lv_label_set_text(gauge_title_label, ""); /* empty on the plain demo wave, set by usinage_icon_cb */
    lv_obj_align(gauge_title_label, LV_ALIGN_CENTER, 0, -70);
    lv_obj_clear_flag(gauge_title_label, LV_OBJ_FLAG_CLICKABLE);

    /* Calibration confirm/cancel: hidden until a long-press captures a
     * value; shown below the number until usinage_end_calibration hides
     * them again (see usinage_confirm_cb / usinage_cancel_cb). They react
     * to LONG_PRESSED rather than CLICKED - a plain tap does nothing - so
     * brushing past one while dragging the value up/down doesn't trigger
     * it by accident; only a deliberate hold does. */
    calib_cancel_btn = lv_btn_create(scr_gauge);
    lv_obj_set_size(calib_cancel_btn, 84, 84);
    lv_obj_set_style_radius(calib_cancel_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(calib_cancel_btn, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_shadow_width(calib_cancel_btn, 0, 0); /* default theme draws a grey drop shadow otherwise */
    lv_obj_set_style_outline_width(calib_cancel_btn, 0, 0);
    lv_obj_align(calib_cancel_btn, LV_ALIGN_CENTER, -70, 90);
    lv_obj_add_flag(calib_cancel_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(calib_cancel_btn, usinage_cancel_cb, LV_EVENT_LONG_PRESSED, NULL);

    lv_obj_t *cancel_icon = lv_label_create(calib_cancel_btn);
    lv_obj_set_style_text_font(cancel_icon, &lv_font_montserrat_48, 0);
    lv_label_set_text(cancel_icon, LV_SYMBOL_CLOSE);
    lv_obj_center(cancel_icon);

    calib_confirm_btn = lv_btn_create(scr_gauge);
    lv_obj_set_size(calib_confirm_btn, 84, 84);
    lv_obj_set_style_radius(calib_confirm_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(calib_confirm_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_shadow_width(calib_confirm_btn, 0, 0); /* default theme draws a grey drop shadow otherwise */
    lv_obj_set_style_outline_width(calib_confirm_btn, 0, 0);
    lv_obj_align(calib_confirm_btn, LV_ALIGN_CENTER, 70, 90);
    lv_obj_add_flag(calib_confirm_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(calib_confirm_btn, usinage_confirm_cb, LV_EVENT_LONG_PRESSED, NULL);

    lv_obj_t *confirm_icon = lv_label_create(calib_confirm_btn);
    lv_obj_set_style_text_font(confirm_icon, &lv_font_montserrat_48, 0);
    lv_label_set_text(confirm_icon, LV_SYMBOL_OK);
    lv_obj_center(confirm_icon);

    start_wave_anim();

    /* --- battery screen --- */
    scr_battery = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_battery, lv_color_black(), 0);
    lv_obj_clear_flag(scr_battery, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr_battery, gesture_cb, LV_EVENT_GESTURE, NULL);

    batt_arc = lv_arc_create(scr_battery);
    lv_obj_set_size(batt_arc, 440, 440);
    lv_obj_center(batt_arc);
    lv_arc_set_rotation(batt_arc, 270);
    lv_arc_set_bg_angles(batt_arc, 0, 360);
    lv_arc_set_range(batt_arc, 0, 100);

    lv_obj_set_style_arc_width(batt_arc, 34, LV_PART_MAIN);
    lv_obj_set_style_arc_color(batt_arc, lv_palette_darken(LV_PALETTE_GREY, 4), LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(batt_arc, true, LV_PART_MAIN);
    lv_obj_set_style_arc_width(batt_arc, 34, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(batt_arc, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(batt_arc, true, LV_PART_INDICATOR);

    lv_obj_remove_style(batt_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(batt_arc, LV_OBJ_FLAG_CLICKABLE);

    batt_label = lv_label_create(scr_battery);
    lv_obj_set_style_text_font(batt_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(batt_label, lv_color_white(), 0);
    lv_obj_center(batt_label);
    lv_label_set_text(batt_label, "0%");

    /* --- app launcher screen: a horizontal one-at-a-time carousel, up to
     * 15 apps - see apps_carousel_refresh_peek for the peek/dim effect and
     * the CAROUSEL_* defines for the geometry. --- */
    scr_apps = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_apps, lv_color_black(), 0);
    lv_obj_clear_flag(scr_apps, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr_apps, gesture_cb, LV_EVENT_GESTURE, NULL);

    apps_carousel = lv_obj_create(scr_apps);
    lv_obj_remove_style_all(apps_carousel); /* plain layout container, no border/bg of its own */
    lv_obj_set_size(apps_carousel, CAROUSEL_VIEWPORT_W, LV_SIZE_CONTENT);
    lv_obj_center(apps_carousel);
    /* Blank margin before the 1st card and after the last one, exactly the
     * old vertical grid's APP_ROW_PAD trick rotated 90 deg: lets the 1st/
     * last card center at rest too, same as every card in between. */
    lv_obj_set_style_pad_left(apps_carousel, CAROUSEL_SIDE_PAD, 0);
    lv_obj_set_style_pad_right(apps_carousel, CAROUSEL_SIDE_PAD, 0);
    lv_obj_set_style_pad_column(apps_carousel, CAROUSEL_CARD_GAP, 0);
    lv_obj_set_flex_flow(apps_carousel, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(apps_carousel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* Only horizontal scrolling, so a vertical swipe still falls through to
     * gesture_cb (leaving the carousel) instead of being eaten as a scroll -
     * same principle as every other screen here, just the axes swapped. */
    lv_obj_set_scroll_dir(apps_carousel, LV_DIR_HOR);
    lv_obj_set_scroll_snap_x(apps_carousel, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(apps_carousel, LV_SCROLLBAR_MODE_OFF); /* the peek itself is the "there's more" cue */
    lv_obj_add_event_cb(apps_carousel, apps_carousel_scroll_cb, LV_EVENT_SCROLL, NULL);

    for (int i = 0; i < APP_ICON_COUNT; i++)
    {
        lv_obj_t *card = lv_obj_create(apps_carousel);
        lv_obj_remove_style_all(card);
        lv_obj_set_size(card, CAROUSEL_CARD_W, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(card, LV_OPA_TRANSP, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_row(card, 12, 0);

        const char *name;
        const char *subtitle;
        const char *icon_text;
        const lv_font_t *icon_font;
        lv_color_t ring_color;
        lv_event_cb_t open_cb;
        char placeholder_name[8];

        if (i == 0)
        {
            /* First app, wired up for real: Usinage. A gear stands in well
             * for "machining" - already in the bundled symbol font, no
             * custom artwork needed. */
            name = "USINAGE";
            subtitle = "Mesure";
            icon_text = LV_SYMBOL_SETTINGS;
            icon_font = &lv_font_montserrat_48;
            ring_color = lv_palette_main(LV_PALETTE_ORANGE);
            open_cb = usinage_icon_cb;
        }
        else if (i == 1)
        {
            /* 2nd app, wired up for real: Calculatrice de cotes. No
             * built-in "calculator" glyph in the bundled symbol set, so
             * "123" stands in for it. */
            name = "CALCULATRICE";
            subtitle = "De cotes";
            icon_text = "123";
            icon_font = &lv_font_montserrat_48;
            ring_color = lv_palette_main(LV_PALETTE_BLUE);
            open_cb = calc_icon_cb;
        }
        else
        {
            /* Placeholders: dim ring, no feature behind them yet. */
            lv_snprintf(placeholder_name, sizeof(placeholder_name), "APP %d", i + 1);
            name = placeholder_name;
            subtitle = "A venir";
            icon_text = "?";
            icon_font = &lv_font_montserrat_22;
            ring_color = lv_palette_darken(LV_PALETTE_GREY, 2);
            open_cb = app_icon_cb;
        }

        lv_obj_t *ring = lv_obj_create(card);
        lv_obj_set_size(ring, CAROUSEL_RING_DIAM, CAROUSEL_RING_DIAM);
        lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(ring, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(ring, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(ring, CAROUSEL_RING_W, 0);
        lv_obj_set_style_border_color(ring, ring_color, 0);
        lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(ring, LV_OBJ_FLAG_CLICKABLE); /* taps fall through to the card underneath */

        lv_obj_t *icon_glyph = lv_label_create(ring);
        lv_obj_set_style_text_font(icon_glyph, icon_font, 0);
        lv_obj_set_style_text_color(icon_glyph, i < 2 ? lv_color_white() : lv_palette_lighten(LV_PALETTE_GREY, 1), 0);
        lv_label_set_text(icon_glyph, icon_text);
        lv_obj_center(icon_glyph);
        lv_obj_clear_flag(icon_glyph, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t *name_label = lv_label_create(card);
        lv_obj_set_style_text_font(name_label, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(name_label, lv_color_white(), 0);
        lv_label_set_text(name_label, name);
        lv_obj_clear_flag(name_label, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t *sub_label = lv_label_create(card);
        lv_obj_set_style_text_font(sub_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(sub_label, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
        lv_label_set_text(sub_label, subtitle);
        lv_obj_clear_flag(sub_label, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_add_event_cb(card, open_cb, LV_EVENT_CLICKED, NULL);
    }

    apps_carousel_refresh_peek(apps_carousel); /* set the correct dim/zoom before the first scroll ever happens */

    /* --- Calculatrice de cotes --- */
    calc_init(&calc_state);

    scr_calc = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_calc, lv_color_black(), 0);
    lv_obj_clear_flag(scr_calc, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr_calc, gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(scr_calc, calc_capture_cb, LV_EVENT_LONG_PRESSED, NULL);

    /* LIVE: always the current shared reading, never touched by anything
     * below it - visually small/cyan on purpose, so it can never be
     * mistaken for the (much bigger, white) working value (spec 16). */
    calc_live_value_label = lv_label_create(scr_calc);
    lv_obj_set_style_text_font(calc_live_value_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(calc_live_value_label, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_label_set_text(calc_live_value_label, "LIVE --");
    lv_obj_align(calc_live_value_label, LV_ALIGN_CENTER, 0, -190);
    lv_obj_clear_flag(calc_live_value_label, LV_OBJ_FLAG_CLICKABLE);

    calc_expr_label = lv_label_create(scr_calc);
    lv_obj_set_style_text_font(calc_expr_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(calc_expr_label, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
    lv_label_set_text(calc_expr_label, "");
    lv_obj_align(calc_expr_label, LV_ALIGN_CENTER, 0, -160);
    lv_obj_clear_flag(calc_expr_label, LV_OBJ_FLAG_CLICKABLE);

    /* The working value/result: the main visual focus of the screen. */
    calc_result_label = lv_label_create(scr_calc);
    lv_obj_set_style_text_font(calc_result_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(calc_result_label, lv_color_white(), 0);
    lv_label_set_text(calc_result_label, "--");
    lv_obj_align(calc_result_label, LV_ALIGN_CENTER, 0, -95);
    lv_obj_clear_flag(calc_result_label, LV_OBJ_FLAG_CLICKABLE);

    /* Operator row: +, -, x, /. Single glyphs -> circles, like every other
     * one-glyph button in this app (confirm/cancel, app icons). */
    {
        const char *op_labels[4] = {"+", "-", "x", "/"};
        const calc_op_t ops[4] = {CALC_OP_ADD, CALC_OP_SUB, CALC_OP_MUL, CALC_OP_DIV};
        int total_w = 4 * CALC_OP_BTN_SIZE + 3 * CALC_OP_BTN_GAP;
        int start_x = -total_w / 2 + CALC_OP_BTN_SIZE / 2;

        for (int i = 0; i < 4; i++)
        {
            lv_obj_t *btn_op = lv_btn_create(scr_calc);
            lv_obj_set_size(btn_op, CALC_OP_BTN_SIZE, CALC_OP_BTN_SIZE);
            lv_obj_set_style_radius(btn_op, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(btn_op, lv_palette_darken(LV_PALETTE_GREY, 3), 0);
            lv_obj_set_style_shadow_width(btn_op, 0, 0);
            lv_obj_set_style_outline_width(btn_op, 0, 0);
            lv_obj_align(btn_op, LV_ALIGN_CENTER, start_x + i * (CALC_OP_BTN_SIZE + CALC_OP_BTN_GAP), 10);
            lv_obj_add_event_cb(btn_op, calc_op_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)ops[i]);

            lv_obj_t *op_glyph = lv_label_create(btn_op);
            lv_obj_set_style_text_font(op_glyph, &lv_font_montserrat_48, 0);
            lv_obj_set_style_text_color(op_glyph, lv_color_white(), 0);
            lv_label_set_text(op_glyph, op_labels[i]);
            lv_obj_center(op_glyph);
        }
    }

    /* Quick row: /2 and x2 (spec 6: must be especially accessible), ARR
     * (rounding) and "..." for everything secondary (spec 12/21: keep the
     * main screen uncluttered). Text labels -> rounded squares, easier to
     * fit multi-character text in than a circle. */
    {
        const char *quick_labels[4] = {"/2", "x2", "ARR", "..."};
        lv_color_t quick_colors[4] = {
            lv_palette_main(LV_PALETTE_BLUE),
            lv_palette_main(LV_PALETTE_BLUE),
            lv_palette_main(LV_PALETTE_ORANGE),
            lv_palette_darken(LV_PALETTE_GREY, 3),
        };
        int total_w = 4 * CALC_OP_BTN_SIZE + 3 * CALC_OP_BTN_GAP;
        int start_x = -total_w / 2 + CALC_OP_BTN_SIZE / 2;

        for (int i = 0; i < 4; i++)
        {
            lv_obj_t *btn_q = lv_btn_create(scr_calc);
            lv_obj_set_size(btn_q, CALC_OP_BTN_SIZE, CALC_OP_BTN_SIZE);
            lv_obj_set_style_radius(btn_q, 16, 0);
            lv_obj_set_style_bg_color(btn_q, quick_colors[i], 0);
            lv_obj_set_style_shadow_width(btn_q, 0, 0);
            lv_obj_set_style_outline_width(btn_q, 0, 0);
            lv_obj_align(btn_q, LV_ALIGN_CENTER, start_x + i * (CALC_OP_BTN_SIZE + CALC_OP_BTN_GAP), 95);

            lv_obj_t *q_label = lv_label_create(btn_q);
            lv_obj_set_style_text_font(q_label, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(q_label, lv_color_white(), 0);
            lv_label_set_text(q_label, quick_labels[i]);
            lv_obj_center(q_label);

            if (i == 0)
            {
                lv_obj_add_event_cb(btn_q, calc_quick_unary_cb, LV_EVENT_CLICKED, (void *)(intptr_t)CALC_QUICK_HALF);
            }
            else if (i == 1)
            {
                lv_obj_add_event_cb(btn_q, calc_quick_unary_cb, LV_EVENT_CLICKED, (void *)(intptr_t)CALC_QUICK_DOUBLE);
            }
            else if (i == 2)
            {
                lv_obj_add_event_cb(btn_q, calc_round_open_cb, LV_EVENT_CLICKED, NULL);
            }
            else
            {
                lv_obj_add_event_cb(btn_q, calc_more_open_cb, LV_EVENT_CLICKED, NULL);
            }
        }
    }

    /* --- ARRONDI overlay: pick a display precision (spec 7). Rounding is
     * display-only - calc_state.value itself is never touched, so it
     * never has to be "undone" and chained calculations stay exact. --- */
    calc_round_overlay = lv_obj_create(scr_calc);
    lv_obj_remove_style_all(calc_round_overlay);
    lv_obj_set_size(calc_round_overlay, lv_pct(100), lv_pct(100));
    lv_obj_center(calc_round_overlay);
    lv_obj_set_style_bg_color(calc_round_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(calc_round_overlay, LV_OPA_COVER, 0);
    lv_obj_clear_flag(calc_round_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(calc_round_overlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *round_title = lv_label_create(calc_round_overlay);
    lv_obj_set_style_text_font(round_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(round_title, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
    lv_label_set_text(round_title, "ARRONDI");
    lv_obj_align(round_title, LV_ALIGN_CENTER, 0, -140);

    {
        const char *round_labels[4] = {"INT", "0.1", "0.01", "0.001"};
        const calc_round_t round_vals[4] = {CALC_ROUND_INT, CALC_ROUND_0_1, CALC_ROUND_0_01, CALC_ROUND_0_001};
        int size = 74, gap = 12;
        int total_w = 4 * size + 3 * gap;
        int start_x = -total_w / 2 + size / 2;

        for (int i = 0; i < 4; i++)
        {
            lv_obj_t *btn_r = lv_btn_create(calc_round_overlay);
            lv_obj_set_size(btn_r, size, size);
            lv_obj_set_style_radius(btn_r, 16, 0);
            lv_obj_set_style_bg_color(btn_r, lv_palette_darken(LV_PALETTE_GREY, 3), 0);
            lv_obj_set_style_shadow_width(btn_r, 0, 0);
            lv_obj_set_style_outline_width(btn_r, 0, 0);
            lv_obj_align(btn_r, LV_ALIGN_CENTER, start_x + i * (size + gap), 0);
            lv_obj_add_event_cb(btn_r, calc_round_choice_cb, LV_EVENT_CLICKED, (void *)(intptr_t)round_vals[i]);

            lv_obj_t *r_label = lv_label_create(btn_r);
            lv_obj_set_style_text_font(r_label, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(r_label, lv_color_white(), 0);
            lv_label_set_text(r_label, round_labels[i]);
            lv_obj_center(r_label);
        }
    }

    lv_obj_t *round_close = lv_btn_create(calc_round_overlay);
    lv_obj_set_size(round_close, 70, 70);
    lv_obj_set_style_radius(round_close, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(round_close, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_shadow_width(round_close, 0, 0);
    lv_obj_set_style_outline_width(round_close, 0, 0);
    lv_obj_align(round_close, LV_ALIGN_CENTER, 0, 110);
    lv_obj_add_event_cb(round_close, calc_round_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *round_close_glyph = lv_label_create(round_close);
    lv_obj_set_style_text_font(round_close_glyph, &lv_font_montserrat_48, 0);
    lv_label_set_text(round_close_glyph, LV_SYMBOL_CLOSE);
    lv_obj_center(round_close_glyph);

    /* --- "..." overlay: everything secondary (spec 12/21) - ABS, +/-,
     * clear, mm/in, history, manual entry, memory. Laid out with the same
     * flex-wrap grid trick as the app launcher, so it doesn't need manual
     * per-button coordinates. --- */
    calc_more_overlay = lv_obj_create(scr_calc);
    lv_obj_remove_style_all(calc_more_overlay);
    lv_obj_set_size(calc_more_overlay, lv_pct(100), lv_pct(100));
    lv_obj_center(calc_more_overlay);
    lv_obj_set_style_bg_color(calc_more_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(calc_more_overlay, LV_OPA_COVER, 0);
    lv_obj_clear_flag(calc_more_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(calc_more_overlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *more_title = lv_label_create(calc_more_overlay);
    lv_obj_set_style_text_font(more_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(more_title, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
    lv_label_set_text(more_title, "FONCTIONS");
    lv_obj_align(more_title, LV_ALIGN_CENTER, 0, -185);

    lv_obj_t *more_grid = lv_obj_create(calc_more_overlay);
    lv_obj_remove_style_all(more_grid);
    lv_obj_set_style_bg_opa(more_grid, LV_OPA_TRANSP, 0);
    lv_obj_set_size(more_grid, CALC_MORE_COLS * CALC_MORE_BTN_SIZE + (CALC_MORE_COLS - 1) * CALC_MORE_BTN_GAP, LV_SIZE_CONTENT);
    lv_obj_align(more_grid, LV_ALIGN_CENTER, 0, 15);
    lv_obj_clear_flag(more_grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_row(more_grid, CALC_MORE_BTN_GAP, 0);
    lv_obj_set_style_pad_column(more_grid, CALC_MORE_BTN_GAP, 0);
    lv_obj_set_flex_flow(more_grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(more_grid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    {
        const char *more_labels[11] = {"ABS", "+/-", "C", "MM", "HIST", "123", "M+", "M-", "MR", "OP-X", NULL};
        const calc_more_id_t more_ids[11] = {
            CALC_MORE_ABS,    CALC_MORE_NEGATE, CALC_MORE_CLEAR,   CALC_MORE_UNIT,      CALC_MORE_HISTORY, CALC_MORE_KEYPAD,
            CALC_MORE_MPLUS,  CALC_MORE_MMINUS, CALC_MORE_MRECALL, CALC_MORE_CANCEL_OP, CALC_MORE_CLOSE,
        };

        for (int i = 0; i < 11; i++)
        {
            lv_obj_t *btn_m = lv_btn_create(more_grid);
            lv_obj_set_size(btn_m, CALC_MORE_BTN_SIZE, CALC_MORE_BTN_SIZE);
            lv_obj_set_style_shadow_width(btn_m, 0, 0);
            lv_obj_set_style_outline_width(btn_m, 0, 0);

            bool is_close = (more_ids[i] == CALC_MORE_CLOSE);
            bool is_clear = (more_ids[i] == CALC_MORE_CLEAR);
            lv_obj_set_style_radius(btn_m, is_close ? (lv_coord_t)LV_RADIUS_CIRCLE : 16, 0);
            lv_obj_set_style_bg_color(btn_m,
                                       is_close ? lv_palette_main(LV_PALETTE_RED)
                                       : is_clear ? lv_palette_darken(LV_PALETTE_RED, 2)
                                                  : lv_palette_darken(LV_PALETTE_GREY, 3),
                                       0);
            lv_obj_add_event_cb(btn_m, calc_more_btn_cb, LV_EVENT_CLICKED, (void *)(intptr_t)more_ids[i]);
            if (more_ids[i] == CALC_MORE_MRECALL)
            {
                lv_obj_add_event_cb(btn_m, calc_mr_long_press_cb, LV_EVENT_LONG_PRESSED, NULL);
            }

            lv_obj_t *m_label = lv_label_create(btn_m);
            if (is_close)
            {
                lv_obj_set_style_text_font(m_label, &lv_font_montserrat_48, 0);
                lv_label_set_text(m_label, LV_SYMBOL_CLOSE);
            }
            else
            {
                lv_obj_set_style_text_font(m_label, &lv_font_montserrat_14, 0);
                lv_label_set_text(m_label, more_labels[i]);
                if (more_ids[i] == CALC_MORE_UNIT)
                {
                    calc_unit_btn_label = m_label; /* kept up to date by calc_more_btn_cb */
                }
            }
            lv_obj_set_style_text_color(m_label, lv_color_white(), 0);
            lv_obj_center(m_label);
        }
    }

    /* --- manual numeric entry overlay (spec 9): only ever shown on
     * demand, from the "..." menu - the main screen stays keypad-free
     * (spec 21), which reads much better on a small round display. --- */
    calc_keypad_overlay = lv_obj_create(scr_calc);
    lv_obj_remove_style_all(calc_keypad_overlay);
    lv_obj_set_size(calc_keypad_overlay, lv_pct(100), lv_pct(100));
    lv_obj_center(calc_keypad_overlay);
    lv_obj_set_style_bg_color(calc_keypad_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(calc_keypad_overlay, LV_OPA_COVER, 0);
    lv_obj_clear_flag(calc_keypad_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(calc_keypad_overlay, LV_OBJ_FLAG_HIDDEN);

    calc_keypad_preview_label = lv_label_create(calc_keypad_overlay);
    lv_obj_set_style_text_font(calc_keypad_preview_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(calc_keypad_preview_label, lv_color_white(), 0);
    lv_label_set_text(calc_keypad_preview_label, "0");
    lv_obj_align(calc_keypad_preview_label, LV_ALIGN_CENTER, 0, -175);

    lv_obj_t *key_grid = lv_obj_create(calc_keypad_overlay);
    lv_obj_remove_style_all(key_grid);
    lv_obj_set_style_bg_opa(key_grid, LV_OPA_TRANSP, 0);
    lv_obj_set_size(key_grid, CALC_KEY_COLS * CALC_KEY_BTN_SIZE + (CALC_KEY_COLS - 1) * CALC_KEY_BTN_GAP, LV_SIZE_CONTENT);
    lv_obj_align(key_grid, LV_ALIGN_CENTER, 0, 15);
    lv_obj_clear_flag(key_grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_row(key_grid, CALC_KEY_BTN_GAP, 0);
    lv_obj_set_style_pad_column(key_grid, CALC_KEY_BTN_GAP, 0);
    lv_obj_set_flex_flow(key_grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(key_grid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    {
        static const char CALC_KEYPAD_CHARS[12] = {'7', '8', '9', '4', '5', '6', '1', '2', '3', '.', '0', '\b'};

        for (int i = 0; i < 12; i++)
        {
            char ch = CALC_KEYPAD_CHARS[i];
            lv_obj_t *btn_k = lv_btn_create(key_grid);
            lv_obj_set_size(btn_k, CALC_KEY_BTN_SIZE, CALC_KEY_BTN_SIZE);
            lv_obj_set_style_radius(btn_k, 14, 0);
            lv_obj_set_style_bg_color(btn_k, lv_palette_darken(LV_PALETTE_GREY, 3), 0);
            lv_obj_set_style_shadow_width(btn_k, 0, 0);
            lv_obj_set_style_outline_width(btn_k, 0, 0);
            lv_obj_add_event_cb(btn_k, calc_keypad_key_cb, LV_EVENT_CLICKED, (void *)(intptr_t)ch);

            lv_obj_t *k_label = lv_label_create(btn_k);
            lv_obj_set_style_text_color(k_label, lv_color_white(), 0);
            if (ch == '\b')
            {
                lv_obj_set_style_text_font(k_label, &lv_font_montserrat_14, 0);
                lv_label_set_text(k_label, LV_SYMBOL_BACKSPACE);
            }
            else
            {
                char one_char[2] = {ch, '\0'};
                lv_obj_set_style_text_font(k_label, &lv_font_montserrat_14, 0);
                lv_label_set_text(k_label, one_char);
            }
            lv_obj_center(k_label);
        }
    }

    {
        lv_obj_t *key_cancel = lv_btn_create(calc_keypad_overlay);
        lv_obj_set_size(key_cancel, 80, 80);
        lv_obj_set_style_radius(key_cancel, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(key_cancel, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_set_style_shadow_width(key_cancel, 0, 0);
        lv_obj_set_style_outline_width(key_cancel, 0, 0);
        lv_obj_align(key_cancel, LV_ALIGN_CENTER, -50, 195);
        lv_obj_add_event_cb(key_cancel, calc_keypad_cancel_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t *key_cancel_glyph = lv_label_create(key_cancel);
        lv_obj_set_style_text_font(key_cancel_glyph, &lv_font_montserrat_48, 0);
        lv_label_set_text(key_cancel_glyph, LV_SYMBOL_CLOSE);
        lv_obj_center(key_cancel_glyph);

        lv_obj_t *key_ok = lv_btn_create(calc_keypad_overlay);
        lv_obj_set_size(key_ok, 80, 80);
        lv_obj_set_style_radius(key_ok, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(key_ok, lv_palette_main(LV_PALETTE_GREEN), 0);
        lv_obj_set_style_shadow_width(key_ok, 0, 0);
        lv_obj_set_style_outline_width(key_ok, 0, 0);
        lv_obj_align(key_ok, LV_ALIGN_CENTER, 50, 195);
        lv_obj_add_event_cb(key_ok, calc_keypad_ok_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t *key_ok_glyph = lv_label_create(key_ok);
        lv_obj_set_style_text_font(key_ok_glyph, &lv_font_montserrat_48, 0);
        lv_label_set_text(key_ok_glyph, LV_SYMBOL_OK);
        lv_obj_center(key_ok_glyph);
    }

    /* --- short history overlay (spec 11): tap a past line to reuse its
     * result. Rebuilt from scratch every time it's opened (calc_history_open_cb),
     * so it always reflects calc_state.history as of that moment. --- */
    calc_history_overlay = lv_obj_create(scr_calc);
    lv_obj_remove_style_all(calc_history_overlay);
    lv_obj_set_size(calc_history_overlay, lv_pct(100), lv_pct(100));
    lv_obj_center(calc_history_overlay);
    lv_obj_set_style_bg_color(calc_history_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(calc_history_overlay, LV_OPA_COVER, 0);
    lv_obj_clear_flag(calc_history_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(calc_history_overlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *hist_title = lv_label_create(calc_history_overlay);
    lv_obj_set_style_text_font(hist_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hist_title, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
    lv_label_set_text(hist_title, "HISTORIQUE");
    lv_obj_align(hist_title, LV_ALIGN_CENTER, 0, -185);

    calc_history_list = lv_obj_create(calc_history_overlay);
    lv_obj_remove_style_all(calc_history_list);
    lv_obj_set_style_bg_opa(calc_history_list, LV_OPA_TRANSP, 0);
    lv_obj_set_size(calc_history_list, 320, 300);
    lv_obj_align(calc_history_list, LV_ALIGN_CENTER, 0, 10);
    lv_obj_set_flex_flow(calc_history_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(calc_history_list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(calc_history_list, 10, 0);
    lv_obj_set_scroll_dir(calc_history_list, LV_DIR_VER);

    lv_obj_t *hist_close = lv_btn_create(calc_history_overlay);
    lv_obj_set_size(hist_close, 70, 70);
    lv_obj_set_style_radius(hist_close, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(hist_close, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_shadow_width(hist_close, 0, 0);
    lv_obj_set_style_outline_width(hist_close, 0, 0);
    lv_obj_align(hist_close, LV_ALIGN_CENTER, 0, 195);
    lv_obj_add_event_cb(hist_close, calc_history_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *hist_close_glyph = lv_label_create(hist_close);
    lv_obj_set_style_text_font(hist_close_glyph, &lv_font_montserrat_48, 0);
    lv_label_set_text(hist_close_glyph, LV_SYMBOL_CLOSE);
    lv_obj_center(hist_close_glyph);
}
