#include <math.h>
#include <stdlib.h>
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

#define APP_GRID_COLS 3   /* "L" - icons per row */
#define APP_ICON_COUNT 15 /* up to 15 apps, i.e. 5 floors of 3 */
#define APP_ICON_SIZE 96
#define APP_ICON_GAP 22
/* Matches the panel's full vertical resolution (LCD_V_RES in main.c) so the
 * scrollable list's own clip boundary sits exactly on the screen edge -
 * already hidden by the round bezel - instead of an artificial inner
 * rectangle that would show up as a visible "square" while scrolling. */
#define APP_GRID_VIEWPORT_H 466
#define APP_ROW_PAD ((APP_GRID_VIEWPORT_H - (2 * APP_ICON_SIZE + APP_ICON_GAP)) / 2)

#define ELEVATOR_DIAM 452        /* hugs the round bezel */
#define ELEVATOR_ARC_SPAN 45     /* total track sweep, in degrees, centered on 3 o'clock */
#define ELEVATOR_HALF_SPAN (ELEVATOR_ARC_SPAN / 2)
#define ELEVATOR_TRACK_START (360 - ELEVATOR_HALF_SPAN)          /* raw bg angle, e.g. 338 */
#define ELEVATOR_TRACK_SPAN (2 * ELEVATOR_HALF_SPAN)              /* unwrapped span, e.g. 44 */
#define ELEVATOR_CURSOR_SPAN 8   /* angular width of the moving cursor within the track */
#define ELEVATOR_TRACK_W 6       /* stroke width of the dim background track */
#define ELEVATOR_CURSOR_W 12     /* stroke width of the bright moving cursor */

#define ROUND_STEP_CMM 1            /* value change per encoder detent or drag threshold crossed: 0.01mm */
#define USINAGE_DRAG_PX_PER_STEP 8  /* px of vertical drag needed to move the value by ROUND_STEP_CMM */

#define TARGET_MARKER_TICK_SPAN 6 /* degrees wide, the green "finish line" tick on the gauge ring */

static const char *USINAGE_TAG = "usinage"; /* phase-transition logging, see idf.py monitor */

static lv_obj_t *scr_gauge;   /* mm wave screen (default) */
static lv_obj_t *scr_battery; /* battery screen, reached by swiping right */
static lv_obj_t *scr_apps;    /* app launcher screen, reached by swiping left */

static lv_obj_t *arc;
static lv_obj_t *percent_label;
static lv_obj_t *gauge_title_label; /* small caption above the value; shows which app is active */
static lv_obj_t *calib_confirm_btn; /* green check: validates / advances */
static lv_obj_t *calib_cancel_btn;  /* red cross: discards and stops */
static lv_obj_t *target_marker_arc; /* green tick on the ring: where the target value sits */

static lv_obj_t *batt_arc;
static lv_obj_t *batt_label;

static lv_obj_t *elevator_arc; /* curved scroll indicator on the apps screen */

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

/* Keeps the curved cursor in sync with how far the app list is scrolled:
 * scroll_top + scroll_bottom is the total scrollable distance regardless of
 * the current position, so their ratio gives a stable 0-100% position.
 * The cursor's start/end angles are set directly (not via lv_arc_set_value)
 * so it stays a fixed-width curved segment sliding along the track, rather
 * than a pie-slice that grows from one end. */
static void app_grid_scroll_cb(lv_event_t *e)
{
    lv_obj_t *grid = lv_event_get_target(e);
    lv_coord_t scrolled = lv_obj_get_scroll_top(grid);
    lv_coord_t total = scrolled + lv_obj_get_scroll_bottom(grid);
    int32_t pct = (total > 0) ? (int32_t)(((int64_t)scrolled * 100) / total) : 0;

    int32_t travel = ELEVATOR_TRACK_SPAN - ELEVATOR_CURSOR_SPAN;
    int32_t cursor_start = ELEVATOR_TRACK_START + (pct * travel) / 100;
    lv_arc_set_angles(elevator_arc, cursor_start, cursor_start + ELEVATOR_CURSOR_SPAN);
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
static void set_arc_value(void *obj, int32_t t)
{
    (void)obj;
    float angle_rad = (2.0f * (float)M_PI) * ((float)t / CYCLE_UNITS);
    current_cmm = (int32_t)lroundf(MM_PEAK_CMM * sinf(angle_rad));
    apply_mm_display(current_cmm);
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
    else if (scr_cur == scr_apps && dir == LV_DIR_RIGHT)
    {
        lv_scr_load_anim(scr_gauge, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
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

    /* --- app launcher screen: up to 15 placeholder circles, 3 per row,
     * scrolling down through the "floors" like an elevator --- */
    scr_apps = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_apps, lv_color_black(), 0);
    lv_obj_clear_flag(scr_apps, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr_apps, gesture_cb, LV_EVENT_GESTURE, NULL);

    lv_obj_t *grid = lv_obj_create(scr_apps);
    lv_obj_remove_style_all(grid); /* plain layout container, no border/bg of its own */
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    /* Width is exactly the icon content (no extra gutter) so lv_obj_center
     * below puts the icons themselves on the true screen center - the
     * elevator sits well clear of them anyway (it's a thin cursor near the
     * bezel, not a full-width bar). */
    lv_obj_set_size(grid, APP_GRID_COLS * APP_ICON_SIZE + (APP_GRID_COLS - 1) * APP_ICON_GAP, APP_GRID_VIEWPORT_H);
    lv_obj_center(grid);
    lv_obj_set_style_pad_row(grid, APP_ICON_GAP, 0);
    lv_obj_set_style_pad_column(grid, APP_ICON_GAP, 0);
    /* Blank margin before the 1st row and after the last one: at rest (not
     * scrolled) this centers the first 2 rows / 6 icons in the viewport,
     * and mirrors it at the far end once the last row is reached. */
    lv_obj_set_style_pad_top(grid, APP_ROW_PAD, 0);
    lv_obj_set_style_pad_bottom(grid, APP_ROW_PAD, 0);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    /* Only vertical scrolling, so a horizontal swipe still falls through to
     * gesture_cb (switching screens) instead of being eaten as a scroll. */
    lv_obj_set_scroll_dir(grid, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(grid, LV_SCROLLBAR_MODE_OFF); /* replaced by elevator_arc below */
    lv_obj_add_event_cb(grid, app_grid_scroll_cb, LV_EVENT_SCROLL, NULL);

    for (int i = 0; i < APP_ICON_COUNT; i++)
    {
        lv_obj_t *icon = lv_obj_create(grid);
        lv_obj_set_size(icon, APP_ICON_SIZE, APP_ICON_SIZE);
        lv_obj_set_style_radius(icon, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(icon, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_set_style_bg_color(icon, lv_palette_darken(LV_PALETTE_GREY, 2), LV_STATE_PRESSED);
        lv_obj_set_style_border_width(icon, 0, 0);
        lv_obj_clear_flag(icon, LV_OBJ_FLAG_SCROLLABLE);

        if (i == 0)
        {
            /* First app, wired up for real: Usinage. A gear stands in well
             * for "machining" and is already in the bundled symbol font -
             * no custom artwork needed. The rest stay grey placeholders
             * until they get their own feature/icon. */
            lv_obj_t *icon_glyph = lv_label_create(icon);
            lv_obj_set_style_text_font(icon_glyph, &lv_font_montserrat_48, 0);
            lv_obj_set_style_text_color(icon_glyph, lv_color_white(), 0);
            lv_label_set_text(icon_glyph, LV_SYMBOL_SETTINGS);
            lv_obj_center(icon_glyph);
            lv_obj_add_event_cb(icon, usinage_icon_cb, LV_EVENT_CLICKED, NULL);
        }
        else
        {
            lv_obj_add_event_cb(icon, app_icon_cb, LV_EVENT_CLICKED, NULL);
        }
    }

    /* Curved "elevator" scroll indicator: a dim 45 deg track hugging the
     * right edge of the round screen, with a short bright segment (the
     * cursor) sliding along it. Both are real arc strokes - genuinely bent
     * along the bezel's curvature, not a straight rectangle - and there's
     * no knob at all here; app_grid_scroll_cb drives the cursor's angles
     * directly via lv_arc_set_angles(). */
    elevator_arc = lv_arc_create(scr_apps);
    lv_obj_set_size(elevator_arc, ELEVATOR_DIAM, ELEVATOR_DIAM);
    lv_obj_center(elevator_arc);
    lv_arc_set_rotation(elevator_arc, 0);
    lv_arc_set_bg_angles(elevator_arc, ELEVATOR_TRACK_START, ELEVATOR_HALF_SPAN);
    lv_obj_remove_style(elevator_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(elevator_arc, LV_OBJ_FLAG_CLICKABLE); /* display only, not draggable */

    lv_obj_set_style_arc_color(elevator_arc, lv_palette_darken(LV_PALETTE_GREY, 3), LV_PART_MAIN);
    lv_obj_set_style_arc_width(elevator_arc, ELEVATOR_TRACK_W, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(elevator_arc, true, LV_PART_MAIN);

    lv_obj_set_style_arc_color(elevator_arc, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(elevator_arc, ELEVATOR_CURSOR_W, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(elevator_arc, true, LV_PART_INDICATOR);
    /* Start at rest (scroll = 0%): cursor sits at the near end of the track. */
    lv_arc_set_angles(elevator_arc, ELEVATOR_TRACK_START, ELEVATOR_TRACK_START + ELEVATOR_CURSOR_SPAN);
}
