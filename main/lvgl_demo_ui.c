#include <math.h>
#include <stdlib.h>
#include "esp_random.h"
#include "lvgl.h"
#include "qmi8658.h"

#define CYCLE_UNITS 3600 /* 0.1 degree resolution over one full sine cycle */
#define MM_PEAK 150      /* wave amplitude, in mm, in both directions */

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

static lv_obj_t *scr_gauge;   /* mm wave screen (default) */
static lv_obj_t *scr_battery; /* battery screen, reached by swiping right */
static lv_obj_t *scr_apps;    /* app launcher screen, reached by swiping left */

static lv_obj_t *arc;
static lv_obj_t *percent_label;

static lv_obj_t *batt_arc;
static lv_obj_t *batt_label;

static lv_obj_t *elevator_arc; /* curved scroll indicator on the apps screen */

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

/* t runs linearly 0..CYCLE_UNITS forever; mm = MM_PEAK*sin(t) gives a
 * perfectly smooth, continuously-differentiable 0->150->0->-150->0 wave
 * (no velocity discontinuity at the turning points, unlike a piecewise
 * linear ramp). */
static void set_arc_value(void *obj, int32_t t)
{
    (void)obj;
    float angle_rad = (2.0f * (float)M_PI) * ((float)t / CYCLE_UNITS);
    int32_t mm = (int32_t)lroundf(MM_PEAK * sinf(angle_rad));

    lv_arc_set_value(arc, abs((int)mm));
    lv_label_set_text_fmt(percent_label, "%dmm", (int)mm);

    lv_color_t color = (mm < 0) ? lv_palette_main(LV_PALETTE_RED) : lv_palette_main(LV_PALETTE_BLUE);
    lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR);
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

    arc = lv_arc_create(scr_gauge);
    lv_obj_set_size(arc, 440, 440);
    lv_obj_center(arc);
    lv_arc_set_rotation(arc, 270);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_range(arc, 0, MM_PEAK);

    lv_obj_set_style_arc_width(arc, 34, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_palette_darken(LV_PALETTE_GREY, 4), LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 34, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_INDICATOR);

    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);

    percent_label = lv_label_create(scr_gauge);
    lv_obj_set_style_text_font(percent_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(percent_label, lv_color_white(), 0);
    lv_obj_center(percent_label);
    lv_label_set_text(percent_label, "0mm");

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
        lv_obj_add_event_cb(icon, app_icon_cb, LV_EVENT_CLICKED, NULL);
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
