#include <math.h>
#include <stdlib.h>
#include "lvgl.h"
#include "qmi8658.h"

#define CYCLE_UNITS 3600 /* 0.1 degree resolution over one full sine cycle */

static lv_obj_t *arc;
static lv_obj_t *percent_label;

lv_obj_t *btn = NULL;

void btn_cb(lv_event_t *e)
{
    (void)e;
}

/* t runs linearly 0..CYCLE_UNITS forever; pct = 100*sin(t) gives a
 * perfectly smooth, continuously-differentiable 0->100->0->-100->0 wave
 * (no velocity discontinuity at the turning points, unlike a piecewise
 * linear ramp). */
static void set_arc_value(void *obj, int32_t t)
{
    (void)obj;
    float angle_rad = (2.0f * (float)M_PI) * ((float)t / CYCLE_UNITS);
    int32_t pct = (int32_t)lroundf(100.0f * sinf(angle_rad));

    lv_arc_set_value(arc, abs((int)pct));
    lv_label_set_text_fmt(percent_label, "%d%%", (int)pct);

    lv_color_t color = (pct < 0) ? lv_palette_main(LV_PALETTE_RED) : lv_palette_main(LV_PALETTE_BLUE);
    lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR);
}

void lvgl_demo_ui(lv_disp_t *disp)
{
    lv_disp_set_rotation(disp, LV_DISP_ROT_180);

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    arc = lv_arc_create(scr);
    lv_obj_set_size(arc, 440, 440);
    lv_obj_center(arc);
    lv_arc_set_rotation(arc, 270);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_range(arc, 0, 100);

    lv_obj_set_style_arc_width(arc, 34, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_palette_darken(LV_PALETTE_GREY, 4), LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 34, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_INDICATOR);

    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);

    percent_label = lv_label_create(scr);
    lv_obj_set_style_text_font(percent_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(percent_label, lv_color_white(), 0);
    lv_obj_center(percent_label);
    lv_label_set_text(percent_label, "0%");

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
