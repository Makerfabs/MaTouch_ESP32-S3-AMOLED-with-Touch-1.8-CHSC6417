// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: 1.8

#include "ui.h"


void ui_event_comp_Slider1_Slider1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    lv_obj_t ** comp_Slider1 = lv_event_get_user_data(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_slider_set_text_value(comp_Slider1[UI_COMP_SLIDER1_LABEL7], target, "", "");
    }
}

// COMPONENT Slider1

lv_obj_t * ui_Slider1_create(lv_obj_t * comp_parent)
{

    lv_obj_t * cui_Slider1;
    cui_Slider1 = lv_slider_create(comp_parent);
    lv_slider_set_value(cui_Slider1, 20, LV_ANIM_OFF);
    if(lv_slider_get_mode(cui_Slider1) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(cui_Slider1, 0, LV_ANIM_OFF);
    lv_obj_set_width(cui_Slider1, 200);
    lv_obj_set_height(cui_Slider1, 30);
    lv_obj_set_x(cui_Slider1, 0);
    lv_obj_set_y(cui_Slider1, 133);
    lv_obj_set_align(cui_Slider1, LV_ALIGN_CENTER);
    lv_obj_set_style_radius(cui_Slider1, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(cui_Slider1, lv_color_hex(0x454040), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(cui_Slider1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(cui_Slider1, lv_color_hex(0x0DD0F4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(cui_Slider1, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(cui_Slider1, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(cui_Slider1, 5, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(cui_Slider1, 5, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(cui_Slider1, lv_color_hex(0x0DD0F4), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(cui_Slider1, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(cui_Slider1, lv_color_hex(0x0DD0F4), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(cui_Slider1, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(cui_Slider1, 2, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(cui_Slider1, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(cui_Slider1, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

    lv_obj_t * cui_Label7;
    cui_Label7 = lv_label_create(cui_Slider1);
    lv_obj_set_width(cui_Label7, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(cui_Label7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(cui_Label7, 84);
    lv_obj_set_y(cui_Label7, 0);
    lv_obj_set_align(cui_Label7, LV_ALIGN_CENTER);
    lv_label_set_text(cui_Label7, "20");
    lv_obj_set_style_text_color(cui_Label7, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(cui_Label7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t ** children = lv_mem_alloc(sizeof(lv_obj_t *) * _UI_COMP_SLIDER1_NUM);
    children[UI_COMP_SLIDER1_SLIDER1] = cui_Slider1;
    children[UI_COMP_SLIDER1_LABEL7] = cui_Label7;
    lv_obj_add_event_cb(cui_Slider1, get_component_child_event_cb, LV_EVENT_GET_COMP_CHILD, children);
    lv_obj_add_event_cb(cui_Slider1, del_component_child_event_cb, LV_EVENT_DELETE, children);
    lv_obj_add_event_cb(cui_Slider1, ui_event_comp_Slider1_Slider1, LV_EVENT_ALL, children);
    ui_comp_Slider1_create_hook(cui_Slider1);
    return cui_Slider1;
}
