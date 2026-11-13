#ifndef CODE_MENU_MENU_PAGES_H_
#define CODE_MENU_MENU_PAGES_H_

#include "menu_nav.h"

void menu_page_set_key(MenuContext *context, MenuKey key);
void menu_page_set_render(MenuContext *context);

void menu_page_img_enter(MenuContext *context);
void menu_page_img_key(MenuContext *context, MenuKey key);
void menu_page_img_tick(MenuContext *context);
void menu_page_img_render(MenuContext *context);

void menu_page_gyro_calibrate(MenuContext *context);
void menu_page_encoder_render(MenuContext *context);

#endif
