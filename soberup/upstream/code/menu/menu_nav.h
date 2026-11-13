#ifndef CODE_MENU_MENU_NAV_H_
#define CODE_MENU_MENU_NAV_H_

#include <stdint.h>
#include <stddef.h>

#define MENU_NAV_MAX_DEPTH 4U

typedef enum {
    MENU_KEY_NONE = 0,
    MENU_KEY_PRESS,
    MENU_KEY_UP,
    MENU_KEY_DOWN,
    MENU_KEY_LEFT,
    MENU_KEY_RIGHT
} MenuKey;

typedef enum {
    MENU_ITEM_SUBMENU = 0,
    MENU_ITEM_ACTION,
    MENU_ITEM_FLOAT,
    MENU_ITEM_INT,
    MENU_ITEM_CHOICE,
    MENU_ITEM_TOGGLE,
    MENU_ITEM_STATUS,
    MENU_ITEM_CUSTOM
} MenuItemType;

typedef enum {
    MENU_MODE_BROWSE = 0,
    MENU_MODE_EDIT
} MenuMode;

typedef struct MenuContext MenuContext;
typedef struct MenuPage MenuPage;

typedef void (*MenuActionFn)(MenuContext *context);
typedef uint8_t (*MenuChoiceGetFn)(void);
typedef void (*MenuChoiceSetFn)(uint8_t value);
typedef uint8_t (*MenuToggleGetFn)(void);
typedef void (*MenuToggleFn)(void);
typedef void (*MenuStatusFn)(char *buffer, size_t buffer_size);
typedef void (*MenuPageHookFn)(MenuContext *context);
typedef void (*MenuCustomKeyFn)(MenuContext *context, MenuKey key);
typedef void (*MenuCustomTickFn)(MenuContext *context);
typedef void (*MenuCustomRenderFn)(MenuContext *context);

typedef struct {
    float *target;
    float minimum;
    float maximum;
    uint8_t decimal_digits;
    MenuActionFn on_change;
} MenuFloatBinding;

typedef struct {
    int *target;
    int minimum;
    int maximum;
    int step;
} MenuIntBinding;

typedef struct MenuItem {
    const char *label;
    MenuItemType type;
    union {
        const MenuPage *page;
        MenuActionFn action;
        MenuFloatBinding float_value;
        MenuIntBinding int_value;
        struct {
            MenuChoiceGetFn get;
            MenuChoiceSetFn set;
            const char *const *labels;
            uint8_t count;
        } choice;
        struct {
            MenuToggleGetFn get;
            MenuToggleFn toggle;
        } toggle;
        MenuStatusFn status;
    } data;
} MenuItem;

struct MenuPage {
    const char *title;
    const MenuItem *items;
    uint8_t item_count;
    uint8_t live;
    MenuPageHookFn on_enter;
    MenuPageHookFn on_leave;
    MenuCustomKeyFn custom_key;
    MenuCustomTickFn custom_tick;
    MenuCustomRenderFn custom_render;
};

typedef struct {
    const MenuPage *page;
    uint8_t cursor;
    uint8_t scroll;
} MenuFrame;

struct MenuContext {
    MenuFrame frames[MENU_NAV_MAX_DEPTH];
    uint8_t depth;
    MenuMode mode;
    uint8_t digit_position;
    uint8_t redraw;
};

void menu_nav_init(MenuContext *context, const MenuPage *root_page);
void menu_nav_handle_key(MenuContext *context, MenuKey key);
void menu_nav_tick(MenuContext *context);
void menu_nav_go_back(MenuContext *context);
void menu_nav_open_page(MenuContext *context, const MenuPage *page);
const MenuPage *menu_nav_current_page(const MenuContext *context);
const MenuItem *menu_nav_current_item(const MenuContext *context);
MenuFrame *menu_nav_current_frame(MenuContext *context);
void menu_nav_set_cursor(MenuContext *context, uint8_t cursor);
void menu_nav_request_redraw(MenuContext *context);
float menu_nav_item_numeric_value(const MenuItem *item);
uint8_t menu_nav_integer_digits(float value);

#endif
