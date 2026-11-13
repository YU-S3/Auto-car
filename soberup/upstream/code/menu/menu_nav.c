#include "menu_nav.h"

#include <limits.h>
#include <string.h>

static uint8_t menu_nav_key_is_valid(MenuKey key)
{
    return (uint8_t)(key >= MENU_KEY_NONE && key <= MENU_KEY_RIGHT);
}

static uint8_t menu_nav_page_item_count(const MenuPage *page)
{
    if (page == NULL || page->items == NULL) {
        return 0U;
    }

    return page->item_count;
}

static void menu_nav_normalize_frame(MenuFrame *frame)
{
    uint8_t item_count;

    if (frame == NULL) {
        return;
    }

    item_count = menu_nav_page_item_count(frame->page);
    if (item_count == 0U) {
        frame->cursor = 0U;
        frame->scroll = 0U;
        return;
    }

    if (frame->cursor >= item_count) {
        frame->cursor = (uint8_t)(item_count - 1U);
    }
    if (frame->scroll >= item_count || frame->scroll > frame->cursor) {
        frame->scroll = frame->cursor;
    }
}

static uint8_t menu_nav_float_position_count(const MenuFloatBinding *binding)
{
    uint16_t position_count;
    uint8_t integer_digits;

    if (binding == NULL || binding->target == NULL) {
        return 0U;
    }

    integer_digits = menu_nav_integer_digits(*binding->target);
    position_count = integer_digits;
    if (binding->decimal_digits > 0U) {
        position_count = (uint16_t)(position_count + 1U + binding->decimal_digits);
    }

    if (position_count > UINT8_MAX) {
        position_count = UINT8_MAX;
    }

    return (uint8_t)position_count;
}

static uint8_t menu_nav_int_position_count(const MenuIntBinding *binding)
{
    if (binding == NULL || binding->target == NULL) {
        return 0U;
    }

    if (binding->step > 0) {
        return 1U;
    }
    return menu_nav_integer_digits((float)*binding->target);
}

static uint8_t menu_nav_current_position_count(const MenuItem *item)
{
    if (item == NULL) {
        return 0U;
    }

    switch (item->type) {
        case MENU_ITEM_FLOAT:
            return menu_nav_float_position_count(&item->data.float_value);

        case MENU_ITEM_INT:
            return menu_nav_int_position_count(&item->data.int_value);

        default:
            return 0U;
    }
}

static void menu_nav_normalize_digit_position(MenuContext *context,
                                               const MenuItem *item)
{
    uint8_t position_count;

    if (context == NULL) {
        return;
    }

    position_count = menu_nav_current_position_count(item);
    if (position_count == 0U) {
        context->digit_position = 0U;
    } else if (context->digit_position >= position_count) {
        context->digit_position = (uint8_t)(position_count - 1U);
    }
}

static float menu_nav_float_power_of_ten(uint8_t exponent, uint8_t reciprocal)
{
    float result = 1.0f;

    while (exponent > 0U) {
        result = reciprocal ? result / 10.0f : result * 10.0f;
        exponent--;
    }

    return result;
}

static int64_t menu_nav_int_power_of_ten(uint8_t exponent)
{
    int64_t result = 1;

    while (exponent > 0U && result <= (INT64_MAX / 10)) {
        result *= 10;
        exponent--;
    }

    return result;
}

static void menu_nav_mark_changed(MenuContext *context)
{
    menu_nav_request_redraw(context);
}

static void menu_nav_adjust_float(MenuContext *context,
                                  const MenuItem *item,
                                  int8_t direction)
{
    const MenuFloatBinding *binding;
    uint8_t integer_digits;
    uint8_t digit_position;
    float step;
    float old_value;
    float new_value;

    if (context == NULL || item == NULL || item->type != MENU_ITEM_FLOAT) {
        return;
    }

    binding = &item->data.float_value;
    if (binding->target == NULL || direction == 0 ||
        binding->minimum != binding->minimum ||
        binding->maximum != binding->maximum ||
        binding->minimum > binding->maximum) {
        return;
    }

    old_value = *binding->target;
    if (old_value != old_value) {
        return;
    }

    integer_digits = menu_nav_integer_digits(old_value);
    digit_position = context->digit_position;
    if (binding->decimal_digits > 0U && digit_position == integer_digits) {
        return;
    }

    if (digit_position < integer_digits) {
        step = menu_nav_float_power_of_ten(
            (uint8_t)(integer_digits - digit_position - 1U), 0U);
    } else if (binding->decimal_digits > 0U && digit_position > integer_digits) {
        step = menu_nav_float_power_of_ten(
            (uint8_t)(digit_position - integer_digits), 1U);
    } else {
        return;
    }

    new_value = old_value + ((direction > 0) ? step : -step);
    if (new_value < binding->minimum) {
        new_value = binding->minimum;
    } else if (new_value > binding->maximum) {
        new_value = binding->maximum;
    }

    if (new_value != old_value) {
        *binding->target = new_value;
        menu_nav_mark_changed(context);
        if (binding->on_change != NULL) {
            binding->on_change(context);
        }
    }
}

static void menu_nav_adjust_int(MenuContext *context,
                                const MenuItem *item,
                                int8_t direction)
{
    const MenuIntBinding *binding;
    uint8_t integer_digits;
    int64_t step;
    int64_t new_value;
    int old_value;

    if (context == NULL || item == NULL || item->type != MENU_ITEM_INT) {
        return;
    }

    binding = &item->data.int_value;
    if (binding->target == NULL || direction == 0 ||
        binding->minimum > binding->maximum) {
        return;
    }

    old_value = *binding->target;
    if (binding->step > 0) {
        step = binding->step;
    } else {
        integer_digits = menu_nav_integer_digits((float)old_value);
        if (context->digit_position >= integer_digits) {
            return;
        }
        step = menu_nav_int_power_of_ten(
            (uint8_t)(integer_digits - context->digit_position - 1U));
    }
    new_value = (int64_t)old_value + ((direction > 0) ? step : -step);
    if (new_value < (int64_t)binding->minimum) {
        new_value = binding->minimum;
    } else if (new_value > (int64_t)binding->maximum) {
        new_value = binding->maximum;
    }

    if ((int)new_value != old_value) {
        *binding->target = (int)new_value;
        menu_nav_mark_changed(context);
    }
}

static void menu_nav_handle_edit_key(MenuContext *context, MenuKey key)
{
    const MenuItem *item;
    uint8_t position_count;

    if (context == NULL) {
        return;
    }

    item = menu_nav_current_item(context);
    if (item == NULL ||
        (item->type != MENU_ITEM_FLOAT && item->type != MENU_ITEM_INT)) {
        context->mode = MENU_MODE_BROWSE;
        context->digit_position = 0U;
        menu_nav_request_redraw(context);
        return;
    }

    menu_nav_normalize_digit_position(context, item);
    position_count = menu_nav_current_position_count(item);
    if (position_count == 0U) {
        context->mode = MENU_MODE_BROWSE;
        context->digit_position = 0U;
        menu_nav_request_redraw(context);
        return;
    }

    switch (key) {
        case MENU_KEY_PRESS:
            context->mode = MENU_MODE_BROWSE;
            context->digit_position = 0U;
            menu_nav_request_redraw(context);
            break;

        case MENU_KEY_UP:
            if (item->type == MENU_ITEM_FLOAT) {
                menu_nav_adjust_float(context, item, 1);
            } else {
                menu_nav_adjust_int(context, item, 1);
            }
            menu_nav_normalize_digit_position(context, item);
            menu_nav_request_redraw(context);
            break;

        case MENU_KEY_DOWN:
            if (item->type == MENU_ITEM_FLOAT) {
                menu_nav_adjust_float(context, item, -1);
            } else {
                menu_nav_adjust_int(context, item, -1);
            }
            menu_nav_normalize_digit_position(context, item);
            menu_nav_request_redraw(context);
            break;

        case MENU_KEY_LEFT:
            if (context->digit_position == 0U) {
                context->digit_position = (uint8_t)(position_count - 1U);
            } else {
                context->digit_position--;
            }
            menu_nav_request_redraw(context);
            break;

        case MENU_KEY_RIGHT:
            context->digit_position++;
            if (context->digit_position >= position_count) {
                context->digit_position = 0U;
            }
            menu_nav_request_redraw(context);
            break;

        case MENU_KEY_NONE:
        default:
            break;
    }
}

void menu_nav_go_back(MenuContext *context)
{
    MenuFrame *frame;

    if (context == NULL || context->depth <= 1U ||
        context->depth > MENU_NAV_MAX_DEPTH) {
        return;
    }

    frame = &context->frames[context->depth - 1U];
    if (frame->page != NULL && frame->page->on_leave != NULL) {
        frame->page->on_leave(context);
    }

    memset(frame, 0, sizeof(*frame));
    context->depth--;
    context->mode = MENU_MODE_BROWSE;
    context->digit_position = 0U;
    menu_nav_current_frame(context);
    menu_nav_request_redraw(context);
}

void menu_nav_open_page(MenuContext *context, const MenuPage *page)
{
    MenuFrame *frame;

    if (context == NULL || page == NULL || context->depth == 0U ||
        context->depth >= MENU_NAV_MAX_DEPTH) {
        return;
    }

    frame = &context->frames[context->depth];
    memset(frame, 0, sizeof(*frame));
    frame->page = page;
    context->depth++;
    context->mode = MENU_MODE_BROWSE;
    context->digit_position = 0U;
    menu_nav_request_redraw(context);

    if (page->on_enter != NULL) {
        page->on_enter(context);
    }
    menu_nav_current_frame(context);
}

static void menu_nav_handle_browse_press(MenuContext *context,
                                         const MenuItem *item)
{
    uint8_t current_choice;

    if (context == NULL || item == NULL) {
        return;
    }

    switch (item->type) {
        case MENU_ITEM_SUBMENU:
        case MENU_ITEM_CUSTOM:
            menu_nav_open_page(context, item->data.page);
            break;

        case MENU_ITEM_ACTION:
            if (item->data.action != NULL) {
                item->data.action(context);
                menu_nav_request_redraw(context);
            }
            break;

        case MENU_ITEM_FLOAT:
            if (item->data.float_value.target != NULL) {
                context->mode = MENU_MODE_EDIT;
                context->digit_position = 0U;
                menu_nav_request_redraw(context);
            }
            break;

        case MENU_ITEM_INT:
            if (item->data.int_value.target != NULL) {
                context->mode = MENU_MODE_EDIT;
                context->digit_position = 0U;
                menu_nav_request_redraw(context);
            }
            break;

        case MENU_ITEM_CHOICE:
            if (item->data.choice.get != NULL &&
                item->data.choice.set != NULL &&
                item->data.choice.count > 0U) {
                current_choice = item->data.choice.get();
                current_choice = (uint8_t)(((uint16_t)current_choice + 1U) %
                                           item->data.choice.count);
                item->data.choice.set(current_choice);
                menu_nav_mark_changed(context);
            }
            break;

        case MENU_ITEM_TOGGLE:
            if (item->data.toggle.toggle != NULL) {
                item->data.toggle.toggle();
                menu_nav_mark_changed(context);
            }
            break;

        case MENU_ITEM_STATUS:
        default:
            break;
    }
}

void menu_nav_init(MenuContext *context, const MenuPage *root_page)
{
    if (context == NULL) {
        return;
    }

    memset(context, 0, sizeof(*context));
    context->mode = MENU_MODE_BROWSE;

    if (root_page == NULL) {
        return;
    }

    context->frames[0].page = root_page;
    context->depth = 1U;
    context->redraw = 1U;

    if (root_page->on_enter != NULL) {
        root_page->on_enter(context);
    }
    menu_nav_current_frame(context);
}

void menu_nav_handle_key(MenuContext *context, MenuKey key)
{
    MenuFrame *frame;
    const MenuPage *page;
    const MenuItem *item;
    uint8_t item_count;

    if (context == NULL || !menu_nav_key_is_valid(key)) {
        return;
    }

    frame = menu_nav_current_frame(context);
    if (frame == NULL || frame->page == NULL) {
        return;
    }
    page = frame->page;

    if (context->mode == MENU_MODE_EDIT) {
        menu_nav_handle_edit_key(context, key);
        return;
    }

    if (key == MENU_KEY_LEFT && page->live && page->custom_key != NULL) {
        page->custom_key(context, key);
        return;
    }

    if (key == MENU_KEY_LEFT) {
        menu_nav_go_back(context);
        return;
    }

    if (page->custom_key != NULL) {
        page->custom_key(context, key);
        return;
    }

    item_count = menu_nav_page_item_count(page);
    if (item_count == 0U) {
        return;
    }

    switch (key) {
        case MENU_KEY_UP:
            if (frame->cursor == 0U) {
                frame->cursor = (uint8_t)(item_count - 1U);
            } else {
                frame->cursor--;
            }
            menu_nav_normalize_frame(frame);
            menu_nav_request_redraw(context);
            break;

        case MENU_KEY_DOWN:
            frame->cursor++;
            if (frame->cursor >= item_count) {
                frame->cursor = 0U;
            }
            menu_nav_normalize_frame(frame);
            menu_nav_request_redraw(context);
            break;

        case MENU_KEY_PRESS:
            item = menu_nav_current_item(context);
            menu_nav_handle_browse_press(context, item);
            break;

        case MENU_KEY_NONE:
        case MENU_KEY_RIGHT:
        default:
            break;
    }
}

void menu_nav_tick(MenuContext *context)
{
    const MenuPage *page = menu_nav_current_page(context);

    if (page != NULL && page->custom_tick != NULL) {
        page->custom_tick(context);
    }
}

const MenuPage *menu_nav_current_page(const MenuContext *context)
{
    if (context == NULL || context->depth == 0U ||
        context->depth > MENU_NAV_MAX_DEPTH) {
        return NULL;
    }

    return context->frames[context->depth - 1U].page;
}

const MenuItem *menu_nav_current_item(const MenuContext *context)
{
    const MenuFrame *frame;
    const MenuPage *page;

    if (context == NULL || context->depth == 0U ||
        context->depth > MENU_NAV_MAX_DEPTH) {
        return NULL;
    }

    frame = &context->frames[context->depth - 1U];
    page = frame->page;
    if (page == NULL || page->items == NULL || page->item_count == 0U ||
        frame->cursor >= page->item_count) {
        return NULL;
    }

    return &page->items[frame->cursor];
}

MenuFrame *menu_nav_current_frame(MenuContext *context)
{
    MenuFrame *frame;

    if (context == NULL || context->depth == 0U ||
        context->depth > MENU_NAV_MAX_DEPTH) {
        return NULL;
    }

    frame = &context->frames[context->depth - 1U];
    if (frame->page == NULL) {
        return NULL;
    }

    menu_nav_normalize_frame(frame);
    return frame;
}

void menu_nav_set_cursor(MenuContext *context, uint8_t cursor)
{
    MenuFrame *frame = menu_nav_current_frame(context);
    uint8_t item_count;

    if (frame == NULL) {
        return;
    }

    item_count = menu_nav_page_item_count(frame->page);
    if (item_count == 0U) {
        frame->cursor = 0U;
        frame->scroll = 0U;
    } else {
        frame->cursor = (cursor < item_count) ? cursor
                                              : (uint8_t)(item_count - 1U);
        if (frame->scroll > frame->cursor || frame->scroll >= item_count) {
            frame->scroll = frame->cursor;
        }
    }

    menu_nav_request_redraw(context);
}

void menu_nav_request_redraw(MenuContext *context)
{
    if (context != NULL) {
        context->redraw = 1U;
    }
}

float menu_nav_item_numeric_value(const MenuItem *item)
{
    if (item == NULL) {
        return 0.0f;
    }

    if (item->type == MENU_ITEM_FLOAT &&
        item->data.float_value.target != NULL) {
        return *item->data.float_value.target;
    }
    if (item->type == MENU_ITEM_INT && item->data.int_value.target != NULL) {
        return (float)*item->data.int_value.target;
    }

    return 0.0f;
}

uint8_t menu_nav_integer_digits(float value)
{
    uint8_t digits = 1U;

    if (value != value) {
        return digits;
    }
    if (value < 0.0f) {
        value = -value;
    }

    while (value >= 10.0f && digits < 10U) {
        value /= 10.0f;
        digits++;
    }

    return digits;
}
