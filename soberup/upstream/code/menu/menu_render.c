#include "menu_render.h"

#include "zf_common_headfile.h"
#include "zf_device_ips114.h"

#include <stdio.h>
#include <string.h>

#define MENU_RENDER_VISIBLE_ROWS  (7U)
#define MENU_RENDER_ROW_HEIGHT    (16U)
#define MENU_RENDER_LABEL_X       (16U)
#define MENU_RENDER_VALUE_X       (120U)

static void menu_render_clear_row(uint16 y)
{
    ips114_show_string(0, y, "                            ");
}

static void menu_render_float_text(float value,
                                   uint8 decimal_digits,
                                   char *buffer,
                                   size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0U) {
        return;
    }

    (void)snprintf(buffer, buffer_size, "%.*f", (int)decimal_digits, value);
}

static void menu_render_highlight_text(uint16 x,
                                       uint16 y,
                                       const char *text,
                                       uint8 digit_position,
                                       uint8 integer_digits,
                                       uint8 decimal_digits)
{
    uint8 text_length;
    uint8 highlight_index;
    uint8 decimal_index = integer_digits;
    uint8 i;

    if (text == NULL) {
        return;
    }

    text_length = (uint8)strlen(text);
    if (text_length == 0U) {
        return;
    }

    if (decimal_digits == 0U) {
        highlight_index = digit_position;
    } else {
        for (i = 0U; i < text_length; i++) {
            if (text[i] == '.') {
                decimal_index = i;
                break;
            }
        }

        if (digit_position < integer_digits) {
            highlight_index = digit_position;
        } else if (digit_position == integer_digits) {
            highlight_index = decimal_index;
        } else {
            highlight_index = (uint8)(decimal_index + digit_position - integer_digits);
        }
    }

    if (highlight_index >= text_length) {
        highlight_index = (uint8)(text_length - 1U);
    }

    for (i = 0U; i < text_length; i++) {
        if (i == highlight_index) {
            ips114_set_color(RGB565_RED, RGB565_WHITE);
        } else {
            ips114_set_color(RGB565_BLACK, RGB565_WHITE);
        }
        ips114_show_char((uint16)(x + i * 8U), y, text[i]);
    }
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
}

static void menu_render_item_value(MenuContext *context,
                                   const MenuItem *item,
                                   uint16 y,
                                   uint8 selected)
{
    char value_text[32] = {0};

    if (context == NULL || item == NULL) {
        return;
    }

    switch (item->type) {
        case MENU_ITEM_SUBMENU:
        case MENU_ITEM_CUSTOM:
            ips114_show_string(216, y, ">");
            break;

        case MENU_ITEM_FLOAT:
            if (item->data.float_value.target == NULL) {
                break;
            }
            menu_render_float_text(*item->data.float_value.target,
                                   item->data.float_value.decimal_digits,
                                   value_text,
                                   sizeof(value_text));
            if (selected && context->mode == MENU_MODE_EDIT) {
                menu_render_highlight_text(MENU_RENDER_VALUE_X,
                                           y,
                                           value_text,
                                           context->digit_position,
                                           menu_nav_integer_digits(*item->data.float_value.target),
                                           item->data.float_value.decimal_digits);
            } else {
                ips114_show_string(MENU_RENDER_VALUE_X, y, value_text);
            }
            break;

        case MENU_ITEM_INT:
            if (item->data.int_value.target == NULL) {
                break;
            }
            (void)snprintf(value_text,
                           sizeof(value_text),
                           "%d",
                           *item->data.int_value.target);
            if (selected && context->mode == MENU_MODE_EDIT) {
                menu_render_highlight_text(MENU_RENDER_VALUE_X,
                                           y,
                                           value_text,
                                           context->digit_position,
                                           menu_nav_integer_digits((float)*item->data.int_value.target),
                                           0U);
            } else {
                ips114_show_string(MENU_RENDER_VALUE_X, y, value_text);
            }
            break;

        case MENU_ITEM_CHOICE:
            if (item->data.choice.get != NULL &&
                item->data.choice.labels != NULL &&
                item->data.choice.count > 0U) {
                uint8 choice = item->data.choice.get();
                if (choice < item->data.choice.count) {
                    ips114_show_string(MENU_RENDER_VALUE_X,
                                       y,
                                       item->data.choice.labels[choice]);
                }
            }
            break;

        case MENU_ITEM_TOGGLE:
            if (item->data.toggle.get != NULL) {
                ips114_show_string(MENU_RENDER_VALUE_X,
                                   y,
                                   item->data.toggle.get() ? "ON " : "OFF");
            }
            break;

        case MENU_ITEM_STATUS:
            if (item->data.status != NULL) {
                item->data.status(value_text, sizeof(value_text));
                ips114_show_string(MENU_RENDER_VALUE_X, y, value_text);
            }
            break;

        case MENU_ITEM_ACTION:
        default:
            break;
    }
}

static void menu_render_list(MenuContext *context, const MenuPage *page)
{
    MenuFrame *frame;
    uint8 row;

    if (context == NULL || page == NULL) {
        return;
    }

    frame = menu_nav_current_frame(context);
    if (frame == NULL) {
        return;
    }

    if (page->item_count <= MENU_RENDER_VISIBLE_ROWS) {
        frame->scroll = 0U;
    } else {
        uint8 max_scroll = (uint8)(page->item_count - MENU_RENDER_VISIBLE_ROWS);
        if (frame->scroll > max_scroll) {
            frame->scroll = max_scroll;
        }
        if (frame->cursor < frame->scroll) {
            frame->scroll = frame->cursor;
        } else if (frame->cursor >=
                   (uint8)(frame->scroll + MENU_RENDER_VISIBLE_ROWS)) {
            frame->scroll = (uint8)(frame->cursor - MENU_RENDER_VISIBLE_ROWS + 1U);
        }
    }

    menu_render_clear_row(0U);
    if (page->title != NULL) {
        ips114_show_string(8, 0, page->title);
    }

    for (row = 0U; row < MENU_RENDER_VISIBLE_ROWS; row++) {
        uint8 item_index = (uint8)(frame->scroll + row);
        uint16 y = (uint16)((row + 1U) * MENU_RENDER_ROW_HEIGHT);

        menu_render_clear_row(y);
        if (item_index >= page->item_count || page->items == NULL) {
            continue;
        }

        if (item_index == frame->cursor) {
            ips114_show_string(0, y, ">");
        }
        if (page->items[item_index].label != NULL) {
            ips114_show_string(MENU_RENDER_LABEL_X,
                               y,
                               page->items[item_index].label);
        }
        menu_render_item_value(context,
                               &page->items[item_index],
                               y,
                               (uint8)(item_index == frame->cursor));
    }
}

void menu_render_current(MenuContext *context)
{
    const MenuPage *page;
    uint8 needs_redraw;

    if (context == NULL) {
        return;
    }

    page = menu_nav_current_page(context);
    if (page == NULL) {
        return;
    }

    needs_redraw = context->redraw;
    if (!needs_redraw && !page->live) {
        return;
    }

    if (needs_redraw) {
        ips114_clear();
    }

    if (page->custom_render != NULL) {
        page->custom_render(context);
    } else {
        menu_render_list(context, page);
    }

    context->redraw = 0U;
}
