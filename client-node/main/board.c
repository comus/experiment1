#include <stdio.h>
#include "esp_log.h"
#include "iot_button.h"

#define TAG "BOARD"

#define BUTTON_ACTIVE_LEVEL     0

extern void example_ble_mesh_send_vendor_message(bool resend);

static void button_tap_cb(void* arg)
{
    example_ble_mesh_send_vendor_message(false);
}

static void board_button_init(void)
{
    button_handle_t btn_handle1 = iot_button_create(32, BUTTON_ACTIVE_LEVEL);
    iot_button_set_evt_cb(btn_handle1, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");

    button_handle_t btn_handle2 = iot_button_create(0, BUTTON_ACTIVE_LEVEL);
    iot_button_set_evt_cb(btn_handle2, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
}

void board_init(void)
{
    board_button_init();
}