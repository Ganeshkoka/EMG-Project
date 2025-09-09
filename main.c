#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/settings/settings.h>
#include "button_service.h"

// Primary advertising data: flags + service UUID
static const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BUTTON_SERVICE_VAL),
};

// Scan response data: device name
static const struct bt_data sd[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void bt_ready(void)
{
        int err;
        bt_addr_le_t addr;
        char addr_str[BT_ADDR_LE_STR_LEN];

        printk("Bluetooth initialized\n");

        if (IS_ENABLED(CONFIG_SETTINGS)) {
                settings_load();
        }

        // Get and print our MAC address
        size_t count = 1;
        bt_id_get(&addr, &count);
        bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
        printk("Our MAC address: %s\n", addr_str);

        // Initialize button service
        err = button_service_init();
        if (err) {
                printk("Button service init failed (err %d)\n", err);
                return;
        }
        printk("Button service init successful\n");

        // Start advertising so Jetson can find us
        err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
        if (err) {
                printk("Advertising failed to start (err %d)\n", err);
                return;
        }
        printk("Advertising started successfully\n");
}

int main(void)
{
        int err;

        //kernel-level print used in embedded systems and kernels
        //different from standard C's printf because designed for embedded, works without standard C library,
                //can print even when full system isn't initialized, commonly used in bare metal and RTOS environments
        printk("Hello! Starting BLE application\n");

        //Initialize BLE
        err = bt_enable(NULL); // if err is 1, it means there is an error
        if (err) {
                printk("Bluetooth init failed (err %d)\n", err); // %d stands for signed decimal integer, gets replaced with the int you provide as argument
                return err;
        }

        bt_ready();

        // Main loop (in embedded systems, often an infinite loop, just sleeps)
        // when an event is received, wakes up and handles it, then goes back to sleep
        // doesn't block code execution, enters low-power state until an event occurs (hardware interrupt, timer, BLE events, etc.)
        while (1) {
                k_sleep(K_FOREVER);
        }

        return 0;
}

