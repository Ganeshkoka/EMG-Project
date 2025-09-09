#include "button_service.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>    // For button handling


//Button GPIO configuration (defined on back of board)
#define BUTTON1_PIN 11
#define BUTTON2_PIN 12

// gpio_callback is predefined struct in Zephyr for GPIO callbacks
// button_cb_data is instance of this struct; stores information about which functon to call when button is pressed
// static means only accessible within this file (button_service.c) like a private variable in C++
static struct gpio_callback button_cb_data; // used for STORAGE, that's it
// tells if button is pressed or not
static uint8_t button_state = BUTTON_STATE_IDLE;

// BLE service and characteristic UUIDs
static struct bt_uuid_128 button_service_uuid = BT_UUID_INIT_128(BT_UUID_BUTTON_SERVICE_VAL);
// we are only doing button presses, but under the same service we can have another char_id for button hold, double clicks, etc.
static struct bt_uuid_128 button_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0xF77F0D8F, 0xF3B9, 0x402E, 0xAD6A, 0x8D6928E56F82));

// Characteristic notification state
static bool notify_enabled;
// tells you which button was pressed
static uint8_t button_value;

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    notify_enabled = (value == BT_GATT_CCC_NOTIFY); // if value is 1, notify_enabled is true, else false
    printk("Notifications %s\n", notify_enabled ? "enabled" : "disabled");
}

// Define our service using auto-registration macro
BT_GATT_SERVICE_DEFINE(button_svc,
    BT_GATT_PRIMARY_SERVICE(&button_service_uuid),
    BT_GATT_CHARACTERISTIC(&button_char_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// This function will be called when a button is pressed
static void button_pressed_callback(
    const struct device *dev,    // Pointer to GPIO device that triggered interrupt
    struct gpio_callback *cb,    // Pointer to our callback structure, the connection is made to in the gpio_init_callback function
    uint32_t pins               // Bitmap showing which pins triggered the interrupt, for instance pin 11 is 0000100000000000
)
{
    // check which pin pressed using bitwises AND
    if (pins & BIT(BUTTON1_PIN)) {
        printk("Button 1 pressed\n");
        button_state = BUTTON_STATE_PRESSED;
        button_value = 0x01;  // Value for button 1
        if (notify_enabled) {
            bt_gatt_notify(NULL, &button_svc.attrs[2], &button_value, sizeof(button_value));
        }
        button_state = BUTTON_STATE_IDLE;  // Reset state after notification
    }
    else if (pins & BIT(BUTTON2_PIN)) {
        printk("Button 2 pressed\n");
        button_state = BUTTON_STATE_PRESSED;
        button_value = 0x02;  // Value for button 2
        if (notify_enabled) {
            bt_gatt_notify(NULL, &button_svc.attrs[2], &button_value, sizeof(button_value));
        }
        button_state = BUTTON_STATE_IDLE;  // Reset state after notification
    }
}


// Initialize our button service
// setup happens ONCE at startup
// this is the function called from main
int button_service_init(void)
{
    // read only, so you would never dereference it, just use it to read and pass memory to other functions (that's how it's designed)
    // note if you had multiple boards, each board runs its own separate firmware, so don't connect multiple nRF boards to one piece of code
    // gpio_dev stores memory address of this GPIO controller on specific board, not entire board itself
    // DT_NODELABEL gets the node label from device tree, gpio0 is the label for GPIO controller on nRF52840
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    int err;
    
    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return -1;
    }

    // Configure buttons as inputs with pull-up
    err = gpio_pin_configure(gpio_dev, BUTTON1_PIN, GPIO_INPUT | GPIO_PULL_UP); // tell the GPIO controller gpio_dev to configure pin 11 as an input and pull_up pin (enables tiny internal resistor that pulls pin's voltage to HIGH)
    err = gpio_pin_configure(gpio_dev, BUTTON2_PIN, GPIO_INPUT | GPIO_PULL_UP);

    // Initialize GPIO callback structure
    // gpio_init_callback() sets up an interrupt handler:
        // stores callbak function pointer (button_pressed_callback) in button_cb_data struct
        // when GPIO hardware detects change on pins 11 or 12, it triggers an interrupt, this interrupt handler looks at callback function and calls it
        // you need this interrupt beucause otherwise need constant polling which wastes power and misses quick presses
    gpio_init_callback(&button_cb_data,         // Address of our callback structure (WHERE to store info)
                      button_pressed_callback,   // points to function to call when button pressed (WHAT to call)
                      BIT(BUTTON1_PIN) | BIT(BUTTON2_PIN)); // pin_mask used by Zephyr to decide whether to call callback at all (it compares the triggered pin with this mask). Otherwise button_callback runs all the time, which is fine becuase you have a filter, but it's just inefficient.

    // NOTE: before the gpio_init_callback(), gpio_callback was just empty storage, the handler (function), pin mask, and linked list nodes not set
        // after, initialization, the handler is set to button_pressed_callback, pin mask set to 0x1800 (bits 11 and 12 set), ready for linked lists (merely initialized)
    
    // NOW, you are passing in your actual GPIO controller as well as address of callback structure
        // take initialized struct, add it to internal linked list of callbacks for that GPIO port, when interrupt occurs, it walks this list and calls each callback function in the list.
        // Note the node is initialized on the button_cb_data struct, and that node is added to GPIO port linked list!

    // this way you separate your concerns, you set up the callback data with initialization, and then you tell the driver about it with gpio_add_callback()

    // Add callback to GPIO
    // give button_cb_data to the GPIO controller gpio_dev, so it knows that if interrupt occurs on pins 11 or 12, execute function located at adddress stored in button_cb_data struct
    err = gpio_add_callback(gpio_dev, &button_cb_data);
    if (err) {
        printk("Could not add GPIO callback\n");
        return err;
    }

//   YOU COULD HAVE MULTIPLE CALLBACKS FOR SAME PINS!
        // one pin action can perform multiple actions
//   struct gpio_callback callback1, callback2;

//   gpio_init_callback(&callback1, function1, BIT(11));
//   gpio_init_callback(&callback2, function2, BIT(11));

//   gpio_add_callback(gpio_dev, &callback1);  // Both will be called
//   gpio_add_callback(gpio_dev, &callback2);  // when pin 11 changes

    // Enable interrupts on button pins
    err = gpio_pin_interrupt_configure(gpio_dev, BUTTON1_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    if (err) {
        printk("Could not configure Button 1 interrupt\n");
        return err;
    }

    err = gpio_pin_interrupt_configure(gpio_dev, BUTTON2_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    if (err) {
        printk("Could not configure Button 2 interrupt\n");
        return err;
    }

    printk("Button service registered automatically\n");

    return 0;
}