# nRF BLE Button Service Project

## Overview

This project implements a Bluetooth Low Energy (BLE) button service on an nRF development board using the Zephyr RTOS. The system uses interrupt-driven GPIO handling to detect button presses and sends notifications to connected BLE clients (like a Jetson) in a power-efficient manner.

## Architecture Overview

### Core Components

1. **Main Application** (`src/main.c`): 
   - Initializes the button service
   - Enables Bluetooth subsystem
   - Enters infinite sleep mode for power efficiency

2. **Button Service** (`src/button_service.c`):
   - Custom BLE GATT service for button interactions
   - GPIO interrupt handling for physical buttons
   - BLE notification system to communicate button states

## How It Works - The Complete Flow

### 1. System Initialization (`button_service_init()`)

The system starts by calling `button_service_init()` before entering sleep mode. This one-time setup function:

#### GPIO Controller Setup
```c
const struct device *gpio_dev = device_get_binding("GPIO_0");
```
- Gets a pointer to the GPIO controller on the nRF board
- **Under the hood**: Zephyr maps physical hardware GPIO controller addresses to memory when booting using device tree descriptions
- `gpio_dev` literally points to hardware registers for GPIO control

#### Pin Configuration
```c
gpio_pin_configure(gpio_dev, BUTTON1_PIN, GPIO_INPUT | GPIO_PULL_UP);
```
- Configures pins 11 and 12 as input pins with pull-up resistors
- **Pull-up choice**: Simpler circuitry design - internal resistor pulls pin voltage HIGH by default
- Button press connects pin to ground (LOW), creating a detectable voltage change

#### Interrupt Callback Setup
This is where the magic happens with a two-step process:

**Step 1: Initialize the callback struct**
```c
static struct gpio_callback button_cb_data;  // Storage allocation
gpio_init_callback(&button_cb_data, button_pressed_callback, BIT(BUTTON1_PIN) | BIT(BUTTON2_PIN));
```

**What this does**:
- Fills `button_cb_data` struct with:
  - `handler`: Points to your `button_pressed_callback` function
  - `pin_mask`: 32-bit number with bits 11&12 set to 1 (0x1800)
  - `node`: Linked list node for chaining callbacks

**Step 2: Register with GPIO controller**
```c
gpio_add_callback(gpio_dev, &button_cb_data);
```
- Adds your callback struct to the GPIO controller's internal linked list
- Now when ANY pin on this GPIO controller interrupts, Zephyr will check your callback

#### Interrupt Configuration
```c
gpio_pin_interrupt_configure(gpio_dev, BUTTON1_PIN, GPIO_INT_EDGE_TO_ACTIVE);
```
- Enables hardware interrupts on pins 11&12
- `GPIO_INT_EDGE_TO_ACTIVE`: Triggers when pin goes from HIGH to LOW (button press)

#### BLE Service Registration
```c
bt_gatt_service_register(&button_svc);
```
- Registers the BLE GATT service with Zephyr's Bluetooth stack
- Makes the service discoverable to BLE clients

### 2. The Interrupt-Driven Flow

Once initialization is complete, the system enters `k_sleep(K_FOREVER)` and waits for interrupts.

#### When Button is Pressed:

1. **Hardware Level**: Button press pulls GPIO pin LOW
2. **CPU Interrupt**: GPIO controller triggers hardware interrupt
3. **Zephyr GPIO ISR**: Interrupt service routine reads which pins triggered
4. **Callback List Walk**: 
   ```c
   // Zephyr internally does this:
   triggered_pins = read_interrupt_register();  // e.g., 0x0800 for pin 11
   
   for (each callback in gpio_dev->callback_list) {
       if (triggered_pins & callback->pin_mask) {  // 0x0800 & 0x1800 = 0x0800 (match!)
           callback->handler(dev, callback, triggered_pins);  // Call your function!
       }
   }
   ```
5. **Your Callback Executes**: `button_pressed_callback()` runs with the triggered pins

#### Inside `button_pressed_callback()`:

**Pin Identification**:
```c
// pins parameter contains which pins actually triggered (not your full mask!)
if (pins & BIT(BUTTON1_PIN)) {  // Check if pin 11 was in the triggered set
    // pins could be 0x0800 (only pin 11) or 0x1800 (both pins)
    // BIT(BUTTON1_PIN) is 0x0800 (only pin 11)
    // 0x0800 & 0x0800 = 0x0800 (non-zero = TRUE)
}
```

**State Management & Notification**:
- Sets `button_state = BUTTON_STATE_PRESSED`
- Sets `button_value` to identify which button (0x01 or 0x02)
- If notifications enabled, sends BLE notification via `bt_gatt_notify()`

### 3. BLE GATT Service Architecture

#### The GATT Attribute Array
```c
static const struct bt_gatt_attr attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&button_service_uuid),     // Service declaration
    BT_GATT_CHARACTERISTIC(&button_char_uuid.uuid,     // Characteristic declaration
                          BT_GATT_CHRC_NOTIFY,         // Supports notifications
                          BT_GATT_PERM_NONE,           // No read/write permissions
                          NULL, NULL, NULL),            // No read/write callbacks
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),  // Client config descriptor
};
```

**What each macro creates**:

1. **`BT_GATT_PRIMARY_SERVICE`**: Creates a GATT attribute that declares "here's a service with this UUID"
2. **`BT_GATT_CHARACTERISTIC`**: Creates a characteristic attribute with notification properties
3. **`BT_GATT_CCC`**: Creates a Client Characteristic Configuration descriptor that allows BLE clients to enable/disable notifications

#### The Service Wrapper
```c
static struct bt_gatt_service button_svc = BT_GATT_SERVICE(attrs);
```
- Packages the individual attributes into a cohesive service
- Provides the interface for Zephyr's BLE stack to manage the service
- Required for service registration with `bt_gatt_service_register()`

### 4. BLE Client Interaction (Jetson Side)

When your Jetson wants to receive button notifications:

1. **Service Discovery**: Jetson scans and finds your button service UUID
2. **Characteristic Discovery**: Finds the button characteristic and its CCC descriptor
3. **Enable Notifications**: Jetson writes `0x0001` to the CCC descriptor
4. **CCC Callback**: Should trigger `ccc_cfg_changed()` to set `notify_enabled = true`
5. **Receive Notifications**: Jetson gets notified when buttons are pressed

## Technical Details

### Power Efficiency Design
- **Event-driven architecture**: CPU sleeps until hardware wakes it up
- **No polling**: Interrupts provide instant response without wasting CPU cycles
- **Low power modes**: Zephyr can enter deep sleep between button presses

### GPIO Interrupt Mechanism
- **Interrupt vs Polling**: Interrupts are vastly more efficient than constantly checking pin states
- **Multiple Callbacks**: GPIO controller maintains a linked list of callbacks per GPIO port
- **Pin Filtering**: Each callback has a pin mask - only relevant callbacks execute
- **Hardware Integration**: Direct connection to CPU interrupt controller for minimal latency

### BLE Stack Integration
- **GATT Table**: Attributes array becomes part of the device's GATT database
- **Standard Compliance**: Uses standard Bluetooth GATT service/characteristic structure
- **Notification System**: Leverages BLE's built-in notification mechanism for efficient data transfer

## Current Code Issues

### Missing CCC Callback
**Problem**: The `BT_GATT_CCC` macro has `NULL` for the callback function, so `notify_enabled` is never set to `true`.

**Fix needed**:
```c
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notifications %s\n", notify_enabled ? "enabled" : "disabled");
}

// In attrs array:
BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
```

### Button State Management
**Issue**: `button_state` never returns to `BUTTON_STATE_IDLE` after being pressed.

## Hardware Configuration

- **Target**: nRF52840 development board
- **Button pins**: GPIO 11 and 12 (physical buttons on board)
- **GPIO controller**: "GPIO_0" (primary GPIO port on nRF52840)
- **Bluetooth**: Nordic's integrated BLE hardware with Zephyr's Bluetooth stack

## Build Instructions

This project uses Zephyr's build system:

```bash
west build
west flash
```

Configuration is managed through `prj.conf` and `CMakeLists.txt`.

## Service Details

- **Service UUID**: `E3C5D1FC-1E61-4133-8B8E-BA7270ADA49F`
- **Characteristic UUID**: `E3C5D1FD-1E61-4133-8B8E-BA7270ADA49F`
- **Button Values**: 
  - Button 1: `0x01`
  - Button 2: `0x02`
- **Button States**:
  - Idle: `0x00`
  - Pressed: `0x01`

## Usage

1. Flash the firmware to nRF board
2. Connect BLE client (Jetson) to the device
3. Subscribe to button characteristic notifications
4. Press buttons to receive notifications with button ID values

This system enables real-time, low-power communication between physical button interactions and remote BLE clients.



NOTE when building and flashing:
    - you should be able to build normally when you go to the nRF Connect extension and create build configuration and fix errors
    - the flashing is an issue, you can't use nrfutil and it becomes weird. you need to use sudo. and you need to be in the Nordic workspace context
        - cd /opt/nordic/ncs/v3.0.2
        - sudo /opt/nordic/ncs/toolchains/ef4fc6722e/bin/west flash -d /Users/ganeshkoka/Documents/GitHub/EMG-Project/nRF/build -r nrfjprog --dev-id 683199299

        - sudo /opt/nordic/ncs/toolchains/ef4fc6722e/bin/west flash -d /opt/nordic/ncs/v3.0.2/zephyr/samples/bluetooth/peripheral/build/peripheral -r nrfjprog --dev-id 683199299


Summary of Advertising Fix

  Original Problem: nRF device not discoverable by Jetson Opal - showing only unnamed MAC addresses in
  Bluetooth scans.

  Root Causes Found:

  1. Missing Scan Response Data (src/main.c:14-16)
    - Only had advertising data (ad), no scan response data (sd) with device name
    - Added: BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, ...)
  2. Wrong Advertising Timing (src/main.c:20-50)
    - Started advertising immediately after bt_enable() - too early
    - Fixed: Moved advertising to bt_ready() callback function
  3. Missing Configuration (prj.conf)
    - Added CONFIG_BT_SMP=y (required for privacy)
    - Added CONFIG_SETTINGS=y, CONFIG_FLASH=y, CONFIG_FLASH_MAP=y, CONFIG_NVS=y
    - Changed CONFIG_BT_PRIVACY=n to =y
  4. Settings Crash
    - settings_load() caused MPU fault without flash storage support
    - Fixed by adding flash/NVS configuration

  Final Result: Device now advertises as "nRF Button Service" and is discoverable.

  Re: UART: No, we used serial monitor over USB (not UART) to debug the crash and see MAC addresses. UART
   would be separate GPIO pins.