//as opposed to C++, use header files to declare functions and types

#ifndef BUTTON_SERVICE_H_    // Guards against multiple inclusion; IF BUTTON_SERVICE_H_ is not defined...
#define BUTTON_SERVICE_H_   // ...define BUTTON_SERVICE_H_

#include <zephyr/bluetooth/uuid.h>   // Defines BT_UUID_128_ENCODE
#include <zephyr/bluetooth/gatt.h>

// Define our custom service UUID
// Note: use uuidgen to generate a random UUID for production code E3C5D1FC-1E61-4133-8B8E-BA7270ADA49F
#define BT_UUID_BUTTON_SERVICE_VAL \
    BT_UUID_128_ENCODE(0xE3C5D1FC, 0x1E61, 0x4133, 0x8B8E, 0xBA7270ADA49F)

// Button states
#define BUTTON_STATE_IDLE     0x00
#define BUTTON_STATE_PRESSED  0x01

// Function declarations
int button_service_init(void);

#endif /* BUTTON_SERVICE_H_ */  // End of the conditional