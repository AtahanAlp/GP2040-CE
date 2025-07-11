#ifndef _ANALOG_MUX_H_
#define _ANALOG_MUX_H_

#include "gpaddon.h"         // Base class
#include "GamepadEnums.h"    // DpadMode, InvertMode etc.
#include "BoardConfig.h"     // May contain defaults
#include "enums.pb.h"        // For configuration enums potentially used in web UI
#include "types.h"           // For Pin_t etc.
#include "hardware/adc.h"    // Required for ADC functions
#include "hardware/gpio.h"   // Required for controlling MUX select pins

// Master enable for this addon
#ifndef ANALOG_MUX_ENABLED
#define ANALOG_MUX_ENABLED 1
#endif

// ----- MUX Configuration -----

// GPIO pins connected to the MUX select lines (S0, S1, S2, ...). -1 means unused.
// Order: S0 = LSB. Define as many as your MUX needs.
#ifndef ANALOG_MUX_S0_PIN
#define ANALOG_MUX_S0_PIN 00
#endif
#ifndef ANALOG_MUX_S1_PIN
#define ANALOG_MUX_S1_PIN 01
#endif
#ifndef ANALOG_MUX_S2_PIN
#define ANALOG_MUX_S2_PIN 15
#endif

// The ADC-capable GPIO pin connected to the MUX common output (SIG/Z).
// Must be GPIO 26, 27, 28, or 29 (ADC0, ADC1, ADC2, or ADC3).
#ifndef ANALOG_MUX_OUT_PIN
#define ANALOG_MUX_OUT_PIN 26
#endif

// MUX Channel assignments (0-indexed). Assign -1 if an input is not used.
#ifndef ANALOG_MUX_LX_CHANNEL
#define ANALOG_MUX_LX_CHANNEL 0 // Left Stick X
#endif
#ifndef ANALOG_MUX_LY_CHANNEL
#define ANALOG_MUX_LY_CHANNEL 1 // Left Stick Y
#endif
#ifndef ANALOG_MUX_RX_CHANNEL
#define ANALOG_MUX_RX_CHANNEL 2 // Right Stick X
#endif
#ifndef ANALOG_MUX_RY_CHANNEL
#define ANALOG_MUX_RY_CHANNEL 3 // Right Stick Y
#endif
#ifndef ANALOG_MUX_LT_CHANNEL
#define ANALOG_MUX_LT_CHANNEL 4 // Left Trigger
#endif
#ifndef ANALOG_MUX_RT_CHANNEL
#define ANALOG_MUX_RT_CHANNEL 5 // Right Trigger
#endif
#ifndef ANALOG_MUX_WHEEL_CHANNEL
#define ANALOG_MUX_WHEEL_CHANNEL 6 // Wheel
#endif

// ----- Left Stick Configuration -----

#ifndef ANALOG_MUX_L_MODE
#define ANALOG_MUX_L_MODE DPAD_MODE_LEFT_ANALOG
#endif
#ifndef ANALOG_MUX_L_INVERT
#define ANALOG_MUX_L_INVERT INVERT_NONE
#endif

// ----- Right Stick Configuration -----

#ifndef ANALOG_MUX_R_MODE
#define ANALOG_MUX_R_MODE DPAD_MODE_RIGHT_ANALOG
#endif
#ifndef ANALOG_MUX_R_INVERT
#define ANALOG_MUX_R_INVERT INVERT_NONE
#endif

// ----- Basic Analog Settings -----

// Deadzone settings for Joysticks (percentage)
#ifndef DEFAULT_INNER_DEADZONE
#define DEFAULT_INNER_DEADZONE 8 // Center deadzone percentage
#endif
#ifndef DEFAULT_OUTER_DEADZONE
#define DEFAULT_OUTER_DEADZONE 3 // Outer saturation percentage
#endif

// Deadzone settings for Triggers (raw ADC values, 0-4095 range)
#ifndef DEFAULT_TRIGGER_DEADZONE_MIN
#define DEFAULT_TRIGGER_DEADZONE_MIN 50   // Raw value below this is considered 0 pressure
#endif
#ifndef DEFAULT_TRIGGER_DEADZONE_MAX
#define DEFAULT_TRIGGER_DEADZONE_MAX 4000 // Raw value above this is considered max pressure
#endif

// Deadzone settings for Wheel (percentage)
#ifndef DEFAULT_WHEEL_DEADZONE_MIN
#define DEFAULT_WHEEL_INNER_DEADZONE 2 // Inner deadzone percentage
#endif
#ifndef DEFAULT_WHEEL_DEADZONE_MAX
#define DEFAULT_WHEEL_OUTER_DEADZONE 5 // Outer deadzone percentage
#endif

// Analog Mux Module Name
#define AnalogMuxName "AnalogMux"

#define ANALOG_MUX_JOYSTICK_COUNT 2
#define ANALOG_MUX_TRIGGER_COUNT 2

// Structure to hold data for a single joystick (X/Y pair)
typedef struct {
    int8_t x_channel;           // MUX channel for X-axis (-1 if unused)
    int8_t y_channel;           // MUX channel for Y-axis (-1 if unused)
    float x_value;              // Processed X value (-1.0 to 1.0) - *Will be mapped to 0-65535 later*
    float y_value;              // Processed Y value (-1.0 to 1.0) - *Will be mapped to 0-65535 later*
    uint16_t x_raw;             // Last raw ADC reading for X (0-4095)
    uint16_t y_raw;             // Last raw ADC reading for Y (0-4095)
    uint16_t x_center;          // Assumed center for X (raw ADC, e.g., 2048) - *Can be refined later*
    uint16_t y_center;          // Assumed center for Y (raw ADC, e.g., 2048) - *Can be refined later*
    InvertMode analog_invert;   // Inversion setting for this stick
    DpadMode analog_dpad;       // Dpad emulation mode for this stick
} analog_mux_stick_instance;

// Structure to hold data for a single trigger
typedef struct {
    int8_t channel;             // MUX channel for this trigger (-1 if unused)
    float value;                // Processed trigger value (0.0 to 1.0) - *Will be mapped to 0-65535 later*
    uint16_t raw;               // Last raw ADC reading (0-4095)
    uint16_t deadzone_min;      // Minimum raw value considered active
    uint16_t deadzone_max;      // Maximum raw value for full activation
} analog_mux_trigger_instance;


// Structure to hold data for wheel input
typedef struct {
    int8_t channel;             // MUX channel for Wheel (-1 if unused)
    float value;                // Processed Wheel value (-1.0 to 1.0) - *Will be mapped to 0-65535 later*
    uint16_t raw;               // Last raw ADC reading (0-4095)
    uint16_t center;            // Assumed center for Wheel (raw ADC, e.g., 2048) - *Can be refined later*
} analog_mux_wheel_instance;

class AnalogMuxInput : public GPAddon {
public:
    virtual bool available(); // Check if the addon is configured correctly
    virtual void setup();     // Initialize MUX pins, ADC
    virtual void process();   // Read all channels, process values, update gamepad state
    virtual void preprocess() {} // Standard addon interface method
    virtual void postprocess(bool sent) {} // Standard addon interface method
    virtual void reinit() {}   // Standard addon interface method
    virtual std::string name() { return AnalogMuxName; }

private:
    // Initialization helpers
    void calibrateSticks();
    bool setupMuxPins();
    bool setupAdc();
    void loadConfig(); // Helper to load settings from BoardConfig/defines

    // MUX control and reading
    void readAllMuxChannels();
    void selectMuxChannel(uint8_t channel);
    uint16_t readMuxChannel(uint8_t channel); // Selects and reads a single channel

    // Processing helpers
    void processStick(analog_mux_stick_instance &stick);
    void processTrigger(analog_mux_trigger_instance &trigger);

    // Basic scaling and deadzone application
    void applyStickDeadzoneAndScale(analog_mux_stick_instance &stick);
    void applyTriggerDeadzoneAndScale(analog_mux_trigger_instance &trigger);
    void applyWheelDeadzoneAndScale(analog_mux_wheel_instance &wheel);

    // Member Variables
    uint16_t channel_values[16];  // Array to store the latest reading for each channel
    Pin_t mux_select_pins[3]; // Store configured S0, S1, S2 pins (expand if needed)
    uint8_t mux_select_pin_count; // How many select pins are actually used
    Pin_t mux_output_pin;         // The GPIO pin number for MUX output
    uint8_t mux_adc_channel;      // The corresponding ADC channel (0-3) for the output pin

    // Data storage for inputs
    analog_mux_stick_instance sticks[ANALOG_MUX_JOYSTICK_COUNT];
    analog_mux_trigger_instance triggers[ANALOG_MUX_TRIGGER_COUNT];
    analog_mux_wheel_instance wheel;

    // Basic configuration values loaded during setup
    float stick_inner_deadzone_scaled; // Store as 0.0 to 1.0 range internally
    float stick_outer_deadzone_scaled; // Store as 0.0 to 1.0 range internally
    // trigger deadzones stored in trigger_instance

    bool is_initialized; // Flag to track successful setup
};

#endif // _ANALOG_MUX_H_