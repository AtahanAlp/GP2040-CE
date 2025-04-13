#include "addons/analog_mux/analog_mux.h" // Include our header
#include "storagemanager.h"             // To access gamepad state
#include "drivermanager.h"              // To potentially get joystick mid value
#include "helper.h"                   // For isValidPin, potentially
#include "config.pb.h"                // If accessing storage options directly (not used in this simplified version yet)

#include <cmath>
#include <algorithm>

// Define ADC constants
#define ADC_MAX ((1 << 12) - 1)       // Max value for RP2040 ADC (4095)
#define ADC_PIN_OFFSET 26             // GPIO pins for ADC start at 26
#define ADC_MAX_FLOAT (float)ADC_MAX  // Float version of ADC_MAX
#define ADC_CENTER_DEFAULT (ADC_MAX / 2 + 1) // Default center value (2048)

// Define Analog scaling constants (output range 0.0 to 1.0)
#define ANALOG_MAX_FLOAT 1.0f
#define ANALOG_CENTER_FLOAT 0.5f
#define ANALOG_MIN_FLOAT 0.0f

// Check if the addon is enabled via the define in the header TODO: make it runtime configurable
// This is a compile-time check, so it won't be affected by runtime changes in the config!!!
bool AnalogMuxInput::available() {
    // Basic check: is the addon enabled in the header?
    // More robust checks (like ensuring pins are assigned) could be added here or in setup.
#if ANALOG_MUX_ENABLED == 1
    return true;
#else
    return false;
#endif
}

// Setup the addon
void AnalogMuxInput::setup() {
    is_initialized = false; // Assume failure until proven otherwise

    // Load configuration from defines into member variables and structs
    loadConfig();

    // Initialize GPIO pins for MUX select lines
    if (!setupMuxPins()) {
        // Optionally log an error here
        return; // Failed to setup MUX pins
    }

    // Initialize the ADC pin connected to the MUX output
    if (!setupAdc()) {
        // Optionally log an error here
        return; // Failed to setup ADC
    }

    // If we reached here, setup was successful
    is_initialized = true;
}

// Load configuration from header defines
void AnalogMuxInput::loadConfig() {
    // Store MUX select pins
    mux_select_pins[0] = ANALOG_MUX_S0_PIN;
    mux_select_pins[1] = ANALOG_MUX_S1_PIN;
    mux_select_pins[2] = ANALOG_MUX_S2_PIN;

    // Determine how many select pins are actually configured
    mux_select_pin_count = 0;
    for (int i = 0; i < (sizeof(mux_select_pins)/sizeof(mux_select_pins[0])); ++i) {
        if (isValidPin(mux_select_pins[i])) {
            // Use the highest index + 1, assuming contiguous pins are defined (S0, S1, S2...)
            mux_select_pin_count = i + 1;
        } else {
            // Stop counting at the first invalid pin in the sequence
            // Or, if non-contiguous pins are allowed, adjust logic
            break;
        }
    }

    // Store MUX output pin
    mux_output_pin = ANALOG_MUX_OUT_PIN;

    // Configure Left Stick (Index 0)
    sticks[0].x_channel = ANALOG_MUX_LX_CHANNEL;
    sticks[0].y_channel = ANALOG_MUX_LY_CHANNEL;
    sticks[0].analog_dpad = ANALOG_MUX_L_MODE;
    sticks[0].analog_invert = ANALOG_MUX_L_INVERT;
    sticks[0].x_center = ADC_CENTER_DEFAULT; // Use default center for now
    sticks[0].y_center = ADC_CENTER_DEFAULT; // Use default center for now
    sticks[0].x_value = ANALOG_CENTER_FLOAT; // Initialize processed value to center
    sticks[0].y_value = ANALOG_CENTER_FLOAT; // Initialize processed value to center

    // Configure Right Stick (Index 1)
    sticks[1].x_channel = ANALOG_MUX_RX_CHANNEL;
    sticks[1].y_channel = ANALOG_MUX_RY_CHANNEL;
    sticks[1].analog_dpad = ANALOG_MUX_R_MODE;
    sticks[1].analog_invert = ANALOG_MUX_R_INVERT;
    sticks[1].x_center = ADC_CENTER_DEFAULT; // Use default center for now
    sticks[1].y_center = ADC_CENTER_DEFAULT; // Use default center for now
    sticks[1].x_value = ANALOG_CENTER_FLOAT; // Initialize processed value to center
    sticks[1].y_value = ANALOG_CENTER_FLOAT; // Initialize processed value to center

    // Configure Left Trigger (Index 0)
    triggers[0].channel = ANALOG_MUX_LT_CHANNEL;
    triggers[0].deadzone_min = DEFAULT_TRIGGER_DEADZONE_MIN;
    triggers[0].deadzone_max = DEFAULT_TRIGGER_DEADZONE_MAX;
    triggers[0].value = ANALOG_MIN_FLOAT; // Initialize processed value to min

    // Configure Right Trigger (Index 1)
    triggers[1].channel = ANALOG_MUX_RT_CHANNEL;
    triggers[1].deadzone_min = DEFAULT_TRIGGER_DEADZONE_MIN;
    triggers[1].deadzone_max = DEFAULT_TRIGGER_DEADZONE_MAX;
    triggers[1].value = ANALOG_MIN_FLOAT; // Initialize processed value to min

    // Scale joystick deadzones from percentage to 0.0-0.5 range (relative to center)
    stick_inner_deadzone_scaled = (DEFAULT_INNER_DEADZONE / 100.0f) * ANALOG_CENTER_FLOAT;
    stick_outer_deadzone_scaled = (DEFAULT_OUTER_DEADZONE / 100.0f) * ANALOG_CENTER_FLOAT;
}

// Initialize MUX select GPIO pins
bool AnalogMuxInput::setupMuxPins() {
    for (int i = 0; i < mux_select_pin_count; ++i) {
        Pin_t pin = mux_select_pins[i];
        if (!isValidPin(pin)) {
            // Log error: Invalid MUX select pin defined
            return false;
        }
        gpio_init(pin);             // Initialize GPIO
        gpio_set_dir(pin, GPIO_OUT); // Set as output
        gpio_put(pin, 0);           // Default to low (select channel 0 initially)
    }
    return true;
}

// Initialize ADC for MUX output pin
bool AnalogMuxInput::setupAdc() {
    if (!isValidPin(mux_output_pin)) {
        // Log error: Invalid MUX output pin defined
        return false;
    }

    // Check if the pin is ADC capable (GPIO 26-29)
    if (mux_output_pin < ADC_PIN_OFFSET || mux_output_pin >= (ADC_PIN_OFFSET + ADC_COUNT)) {
         // Log error: MUX output pin is not ADC capable
        return false;
    }

    adc_init();                     // Initialize ADC system
    adc_gpio_init(mux_output_pin); // Initialize the GPIO for ADC function
    mux_adc_channel = mux_output_pin - ADC_PIN_OFFSET; // Determine ADC channel (0-3)

    return true;
}


// Main process loop called by the system
void AnalogMuxInput::process() {
    // Don't run if not initialized or not enabled
    if (!is_initialized || !available()) {
        return;
    }

    // Get the gamepad state object
    Gamepad *gamepad = Storage::getInstance().GetGamepad();
    if (!gamepad) {
        return; // Should not happen, but safety check
    }

    // Determine the joystick center value based on the current driver
    uint16_t joystickMid = GAMEPAD_JOYSTICK_MID; // Default GP2040-CE center (0x8000)
    if (DriverManager::getInstance().getDriver() != nullptr) {
        joystickMid = DriverManager::getInstance().getDriver()->GetJoystickMidValue(); // Get driver specific center (e.g., 0x7FFF for XInput)
    }

    // Process Joysticks
    for (int i = 0; i < ANALOG_MUX_JOYSTICK_COUNT; ++i) {
        processStick(sticks[i]); // Read raw values and apply deadzone/scaling

        // Map processed float value (0.0 to 1.0) to gamepad uint16_t range (0 to 65535)
        uint16_t mapped_x = static_cast<uint16_t>(sticks[i].x_value * 65535.0f);
        uint16_t mapped_y = static_cast<uint16_t>(sticks[i].y_value * 65535.0f);

        // Assign to correct gamepad state based on mode
        if (sticks[i].analog_dpad == DpadMode::DPAD_MODE_LEFT_ANALOG) {
            gamepad->state.lx = mapped_x;
            gamepad->state.ly = mapped_y;
        } else if (sticks[i].analog_dpad == DpadMode::DPAD_MODE_RIGHT_ANALOG) {
            gamepad->state.rx = mapped_x;
            gamepad->state.ry = mapped_y;
        }
        // DPAD_MODE_DIGITAL is not handled here, assumes analog output
    }

    // Process Triggers
    for (int i = 0; i < ANALOG_MUX_TRIGGER_COUNT; ++i) {
        processTrigger(triggers[i]); // Read raw value and apply deadzone/scaling

        // Map processed float value (0.0 to 1.0) to gamepad uint16_t range (0 to 65535)
        uint16_t mapped_trigger = static_cast<uint16_t>(triggers[i].value * 65535.0f);

        // Assign to correct gamepad state (index 0 = LT, index 1 = RT)
        if (i == 0) { // Left Trigger
            gamepad->state.lt = mapped_trigger;
        } else { // Right Trigger
            gamepad->state.rt = mapped_trigger;
        }
    }
}

// Read and process a single joystick
void AnalogMuxInput::processStick(analog_mux_stick_instance &stick) {
    // Read X-axis raw value if channel is valid
    if (stick.x_channel >= 0) {
        stick.x_raw = readMuxChannel(stick.x_channel);
    } else {
        stick.x_raw = stick.x_center; // Default to center if not configured
    }

    // Read Y-axis raw value if channel is valid
    if (stick.y_channel >= 0) {
        stick.y_raw = readMuxChannel(stick.y_channel);
    } else {
        stick.y_raw = stick.y_center; // Default to center if not configured
    }

    // Apply deadzone and scaling to get processed float values (0.0 to 1.0)
    applyStickDeadzoneAndScale(stick);
}

// Read and process a single trigger
void AnalogMuxInput::processTrigger(analog_mux_trigger_instance &trigger) {
    // Read raw value if channel is valid
    if (trigger.channel >= 0) {
        trigger.raw = readMuxChannel(trigger.channel);
    } else {
        trigger.raw = 0; // Default to 0 if not configured
    }

    // Apply deadzone and scaling to get processed float value (0.0 to 1.0)
    applyTriggerDeadzoneAndScale(trigger);
}

// Set the MUX select lines to choose a specific channel
void AnalogMuxInput::selectMuxChannel(uint8_t channel) {
    if (!is_initialized) return; // Don't operate if pins aren't setup

    for (int i = 0; i < mux_select_pin_count; ++i) {
        gpio_put(mux_select_pins[i], (channel >> i) & 1); // Set S0, S1, S2... based on channel bits
    }
    // Optional: Add a small delay here if needed for MUX settling time
    sleep_us(10); // Example: 10 microsecond delay
}

// Select a MUX channel and read the ADC value from the output pin
uint16_t AnalogMuxInput::readMuxChannel(uint8_t channel) {
    if (!is_initialized) return ADC_CENTER_DEFAULT; // Return center if not setup

    selectMuxChannel(channel);          // Set the MUX to the desired channel
    adc_select_input(mux_adc_channel); // Select the ADC input connected to the MUX output
    return adc_read();                  // Perform ADC conversion and return result (0-4095)
}

// Apply deadzone and scaling for joysticks
void AnalogMuxInput::applyStickDeadzoneAndScale(analog_mux_stick_instance &stick) {
    // 1. Map raw ADC values (0-4095) to a float range around 0.0 (-0.5 to +0.5)
    //    relative to the calibrated center.
    //    Note: This assumes center is roughly ADC_MAX/2. More complex mapping
    //    might be needed if the center is significantly skewed.
    float dx = 0.0f;
    if (stick.x_raw > stick.x_center) {
        dx = (float)(stick.x_raw - stick.x_center) / (ADC_MAX - stick.x_center); // 0 to 1 range
    } else if (stick.x_raw < stick.x_center) {
        dx = (float)(stick.x_raw - stick.x_center) / stick.x_center; // -1 to 0 range
    } // else dx remains 0.0 if raw == center

    float dy = 0.0f;
     if (stick.y_raw > stick.y_center) {
        dy = (float)(stick.y_raw - stick.y_center) / (ADC_MAX - stick.y_center); // 0 to 1 range
    } else if (stick.y_raw < stick.y_center) {
        dy = (float)(stick.y_raw - stick.y_center) / stick.y_center; // -1 to 0 range
    } // else dy remains 0.0 if raw == center

    // dx and dy are now roughly in the range -1.0 to +1.0

    // 2. Calculate magnitude (distance from center, scaled relative to max radius 1.0)
    //    We scale dx/dy by 0.5 because they represent the full range (-1 to 1),
    //    but we want magnitude relative to a center of 0.0 and max radius of 0.5
    //    in this coordinate system before applying deadzone.
    float magnitude = sqrtf( (dx * dx) + (dy * dy) ) * ANALOG_CENTER_FLOAT; // Magnitude now 0.0 to ~0.707 (or higher if not circular)

    // 3. Apply radial deadzone
    if (magnitude < stick_inner_deadzone_scaled) {
        // Inside deadzone, snap to center
        stick.x_value = ANALOG_CENTER_FLOAT;
        stick.y_value = ANALOG_CENTER_FLOAT;
    } else {
        // Outside deadzone, calculate scaling factor
        // Rescale magnitude from (inner_deadzone..outer_deadzone) to (0..0.5)
        float scale = (magnitude - stick_inner_deadzone_scaled) / (stick_outer_deadzone_scaled - stick_inner_deadzone_scaled);
        scale = std::max(0.0f, std::min(scale, ANALOG_CENTER_FLOAT)); // Clamp scale 0.0 to 0.5

        // Apply scaling to the original direction vector (dx, dy)
        // Need to normalize the direction vector (dx, dy) before scaling by 'scale'
        float norm_magnitude = sqrtf(dx*dx + dy*dy); // Original magnitude in -1..1 space
        if (norm_magnitude < 1e-6f) norm_magnitude = 1e-6f; // Avoid division by zero

        float scaled_dx = (dx / norm_magnitude) * scale;
        float scaled_dy = (dy / norm_magnitude) * scale;

        // Convert back to 0.0 to 1.0 range centered at 0.5
        stick.x_value = ANALOG_CENTER_FLOAT + scaled_dx;
        stick.y_value = ANALOG_CENTER_FLOAT + scaled_dy;
    }

    // 4. Apply inversion if necessary (invert around the center 0.5)
    if (stick.analog_invert == InvertMode::INVERT_X || stick.analog_invert == InvertMode::INVERT_XY) {
        stick.x_value = ANALOG_MAX_FLOAT - stick.x_value;
    }
    if (stick.analog_invert == InvertMode::INVERT_Y || stick.analog_invert == InvertMode::INVERT_XY) {
        stick.y_value = ANALOG_MAX_FLOAT - stick.y_value;
    }

    // 5. Final clamp to ensure values are strictly within 0.0 to 1.0
    stick.x_value = std::max(ANALOG_MIN_FLOAT, std::min(stick.x_value, ANALOG_MAX_FLOAT));
    stick.y_value = std::max(ANALOG_MIN_FLOAT, std::min(stick.y_value, ANALOG_MAX_FLOAT));
}

// Apply deadzone and scaling for triggers
void AnalogMuxInput::applyTriggerDeadzoneAndScale(analog_mux_trigger_instance &trigger) {
    // Ensure min/max are valid
    uint16_t min_dead = std::min(trigger.deadzone_min, trigger.deadzone_max);
    uint16_t max_dead = std::max(trigger.deadzone_min, trigger.deadzone_max);
    uint16_t range = (max_dead > min_dead) ? (max_dead - min_dead) : 1; // Avoid division by zero

    // Apply deadzone and scale
    if (trigger.raw <= min_dead) {
        trigger.value = 0.0f;
    } else if (trigger.raw >= max_dead) {
        trigger.value = 1.0f;
    } else {
        // Linearly scale between min and max deadzone
        trigger.value = (float)(trigger.raw - min_dead) / range;
    }

    // Final clamp (should be redundant if logic above is correct, but safe)
    trigger.value = std::max(0.0f, std::min(trigger.value, 1.0f));
}
