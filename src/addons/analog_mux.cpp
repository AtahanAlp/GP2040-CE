#include "addons/analog_mux.h" // Include our header
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
#define ADC_COUNT_FOR_MUX 4                   // Number of ADC channels (0-3)

#define ANALOG_MAX_FLOAT 1.0f
#define ANALOG_CENTER_FLOAT 0.5f
#define ANALOG_MIN_FLOAT 0.0f


bool AnalogMuxInput::available() {
    #if ANALOG_MUX_ENABLED == 1
        return true;
    #else
        return false;
    #endif
}


void AnalogMuxInput::setup() {
    is_initialized = false; 

    loadConfig();

    if (!setupMuxPins()) {
        return;
    }

    if (!setupAdc()) {
        return;
    }

    calibrateSticks();

    is_initialized = true;
}

void AnalogMuxInput::calibrateSticks() {
    sleep_us(100);
    for (int i = 0; i < ANALOG_MUX_JOYSTICK_COUNT; ++i) {
        if (sticks[i].x_channel >= 0) {
            sticks[i].x_center = readMuxChannel(sticks[i].x_channel);
        }
        sleep_us(10);
        if (sticks[i].y_channel >= 0) {
            sticks[i].y_center = readMuxChannel(sticks[i].y_channel);
        }
    }
}


void AnalogMuxInput::loadConfig() {
    mux_select_pins[0] = ANALOG_MUX_S0_PIN;
    mux_select_pins[1] = ANALOG_MUX_S1_PIN;
    mux_select_pins[2] = ANALOG_MUX_S2_PIN;

    // Determine how many select pins are actually configured
    mux_select_pin_count = 0;
    for (size_t i = 0; i < (sizeof(mux_select_pins)/sizeof(mux_select_pins[0])); ++i) {
        if (isValidPin(mux_select_pins[i])) {
            // Use the highest index + 1, assuming contiguous pins are defined (S0, S1, S2...)
            mux_select_pin_count = static_cast<int>(i) + 1;
        } else {
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

bool AnalogMuxInput::setupAdc() {
    if (!isValidPin(mux_output_pin)) {
        // Log error: Invalid MUX output pin defined
        return false;
    }

    // Check if the pin is ADC capable (GPIO 26-29)
    if (mux_output_pin < ADC_PIN_OFFSET || mux_output_pin >= (ADC_PIN_OFFSET + ADC_COUNT_FOR_MUX)) {
         // Log error: MUX output pin is not ADC capable
        return false;
    }

    adc_init();                     // Initialize ADC system
    adc_gpio_init(mux_output_pin); // Initialize the GPIO for ADC function
    mux_adc_channel = mux_output_pin - ADC_PIN_OFFSET; // Determine ADC channel (0-3)

    return true;
}


void AnalogMuxInput::process() {
    if (!is_initialized || !available()) {
        return;
    }

    // --- Part 1: Read all active MUX channels in a tight, fast loop ---
    readAllMuxChannels();

    // --- Part 2: Process all inputs using the fresh, stored values ---
    Gamepad *gamepad = Storage::getInstance().GetGamepad();
    if (!gamepad) {
        return;
    }

    // Process Joysticks
    for (int i = 0; i < ANALOG_MUX_JOYSTICK_COUNT; ++i) {
        if (sticks[i].x_channel >= 0) sticks[i].x_raw = channel_values[sticks[i].x_channel];
        if (sticks[i].y_channel >= 0) sticks[i].y_raw = channel_values[sticks[i].y_channel];

        applyStickDeadzoneAndScale(sticks[i]); // apply deadzone/scaling

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
    // Enable analog triggers as per documentation
    gamepad->hasAnalogTriggers = true;
    for (int i = 0; i < ANALOG_MUX_TRIGGER_COUNT; ++i) {
        if (triggers[i].channel >= 0) triggers[i].raw = channel_values[triggers[i].channel];

        applyTriggerDeadzoneAndScale(triggers[i]); // apply deadzone/scaling

        // Map processed float value (0.0 to 1.0) to gamepad uint8_t range (0 to 255)
        uint8_t mapped_trigger = static_cast<uint8_t>(triggers[i].value * 255.0f);

        // Assign to correct gamepad state (index 0 = LT, index 1 = RT)
        if (i == 0) { // Left Trigger
            gamepad->state.lt = mapped_trigger;
        } else { // Right Trigger
            gamepad->state.rt = mapped_trigger;
        }
    }
}


void AnalogMuxInput::applyStickDeadzoneAndScale(analog_mux_stick_instance &stick) {
    float dx = 0.0f;
    float dy = 0.0f;

    // Use the outer deadzone setting to define the 'usable' electrical range.
    // This compresses the effective range to prevent hitting 1.0 too early.
    float outer_deadzone_percent = DEFAULT_OUTER_DEADZONE / 100.0f;
    float effective_max = ADC_MAX_FLOAT * (1.0f - outer_deadzone_percent);
    float effective_min = ADC_MAX_FLOAT * outer_deadzone_percent;


    // --- X-Axis Calculation ---
    int16_t raw_x_deflection = stick.x_raw - stick.x_center;

    if (raw_x_deflection > 0) {
        // Normalize by the distance from center to the effective maximum
        float positive_range = effective_max - stick.x_center;
        if (positive_range < 1.0f) positive_range = 1.0f; // Avoid division by small/zero numbers
        dx = (float)raw_x_deflection / positive_range;
    } else if (raw_x_deflection < 0) {
        // Normalize by the distance from center to the effective minimum
        float negative_range = stick.x_center - effective_min;
        if (negative_range < 1.0f) negative_range = 1.0f;
        dx = (float)raw_x_deflection / negative_range;
    }

    // --- Y-Axis Calculation ---
    int16_t raw_y_deflection = stick.y_raw - stick.y_center;

    if (raw_y_deflection > 0) {
        float positive_range = effective_max - stick.y_center;
        if (positive_range < 1.0f) positive_range = 1.0f;
        dy = (float)raw_y_deflection / positive_range;
    } else if (raw_y_deflection < 0) {
        float negative_range = stick.y_center - effective_min;
        if (negative_range < 1.0f) negative_range = 1.0f;
        dy = (float)raw_y_deflection / negative_range;
    }
    
    // --- Final Processing ---

    // Clamp the values. Any movement beyond the 'effective' range will be clamped to 1.0 or -1.0.
    dx = std::max(-1.0f, std::min(dx, 1.0f));
    dy = std::max(-1.0f, std::min(dy, 1.0f));

    // Apply inner deadzone and rescale
    float inner_deadzone = DEFAULT_INNER_DEADZONE / 100.0f;
    float inner_range = 1.0f - inner_deadzone;
    if (inner_range < 1e-6f) inner_range = 1.0f;

    if (std::abs(dx) < inner_deadzone) {
        dx = 0.0f;
    } else {
        dx = ((dx > 0) ? 1.0f : -1.0f) * (std::abs(dx) - inner_deadzone) / inner_range;
    }

    if (std::abs(dy) < inner_deadzone) {
        dy = 0.0f;
    } else {
        dy = ((dy > 0) ? 1.0f : -1.0f) * (std::abs(dy) - inner_deadzone) / inner_range;
    }

    // Convert to 0.0 to 1.0 range for gamepad output
    stick.x_value = (dx + 1.0f) / 2.0f;
    stick.y_value = (dy + 1.0f) / 2.0f;

    // Apply inversion if necessary
    if (stick.analog_invert == InvertMode::INVERT_X || stick.analog_invert == InvertMode::INVERT_XY) {
        stick.x_value = ANALOG_MAX_FLOAT - stick.x_value;
    }
    if (stick.analog_invert == InvertMode::INVERT_Y || stick.analog_invert == InvertMode::INVERT_XY) {
        stick.y_value = ANALOG_MAX_FLOAT - stick.y_value;
    }
}


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


void AnalogMuxInput::readAllMuxChannels() {
    // Select the ADC input pin ONCE at the start
    adc_select_input(mux_adc_channel);

    // Read all joystick channels
    for (int i = 0; i < ANALOG_MUX_JOYSTICK_COUNT; ++i) {
        if (sticks[i].x_channel >= 0) {
            selectMuxChannel(sticks[i].x_channel);
            sleep_us(10); // Crucial delay for settling
            channel_values[sticks[i].x_channel] = adc_read();
        }
        if (sticks[i].y_channel >= 0) {
            selectMuxChannel(sticks[i].y_channel);
            sleep_us(10); // Crucial delay for settling
            channel_values[sticks[i].y_channel] = adc_read();
        }
    }

    // Read all trigger channels
    for (int i = 0; i < ANALOG_MUX_TRIGGER_COUNT; ++i) {
        if (triggers[i].channel >= 0) {
            selectMuxChannel(triggers[i].channel);
            sleep_us(10); // Crucial delay for settling
            channel_values[triggers[i].channel] = adc_read();
        }
    }
}

void AnalogMuxInput::selectMuxChannel(uint8_t channel) {
    if (!is_initialized) return;

    for (int i = 0; i < mux_select_pin_count; ++i) {
        gpio_put(mux_select_pins[i], (channel >> i) & 1);
    }
}

uint16_t AnalogMuxInput::readMuxChannel(uint8_t channel) {
    selectMuxChannel(channel); 
    sleep_us(5);
    adc_select_input(mux_adc_channel);
    sleep_us(10);
    return adc_read(); 
}