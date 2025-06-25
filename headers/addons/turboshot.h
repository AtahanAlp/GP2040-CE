// In headers/addons/TurboShot.h

#ifndef TURBOSHOT_H_
#define TURBOSHOT_H_

#include "gpaddon.h"
#include "GamepadEnums.h"
#include "BoardConfig.h"
#include "enums.pb.h"

// Set this to 1 to enable the add-on
#ifndef TURBOSHOT_ENABLED
#define TURBOSHOT_ENABLED 0
#endif

// Hard-code the GPIO pin we will use for our trigger
#define TURBO_SHOT_PIN 15

// The name of our add-on
#define TurboShotName "TurboShot"

// Define the class for our add-on
class TurboShotAddon : public GPAddon
{
public:
    // This function is called once at boot.
    virtual void bootProcess() {} 

    // This function is called to check if the add-on should be enabled.
    virtual bool available(); 

    // This function is called once when the add-on is enabled to set things up.
    virtual void setup(); 

    // This is called on every frame BEFORE the main controller logic.
    virtual void preprocess(); 

    // This is called on every frame AFTER the main controller logic.
    virtual void process(); 

    // Returns the name of the add-on.
    virtual std::string name() { return TurboShotName; }

private:
    // We can add private variables here later if needed
};

#endif