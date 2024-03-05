#ifndef RCMUTIL_H
#define RCMUTIL_H

// contains functions common to all RCMv2 projects

#include <Arduino.h>

#include "rcm.h"

extern void PowerOn();
extern void Enable();
extern void Disable();
extern void Enabled();
extern void Always();
extern void configWifi();
extern void WifiDataToParse();
extern void WifiDataToSend();
extern void setupMotors();

#ifndef RCM_HARDWARE_VERSION

void setupRSL()
{
    pinMode(ONBOARD_LED, OUTPUT);
}
void enabledRSL()
{
    digitalWrite(ONBOARD_LED, millis() % 500 < 250); // flash, enabled
}
void wifiFailRSL()
{
    digitalWrite(ONBOARD_LED, millis() % 1000 <= 100); // short flash, wifi connection fail
}
void wifiDisconnectedRSL()
{
    digitalWrite(ONBOARD_LED, millis() % 1000 >= 100); // long flash, no driver station connected
}
void disabledRSL()
{
    digitalWrite(ONBOARD_LED, HIGH); // on, disabled
}

#elif RCM_HARDWARE_VERSION == 10

void setupRSL()
{
#if defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
    FastLED.addLeds<NEOPIXEL, PIN_NEOPIXEL>(RSL_leds, 1);
    RSL = CRGB(0, 0, 0);
    FastLED.show();
}
void enabledRSL()
{
#ifndef OVERWRITE_RSL
    if (millis() % 500 < 250) {
        setRSL(RSLcolor);
    } else {
        setRSL(CRGB(0, 0, 0));
    }
#endif
}
void wifiFailRSL()
{
#ifndef OVERWRITE_RSL
    if (millis() % 1000 <= 100) {
        setRSL(RSLcolor);
    } else {
        setRSL(CRGB(0, 0, 0));
    }
#endif
}
void wifiDisconnectedRSL()
{
#ifndef OVERWRITE_RSL
    if (millis() % 1000 >= 100) {
        setRSL(RSLcolor);
    } else {
        setRSL(CRGB(0, 0, 0));
    }
#endif
}
void disabledRSL()
{
#ifndef OVERWRITE_RSL
    setRSL(RSLcolor);
#endif
}

#endif

void setup()
{
    Serial.begin(115200);
    setupRSL(); //     pinMode(ONBOARD_LED, OUTPUT);
    setupMotors();
    PowerOn();
    Disable();
#ifndef RCM_ROS
    configWifi();
    EWD::setupWifi(WifiDataToParse, WifiDataToSend);
#else
    setupROS();
#endif
}

boolean connectedToWifi()
{
#ifndef RCM_ROS
    return EWD::wifiConnected;
#else
    return !ROSCheckFail;
#endif
}
boolean connectionTimedOut()
{
#ifndef RCM_ROS
    return EWD::timedOut();
#else
    return (millis() - lastEnableSentMillis) > rosWifiTimeout;
#endif
}

extern void ROSrun();

void loop()
{
#ifndef RCM_ROS
    EWD::runWifiCommunication();
#else
    ROSrun();
#endif
    if (!connectedToWifi() || connectionTimedOut()) {
        enabled = false;
    }
    Always();
    if (enabled && !wasEnabled) {
#if RCM_HARDWARE_VERSION == 10
#ifndef RCM_BYTE_DO_NOT_USE_SAFE_DISABLE
        digitalWrite(motorsEnablePin, HIGH);
#endif
#endif

        Enable();
    }
    if (!enabled && wasEnabled) {
        Disable();

#if RCM_HARDWARE_VERSION == 10
#ifndef RCM_BYTE_DO_NOT_USE_SAFE_DISABLE
        digitalWrite(motorsEnablePin, LOW);
#endif
#endif
    }
    if (enabled) {
        Enabled();
        enabledRSL(); //        digitalWrite(ONBOARD_LED, millis() % 500 < 250); // flash, enabled
    } else {
        if (!connectedToWifi())
            wifiFailRSL(); //            digitalWrite(ONBOARD_LED, millis() % 1000 <= 100); // short flash, wifi connection fail
        else if (connectionTimedOut())
            wifiDisconnectedRSL(); //            digitalWrite(ONBOARD_LED, millis() % 1000 >= 100); // long flash, no driver station connected
        else
            disabledRSL(); //            digitalWrite(ONBOARD_LED, HIGH); // on, disabled
    }
    wasEnabled = enabled;
}

#endif // RCMUTIL_H
