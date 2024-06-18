/****
 * @file main.cpp
 * @version 0.1.0
 * @date June 14, 2024
 * 
 * This is the firmware for a moon phase display. The display consists of a picture of the full 
 * moon, on top of which there is a springy, flexible "terminator" that runs between two pivots 
 * placed at the northernmost and southernmost points of the moon photo. The northern one is free 
 * to turn. One end of the terminator is attached to this pivot. The terminator material passes 
 * across the photo to the southern pivot. It slides through that pivot and continues beyond it. 
 * The southern pivot can be twisted by a stepper motor (pvMotor). This twists the terminator 
 * material, set the angle it makes with the southern extremity of the photo. The northern, 
 * freely rotating, pivot assumes whatever angle the forces on it dictate. 
 * 
 * As noted, the material forming the terminator is longer than the distance from the northern 
 * pivot to the southern one. After it passes through the southern (driven) pivot, its end is 
 * attached to the travelling part of a leadscrew mechanism the whole of which rotates with the 
 * pivot. The leadscrew mechanism, driven by another stepper motor (lsMotor) pushes or pulls the 
 * terminator material, sliding it through the pivot. Thus the position of the leadscrew 
 * determins the length of the terminator material the runs across the photo between the two 
 * pivots. 
 * 
 * By coordinating the operation of the two steppers we can form the terminator into various 
 * curves crossing the face of the photo. By doing so correctly we can approximate the the curves 
 * real terminator takes as it crosses the face of the real moon. Well, close enough to make a 
 * nice mechanical moon phase display, anyway.
 * 
 * Based on calibration runs of the as-built mechanism, if the position of the leadscrew (ls) as 
 * a function of pivot position (pv) is
 * 
 *      ls = 497671 + 30.5pv - 0.201pv^2
 * 
 * where -1600 <= pv <= 1600, the result is a (semi-credible) terminator path across the moon 
 * photo. (Here, ls is a measure of how much terminator material is stored in the leadscrew 
 * mechanism, so the length across the photo in inversely proportional to it. Positions of both 
 * motors is measured in steps.) 
 * 
 * For the purposes of the display, I've divided a lunation into 60 phases. Phase 0 is a new moon, 
 * phase 16 is the first quarter, phase 30 is the full moon and phase 45 is the third quarer moon. 
 * The transition from phase 59 to 0 brings us back to the new moon of the next lunation.
 * 
 * At the start of a lunation (new moon -- phase 0), pv is at its minimum (-1600) and the 
 * terminator is at its longest and is bent strongly to the right (the display is for earth's 
 * northern hemisphere). As time progresses, the phases increase step by step, showing the waxing 
 * crescent progression. At each phase, the steppers are operated to form the appropriate 
 * terminator. During the transition from phase 15 to 16, the terminator begins to bend to the 
 * left marking the progress of the gibbous moon until, at phase 29, it is at its maximum, bent 
 * strongly to the left at full moon. The transition from phase 29 to 30 involves moving the 
 * terminator from full left extension to full right so that it is in position to show the start 
 * of the waning gibbous moon. As the phases continue, the terminator makes its way across the 
 * moon photo, marking the lunation's waning phases, until, at the transition from phase 59 to 0, 
 * we're back at the new moon. Like the transition from phase 29 to 30, the transition from phase 
 * 59 to 0 requires the terminator to be reset from its extreme left position to its extreme 
 * right one.
 * 
 * In addition to the terminator and moon photo, the display has two low angle light sources that 
 * shine aross the moon photo, one from right to left, and the other from left to right. The 
 * terminator material stands tall enough above the photo to keep the light from shining on the 
 * part of the photo on the side away from the light source. Thus when one light source is on and 
 * the other is off, the part of the moon photo on the same side is illuminated, but the part of 
 * the photo on the other side is in (relative) darkness. 
 * 
 * In phases 0 - 29, only the right source is turned on, thus illuminating part of the moon phot 
 * corresponding to what's lit during the waxing phases of the lunation. During the full moon 
 * transition from phase 29 to 30, during which the terminator is reset, both sources are lit. 
 * During phases 30 - 59, only the right source is lit, lighting the part corresponding to what's 
 * lit during the moon's waning phases. During the new-moon transition from phase 59 to phase 0, 
 * while the terminator is reset, neither light source is lit.
 * 
 *****
 * 
 * Copyright (C) 2024 D.L. Ehnebuske
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 * and associated documentation files (the "Software"), to deal in the Software without 
 * restriction, including without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 * 
 ****/
#include <Arduino.h>                        // Ritual include of Arduino base stuff
#include <avr/eeprom.h>                     // Access to eeprom for non-volatile state
#include "ULN2003.h"                        // Encapsulation for ULN2003-driven Stepper motor
#include "CommandLine.h"                    // Support for command line interpreter

// Misc compile-time constants
#define BANNER                  "MoonDisplay V0.1.0"
#define SERIAL_WAIT_MILLIS      (125)       // Duration (millis()) of built-in LED flashes while waiting for Serial to connect
#define TOP_SPEED               (800)       // Top speed for steppers
#define MAX_PV_ANGLE            (78.125)    // Maximum pivot angle (degrees)
#define MIN_PV_ANGLE            (0.0)       // Minimum pivot angle (degrees)
#define MAX_LS_LOC              (500000)    // Maximum leadscrew location (steps)
#define MIN_LS_LOC              (34000)     // Minimum leadscrew location (steps)
#define FINGERPRINT             (0x1656)    // Our fingerprint (to see if EEProm has our stuff)

// Stepper motor pin definitions
#define PV_IN1                 (2)      // Pivot motor
#define PV_IN2                 (3)
#define PV_IN3                 (4)
#define PV_IN4                 (5)
#define LS_IN1                 (6)      // Leadscrew motor
#define LS_IN2                 (7)
#define LS_IN3                 (8)
#define LS_IN4                 (9)

// Types
struct nvState_t {
    int16_t fingerprint;                // Value to tell whether EEPROM contents is ours
    int16_t curPhase;                   // The currently displayed phase
};

// Globals
ULN2003 lsMotor {LS_IN1, LS_IN2, LS_IN3, LS_IN4};       //Leadscrew stepper motor
ULN2003 pvMotor {PV_IN1, PV_IN2, PV_IN3, PV_IN4};       // Bottom stepper motor
CommandLine ui;                                         // Command line interpreter object
nvState_t state = {FINGERPRINT, 15};                    // State for cold start
int16_t tgtPhase;                                       // The phase we'd like to display

static const float stepPva[60] = {                      // Pivot angle (degrees) for each lunation display step
    -78, -78, -75.5, -73.5, -71, -67, -63,  -58,  -53, -47,
    -40, -33, -25,   -18,   -10,  10,  18,   25,   33,  40,
     47,  53,  58,    63,    67,  71,  73.5, 75.5, 78,  78,
    -78, -78, -75.5, -73.5, -71, -67, -63,  -58,  -53, -47,
    -40, -33, -25,   -18,   -10,  10,  18,   25,   33,  40,
     47,  53,  58,    63,    67,  71,  73.5, 75.5, 78,  78
};

/**
 * @brief   Return the position (in steps) the leadscrew should have given the positon (in steps) 
 *          of the pivot to form a good-looking moon terminator. 
 * 
 * @note    The assumption that -1600 <= pv <= 1600 is not checked.
 * 
 * @details This functon is based on curve fitting calibration data. When pv is 0, the terminator 
 *          is a straight vertical line with 500,000 steps (each step is 1/131072") worth of 
 *          material to be pushed out. The curve formed when pv = 1600 is all the way to the rim 
 *          of the moon (approximately), so not on the displayed face.
 * 
 * @param pv    The position (in steps) of the pivot
 * @return int32_t 
 */
int32_t inline pvToLs(int32_t pv) {
    return (int32_t)(497671.0 + 30.5 * pv - 0.201 * pv * pv);
}

/**
 * @brief   Return the position (in steps) of the pivot given its angle in degrees.
 * 
 * @note    The assumtion that -78.125 <= angle <= 78.125 is not checked.
 * 
 * @details The pivot is driven by a 4096 step/turn stepper using a toothed belt drive with a 
 *          20-tooth pullet on the stepper and a 32-tooth pulley on the pivot.
 * 
 * @param angle 
 * @return int32_t 
 */
int32_t inline degToPv(float angle) {
    return (int32_t)(20.48 * angle);
}

/**
 * @brief   Read data from EEPROM, and if it looks like it's ours, put the data in theState. 
 *          Otherwise, put a synthesized "cold start" state in theState
 * 
 * @param theState The place to put the state data
 * @return boolean True if the data was read from EEPROM; false if synthesized
 */
boolean readEEPROM(nvState_t *theState) {
	eeprom_read_block((void*)&theState, (const void*)0, sizeof(nvState_t));
	if (theState->fingerprint == FINGERPRINT) {     // If it looks like ours
		return true;                            //  Say we read it okay
	} else {                                    // Otherwise
        theState->fingerprint = FINGERPRINT;    //  Use cold start values
        theState->curPhase = 0;
        theState->curPhase = 45;    // <-- NB: Temporary until I fix the device
		return false;                           //  Say we synthesized it
	}
}
/**
 * @brief   Write the contents of thesState to EEPROM, marking it with our fingerprint so we'll 
 *          recognize it as ours when we read it again later at startup
 * 
 * @param theState 
 */
void writeEEPROM(nvState_t *theState) {
	state.fingerprint = FINGERPRINT;            // Put our fingerprint on it
	eeprom_write_block((const void*)theState, (void*)0, sizeof(nvState_t)); // Write it to EEPROM
}

/**
 * @brief help command handler: Display command summary
 * 
 * @param h         Pointer to CommandHandlerHelper object
 * @return String   The String to dsiplay
 */
String onHelp(CommandHandlerHelper *h) {
    return
        "help                   Display this text to the user\n"
        "h                      Same as \"help\"\n"
        "assume <pv> <ls>       Assume pv is at to <pv> and ls is at <ls>\n"
        "ls <steps>             Drive leadscrew by <steps>. + ==> out, - ==> in\n"
        "pv <steps>             Drive pivot by <steps>. + ==> CC, - ==> CW viewed from front\n"
        "show <step>            Show step <step> in lunation 0 .. 59"
        "stop                   Stop all motion immediately\n"
        "s                      Same as \"stop\"\n"
        "to <pva>               Move to display with pv angle <pva>\n"
        "where                  Print location of pv and ls\n";
}

/**
 * @brief assume command handler: Set assumed location of pv to <pv> and ls to <ls>
 * 
 * @param h         CommandLineHelper to talk to command processor
 * @return String   What command processor should show to the user
 */
String onAssume(CommandHandlerHelper *h) {
    int32_t pvLoc = h->getWord(1).toInt();
    int32_t lsLoc = h->getWord(2).toInt();
    pvMotor.setLocation(pvLoc);
    lsMotor.setLocation(lsLoc);
    return "Assuming location is (" + String(pvLoc) + ", " + String(lsLoc) + ").\n";
}

/**
 * @brief   ls command handler: Drive leadscrew by <steps>. + ==> out, - ==> in,
 *          but don't change thes location the motor thinks it's at
 * 
 * @param h         CommandLineHelper to talk to command processor
 * @return String   What command processor should show to the user
 */
String onLs(CommandHandlerHelper *h) {
    int32_t steps = h->getWord(1).toInt();
    long loc = lsMotor.getLocation();
    lsMotor.drive(steps);
    while (lsMotor.isMoving()) {
        //Spin
    }
    lsMotor.setLocation(loc);
    return "Driving leadscrew by " + String(steps) + ".\n";
}

/**
 * @brief   pv command handler: Drive pivot by <steps>. + ==> CC, - ==> CW viewed from front, 
 *          but don't change the location the motor thinks it's at.
 * 
 * @param h         CommandLineHelper to talk to command processor
 * @return String   What command processor should show to the user
 */
String onPv(CommandHandlerHelper *h) {
    int32_t steps = h->getWord(1).toInt();
    long loc = pvMotor.getLocation();
    pvMotor.drive(steps);
    while (pvMotor.isMoving()) {
        // Spin
    }
    pvMotor.setLocation(loc);
    return "Driving pivot by " + String(steps) + ".\n";
}

String onShow(CommandHandlerHelper *h) {
    int16_t ph = h->getWord(1).toInt();
    if (ph < 0 || ph > sizeof(stepPva) / sizeof(stepPva[0])) {
        return "Phase to display must be 0 .. 59.\n";
    }
    tgtPhase = ph;
    return "Changing to display phase " + String(ph) + ".\n";
}

/**
 * @brief stop and s command handler: Stop all movement immediately
 * 
 * @param h         CommandLineHelper to talk to command processor
 * @return String   What command processor should show to the user
 */
String onStop(CommandHandlerHelper *h) {
    lsMotor.stop();
    pvMotor.stop();
    return "Ok.\n";
}

/**
 * @brief   to command handler: Move the display so that is displays the moon phase associated 
 *          with the pivot angle (in degrees) <pva>; 0 <= <pva> <= 78.125.
 * 
 * @param h         CommandLineHelper to talk to command processor
 * @return String   What command processor should show to the user
 */
String onTo(CommandHandlerHelper *h) {
    int32_t pv = degToPv(h->getWord(1).toFloat());
    if (pv < 0 || pv > degToPv(78.125)) {
        return "Bad angle: 0.0 < angle < 78.125; Can't go there.\n";
    }
    int32_t ls = pvToLs(pv);
    lsMotor.driveTo(ls);
    pvMotor.driveTo(pv);
    return "Going to (" + String(pv) + ", " + String(ls) + ").\n";
}

/**
 * @brief where command handler: Print current location (in steps) of pv and ls
 * 
 * @param h         CommandLineHelper to talk to command processor
 * @return String   What command processor should show to the user
 */
String onWhere(CommandHandlerHelper *h) {
    return "At: (" + String(pvMotor.getLocation()) + ", " + String(lsMotor.getLocation()) + ").\n";
}


/**
 * @brief Arduino setup() function: Called once at power-on or reset
 * 
 */
void setup() {
    //Initialize built-in LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Initialize Serial
    Serial.begin(9600);
    while (!Serial) {
        digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) == LOW ? HIGH : LOW);
        delay(SERIAL_WAIT_MILLIS);
    }
    Serial.println(BANNER);

    // Restart from EEPROM data if it exists; else cold start
    Serial.println(readEEPROM(&state) ? "Restarting from EEPROM data." : "Cold starting.");
    int32_t initPv = degToPv(stepPva[state.curPhase]);
    int32_t initLs = pvToLs(initPv);
    tgtPhase = state.curPhase;
    
    // Initialize stepper motors
    lsMotor.begin();
    lsMotor.setModulus(0);
    lsMotor.setSpeed(TOP_SPEED);
    lsMotor.setLocation(initLs);
    pvMotor.begin();
    pvMotor.setModulus(0);
    pvMotor.setSpeed(TOP_SPEED / 2);
    pvMotor.setLocation(initPv);
    Serial.println("Starting at phase" + String(state.curPhase));

    // Initialize ui
    if (!(ui.attachCmdHandler("help", onHelp) && ui.attachCmdHandler("h", onHelp) &&
          ui.attachCmdHandler("assume", onAssume) &&
          ui.attachCmdHandler("ls", onLs) && 
          ui.attachCmdHandler("pv", onPv) &&
          ui.attachCmdHandler("stop", onStop) && ui.attachCmdHandler("s", onStop) &&
          ui.attachCmdHandler("to", onTo) &&
          ui.attachCmdHandler("where", onWhere))) {
        Serial.println("Too many commandHandlers\n");
    }
    Serial.println("Ready. Type 'help' or 'h' for command summary.");
}

/**
 * @brief Arduino loop() function: Called repeatedly after setuo() returns
 * 
 */
void loop() {
    static boolean resetting = false;           // true when driving the display backwards to go from ph 29 to 30 or 59 to 0
                                                // false otherwise.
    // Let the ui do its thing
    ui.run();

    // If no motors are running we might need to do something
    if (!lsMotor.isMoving() && !pvMotor.isMoving()) {
        // If the current and target phases don't match, we need to move the display
        if (state.curPhase != tgtPhase) {
            // Choose the next step on our way
            state.curPhase += resetting ? -1 : 1;
            // If it's a transition from phase 59 to phase 0, we need to begin by resetting to phase 30 
            if (state.curPhase == 60) {
                tgtPhase = 30;
                resetting = true;
            }
            //if it's a transition from phase 29 to phase 30, need to begin by resetting to phase 0
            if (state.curPhase == 30) {
                tgtPhase = 0;
                resetting = true;
            }
            if (state.curPhase != tgtPhase) {
                int32_t pv = degToPv(stepPva[state.curPhase]);
                int32_t ls = pvToLs(pv);
                lsMotor.driveTo(ls);
                pvMotor.driveTo(pv);
            }
        // Otherwise if we're resetting, we might be done with doing that
        } else if (resetting) {
            // if we're at phase 30, that means a reset to phase 0 is complete
            if (state.curPhase == 30) {
                state.curPhase = 0;
                resetting = false;
            // Otherwise, if w're at phase 0, that means we just completed a reset to phase 30
            } else if (state.curPhase == 0) {
                state.curPhase = 30;
                resetting = false;
            }
        }
    }
}