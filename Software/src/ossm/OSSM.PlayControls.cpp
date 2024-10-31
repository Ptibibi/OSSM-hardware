#include "OSSM.h"

#include "extensions/u8g2Extensions.h"
#include "services/tasks.h"
#include "utils/analog.h"
#include "utils/format.h"

void OSSM::drawPlayControlsTask(void *pvParameters) {
    // parse ossm from the parameters
    OSSM *ossm = (OSSM *)pvParameters;
    ossm->encoder.setAcceleration(10);
    ossm->encoder.setBoundaries(0, 100, false);
    // Clean up!
    switch (ossm->playControl) {
        case PlayControls::STROKE:
            ossm->encoder.setEncoderValue(ossm->setting.stroke);
            break;
        case PlayControls::SENSATION:
            ossm->encoder.setEncoderValue(ossm->setting.sensation);
            break;
        case PlayControls::DEPTH:
            ossm->encoder.setEncoderValue(ossm->setting.depth);
            break;
    }

    auto menuString = menuStrings[ossm->menuOption];

    SettingPercents next;
    next.speed = ossm->setting.speed;
    next.stroke = ossm->setting.stroke;
    next.depth = ossm->setting.depth;
    next.sensation = ossm->setting.sensation;

    SettingPercents target;
    target.speed = ossm->setting.speed;
    target.stroke = ossm->setting.stroke;
    target.depth = ossm->setting.depth;
    target.sensation = ossm->setting.sensation;

    unsigned long displayLastUpdated = 0;

    /**
     * /////////////////////////////////////////////
     * /////////// Play Controls Display ///////////
     * /////////////////////////////////////////////
     *
     * This is a safety feature to prevent the user from accidentally beginning
     * a session at max speed. After the user decreases the speed to 0.5% or
     * less, the state machine will be allowed to continue.
     */
    auto isInCorrectState = [](OSSM *ossm) {
        // Add any states that you want to support here.
        return ossm->sm->is("simplePenetration"_s) ||
               ossm->sm->is("simplePenetration.idle"_s) ||
               ossm->sm->is("strokeEngine"_s) ||
               ossm->sm->is("strokeEngine.idle"_s);
    };

    // Line heights
    short lh3 = 56;
    short lh4 = 64;
    static float encoder = 0;

    bool isStrokeEngine =
        ossm->sm->is("strokeEngine"_s) || ossm->sm->is("strokeEngine.idle"_s);

    // This small break gives the encoder a minute to settle.
    vTaskDelay(100);
    while (isInCorrectState(ossm)) {
        // Always assume the display should not update.
        bool shouldUpdateDisplay = false;

        /**
         * ////////////////////////////////////////////
         * ///////////    Manage analog    ////////////
         * ////////////////////////////////////////////
         */
        target.speed = round(getAnalogAveragePercent(SampleOnPin{Pins::Remote::speedPotPin, 50}));

        /**
         * /////////////////////////////////////////////
         * ///////////    Manage encoder    ////////////
         * /////////////////////////////////////////////
         */
        encoder = ossm->encoder.readEncoder();
        switch (ossm->playControl) {
            // STROKE used for SinglePenetration and StrokeEngine
            case PlayControls::STROKE:
                target.stroke = encoder;
                break;
            // SENSATION used for StrokeEngine
            case PlayControls::SENSATION:
                target.sensation = encoder;
                break;
            // DEPTH used for StrokeEngine
            case PlayControls::DEPTH:
                target.depth = encoder;
                break;
        }

        /**
         * /////////////////////////////////////////////
         * ///////////   Apply input ramp   ////////////
         * /////////////////////////////////////////////
         */
        next.speed = applyRamp(next.speed, target.speed,
                               Config::Remote::speedUpPercentPerCycle,
                               Config::Remote::speedDownPercentPerCycle);
        next.stroke = applyRamp(next.stroke, target.stroke,
                                Config::Remote::strokeUpPercentPerCycle,
                                Config::Remote::strokeDownPercentPerCycle);
        next.sensation = applyRamp(next.sensation, target.sensation,
                                   Config::Remote::sensationUpPercentPerCycle,
                                   Config::Remote::sensationDownPercentPerCycle);
        next.depth = applyRamp(next.depth, target.depth,
                               Config::Remote::depthUpPercentPerCycle,
                               Config::Remote::depthDownPercentPerCycle);

        /**
         * //////////////////////////////////////////////
         * /////////// Inputs update display ////////////
         * //////////////////////////////////////////////
         */
        shouldUpdateDisplay = shouldUpdateDisplay || next.speed - ossm->setting.speed != 0;
        shouldUpdateDisplay = shouldUpdateDisplay || next.stroke - ossm->setting.stroke != 0;
        shouldUpdateDisplay = shouldUpdateDisplay || next.sensation - ossm->setting.sensation != 0;
        shouldUpdateDisplay = shouldUpdateDisplay || next.depth - ossm->setting.depth != 0;

        shouldUpdateDisplay = shouldUpdateDisplay || millis() - displayLastUpdated > 1000;

        /**
         * ///////////////////////////////////////////////
         * /////////// Apply next input value ////////////
         * ///////////////////////////////////////////////
         */
        ossm->setting.speed = next.speed;
        ossm->setting.stroke = next.stroke;
        ossm->setting.sensation = next.sensation;
        ossm->setting.depth = next.depth;

        // Waiting and continue if display not updated
        if (!shouldUpdateDisplay) {
            vTaskDelay(100);
            continue;
        }

        displayLastUpdated = millis();

        String strokeString = UserConfig::language.Stroke;
        auto stringWidth = ossm->display.getUTF8Width(strokeString.c_str());

        displayMutex.lock();
        ossm->display.clearBuffer();
        ossm->display.setFont(Config::Font::base);

        drawShape::settingBar(UserConfig::language.Speed, target.speed, next.speed);

        if (isStrokeEngine) {
            switch (ossm->playControl) {
                case PlayControls::STROKE:
                    drawShape::settingBarSmall(target.sensation, next.sensation, 125);
                    drawShape::settingBarSmall(target.depth, next.depth, 120);
                    drawShape::settingBar(strokeString, target.stroke, next.stroke,
                                          118, 0, RIGHT_ALIGNED);
                    break;
                case PlayControls::SENSATION:
                    drawShape::settingBar("Sensation", target.sensation, next.sensation,
                                          128, 0, RIGHT_ALIGNED, 10);
                    drawShape::settingBarSmall(target.depth, next.depth, 113);
                    drawShape::settingBarSmall(target.stroke, next.stroke, 108);

                    break;
                case PlayControls::DEPTH:
                    drawShape::settingBarSmall(target.sensation, next.sensation, 125);
                    drawShape::settingBar("Depth", target.depth, next.depth,
                                          123, 0, RIGHT_ALIGNED, 5);
                    drawShape::settingBarSmall(target.stroke, next.stroke, 108);

                    break;
            }
        } else {
            drawShape::settingBar(strokeString, target.stroke, next.stroke,
                                  118, 0, RIGHT_ALIGNED);
        }

        /**
         * /////////////////////////////////////////////
         * /////////// Play Controls Left  ////////////
         * /////////////////////////////////////////////
         *
         * These controls are associated with stroke and distance
         */
        ossm->display.setFont(Config::Font::small);
        strokeString = "# " + String(ossm->sessionStrokeCount);
        ossm->display.drawUTF8(14, lh4, strokeString.c_str());

        /**
         * /////////////////////////////////////////////
         * /////////// Play Controls Right  ////////////
         * /////////////////////////////////////////////
         *
         * These controls are associated with stroke and distance
         */

        if (!isStrokeEngine) {
            strokeString = formatDistance(ossm->sessionDistanceMeters);
            stringWidth = ossm->display.getUTF8Width(strokeString.c_str());
            ossm->display.drawUTF8(104 - stringWidth, lh3,
                                   strokeString.c_str());
        }

        strokeString =
            formatTime(displayLastUpdated - ossm->sessionStartTime).c_str();
        stringWidth = ossm->display.getUTF8Width(strokeString.c_str());
        ossm->display.drawUTF8(104 - stringWidth, lh4, strokeString.c_str());

        ossm->display.sendBuffer();
        displayMutex.unlock();

        vTaskDelay(200);
    }

    vTaskDelete(nullptr);
};

void OSSM::drawPlayControls() {
    int stackSize = 3 * configMINIMAL_STACK_SIZE;
    xTaskCreate(drawPlayControlsTask, "drawPlayControlsTask", stackSize, this,
                1, &drawPlayControlsTaskH);
}

// Apply ramp in current value to move to target value
float OSSM::applyRamp(float current, float target,
                      float increaseValue, float decreaseValue) {
    if (target > current)
        current = constrain(current + increaseValue, current, target);
    else
        current = constrain(current - decreaseValue, target, current);
    return current;
}
