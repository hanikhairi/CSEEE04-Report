/***************************************************************
 * $ Bioreactor :: CSEEE Group4 $
 *
 * Title: subsystem_stirring.cpp
 *
 * Author: Teii Ri (teii.ri.21@ucl.ac.uk)
 * Author: Rui Huang (rui.huang.21@ucl.ac.uk)
 * Date: 2021/12/11
 * Version: 1.0
 **************************************************************/

#include "subsystem_stirring.h"
#include <Arduino.h>

/**
 * @brief Get motor speed in RPM
 */
void StirringSubsystem::getMotorSpeedCurrent() {
    uint64_t start_time = millis();
    uint64_t end_time = start_time + 1000;
    uint16_t count = 0;

    while (millis() < end_time) {
        if ((PINC & (1 << PINC1)) > 0) {
            count++;
            while (((PINC & (1 << PINC1)) > 0) && (millis() < end_time));
        }
    }

    speedCurrent = count * 60;
}

/**
 * @brief Control motor
 */
void StirringSubsystem::workCycle() {
    if (speedTarget == 0) {
        digitalWrite(STIRRING_MOTOR_PIN, LOW);
        motorDutyCycle = 0;
        speedCurrent = 0;
        return;
    }

    getMotorSpeedCurrent();

    if (speedCurrent == 0) {
        motorDutyCycle = 150;
        analogWrite(STIRRING_MOTOR_PIN, 254);
        analogWrite(STIRRING_MOTOR_PIN, motorDutyCycle);
        getMotorSpeedCurrent();
    }

    int adjValue = (abs(speedTarget - speedCurrent) > 150)
                   ? 10
                   : (signed) abs(speedTarget - speedCurrent) / 10;

    motorDutyCycle += (speedCurrent < speedTarget) ? adjValue : -adjValue;

    if (motorDutyCycle > 255)
        motorDutyCycle = 255;

    analogWrite(STIRRING_MOTOR_PIN, motorDutyCycle);
}

void StirringSubsystem::setSpeedTarget(const uint16_t speed) { speedTarget = speed; }

uint16_t StirringSubsystem::getMotorDutyCycle() const { return motorDutyCycle; }

uint16_t StirringSubsystem::getSpeedCurrent() const { return speedCurrent; }
