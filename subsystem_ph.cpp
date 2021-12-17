/***************************************************************
 * $ Bioreactor :: CSEEE Group4 $
 *
 * Title: subsystem_ph.cpp
 *
 * Author: Ameera Nor Khairi (ameera.khairi.21@ucl.ac.uk)
 * Author: Teii Ri (teii.ri.21@ucl.ac.uk)
 * Date: 2021/12/11
 * Version: 1.0
 **************************************************************/
//The code for calculating average voltage readings and converting voltage readings to pH are adapted from:
//https://scidle.com/how-to-use-a-ph-sensor-with-arduino/

#include "subsystem_ph.h"
#include <Arduino.h>

/**
 * @brief Get current PH value
 */
void PHSubsystem::readCurrentPH() {
    uint16_t avgValue;
    uint16_t buf[10];

    for (unsigned int &i: buf) {
        i = analogRead(PH_PROBE_PIN);
        delay(10);
    }

    for (uint8_t i = 0; i < 10 - 1; i++)
        for (uint8_t j = i + 1; j < 10; j++)
            if (buf[i] > buf[j]) {
                /* Exchange the values of buf[i] and buf[j] */
                buf[i] ^= buf[j];
                buf[j] ^= buf[i];
                buf[i] ^= buf[j];
            }

    avgValue = 0;
    for (uint8_t i = 2; i < 8; i++)
        avgValue += buf[i];

    float pHVol = (float) avgValue * 5.0f / 1024 / 6;
    currentPH = pHVol + 0.455;
}

/**
 * @brief Control two pumps
 */
void PHSubsystem::workCycle() {
    readCurrentPH();

    if (currentPH < targetPH - 0.5) {
        digitalWrite(PUMP_1_PIN, HIGH);
        digitalWrite(PUMP_2_PIN, LOW);
    } else if (currentPH > targetPH + 0.5) {
        digitalWrite(PUMP_1_PIN, LOW);
        digitalWrite(PUMP_2_PIN, HIGH);
    } else {
        digitalWrite(PUMP_1_PIN, LOW);
        digitalWrite(PUMP_2_PIN, LOW);
    }
}

void PHSubsystem::setTargetPH(const float ph) { targetPH = ph; }

float PHSubsystem::getCurrentPH() const { return currentPH; }
