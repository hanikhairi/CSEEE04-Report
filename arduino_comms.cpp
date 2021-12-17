/***************************************************************
 * $ Bioreactor :: CSEEE Group4 $
 *
 * Title: arduino_comms.cpp
 * Author: Uzair Khan (uzair.khan.21@ucl.ac.uk)
 * Author: Teii Ri (teii.ri.21@ucl.ac.uk)
 * Date: 2021/12/11
 * Version: 1.0
 **************************************************************/

//The functions (sendData, receiveData) for communication between ESP32 and Arduino is adapted from:
//https://moodle.ucl.ac.uk/pluginfile.php/4381816/mod_resource/content/4/Bioreactor%20connectivity%20overview.pdf

//The logic behind sending the float array is adapted from the first answer from:
//https://arduino.stackexchange.com/questions/63586/is-it-possible-to-send-a-float-array-over-i2c 

#define SLAVE_ADD 9

#include <Arduino.h>
#include <Wire.h>

#include "subsystem_heating.h"
#include "subsystem_stirring.h"
#include "subsystem_ph.h"

union bytesToFloat {
    uint8_t buffer[4];
    float floatVal;
} b2f;

HeatingSubsystem subSysHeat;
StirringSubsystem subSyStir;
PHSubsystem subSysPH;

float targetTemp, currentTemp;
float targetRPM, currentRPM;
float targetPH, currentPH;

void sendData() {
    float myData[3] = {currentTemp, currentRPM, currentPH};

    Wire.write((uint8_t *) myData, sizeof(myData));
}

void receiveData(int numBytes) {
    uint8_t index = 0;
    while (Wire.available()) {
        b2f.buffer[index++] = Wire.read();
    }

    if (b2f.floatVal >= 480 && b2f.floatVal <= 1520)
        targetRPM = b2f.floatVal;
    else if (b2f.floatVal >= 24.5 && b2f.floatVal <= 35.5)
        targetTemp = b2f.floatVal;
    else if (b2f.floatVal >= 2.9 && b2f.floatVal <= 7.1)
        targetPH = b2f.floatVal;
}

void setup() {
    /*
     * Set PC0(A0) and PC1(A1) as INPUT Mode
     *     PB1(9) and PB2(10) as OUTPUT Mode
     * Using Data Direction Register
     */
    DDRC &= ~((1 << DDC0) | ((1 << DDC1)));
    DDRB |= ((1 << DDB1) | (1 << DDB2));

    Serial.begin(115200);
    Wire.begin(SLAVE_ADD);

    Wire.onRequest(sendData);
    Wire.onReceive(receiveData);
}

void loop() {
    subSysHeat.setTempTarget(targetTemp);
    subSyStir.setSpeedTarget((uint16_t) targetRPM);
    subSysPH.setTargetPH(targetPH);

    currentTemp = subSysHeat.getTempCurrent();
    currentRPM = subSyStir.getSpeedCurrent();
    currentPH = subSysPH.getCurrentPH();

    subSysHeat.workCycle();
    subSyStir.workCycle();
    subSysPH.workCycle();

    Serial.println(targetTemp);
    Serial.println(targetRPM);
    Serial.println(targetPH);
}
