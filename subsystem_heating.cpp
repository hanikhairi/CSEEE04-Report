/***************************************************************
 * $ Bioreactor :: CSEEE Group4 $
 *
 * Title: subsystem_heating.cpp
 *
 * Author: Teii Ri (teii.ri.21@ucl.ac.uk)
 * Date: 2021/12/11
 * Version: 1.0
 **************************************************************/

#include "subsystem_heating.h"
#include <Arduino.h>

/**
 * @brief Get the current temperature from NTC thermistor
 *
 * NTC thermistor temperature calculation formula:
 *     T = (T0 * B) / (T0 * ln(R / R0) + B)
 *     - R is the resistance of the thermistor at temperature T0
 *     - B is Thermal Index
 */
void HeatingSubsystem::getTempCelsiusCurrent() {
    uint8_t ADCLow, ADCHigh;
    uint16_t analogIn;
    float ntcR;

    /*
     * Set the analog reference of 5 volts
     * Select ADC0
     */
    ADMUX = (DEFAULT << 6) | (0 & 0x07);
    /* Start conversion */
    ADCSRA |= (1 << ADSC);
    /* Wait for conversion */
    while (ADCSRA & (1 << ADSC));
    /* Get result from the ADC Data Register */
    ADCLow = ADCL;
    ADCHigh = ADCH;

    analogIn = (ADCHigh << 8) | ADCLow;
    ntcR = (float) analogIn / (1024 - analogIn) * vdR;

    tempCurrent = (ntcT0 * ntcBeta) / (ntcT0 * (float) log((double) ntcR / ntcR0) + ntcBeta);
    tempCurrent -= 273.15;
}

/**
 * @brief Control heater
 */
void HeatingSubsystem::setHeaterPower() {
    uint8_t proTemp = 2;

    if (tempTarget > tempCurrent + proTemp) /* True ON */ {
        heaterDutyCycle = 1;
        /* Set normal port operation */
        TCCR1A &= ~(1 << COM1A1);
        PORTB |= (1 << DDB1);
    } else if (tempTarget > tempCurrent) /* Enable PWM */ {
        heaterDutyCycle = (tempTarget - tempCurrent) / proTemp;
        /*
         * Connect pwm to pin on timer 1, channel A.
         * The OC1A output overrides the normal port functionality of the I/O pin
         */
        TCCR1A |= (1 << COM1A1);
        OCR1A = (int) (heaterDutyCycle * 255);
    } else /* Turn OFF */ {
        heaterDutyCycle = 0;
        TCCR1A &= ~(1 << COM1A1);
        PORTB &= ~(1 << DDB1);
    }
}

void HeatingSubsystem::workCycle() {
    getTempCelsiusCurrent();
    setHeaterPower();
}

void HeatingSubsystem::setTempTarget(const float temp) { tempTarget = temp; }

float HeatingSubsystem::getTempCurrent() const { return tempCurrent; }

float HeatingSubsystem::getHeaterDutyCycle() const { return heaterDutyCycle; }
