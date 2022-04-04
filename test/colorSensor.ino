#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Example code for the Adafruit TCS34725 breakout library */

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

int buttonPin = 22;
int numReadings = 10;

void setup(void) {
    Serial.begin(9600);

    if (tcs.begin()) {
        Serial.println("Found sensor");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1);
    }
    // Now we're ready to get readings!
}

void loop(void) {
    uint16_t r, g, b, c, colorTemp, lux;
    uint32_t rTotal = 0, gTotal = 0, bTotal = 0, cTotal = 0;

    Serial.print("Measuring new reading... ");
    Serial.println(" ");

    // take numReadings readings and average the RGB values
    for (int i = 0; i < numReadings; i ++)
    {
        tcs.getRawData(&r, &g, &b, &c);
        rTotal += r;
        gTotal += g;
        bTotal += b;
        cTotal += c;
    }
    // colorTemp = tcs.calculateColorTemperature(r, g, b);
    colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    lux = tcs.calculateLux(r, g, b);

    Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
    Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
    Serial.print("R: "); Serial.print(rTotal / numReadings, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(gTotal / numReadings, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(bTotal / numReadings, DEC); Serial.print(" ");
    Serial.print("C: "); Serial.print(cTotal / numReadings, DEC); Serial.print(" ");
    Serial.println(" ");

    float fR, fG, fB; 
    float fRTotal = 0, fBTotal = 0, fGTotal = 0;
    // take numReadings readings and average the RGB values
    for (int i = 0; i < numReadings; i ++)
    {
        tcs.getRGB(&fR, &fG, &fB);
        fRTotal += fR;
        fGTotal += fG;
        fBTotal += fB;
    }

    Serial.print("R: "); Serial.print(fRTotal / numReadings, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(fGTotal / numReadings, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(fBTotal / numReadings, DEC); Serial.print(" ");
    Serial.println(" ");
    Serial.println(" ");
}