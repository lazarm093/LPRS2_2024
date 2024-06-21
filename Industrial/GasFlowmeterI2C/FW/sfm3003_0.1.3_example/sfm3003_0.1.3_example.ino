/*
 * Copyright (c) 2020, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sfm3003.h"
#include <stdio.h>
#include <Wire.h>  // Biblioteka za I2C komunikaciju
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);





void setup() {

    Serial.begin(115200);  // Inicijalizacija serijske komunikacije
    
    lcd.begin(16, 2);  // Inicijalizacija LCD-a 2x16

    Wire.begin();  // Inicijalizacija I2C komunikacije

    /* Reset svih I2C uređaja */
    int16_t error = sensirion_i2c_general_call_reset();
    if (error) {
        Serial.println("General call reset failed");
    }

    /* Sačekajte da se SFM3003 inicijalizuje */
    delayMicroseconds(SFM3003_SOFT_RESET_TIME_US);

	Serial.println(SFM3003_I2C_ADDRESS, HEX);
	bool found_dev = false;
    for(int i = 0; i < 10; i++) {
    	found_dev = !sfm3003_probe();
    	if(found_dev){
    		break;
    	}else{
        	Serial.println("SFM sensor probing failed");
    	}
    }
    if(!found_dev){
    	return;
    }

    uint32_t product_number = 0;
    uint8_t serial_number[8] = {};
    error = sfm_common_read_product_identifier(SFM3003_I2C_ADDRESS,
                                               &product_number, &serial_number);
    if (error) {
        Serial.println("Failed to read product identifier");
    } else {
        Serial.print("product: 0x");
        Serial.println(product_number, HEX);
        Serial.print("serial: 0x");
        for (size_t i = 0; i < 8; ++i) {
            Serial.print(serial_number[i], HEX);
        }
        Serial.println();
    }

    SfmConfig sfm3003 = sfm3003_create();

    error = sfm_common_start_continuous_measurement(
        &sfm3003, SFM3003_CMD_START_CONTINUOUS_MEASUREMENT_AIR);
    if (error) {
        Serial.println("Failed to start measurement");
    }

    /* Sačekajte prvu merenje. Možete sačekati SFM3003_MEASUREMENT_INITIALIZATION_TIME_US
     * umesto SFM3003_MEASUREMENT_INITIALIZATION_TIME_US da biste dobili preciznije rezultate */
    delayMicroseconds(SFM3003_MEASUREMENT_INITIALIZATION_TIME_US);

    for (;;) {
        int16_t flow_raw;
        int16_t temperature_raw;
        uint16_t status;
        error = sfm_common_read_measurement_raw(&sfm3003, &flow_raw,
                                                &temperature_raw, &status);
        if (error) {
            Serial.println("Error while reading measurement");
        } else {
            float flow;
            float temperature;
            error = sfm_common_convert_flow_float(&sfm3003, flow_raw, &flow);
            if (error) {
                Serial.println("Error while converting flow");
            }
            temperature = sfm_common_convert_temperature_float(temperature_raw);
            Serial.print("Flow: ");
            Serial.print(flow, 3);  // Printuj do 3 decimale
            Serial.print(" (");
            Serial.print(flow_raw);
            Serial.print(")  Temperature: ");
            Serial.print(temperature, 2);  // Printuj do 2 decimale
            Serial.print(" (");
            Serial.print(temperature_raw);
            Serial.print(")  Status: ");
            Serial.println(status, HEX);  // Printuj u HEX formatu

            lcd.setCursor(0, 0);
            lcd.print("Flow: ");
            lcd.print(flow, 1);  // Možete prilagoditi broj decimala
            lcd.print(" ");
            lcd.setCursor(0, 1);
            lcd.print("Temp: ");
            lcd.print(temperature, 2);  // Možete prilagoditi broj decimala
            delay(500);

            
        }
    }
}


void loop() {
	
}
