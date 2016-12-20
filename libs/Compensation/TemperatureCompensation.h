/*------------------------------------------------------------------------------
 * Copyright 2016 Sensirion AG, Switzerland. All rights reserved.
 *
 * Software source code and Compensation Algorithms are Sensirion Confidential
 * Information and trade secrets. Licensee shall protect confidentiality of
 * Software source code and Compensation Algorithms.
 *
 * Licensee shall not distribute Software and/or Compensation Algorithms other
 * than in binary form incorporated in Products. DISTRIBUTION OF SOURCE CODE
 * AND/OR COMPENSATION ALGORITHMS IS NOT ALLOWED.
 *
 * Software and Compensation Algorithms are licensed for use with Sensirion
 * sensors only.
 *
 * Licensee shall not change the source code unless otherwise stated by
 * Sensirion.
 *
 * Software and Compensation Algorithms are provided "AS IS" and any and all
 * express or implied warranties are disclaimed.
 *
 * THIS SOFTWARE IS PROVIDED BY SENSIRION "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL SENSIRION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *----------------------------------------------------------------------------*/
#ifndef TEMPERATURE_COMPENSATION_H
#define TEMPERATURE_COMPENSATION_H

#include <Hardware.h>
#include <Arduino.h>

#include "AirTouch.h"

enum struct TemperatureCompensationMode : uint8_t {
    Algebraic = 0,
    Slow,
    Medium,
    Fast,

    NumberOfModes,
};

/**
 * Initialize the engine
 * @param compensationMode              The compensation mode to use
 */
void initTemperatureCompensation(TemperatureCompensationMode compensationMode);

/**
 * Compensation engine to calculate skin, feellike and apparent temperature.
 * All temperatures are in degree Celsius.
 *
 * @param timeStampMs                   Time stamp in ms since the firmware start
 * @param temperatureSkinSensor         Temperature from the sensor close to skin (STS)
 * @param temperatureAmbientSensor      Temperature from sensor away from skin (SHT)
 * @param relativeHumidityAmbientSensor Relative humidity in % from sensor away from skin (SHT)
 * @param[out] skinTemperature          Calculated skin temperature
 * @param[out] heatIndex                Calculated heatIndex temperature (https://en.wikipedia.org/wiki/Heat_index)
 * @param[out] apparentTemperature      Calculated apparent temperature
 * @param[out] humidex                  Canadian humidity index
 * @param[out] airTouchEvent            Event flag which is set when air touch starts or stops
 */
void temperatureCompensation(unsigned long timeStampMs, float temperatureSkinSensor,
                             float temperatureAmbientSensor, float relativeHumidityAmbientSensor,
                             float *skinTemperature, float *heatIndex, float *apparentTemperature,
                             float *humidex, AirTouchEvent *airTouchEvent);

#endif // TEMPERATURE_COMPENSATION_H
