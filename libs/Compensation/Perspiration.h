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
#ifndef PERSPIRATION_H
#define PERSPIRATION_H

#include <Hardware.h>
#include <Arduino.h>

#include "AirTouch.h"

/**
 * Initialize the perspiration engine
 */
void initPerspirationEngine();

/**
 * Compensation engine to calculate persperation.
 *
 * @param timeStampMs                   Time stamp in ms since the firmware start
 * @param temperatureAmbientSensor      Temperature from sensor away from skin
 * @param relativeHumidityAmbientSensor Relative humidity in % from sensor away from skin
 * @param temperatureSkinSensor         Temperature from the sensor close to skin
 * @param relativeHumiditySkinSensor    Relative humidity in % from sensor close to skin
 * @param[out] perspiration             Calculated prespiration factor
 */
void perspirationEngine(unsigned long timeStampMs,
                        float temperatureAmbientSensor, float relativeHumidityAmbientSensor,
                        float temperatureSkinSensor, float relativeHumiditySkinSensor,
                        float *perspiration);

#endif // PERSPIRATION_H
