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
#include "Perspiration.h"
#include <cmath>
#include "LowPassFilter.h"

#define REMOVE_OFFSET_AND_CAP_DATA 1

void initPerspirationEngine()
{
}

static float absoluteHumidity(float temperature, float relativeHumidity)
{
    static const float A = 6.112f; // hPa
    static const float Tn = 243.12f; // °C
    static const float m = 17.62f;
    return 216.7f * (relativeHumidity * 0.01f * A * std::exp(m * temperature / (Tn + temperature)) / (273.15f + temperature));
}

void perspirationEngine(unsigned long timeStampMs,
                        float temperatureAmbientSensor, float relativeHumidityAmbientSensor,
                        float temperatureSkinSensor, float relativeHumiditySkinSensor,
                        float *perspiration)
{
    /* compute the time delta from the timestamp, the time delta will slightly vary but this is desireable for the filter computation */
    static unsigned long timeStampMsLastCall = 0;
    static float delta_t = int(timeStampMs - timeStampMsLastCall) / 1000.0f;
    timeStampMsLastCall = timeStampMs;

    /* relative humidity offset of ambient sensor */
    static const float RH_OFFSET = 0;
    /* temperature offset of ambient sensor */
    static const float T_OFFSET = 0;
    /* distance between sensors in m */
    static const float D_SHT = 0.008f;
    /* diffusion coefficient @ 25°C in m^2/s */
    static const float DIFF_COEF_25_DEGREES = 2.6e-5f;
    /* emperical constant, slope of temperature vs calibration factor */
    static const float T_CORRECTION_FACTOR = 0.04f;
    /* diffusion coefficient temperature dependent */
    float DIFF_COEF = DIFF_COEF_25_DEGREES * T_CORRECTION_FACTOR * temperatureSkinSensor;
    float CONVERSION_FACTOR = 1.0f / D_SHT * DIFF_COEF * 3600.0f;
    /* calibration factor of device */
    static const float CALIBRATION_FACTOR = 1.5f;

    /* define time constant for low pass filter and apply filtering on both temperature values */
    static const float TAU = 30;
    temperatureAmbientSensor = firstOrder_lowPassFilter(temperatureAmbientSensor, TAU, delta_t);
    temperatureSkinSensor = firstOrder_lowPassFilter(temperatureSkinSensor, TAU, delta_t);

    #if REMOVE_OFFSET_AND_CAP_DATA
    /* use adjusted temperature and relative humidity to calculate absolute humidity */
    temperatureAmbientSensor += T_OFFSET;
    relativeHumidityAmbientSensor += RH_OFFSET;
    #endif /* REMOVE_OFFSET_AND_CAP_DATA */
    float absoluteHumidityAmbientSensor = absoluteHumidity(temperatureAmbientSensor, relativeHumidityAmbientSensor);
    float absoluteHumiditySkinSensor = absoluteHumidity(temperatureSkinSensor, relativeHumiditySkinSensor);

    #if REMOVE_OFFSET_AND_CAP_DATA
    if (absoluteHumiditySkinSensor > absoluteHumidityAmbientSensor) {
    #endif /* REMOVE_OFFSET_AND_CAP_DATA */
        *perspiration = CALIBRATION_FACTOR * CONVERSION_FACTOR * (absoluteHumiditySkinSensor - absoluteHumidityAmbientSensor);
    #if REMOVE_OFFSET_AND_CAP_DATA
    } else {
        *perspiration = 0.0f;
    }
    #endif /* REMOVE_OFFSET_AND_CAP_DATA */
}
