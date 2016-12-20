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
#include "TemperatureCompensation.h"

#include "AirTouch.h"
#include "DebugPrint.h"
#include <math.h>

static float sampletime = 0.1f;  // sample time  [sec]

static float fsqrtf(float x)
{
    const float xhalf = 0.5f * x;
    union { float x; int i; } u;
    u.x = x;
    u.i = 0x5f3759df - (u.i >> 1);
    // x * 1/sqrt(x) = sqrt(x)
    return x * u.x * (1.5f - xhalf * u.x * u.x);
}

#define C_TO_F(C) (((C) * 9.0f / 5.0f) + 32.0f)
#define F_TO_C(F) (((F) - 32.0f) * (5.0f / 9.0f))

// change this value to your specific device
static const float CORRECTION_FACTOR = 1.5f;

static float calc_humidex(float RH, float T)
{
    float humidex, temp;
    temp = 6.112f * RH / 100.0f * exp(17.62f * T / (243.12f + T));
    humidex = T + (5.0f / 9.0f * (temp - 10.0f));
    //humidex only defined for higher temperatures
    return humidex > T ? humidex : T;
}

static float calc_heatindex(float RH, float T)
{
    float hi = 0.0f;

    if (T < 4.44f) {
        hi = C_TO_F(T);
    } else {
        float t_f = C_TO_F(T);
        float hitemp = 61.0f + ((t_f - 68.0f) * 1.2f) + (RH * 0.094f);
        float fptemp = t_f;
        float hifinal = 0.5f * (fptemp + hitemp);

        if (hifinal > 79.0f) {
            float t_f2 = t_f * t_f;
            float RH2 = RH * RH;
            hi = -42.379f +
                2.04901523f * t_f +
                10.14333127f * RH -
                0.22475541f * t_f * RH -
                0.00683783f * t_f2 -
                0.05481717f * RH2 +
                0.00122874f * t_f2 * RH +
                0.00085282f * t_f * RH2 -
                0.00000199f * t_f2 * RH2;

            if ((RH <= 13.0f) && (t_f >= 80.0f) && (t_f <= 112.0f)) {
                float adj1 = (13.0f - RH) / 4.0f;
                float adj2 = fsqrtf((17.0f - fabsf(t_f - 95.0f)) / 17.0f);
                float adj = adj1 * adj2;
                hi = hi - adj;
            } else if ((RH > 85.0f) && (t_f >= 80.0f) && (t_f <= 87.0f)) {
                float adj1 = (RH - 85.0f) / 10.0f;
                float adj2 = (87.0f - t_f) / 5.0f;
                float adj = adj1 * adj2;
                hi = hi + adj;
            }

        } else {
            hi = hifinal;
        }
    }

    hi = F_TO_C(hi);
    return (hi > T) ? hi : T;
}

void initTemperatureCompensation(TemperatureCompensationMode compensationMode)
{
    switch (compensationMode) {
        case TemperatureCompensationMode::Medium:
            DEBUG_PRINTLN("Compensation Mode changed to normal");
            break;

        case TemperatureCompensationMode::Fast:
            DEBUG_PRINTLN("Compensation Mode changed to fast");
            break;
        case TemperatureCompensationMode::Slow:
            DEBUG_PRINTLN("Compensation Mode changed to slow");
            break;

        case TemperatureCompensationMode::Algebraic:
            DEBUG_PRINTLN("Compensation Mode changed to algebraic");
            break;

        default:
            DEBUG_PRINTLN("Compensation Mode changed to normal");
            break;
    }
}

void temperatureCompensation(unsigned long timeStampMs, float temperatureSkinSensor,
                             float temperatureAmbientSensor, float relativeHumidityAmbientSensor,
                             float *skinTemperature, float *heatIndex, float *apparentTemperature,
                             float *humidex, AirTouchEvent *airTouchEvent)
{
    static float timeStampMsLastCall = 0;
    sampletime = (timeStampMs - timeStampMsLastCall) / 1000.0f;

    // adapt the CORRECTION_FACTOR to your specific device
    *apparentTemperature = temperatureAmbientSensor - CORRECTION_FACTOR * (temperatureSkinSensor - temperatureAmbientSensor);
    *heatIndex = calc_heatindex(relativeHumidityAmbientSensor, *apparentTemperature);
    *humidex = calc_humidex(relativeHumidityAmbientSensor, *apparentTemperature);

    airtouchEngine(timeStampMs, *apparentTemperature, relativeHumidityAmbientSensor, airTouchEvent);
}
