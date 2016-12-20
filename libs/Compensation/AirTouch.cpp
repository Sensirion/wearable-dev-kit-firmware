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
#include "AirTouch.h"

#define A_SIZE (3)
#define B_SIZE (1)
#define C_SIZE (1)

static const float A_MATRIX[(A_SIZE*A_SIZE)] = {
        9.9416909621e-01f, -1.1661807580e-01f, -2.3323615160e-01f,
        9.4460641399e-02f, 8.8921282799e-01f, -2.2157434402e-01f,
        3.1486880466e-02f, 6.2973760933e-01f, 2.5947521866e-01f,
};

static const float B_MATRIX[(A_SIZE*B_SIZE)] = {
        -4.3191879927e-01f,
        -4.1032285930e-01f,
        2.3323615160e+00f,
};

static const float C_MATRIX[(C_SIZE*A_SIZE)] = {
        1.5743440233e-02f, 3.1486880466e-01f, 6.2973760933e-01f,
};

static const float D_MATRIX[(C_SIZE*B_SIZE)] = {
        1.1661807580e+00f,
};

static const float I_MATRIX[(A_SIZE*B_SIZE)] = {
        3.2237347322e-15f,
        -3.7037037037e+00f,
        0.0000000000e+00f,
};



static const float ENGINE_UP_THRESHOLD  = 0.42f;
static const float ENGINE_LOW_THRESHOLD = 0.1f;

static float old_x[A_SIZE];
static char run = 0;
static bool engine_triggered = false;


void airtouchEngine(unsigned long timeStampMs,
                    float temperatureAmbient,
                    float relativeHumidityAmbient,
                    AirTouchEvent *airTouchEvent) {

    float x[A_SIZE];
    float y[C_SIZE];
    float u[B_SIZE];

    u[0] = relativeHumidityAmbient;

    if (run == 0) {
        for (int row=0; row<A_SIZE; row++) {
            x[row] = 0.0f;
            for (int col=0; col<B_SIZE; col++) {
                x[row] += I_MATRIX[row * B_SIZE + col] * u[col];
            }
        }
        run += 1;
    } else {
        for (int row=0; row<A_SIZE; row++) {
            x[row] = 0.0f;
            for (int col=0; col<A_SIZE; col++) {
                x[row] += A_MATRIX[row * A_SIZE + col] * old_x[col];
            }
            for (int col=0; col<B_SIZE; col++) {
                x[row] += B_MATRIX[row * B_SIZE + col] * u[col];
            }
        }
    }


    for (int i=0; i<A_SIZE; i++) {
        old_x[i] = x[i];
    }

    for (int row=0; row<C_SIZE; row++) {
        y[row]=0.0f;
        for (int col=0; col<A_SIZE; col++) {
            y[row] += C_MATRIX[row * A_SIZE + col] * x[col];
        }
        for (int col=0; col<B_SIZE; col++) {
            y[row] += D_MATRIX[row * B_SIZE + col] * u[col];
        }
    }

    if (engine_triggered) {
        if (y[0] < ENGINE_LOW_THRESHOLD) {
            engine_triggered = false;
            *airTouchEvent = AirTouchEvent::Stop;
        }
        else {
            *airTouchEvent = AirTouchEvent::None;
        }
    } else {
        if (y[0] > ENGINE_UP_THRESHOLD) {
            engine_triggered = true;
            *airTouchEvent = AirTouchEvent::Start;
        }
        else {
            *airTouchEvent = AirTouchEvent::None;
        }
    }
}
