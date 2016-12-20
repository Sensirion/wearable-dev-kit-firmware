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
#ifndef CIRCULARBUFFER_h
#define CIRCULARBUFFER_h

#include <stddef.h>

/**
 * The circular buffer returns always the "(Size-1)"-th value back in time. For
 * example if the circular buffer Size is 3 the get() method returns: value[t-2]
 */

template <typename T, size_t Size>
class CircularBuffer {

public:
    CircularBuffer();
    /** Add elemen to buffer */
    void add(T value);
    /** Get least recently added element */
    T get();
    /** Clear buffer */
    void clear();

private:
    T _buffer[Size];
    size_t _start;
    size_t _end;
    bool _full;
};

/**
 * Make the whole source code of the CircularBuffer template available to the method
 * which instantiate the template class
 */
#include "CircularBuffer.tpp"

#endif
