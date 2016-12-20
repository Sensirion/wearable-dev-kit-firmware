##------------------------------------------------------------------------------
# Copyright 2016 Sensirion AG, Switzerland. All rights reserved.
#
# Software source code and Compensation Algorithms are Sensirion Confidential
# Information and trade secrets. Licensee shall protect confidentiality of
# Software source code and Compensation Algorithms.
#
# Licensee shall not distribute Software and/or Compensation Algorithms other
# than in binary form incorporated in Products. DISTRIBUTION OF SOURCE CODE
# AND/OR COMPENSATION ALGORITHMS IS NOT ALLOWED.
#
# Software and Compensation Algorithms are licensed for use with Sensirion
# sensors only.
#
# Licensee shall not change the source code unless otherwise stated by
# Sensirion.
#
# Software and Compensation Algorithms are provided "AS IS" and any and all
# express or implied warranties are disclaimed.
#
# THIS SOFTWARE IS PROVIDED BY SENSIRION "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
# EVENT SHALL SENSIRION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##----------------------------------------------------------------------------*/ 
import struct
import logging
from collections import namedtuple

LOGGER = logging.getLogger(__name__)

import serial

LOG_VALUE_NAMES_AND_TYPES = [
    # Conversion information of the log values
    # Tuple with (Value name, unpack type, normalization factor)
    ("Tup",                       'i', 0.001), # 0
    ("Hup",                       'i', 0.001), # 1
    ("Tdwn",                      'i', 0.001), # 2
    ("Hdwn",                      'i', 0.001), # 3
    ("Rmox1",                     'i', None),  # 4
    ("Rmox2",                     'i', None),  # 5
    ("ReservedSensor1",           'h', None),  # 6
    ("ReservedSensor2",           'h', None),  # 7
    ("ReservedSensor3",           'h', None),  # 8
    ("ReservedSensor4",           'h', None),  # 9
    ("ReservedSensor5",           'h', None),  # 10
    ("ReservedSensor6",           'h', None),  # 11
    ("ReservedSensor7",           'h', None),  # 12
    ("ReservedSensor8",           'h', None),  # 13
    ("ReservedSensor9",           'h', None),  # 14
    ("ReservedSensor10"           'h', None),  # 15

    # Processed values
    ("Tskin",                    'f', 1.0),  # 16
    ("ApparentTemperature",      'f', 1.0),  # 17
    ("HeatIndex",                'f', 1.0),  # 18
    ("Humidex",                  'f', 1.0),  # 19
    ("CompensationMode",         'B', None), # 20
    ("Perspiration",             'f', None), # 21
    ("OnOffBodyState",           'B', 1.0),  # 22
    ("Reserved0",                'f', None), # 23
    ("Reserved1",                'B', None), # 24
    ("Reserved2",                'B', None), # 25
    ("Reserved3",                'i', None), # 26
    ("Reserved4",                'i', None), # 27
    ("Reserved5",                'i', None), # 28
    ("Reserved6",                'i', None), # 29
    ("Reserved7",                'i', None), # 30
    ("Reserved8",                'i', None), # 31
]

LOGGER_STATE = {
    0: "Empty",
    1: "Dirty",
    2: "Erasing",
    3: "Writing",
    4: "WritingPaused",
    5: "LogFull",
    6: "Reading",
    7: "ReadFinished",
}


def _size_from_enabled_channels_mask(enabled_channels_mask):
    # timestamp is always there
    log_size = 4
    for i in range(32):
        if enabled_channels_mask & (1 << i):
            if LOG_VALUE_NAMES_AND_TYPES[i][1] in ['i', 'I', 'f']:
                log_size += 4
            elif LOG_VALUE_NAMES_AND_TYPES[i][1] in ['h']:
                log_size += 2
            elif LOG_VALUE_NAMES_AND_TYPES[i][1] in ['B']:
                log_size += 1
            else:
                raise IOError("Unsupported log channel")
    return log_size


def _types_from_enabled_channels_mask(enabled_channels_mask):
    # timestamp
    types = '<I'
    for i in range(32):
        if enabled_channels_mask & (1 << i):
            types += LOG_VALUE_NAMES_AND_TYPES[i][1]
    return types


def _names_from_enabled_channels_mask(enabled_channels_mask):
    names = ['Timestamp']
    for i in range(32):
        if enabled_channels_mask & (1 << i):
            names.append(LOG_VALUE_NAMES_AND_TYPES[i][0])
    return names

def _conversion_from_enabled_channels_mask(enabled_channels_mask):
    conversion = [None]
    for i in range(32):
        if enabled_channels_mask & (1 <<i):
            conversion.append(LOG_VALUE_NAMES_AND_TYPES[i][2])
    return conversion


class PebbleBackpack(object):

    LogHeader = namedtuple('LogHeader', ['start_time', 'log_interval_ms', 'enabled_channels_mask'])

    def __init__(self, com_port, baud_rate=115200, prefetch=8):
        """
        Establish serial connection

        @param com_port  Serial port to use, e.g:
                         - /dev/ttyACM3 (Linux)
                         - COM12 (Windows)
        @param baud_rate Baud-rate to use with the serial port
        @param prefetch Number of log entries to request ahead of time. A
                        higher value increases performance, but if chosen too
                        high one may loose data.
        """
        self._serial = serial.Serial(com_port, baud_rate, parity=serial.PARITY_NONE, timeout=1)
        self._prefetch = prefetch
        self._header = None
        self._log_entry_size = None
        self._log_entry_types = None
        self._log_entry_names = None
        self._log_entry_conversion = None

    def _flush_and_send(self, byte):
        self._serial.flushOutput()
        self._serial.write(byte)

    def _request_log_header(self):
        self._flush_and_send('h')

    def _request_log_entry(self):
        self._flush_and_send('r')

    def _request_get_state(self):
        self._flush_and_send('g')

    def _request_quit(self):
        self._flush_and_send('q')

    def get_state(self):
        self._request_get_state()
        state = self._serial.read(1)
        if len(state) != 1:
            raise IOError("Read timeout while reading state")
        return LOGGER_STATE[ord(state[0])]


    def close(self):
        self._request_quit()
        self._serial.close()

    def start(self):
        """
        Start reading the log and return the available fields
        """
        self._read_log_header()
        return self._log_entry_names

    def get_start_time(self):
        return self._header.start_time

    def get_log_interval_ms(self):
        return self._header.log_interval_ms

    def __iter__(self):
        self._check_is_initialized()
        # request a few log entries to fill buffers
        for i in range(self._prefetch):
            self._request_log_entry()
        return self

    def next(self):
        """
        Return the next log entry
        """
        self._check_is_initialized()
        self._request_log_entry()
        entry = self._read_log_entry()
        # if the timestamp is
        if entry[0] == 0xFFFFFFFF:
            raise StopIteration()
        return entry

    def _check_is_initialized(self):
        if self._log_entry_size is None or self._log_entry_types is None or self._log_entry_names is None:
            raise RuntimeError("No log header information! You need to call start() first?")

    def _read_log_header(self):
        """
        Reads the log header

        Sends command 'h' and reads the result.
        """
        self._serial.flushOutput()
        self._serial.write('h')
        header = self._serial.read(size=16)
        if len(header) < 16:
            raise IOError("Read timeout while reading header")

        self._header = PebbleBackpack.LogHeader._make(struct.unpack('<QII', header))
        LOGGER.info('Got header %s', self._header)

        self._log_entry_size = _size_from_enabled_channels_mask(self._header.enabled_channels_mask)
        self._log_entry_types = _types_from_enabled_channels_mask(self._header.enabled_channels_mask)
        self._log_entry_names = _names_from_enabled_channels_mask(self._header.enabled_channels_mask)
        self._log_entry_conversion = _conversion_from_enabled_channels_mask(self._header.enabled_channels_mask)
        LOGGER.info('  log_entry_size: %i', self._log_entry_size)
        LOGGER.info('  log_entry_types: %s', self._log_entry_types)
        return self._header

    def _read_log_entry(self):
        raw_entry = self._serial.read(size=self._log_entry_size)
        # if we don't receive enough data we either finished reading or we have
        # a connection failure
        if len(raw_entry) < self._log_entry_size:
            if self.get_state() == "ReadFinished":
                raw_entry = '\xFF'*self._log_entry_size
            else:
                raise IOError("Read timeout while reading log entry")
        # unpack the raw bytes into a tuple
        values = list(struct.unpack_from(self._log_entry_types, raw_entry))
        for i in range(len(values)):
            if self._log_entry_conversion[i] is not None:
                values[i] = self._log_entry_conversion[i] * values[i]
        return values
