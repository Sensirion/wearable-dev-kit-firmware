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

import argparse
import datetime
import logging
import sys
import os

from wdk_logdata_driver import PebbleBackpack

LOGGER = logging.getLogger(__name__)


def _dump_logs_to_file(pebble, file_name):
    columns = pebble.start()
    start_time = pebble.get_start_time() / 1000.0
    log_interval = pebble.get_log_interval_ms() / 1000.0
    LOGGER.info('Found columns: %s', columns)

    with open(file_name, 'wb') as f:
        header = '\t'.join(['Epoch_UTC'] + columns)
        f.write(header + '\n')

        epoch_utc = start_time
        count = 0
        for entry in pebble:
            data = '\t'.join([str(x) for x in entry])
            f.write(str(epoch_utc) + '\t' + data + '\n')
            epoch_utc += log_interval
            count += 1

        LOGGER.info('Number of log entries dumped: %d', count)


def _prepare(file_name):
    parent_dir = os.path.dirname(file_name)
    if not os.path.exists(parent_dir):
        os.makedirs(parent_dir)


def main(args):
    d = datetime.datetime.now()
    default_file_name = os.path.join('data', 'PB_{0}.edf'.format(d.strftime('%Y-%m-%d_%H%M%S')))

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('device', help='The device to connect to', default='/dev/ttyACM3', nargs='?')
    parser.add_argument('-b', '--baud_rate', help='The baud-rate to use in communation', default=115200, type=int)
    parser.add_argument('-p', '--prefetch', help='The number of entries to prefetch', default=8, type=int)
    parser.add_argument('-o', '--out', default=default_file_name, help='Where to write the results.')
    config = parser.parse_args(args)

    dump_file = config.out
    _prepare(dump_file)

    pebble_backpack = PebbleBackpack(config.device, baud_rate=config.baud_rate, prefetch=config.prefetch)
    try:
        LOGGER.info('Will dump data-log to \'%s\'', dump_file)
        _dump_logs_to_file(pebble_backpack, dump_file)
    finally:
        pebble_backpack.close()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main(sys.argv[1:])
