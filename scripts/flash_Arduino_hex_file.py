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
##----------------------------------------------------------------------------*/ import time
import sys
import serial
from serial.tools import list_ports
import subprocess
import os
 
class flashBinToArduino(object):
    def getPort(self):
        """Get list of description of all COM ports"""
        ports_avaiable = list(list_ports.comports())
        arudino_port = tuple()
        for port in ports_avaiable:
            print port
            if port[1].startswith("Arduino"):
                arduino_port = port
            elif port[1].startswith("Genuino"):
                arduino_port = port
            else:
                arduino_port = []
        return arduino_port
 
    def resetDevice(self, comport=None):
        """Set device to bootloader mode, baud=1200, 8N1"""
        #ser = serial.Serial(self.comport, baudrate=1200, parity=serial.PARITY_NONE, timeout=1)
        ser = serial.Serial(comport, 57600)
        ser.close()
        ser = serial.Serial(comport, 1200)
        ser.setRTS(True)  # RTS line needs to be held high and DTR low
        ser.setDTR(False)  # (see Arduino IDE source code)
        ser.close()
 
    def loadBinFile(self, comport=None):
        # change path according to your specific setup
        bossac = os.path.abspath(r'C:\Users\user\AppData\Local\Arduino15\packages\arduino\tools\bossac\1.6.1-arduino\bossac.exe')
        binary = os.path.abspath(r'C:\Users\user\Downloads\pebble.ino.SensirionArduinoZero')
        output = subprocess.check_call(['{}'.format(bossac), '-i', '-d', '--port={}'.format(comport), '--force_usb_port=true',
                               '-i', '-e', '-w', '-v','{}'.format(binary), '-R'])
        print output
if __name__ == "__main__":
    arduino = flashBinToArduino()
    port = arduino.getPort()
    if not port:
        print "no device found"
    else:
        port = port[0]
    arduino.resetDevice(port)
    # wait for device re-enumerate on USB
    time.sleep(5)
    port = arduino.getPort()
    if not port:
        print "no device found"
    else:
        port = port[0]
    arduino.loadBinFile(port)