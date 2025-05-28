##############################################
# The MIT License (MIT)
# Copyright (c) 2017 Kevin Walchko
# see LICENSE for full details
##############################################
# This is basically the interface between the Create2 and pyserial

import serial # type:ignore
import struct


class SerialCommandInterface(object):
    """
    This class handles sending commands to the Create2. Writes will take in tuples
    and format the data to transfer to the Create.
    """

    def __init__(self):
        """
        Constructor.

        Creates the serial port, but doesn't open it yet. Call open(port) to open
        it.
        """
        self.ser = serial.Serial()

    def __del__(self):
        """
        Destructor.

        Closes the serial port
        """
        self.close()

    def open(self, port, baud=115200, timeout=1):
        """
        Opens a serial port to the create.

        port: the serial port to open, ie, '/dev/ttyUSB0'
        buad: default is 115200, but can be changed to a lower rate via the create api
        """
        try:
            self.ser.port = port
            self.ser.baudrate = baud
            self.ser.timeout = timeout
            if self.ser.is_open:
                self.ser.close()
            self.ser.open()
            if self.ser.is_open:
                print('-'*40)
                print(' Create opened serial connection')
                print('   port: {}'.format(self.ser.port))
                print('   datarate: {} bps'.format(self.ser.baudrate))
                print('-'*40)
            else:
                raise Exception('Failed to open {} at {}'.format(port, baud))
        except serial.SerialException as e:
            print(f"SerialException: {e}")
            raise
        except Exception as e:
            print(f"Exception opening serial port: {e}")
            raise

    def write(self, opcode, data=None):
        """
        Writes a command to the create. There needs to be an opcode and optionally
        data. Not all commands have data associated with it.

        opcode: see creaet api
        data: a tuple with data associated with a given opcode (see api)
        """
        msg = (opcode,)

        if data:
            msg += data
        try:
            self.ser.write(struct.pack('B' * len(msg), *msg))
        except serial.SerialTimeoutException as e:
            print(f"SerialTimeoutException during write: {e}")
            raise
        except serial.SerialException as e:
            print(f"SerialException during write: {e}")
            raise
        except Exception as e:
            print(f"Exception during write: {e}")
            raise

    def read(self, num_bytes):
        """
        Read a string of 'num_bytes' bytes from the robot.

        Arguments:
            num_bytes: The number of bytes we expect to read.
        """
        if not self.ser.is_open:
            raise Exception('You must open the serial port first')

        try:
            data = self.ser.read(num_bytes)
            return data
        except serial.SerialTimeoutException as e:
            print(f"SerialTimeoutException during read: {e}")
            raise
        except serial.SerialException as e:
            print(f"SerialException during read: {e}")
            raise
        except Exception as e:
            print(f"Exception during read: {e}")
            raise

    def close(self):
        """
        Closes the serial connection.
        """
        try:
            if self.ser.is_open:
                print('Closing port {} @ {}'.format(self.ser.port, self.ser.baudrate))
                self.ser.close()
        except serial.SerialException as e:
            print(f"SerialException during close: {e}")
        except Exception as e:
            print(f"Exception during close: {e}")
