#!usr/bin/python3
"""
UART communication on Raspberry Pi using Pyhton
http://www.electronicwings.com
https://www.electronicwings.com/raspberry-pi/raspberry-pi-uart-communication-using-python-and-c
"""

import sys
from time import sleep

import serial


def main(args: list) -> None:
    """"""
    ser = serial.Serial ("/dev/ttyS0", 115200)    # Open port with baud rate
    while True:
        # received_data = ser.read()  # read serial port
        # sleep(0.03)
        # data_left = ser.inWaiting()  # check for remaining byte
        # received_data += ser.read(data_left)
        #print(received_data)  # print received data
        bytes_str = input("Hex bytes to send>")
        n_bytes = (len(bytes_str) - 2) // 2
        ser.write(int(bytes_str, 16).to_bytes(n_bytes, "big"))  # transmit data serially
        

if(__name__ == "__main__"):
    main(sys.argv)
