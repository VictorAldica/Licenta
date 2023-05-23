import serial
import time
from serial.serialutil import SerialException

class SerialCommunication:
    def __init__(self, port, baud_rate=9600, timeout=1):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_port = None
        self.connect()

    def connect(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            print(f"Connected to Arduino on port {self.port}")
        except SerialException:
            print(f"Arduino not found on port {self.port}")
            self.serial_port = None

    def send(self, message):
        if self.serial_port is None:
            print(f"No connection to Arduino on port {self.port}. Attempting to reconnect...")
            self.connect()
            if self.serial_port is None:
                print(f"Failed to reconnect to Arduino on port {self.port}")
                return False

        try:
            self.serial_port.write(str(message).encode())
            time.sleep(0.05)  
            return True
        except SerialException:
            print(f"Lost connection to Arduino on port {self.port}")
            self.serial_port = None
            return False

arduino_communicator = SerialCommunication('/dev/ttyACM0')  

def send_to_arduino(distance):
    success = arduino_communicator.send(distance)
    if not success:
        print("Failed to send data to Arduino")
