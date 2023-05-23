import serial

def send_to_arduino(distance):
    ser = serial.Serial('/dev/ttyACM0', 9600) 
    ser.write(str(distance).encode())
    ser.close()
