import serial

arduino = serial.Serial('COM2', 115200)

while True:
    data = input("set degrees: ")
    data = data + "\n"
    data = data.encode('ascii')
    print(data)
    arduino.write(data)