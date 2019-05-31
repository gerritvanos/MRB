import serial

def input_write_serial(serial_device):
    """
    @brief Sends inputted data to the serial communication of the serial_device.
    @param serial_device The device you want to send the data to.
    """
    data = input("set degrees: ")
    data = data + "\n"
    data = data.encode('ascii')
    print(data)
    arduino.write(data)

if __name__ == "__main__":
    arduino = serial.Serial('COM2', 115200)
    while True:
        input_write_serial(arduino)