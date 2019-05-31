import datetime
import cv2
import serial

def send_x(value, serial_com):
    """
    @brief sends data using serial communication to the arduino with X prefix.
    @param value An integer containing the new x value.
    @param serial_com The serial communication you wan to send to.
    """
    data =  "X " + str(int(value)) + "\n"
    send_serial(data, serial_com)
    

def send_y(value, serial):
    """
    @brief sends data using serial communication to the arduino with Y prefix.
    @param value An integer containing the new y value.
    @param serial_com The serial communication you wan to send to.
    """
    data = "Y " + str(int(value)/2) + "\n"
    send_serial(data, arduino)

def send_serial(data, serial_com):
    """
    @brief Ecodes the data using ascii and sends it to the serial_com.
    @param data The data you want to send to the serial_com.
    @param serial_com The serial device you want to send the data to.
    """

    data = data.encode("ascii")
    arduino.write(data)


class PID:
    """
    @brief PID class used to simulate software PID.
    """
    
    def __init__(self, delta_time):
        """
        @brief The constructor of the PID class.
        @param delta_time The time between two frames.
        """
        self.kp = [9,9]
        self.ki = [0.25,0.25]
        self.kd = [4,4]
        self.set_point = [260,150]
        self.total_errors = [0,0]
        self.prev_error = [0,0]
        self.delta_time = delta_time

    def config_set_point(self, new_set_point):
        """
        @brief sets a new set_point and resets the total_error.
        @param new_set_point the new set_point.
        """
        self.set_point = new_set_point
        self.total_errors = [0,0]

    def calculate_single_PID(self, current_coordinate, axis):
        """
        @brief Calculates the new PID value of one axis.
        @param current_coordinate The current coordinate of the ball.
        @param axis 0 is x 1 is y asix.
        @return The new PID output of one axis.
        """

        current_error = self.set_point[axis] - current_coordinate[axis]

        self.total_errors[axis] += current_error
        d_action = (current_error - self.prev_error[axis])/self.delta_time

        i_action = self.total_errors[axis] * self.delta_time
        total_action = (current_error * self.kp[axis]) + (d_action * self.kd[axis]) + (i_action * self.ki[axis])

        self.prev_error[axis] = current_error

        return (total_action /75)


    def caluclate_PID(self, current_coordinate):
        """
        @brief Calculates the new PID values of both axis.
        @param current_coordinate The current coordinate of the ball.
        @return The new PID output of both axis.
        """
        return [self.calculate_single_PID(current_coordinate, 0), self.calculate_single_PID(current_coordinate, 1)] #calc x,y

arduino = serial.Serial("COM2",115200)

ball_lower = (0, 46, 128)
ball_upper = (30, 121, 255)

camera = cv2.VideoCapture(1)
camera.set(cv2.CAP_PROP_FPS, 120)
fps = camera.get(cv2.CAP_PROP_FPS)

delta_time = 1/fps

start_x = 160
start_y = 120
end_x = 480
end_y = 420

resolution = [end_x-start_x, end_y-start_y]
set_point = [160,150]
servo_pid = PID(delta_time, resolution)

print ("Frames per second : {0}".format(fps) )

def set_point_from_mouse_position(event, x, y, flags, params):
    """
    @brief Sets set_point to mouse position.
    @param event The openCV event that occurred.
    @param x The x coordinate of the mouse.
    @param y The y coordinate of the mouse.
    @param flags
    @param params
    """
    
    global set_point
    if event == cv2.EVENT_LBUTTONDOWN:
            set_point = (x, y)


while True:
    (grabbed, frame) = camera.read()

    cv2.setMouseCallback("Frame", set_point_from_mouse_position)
    servo_pid.config_set_point(set_point)
    start = datetime.datetime.now()
    frame = frame[start_y:end_y,start_x:end_x]

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, ball_lower, ball_upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(cnts) > 0:
        c = max(cnts, key=cv2send_x.contourArea)
        ((current_x, current_y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10 and radius < 20:
            cv2.circle(frame, (int(current_x), int(current_y)), int(radius), (0, 255, 255), 2)
            current_coordinate = [current_x, current_y]
            pid_value = servo_pid.caluclate_PID(current_coordinate)
            send_x(pid_value[0], arduino)
            send_y(pid_value[1], arduino)

        cv2.circle(frame, (set_point[0], set_point[1]), int(5), (0, 255, 127), 2)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)
    end = datetime.datetime.now()

    while((end - start).seconds >= delta_time):
        end = datetime.datetime.now()

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
    elif key == ord("o"):
        servo_pid.kp -= 0.1
    elif key == ord("p"):
        servo_pid.kp += 0.1

    elif key == ord("k"):
        servo_pid.kd -= 0.1
    elif key == ord("l"):
        servo_pid.kd += 0.1

    elif key == ord("n"):
        servo_pid.ki -= 0.1
    elif key == ord("m"):
        servo_pid.ki += 0.1

camera.release()
cv2.destroyAllWindows()