import datetime
import cv2
import serial

def send_x(value,arduino):
    data =  "X " + str(int(value)) + "\n"
    print(data)
    data = data.encode("ascii")

    arduino.write(data)


def send_y(value,arduino):
    data = "Y " + str(int(value)/2) + "\n"
    print(data)
    data = data.encode("ascii")
    arduino.write(data)


class PID:
    def __init__(self,dt,res):
        self.kp = [9,9]
        self.ki = [0.25,0.25]
        self.kd = [4,4]
        self.set_point = [260,150]
        self.total_errors = [0,0]
        self.prev_error = [0,0]
        self.dt = dt
        self.res = res

    def config_set_point(self,new_set_point):
        self.set_point = new_set_point
        self.total_errors = [0,0]

    def calculate_single_PID(self,current_values, axis):
        current_error = self.set_point[axis] - current_values[axis]

        self.total_errors[axis] += current_error
        d_action = (current_error - self.prev_error[axis])/self.dt

        i_action = self.total_errors[axis] * self.dt
        total_action = (current_error * self.kp[axis]) + (d_action * self.kd[axis]) + (i_action * self.ki[axis])

        self.prev_error[axis] = current_error

        return (total_action /75)


    def caluclate_PID(self,current_values):
        return [self.calculate_single_PID(current_values, 0), self.calculate_single_PID(current_values, 1)] #calc x,y

arduino = serial.Serial("COM2",115200)

ball_lower = (0, 46, 128)
ball_upper = (30, 121, 255)

camera = cv2.VideoCapture(1)
camera.set(cv2.CAP_PROP_FPS, 120)
fps = camera.get(cv2.CAP_PROP_FPS)

dt = 1/fps

start_x = 160
start_y = 120
end_x = 480
end_y = 420

res = [end_x-start_x, end_y-start_y]
set_point = [160,150]
servo_pid = PID(dt, res)

print ("Frames per second : {0}".format(fps) )

def mouse_drawing(event, x, y, flags, params):
    global set_point
    if event == cv2.EVENT_LBUTTONDOWN:
            set_point = (x, y)


while True:
    (grabbed, frame) = camera.read()

    cv2.setMouseCallback("Frame", mouse_drawing)
    servo_pid.config_set_point(set_point)
    start = datetime.datetime.now()
    frame = frame[start_y:end_y,start_x:end_x]

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, ball_lower, ball_upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((current_x, current_y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10 and radius < 20:
            cv2.circle(frame, (int(current_x), int(current_y)), int(radius), (0, 255, 255), 2)
            current_values = [current_x, current_y]
            pid_value = servo_pid.caluclate_PID(current_values)
            send_x(pid_value[0], arduino)
            send_y(pid_value[1], arduino)

        cv2.circle(frame, (set_point[0], set_point[1]), int(5), (0, 255, 127), 2)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)
    end = datetime.datetime.now()

    while((end - start).seconds >= dt):
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