############################  Serial ########################## 
######### final version 1 ##############
import serial
import time
import threading
import PID
import numpy as np
from time import sleep
import cv2 as cv


class LEGOPrinter:
    ser = []
    A_encoder_num = 0
    B_encoder_num = 0
    C_encoder_num = 0
    D_encoder_num = 0
    threadLock = threading.Lock()
    error_count = 0
    d_count = 0

    def __init__(self):
        # serial port parameter initialtzation
        self.ser = serial.Serial(
            port='/dev/ttyS0',
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.05
        )
        self.PID_A = PID.PID(1.4, 1.0, 0)
        self.PID_B = PID.PID(2, 0.0, 0.0)
        self.PID_C = PID.PID(0.6, 1.0, 0.01)
        self.PID_D = PID.PID(3, 1, 0.1)
        self.A_last_out = 0
        self.B_last_out = 0
        self.C_last_out = 0
        self.D_last_out = 0
        self.A_target_angle = 0
        self.B_target_angle = 0
        self.C_target_angle = 0
        self.D_target_angle = 0
        self.A_current_angle = 0
        self.B_current_angle = 0
        self.C_current_angle = 0
        self.D_current_angle = 0
        self.next = False
        self.A_ready = False
        self.B_ready = False
        self.C_ready = False
        self.D_ready = False
        self.init_done = False

    # motion part
    # [important]Note here A_motor not use PID,if need change this
    def go_angle(self, a_angle, b_angle, c_angle, d_angle):
        if self.read_data():
            if abs(self.A_current_angle + a_angle) > 2.0:
                # A_out = self.PID_A.PID_Driver(a_angle, -self.A_current_angle)
                A_out = 8
            else:
                A_out = 0
                self.A_ready = True
            if self.A_ready:
                A_out = 0
            if abs(self.B_current_angle + b_angle) > 5.0:
                B_out = self.PID_B.PID_Driver(b_angle, -self.B_current_angle)
            else:
                B_out = 0
                self.B_ready = True
            if self.B_ready:
                B_out = 0
            if abs(self.C_current_angle + c_angle) > 1.0:
                C_out = self.PID_C.PID_Driver(c_angle, -self.C_current_angle)
            else:
                C_out = 0
                self.C_ready = True
            if self.C_ready:
                C_out = 0
            if abs(self.D_current_angle + d_angle) > 5.0:
                D_out = self.PID_D.PID_Driver(d_angle, -self.D_current_angle)
            else:
                D_out = 0
                self.D_ready = True
            if self.D_ready:
                D_out = 0
            if self.A_ready and self.B_ready and self.C_ready and self.D_ready:
                self.next = True
                self.A_ready = False
                self.B_ready = False
                self.C_ready = False
                self.D_ready = False
        else:
            A_out = self.A_last_out
            B_out = self.B_last_out
            C_out = self.C_last_out
            D_out = self.D_last_out
        self.send_out(A_out, B_out, C_out, D_out)

    # init the position
    def run_init(self):
        send = "A0B-15C-20D0T3000EN"
        self.ser.write(send.encode())
        self.ser.readline()
        print("init...")
        time.sleep(3)
        send = "A0B0C0D0T3000EN"
        self.ser.write(send.encode())
        self.ser.readline()
        time.sleep(0.02)
        self.A_target_angle = 0
        self.B_target_angle = 0
        self.C_target_angle = 0
        self.D_target_angle = 0
        self.A_current_angle = 0
        self.B_current_angle = 0
        self.C_current_angle = 0
        self.D_current_angle = 0
        self.next = False
        self.A_ready = False
        self.B_ready = False
        self.C_ready = False
        self.D_ready = False
        self.init_done = True

    # read four motor angle from sensor
    def read_data(self):
        self.threadLock.acquire()
        # readData = self.ser.readline()
        # sleep(0.05)
        readData = self.ser.readline()
        self.threadLock.release()
        if len(readData) > 5:
            readData = str(readData.decode())
            # print(readData)
            if readData[0] == 'A':
                temp = readData.split('A')
                readData = temp[1]
                temp = readData.split('B')
                # if self.D_last_out:
                # self.D_current_angle = self.D_current_angle-abs(int(temp[0]))*abs(self.D_last_out)/self.D_last_out
                self.D_current_angle = self.D_current_angle - (int(temp[0]))
                readData = temp[1]
                temp = readData.split('C')
                # if self.C_last_out:
                #     self.C_current_angle = self.C_current_angle-abs(int(temp[0]))*abs(self.C_last_out)/self.C_last_out
                self.C_current_angle = self.C_current_angle - (int(temp[0]))
                readData = temp[1]
                temp = readData.split('D')
                # if self.B_last_out:
                #    self.B_current_angle = self.B_current_angle-abs(int(temp[0]))*abs(self.B_last_out)/self.B_last_out
                self.B_current_angle = self.B_current_angle - (int(temp[0]))
                readData = temp[1]
                temp = readData.split('E')
                # if self.A_last_out:
                #    self.A_current_angle = self.A_current_angle-abs(int(temp[0]))*abs(self.A_last_out)/self.A_last_out
                self.A_current_angle = self.A_current_angle - (int(temp[0]))
            return 1
        return 0

    # send message
    def send_out(self, A_out, B_out, C_out, D_out):
        send = "A" + str(A_out) + "B" + str(B_out) + "C" + str(C_out) + "D" + str(D_out) + "T40EN"
        # print(send)
        self.A_last_out = A_out
        self.B_last_out = B_out
        self.C_last_out = C_out
        self.D_last_out = D_out
        self.ser.write(send.encode())

    # the lift motor action (lift motor is motorB)
    def lift_motion(self, value):
        self.B_target_angle = value
        b_angle = self.B_target_angle
        if self.read_data():
            if abs(self.B_current_angle + b_angle) > 5.0:
                B_out = self.PID_B.PID_Driver(b_angle, -self.B_current_angle)
            else:
                B_out = 0
                self.B_ready = True
            if self.B_ready:
                B_out = 0
            if self.B_ready:
                self.next = True
                self.A_ready = False
                self.B_ready = False
                self.C_ready = False
                self.D_ready = False
            A_out = 0
            C_out = 0
            D_out = 0
        else:
            A_out = 0
            B_out = self.B_last_out
            C_out = 0
            D_out = 0
        self.send_out(A_out, B_out, C_out, D_out)

    # the value of lift up
    def lift_up(self):
        self.lift_motion(100)

    # the value of lift down
    def lift_down(self):
        self.lift_motion(300)

    # change the point position to motor angle
    def x_position(self, x_point):
        self.C_target_angle = x_point

    # change the point position to motor angle
    def y_position(self, y_point):
        self.A_target_angle = y_point / 1.3

    # end action
    def end(self):
        send = "A20B-20C-20D0T3000EN"
        self.ser.write(send.encode())
        self.ser.readline()
        time.sleep(3)
        print("Draw Done!")

    # main print process
    def point_transfer(self, points):
        x_range_min = 20
        # x_range_max = 660
        y_range_min = 10
        if not self.init_done:
            self.run_init()
        point_x_max = 0
        point_y_max = 0
        point_x_min = 1000
        point_y_min = 1000
        # find range
        for point in points:
            if point[0] > point_x_max:
                point_x_max = point[0]
            if point[0] < point_x_min:
                point_x_min = point[0]
            if point[1] > point_y_max:
                point_y_max = point[1]
            if point[1] < point_y_min:
                point_y_min = point[1]

        # zoom all point and move to (0,0)
        zoom_x = 500 / (point_x_max - point_x_min)
        # zoom_y = 500 / (point_y_max - point_y_min)
        # if zoom_x < zoom_y:
        #    points = np.array(points) / zoom_y - point_y_min
        # else:
        points = (np.array(points)) * zoom_x
        img = np.zeros((700, 700, 3), np.uint8)
        for i in range(len(points)):
            points[i][0] = points[i][0] - point_x_min * zoom_x + x_range_min
            points[i][1] = points[i][1] - point_y_min * zoom_x + y_range_min
            cv.circle(img, (int(points[i][0]), int(points[i][1])), 1, (0, i, 255 - i), 4)
        cv.namedWindow("image")
        cv.imshow('image', img)
        cv.waitKey(1000)
        print(points, point_x_min, zoom_x)
        count_point = 0
        print("<<<<<<<<start print>>>>>>>")

        # print loop
        for point in points:
            print(point)
            complete_count = count_point / 106.0 * 100.0
            print("Compelete:" + str(complete_count) + "%")
            # x,y move action
            while not self.next:
                self.x_position(point[0])
                self.y_position(point[1])
                self.go_angle(self.A_target_angle, self.B_target_angle, self.C_target_angle, self.D_target_angle)
                sleep(0.02)
            self.next = False

            # lift action
            while not self.next:
                self.lift_down()
                # self.go_angle(self.A_target_angle, self.B_target_angle, self.C_target_angle, self.D_target_angle)
                sleep(0.02)
            self.next = False
            while not self.next:
                self.lift_up()
                # self.go_angle(self.A_target_angle, self.B_target_angle, self.C_target_angle, self.D_target_angle)
                sleep(0.02)
            self.next = False
            count_point = count_point + 1
        self.end()


# change txt to numpy
def open_points(name):
    all_points = []
    with open(name, 'r') as file_to_read:
        while True:
            lines = file_to_read.readline()
            if not lines:
                break
                pass
            x_tmp, y_tmp = [float(i) for i in lines.split()]
            all_points.append([x_tmp, y_tmp])
    data = np.array(all_points)
    point = data[np.lexsort(data.T)]
    return point


if __name__ == "__main__":
    LP = LEGOPrinter()
    c_time = time.time()
    filename = 'myfile.txt'
    raw_point = open_points(filename)
    print(raw_point)
    # raw_point = [[1,10],[100,50],[200,100],[300,200],[1,10],[100,50],[200,100],[300,200]]
    LP.point_transfer(raw_point)
    c_time = time.time() - c_time
    print("take time:" + str(c_time))
