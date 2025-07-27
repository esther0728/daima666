import sensor
from pyb import UART
from my_uart import My_uart
from machine import Pin
import time
import math
# import display
from pid import PID
from color_detector import Color_detector

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_auto_exposure(False, exposure_us=25000)
clock = time.clock()
uart = UART(3,115200)
rx_pin = Pin("P5", mode=Pin.IN, pull=Pin.PULL_UP)
uart3 = My_uart(3,115200)
color_threshold = (59, 100, 11, 127, 17, -128)
pid_x = PID(p=0.1,i=0,d=0,imax=50)
pid_y =PID(p=0.1,i=0,d=0,imax=50)

laser_dector = Color_detector(80,60)

img = sensor.snapshot()
w = img.width()
h = img.height()
cx = w // 2
cy = h // 2
while True:
    clock.tick()
    img = sensor.snapshot()
    img.draw_line(0, cy, w, cy, color=(255,255,255))
    img.draw_line(cx, 0, cx, h, color=(255,255,255))

    x,y=laser_dector.find()
    img.draw_cross(x, y)

    output_x=pid_x.get_pid(x-cx,1)
    output_y=pid_y.get_pid(y-cy,1)

    angle_x = math.atan2(output_x*1.1,5)
    angle_y = math.atan2(output_y*1.1,5)
    buffer = [angle_x,angle_y]
    uart3.send_floats(buffer)



