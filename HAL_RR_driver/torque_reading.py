from math import sin, pi
import time
from machinekit import hal
import numpy as np


def main():
    # first we need to unlink the ros_control interface pins
    pin1 = hal.Pin('lcec.0.0.torque-actual-value')
    pin2 = hal.Pin('lcec.0.1.torque-actual-value')
    pin3 = hal.Pin('lcec.0.2.torque-actual-value')
    pin4 = hal.Pin('lcec.0.3.torque-actual-value')
    pin5 = hal.Pin('lcec.0.4.torque-actual-value')
    pin6 = hal.Pin('lcec.0.5.torque-actual-value')

    print(pin1.get())
    # print(pin2.get())
    # print(pin3.get())
    # print(pin4.get())
    # print(pin5.get())
    # print(pin6.get())

    # now=time.time()
    # while time.time()-now<10:
    # 	print(np.int(pin.get()))

if __name__ == '__main__':
    main()


