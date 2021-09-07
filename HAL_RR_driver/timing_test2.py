from math import sin, pi
import time
from machinekit import hal

def main():
    # first we need to unlink the ros_control interface pins
    pos_cmd_pin = hal.Pin('hal_hw_interface.joint_6.pos-cmd')
    if pos_cmd_pin.linked:
        pos_cmd_pin.unlink()
    vel_cmd_pin = hal.Pin('hal_hw_interface.joint_6.vel-cmd')
    if vel_cmd_pin.linked:
        vel_cmd_pin.unlink()
    # unlink this pin only if you provide velocity command values
    vel_calc_pin = hal.Pin('joint6_ros_vel_cmd.out')
    if vel_calc_pin.linked:
        vel_calc_pin.unlink()

    # next create references to the HAL signals
    pos_cmd = hal.Signal('joint6_ros_pos_cmd')
    # vel_ff = hal.Signal('joint6_vel_feedforward')
    vel_cmd = hal.Signal('joint6_ros_vel_cmd')
    pos_fb = hal.Signal('joint6_ros_pos_fb')
    vel_fb = hal.Signal('joint6_ros_vel_fb')

    vel=vel_fb.get()
    current_time=time.time()
    freq=[]
    while True:
        if vel*vel_fb.get()<0:
            if len(freq)>1000:
                freq.pop(0)
            freq.append(1/(time.time()-current_time))
            print(sum(freq)/len(freq),len(freq))
            current_time=time.time()
            vel=vel_fb.get()
if __name__ == '__main__':
    main()
