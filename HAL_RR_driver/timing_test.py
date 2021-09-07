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

    period = 0.01
    start_time = time.time()
    i=0
    while True:
        i+=1
        current_time = time.time()
        if i % 2 ==0:
            
            vel_cmd.set(100)
            pos_cmd.set(0.00001)
        else:
            vel_cmd.set(-100)
            pos_cmd.set(-0.00001)

        # and read back the feedback values
        print(f"Pos {pos_fb.get()} Vel {vel_fb.get()}")
        time.sleep(0.001)        


if __name__ == '__main__':
    main()
