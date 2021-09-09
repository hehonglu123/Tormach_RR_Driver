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

    start_pos = pos_fb.get()
    last_pos = start_pos
    period = 0.01
    start_time = time.time()

    while True:
        current_time = time.time()
        since_start = current_time - start_time
        new_pos = start_pos + sin(since_start * 2 * pi) * 0.05
        vel = (new_pos - last_pos) / period
        last_pos = new_pos
        # in the loop, we set the new cmd values
        vel_cmd.set(vel)
        pos_cmd.set(new_pos)
        # and read back the feedback values
        print(f"Pos {pos_fb.get()} Vel {vel_fb.get()}")
        time.sleep(max(period - (time.time() - current_time), 0.001))


if __name__ == '__main__':
    main()