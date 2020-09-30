from xarm.wrapper import XArmAPI
from pydexarm import Dexarm

class ArmWrapper():
    """ Base Class for Arm
    """

    def __init__(self):
        self.init_offset = [0, 0 ,0]
        pass

    def setup(self):
        pass

    def go_home(self):
        pass 

    def set_pos(self, x, y, z):
        pass

    def get_pos(self):
        return None

    def set_gripper(self, open_pos):
        """
        Args:
            open_pos (float): from 0 to 1.0
        """
        pass

    def reset(self):
        pass

    def close(self):
        pass


class DexArmWrapper(ArmWrapper):
    """ Base Class for Arm
    """

    def __init__(self,
                 logging=False):
        """
        Args:
            logging (bool): log or not.
        """
        self.init_offset = [0, 300 ,0]
        self._arm = Dexarm("/dev/ttyACM0") # COM* in Windows, /dev/tty* in Ubuntu
        self._logging = logging
        self.setup()

    def setup(self):
        self._arm.go_home()

    def go_home(self):
        self._arm.go_home()

    def set_pos(self, x=None, y=None, z=None, wait=False):
        if x is not None: x = - x
        if y is not None: y = y + 300
        self._arm.move_to(x,y,z, wait=False, feedrate=2000)

    def get_pos(self):
        pos = list(self._arm.get_current_position())
        if pos[0] is not None:
            pos[0] = - pos[0]
            pos[1] = pos[1] - 300
        return pos[:3]

    def set_gripper(self, open_pos):
        if 0.3 < open_pos <= 0.5:
            self._arm.soft_gripper_pick()
        elif 0.7 <= open_pos < 0.9:
            self._arm.soft_gripper_place()
        else:
            self._arm.soft_gripper_nature()

    def reset(self, wait=True):
        self._arm.go_home()

    def close(self):
        self._arm.close()


class xArm5Wrapper(ArmWrapper):
    """ Base Class for Arm
    """

    def __init__(self,
                 logging=False):
        """
        Args:
            logging (bool): log or not.
        """
        self._arm = XArmAPI('192.168.1.110', do_not_open=True, is_radian=False)
        self._logging = logging

    def setup(self):
        time.sleep(0.5)
        print('=' * 50)
        print("conneting xARM, please double check the settings, and make sure no human or obstacles in working area")
        os.system('pause')  # press any key to continue
        arm = XArmAPI('192.168.1.110', do_not_open=True, is_radian=False)
        #time.sleep(0.5)
        #if arm.warn_code != 0:
        #    arm.clean_warn()
        #if arm.error_code != 0:
        #    arm.clean_error()
        arm.motion_enable(enable=True)
        arm.set_mode(0) # 0 for position control mode. do not use other mode like 1 for servo motion mode, 2 for joint teaching mode
        arm.set_state(state=0) # 0: enable motion, 3: pause state, 4: stop state
        arm.reset(wait=True)
        print('version:', arm.get_version())
        print('state:', arm.get_state())  # 1 in motion, 2 ready but no motion(hibernate), 3 pause(could have instruction cache), 4 stop 
        print('err_warn_code:', arm.get_err_warn_code())
        print('position(°):', arm.get_position(is_radian=False))
        print('angles(°):', arm.get_servo_angle(is_radian=False))
        print('angles(°)(servo_id=1):', arm.get_servo_angle(servo_id=1, is_radian=False))

        # TODO set_tcp_offset 0 0 20 0 0 0
        code = arm.set_gripper_mode(0) # location mode, xarm gripper only support location mode for now
        print('set gripper mode: location mode, code={}'.format(code))
        code = arm.set_gripper_enable(True)
        print('set gripper enable, code={}'.format(code))
        code = arm.set_gripper_speed(500)  # unit:r/min
        print('set gripper speed, code={}'.format(code))
        code = arm.set_gripper_position(800, wait=True) # -10 to 850
        print('[wait]set gripper pos, code={}'.format(code))
        code, pos = arm.get_gripper_position() # tuple((code, pos)), only when code is 0, the returned result is correct
        print('get gripper pos, pos={}'.format(pos))
        return arm

    def go_home(self):
        pass

    def set_pos(self, x, y, z):
        if self._arm.connected and self._arm.get_cmdnum() <= 2:
            print('cmdnum:', self._arm.get_cmdnum())
            self._arm.set_position(x=x, y=y, z=z, roll=180, pitch=0, yaw=0, radius=50, speed=100, mvacc=200, wait=False)  # speed in mm/s,rad/

    def get_pos(self):
        _, pos = self._arm.get_position(is_radian=False)
        return pos[:3]

    def set_gripper(self, open_pos):
        if self._arm.connected and self._arm.get_cmdnum() <= 2:
            open_pos = open_pos * 850.0  # -10 to 850
            code = self._arm.set_gripper_position(min(open_pos, 850), wait=False)
            print('set gripper pos, code={}'.format(code))

    def reset(self, wait=True):
        print('=' * 50)
        print("reseting xARM, please wait..")
        self._arm.reset(wait)

    def close(self):
        self._arm.reset()
        self._arm.disconnect()



"""
标定时可以试试直接控制servo角度然后get position以获取正向运动学坐标:
speed = 20
arm.set_servo_angle(angle=[0, -30, -60, 0, 0], speed=speed, wait=True)
或者
speed = math.radians(20)
arm.set_servo_angle(angle=[0, math.radians(-30), math.radians(-60), 0, 0], speed=speed, is_radian=True, wait=True)
然后
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.get_position(is_radian=False)
_, angles = arm.get_servo_angle()

XARM5在关节空间时，机械臂有5个自由度可以控制，在需要末端使用其他姿态的情况下，可以切换至关节指令。而后再次使用关节指令回到法兰与底座水平的姿态即可以切换回笛卡尔控制。有一个快速的方法是：仅需要将J4的角度设为-(J2角度+J3角度)即可。

# 其他常用API
    arm.connect(...)
    arm.reset(...)
    arm.move_gohome(...)

    arm.set_position(...)
    arm.set_servo_angle(...)
    arm.set_servo_angle_j(...)
    arm.set_servo_cartesian(...)
    arm.set_position_aa(...)
    arm.set_servo_cartesian_aa(...)

    arm.set_tcp_offset(...)
    arm.set_servo_attach(...)
    arm.set_servo_detach(...)
    arm.set_state(...)
    arm.set_mode(...)
    arm.motion_enable(...)
    arm.set_pause_time(...)

    arm.set_tcp_jerk(...)
    arm.set_tcp_maxacc(...)
    arm.set_joint_jerk(...)
    arm.set_joint_maxacc(...)
    arm.set_tcp_load(...)
    arm.set_collision_sensitivity(...)
    arm.set_teach_sensitivity(...)
    arm.set_gravity_direction(...)
    arm.config_tgpio_reset_when_stop(...)
    arm.config_cgpio_reset_when_stop(...)
    arm.clean_conf()
    arm.save_conf()

    arm.set_gripper_enable(...)
    arm.set_gripper_mode(...)
    arm.set_gripper_speed(...)
    arm.set_gripper_position(...)
    arm.get_gripper_position()
    arm.get_gripper_err_code()
    arm.clean_gripper_error()
"""