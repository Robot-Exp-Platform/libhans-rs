import libhans.libhans

def wait_for_moving(robot: libhans.HansRobot):
    while robot.is_moving():
        pass
    print("robot has reached")

# libhans v0.1.3
def motion_before_confirm(closure: callable[[list[float]], None], motion: list[float]):
    print("you want to ", closure.__name__, "to ", motion)
    input("Press Enter to continue...") 
    closure(motion)
    
# libhans v0.1.6
def motion_before_confirm(closure: callable[[list[float], float], None], motion: list[float], speed: float):
    print("you want to ", closure.__name__, "to ", motion)
    input("Press Enter to continue...") 
    closure(motion, speed)
    


robot = libhans.HansRobot("192.168.10.10")

joint_0 = [-269.377,-15.481,-150.696,-162.461,219.601,-251.614]
joint_1 = [-269.376,-76.614,-124.514,-162.461,219.601,-251.614]
joint_2 = [-248.436,-72.951,-122.814,-166.151,206.827,-271.070]
joint_3 = [-219.181,-80.247,-117.409,-163.388,236.073,-269.941]

# libhans v0.1.3 的所有运动函数中都不携带速度参数

# libhans v0.1.6 的所有运动函数中都会携带速度参数
# robot.move_joint(joint_1, 0.1)
# robot.move_joint(joint_2, 0.1)
# robot.move_joint(joint_3, 0.05)
# robot.move_joint(joint_2, 0.05)
# robot.move_joint(joint_1, 0.1)
# robot.move_path_from_file("traj.json", 0.1)

# libhans v0.1.3
motion_before_confirm(robot.move_joint, joint_0)
motion_before_confirm(robot.move_joint_rel, [10, 0, 0, 0, 0, 0])

# libhans v0.1.6
motion_before_confirm(robot.move_joint, joint_0, 0.1)
motion_before_confirm(robot.move_joint_rel, [10, 0, 0, 0, 0, 0], 0.1)