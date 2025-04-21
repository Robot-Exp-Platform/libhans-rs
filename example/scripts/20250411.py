import libhans.libhans

def wait_for_moving(robot: libhans.HansRobot):
    while robot.is_moving():
        pass
    print("robot has reached")

robot = libhans.HansRobot("192.168.10.10")

joint_0 = [-269.377,-15.481,-150.696,-162.461,219.601,-251.614]
joint_1 = [-269.376,-76.614,-124.514,-162.461,219.601,-251.614]
joint_2 = [-248.436,-72.951,-122.814,-166.151,206.827,-271.070]
joint_3 = [-219.181,-80.247,-117.409,-163.388,236.073,-269.941]

robot.move_joint(joint_1, 0.1)
robot.move_joint(joint_2, 0.1)

robot.move_joint(joint_3, 0.05)
robot.move_joint(joint_2, 0.05)

robot.move_joint(joint_1, 0.1)

robot.move_path_from_file("traj.json", 0.1)