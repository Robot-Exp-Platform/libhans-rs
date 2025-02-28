import libhans.libhans

def wait_for_moving(robot: libhans.HansRobot):
    while robot.is_moving():
        pass
    print("robot has reached")

robot_down = libhans.HansRobot()
robot_up = libhans.HansRobot()

print("now version is " + robot_up.version())

robot_down.connect("192.168.10.2",10003)
robot_up.connect("192.168.10.3",10003)

robot_down.enable()
robot_up.enable()

print("up robot joint is " + robot_up.read_joint())
print("down robot joint is " + robot_down.read_joint())

joint_down_0 = [-42.318, -94.086, 124.892, -32.432, -17.917, -177.154]
joint_down_1 = [-16.803, -93.325, 125.761, -34.221, -17.897, -177.030] # 拆卸定位位姿
joint_down_2 = [-3.901, -92.228, 124.779, -37.925, -6.162, -174.093] # 拆卸准备位姿
joint_down_3 = [-4.713, -86.105, 125.697, -36.897, -7.703, -182.152] # 拆卸操作位姿

joint_up_0 = [27.838, -87.525, -86.58, -5.426, -9.101, 77.22]
joint_up_1 = [14.344, -88.131, -81.403, -9.997, -9.101, 77.220] # 拆卸定位位姿
joint_up_2 = [3.094, -91.993, -78.533, -8.908, -1.675, 75.430] # 拆卸准备位姿
joint_up_3 = [3.09, -92.195, -71.152, -14.493, 2.166, 74.43] # 拆卸操作位姿


robot_down.set_speed(0.1)
robot_up.set_speed(0.1)


input("Press Enter to continue...") 
robot_down.move_joint_async(joint_down_3)
robot_up.move_joint_async(joint_up_3)
wait_for_moving(robot_up)
wait_for_moving(robot_down)

input("Press Enter to continue...") 
robot_down.move_joint_async(joint_down_2)
robot_up.move_joint_async(joint_up_2)
wait_for_moving(robot_up)
wait_for_moving(robot_down)

input("Press Enter to continue...") 
robot_down.move_joint_async(joint_down_1)
robot_up.move_joint_async(joint_up_1)
wait_for_moving(robot_up)
wait_for_moving(robot_down)

input("Press Enter to continue...") 
robot_down.move_joint_async(joint_down_0)
robot_up.move_joint_async(joint_up_0)
wait_for_moving(robot_up)
wait_for_moving(robot_down)

# print("up robot joint is " + robot_up.read_joint())
# print("down robot joint is " + robot_down.read_joint())