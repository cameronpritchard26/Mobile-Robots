"""cameronpritchard_Lab3_Task2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from fairis_tools.cameronpritchard_MyRobot import MyRobot

# create the Robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = '../../worlds/Fall24/maze6.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()

following_right_wall = False # Sets the wall to follow, True for right, False for left
begin_wall_following = False # Switch variable for when the robot should begin wall following

while robot.experiment_supervisor.step(robot.timestep) != -1:
    # Getting objects from the environment and distance from front wall
    recognized_objects = robot.rgb_camera.getRecognitionObjects()
    front_wall_distance = min(robot.get_lidar_range_image()[350:450])
    print(f"Distance to front wall: {front_wall_distance}")
    # No objects found, perform wall following
    if len(recognized_objects) == 0:
        print("Goal object not found, perform wall following")
        # If ready to begin wall following, perform wall following
        if begin_wall_following:
            # Right wall following logic
            if following_right_wall:
                print("Following right wall")
                # Getting distance from right wall and error
                distance_from_right_wall = min(robot.get_lidar_range_image()[525:600])
                print(f"Distance from right wall: {distance_from_right_wall}")
                error = 0.4 - distance_from_right_wall
                print(f"Error: {error}")
                # Calculating new wheel velocities
                p = robot.PID_saturation(error, robot.wall_follow_speed)
                if error > 0:
                    print("Too close to wall, turn away from wall")
                    robot.set_left_motors_velocity(robot.wall_follow_speed)
                    robot.set_right_motors_velocity(robot.wall_follow_speed + abs(p))
                else:
                    print("Too far wall, turn towards wall")
                    robot.set_left_motors_velocity(robot.wall_follow_speed + abs(p))
                    robot.set_right_motors_velocity(robot.wall_follow_speed)
                # If too close to front wall, turn away
                if min(robot.get_lidar_range_image()[390:410]) < 0.5:
                    print("Too close to front wall, turn away")
                    robot.set_left_motors_velocity(-1)
                    robot.set_right_motors_velocity(1)
            # Left wall following logic
            else:
                print("Following left wall")
                # Getting distance from left wall and error
                distance_from_right_wall = min(robot.get_lidar_range_image()[200:275])
                print(f"Distance from right wall: {distance_from_right_wall}")
                error = 0.4 - distance_from_right_wall
                print(f"Error: {error}")
                # Calculating new wheel velocities
                p = robot.PID_saturation(error, robot.wall_follow_speed)
                if error > 0:
                    print("Too close to wall, turn away from wall")
                    robot.set_left_motors_velocity(robot.wall_follow_speed + abs(p))
                    robot.set_right_motors_velocity(robot.wall_follow_speed)
                else:
                    print("Too far from wall, turn towards wall")
                    robot.set_left_motors_velocity(robot.wall_follow_speed)
                    robot.set_right_motors_velocity(robot.wall_follow_speed + abs(p))
                # If too close to front wall, turn away
                if min(robot.get_lidar_range_image()[390:410]) < 0.5:
                    print("Too close to front wall, turn away")
                    robot.set_left_motors_velocity(1)
                    robot.set_right_motors_velocity(-1)
        # If not ready to begin wall following, orient robot so it is ready
        else:
            # Turn robot to face wall
            current_heading = robot.get_compass_reading()
            if current_heading != 90 and front_wall_distance > 0.51:
                print("Rotating to align with wall")
                if current_heading > 90:
                    print("Turning right to face wall")
                    robot.set_left_motors_velocity(1)
                    robot.set_right_motors_velocity(-1)
                else:
                    print("Turning left to face wall")
                    robot.set_left_motors_velocity(-1)
                    robot.set_right_motors_velocity(1)
            # Moving robot to wall
            elif front_wall_distance > 0.51:
                print("Moving to wall")
                error = front_wall_distance - 0.3
                wheel_velocity = robot.PID_saturation(error, 10)
                robot.set_left_motors_velocity(wheel_velocity)
                robot.set_right_motors_velocity(wheel_velocity)
            # Ready to begin wall following
            else:
                print("Close enough to wall, begin wall following")
                begin_wall_following = True
    # Performing motion to goal
    else:
        if following_right_wall:
            front_right_wall_distance = min(robot.get_lidar_range_image()[525:600])
            print(f"Distance to front right wall: {front_right_wall_distance}")
        else:
            front_left_wall_distance = min(robot.get_lidar_range_image()[200:275])
            print(f"Distance to front left wall: {front_left_wall_distance}")
        begin_wall_following = False
        print("Goal object found, motion to goal")
        # Getting the goal object and its position
        goal = recognized_objects[0]
        goal_position = goal.getPosition()
        goal_position_image = goal.getPositionOnImage()
        print(f"Goal position: x = {goal_position[0]}, y = {goal_position[1]}")
        print(f"Goal position on image: x = {goal_position_image[0]}, y = {goal_position_image[1]}")
        # Getting distance to goal and error
        distance_to_goal = goal_position[0]
        error = distance_to_goal - 0.5
        # Centering on goal if not centered
        if not (-0.3 < goal_position[1] < 0.3):
            print("Need to center goal")
            if goal_position_image[0] < 310:
                print("Rotating left to center goal")
                robot.set_left_motors_velocity(-3)
                robot.set_right_motors_velocity(3)
            else:
                print("Rotating right to center goal")
                robot.set_left_motors_velocity(3)
                robot.set_right_motors_velocity(-3)
        else:
            # Moving to goal
            print("Goal centered, moving to goal")
            distance_to_goal = goal_position[0]
            error = distance_to_goal - 0.5
            wheel_velocity = robot.PID_saturation(error, 10)
            robot.set_left_motors_velocity(wheel_velocity)
            robot.set_right_motors_velocity(wheel_velocity)
            # Reached goal
            if goal_position[0] < 0.5:
                print("Goal reached")
                robot.stop()
                break