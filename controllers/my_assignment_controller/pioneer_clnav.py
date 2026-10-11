# pioneer_clnav Class Definition
# File: pioneer_clnav.py
# Date: 12th Nov 2024
# Description: 
# Author: Terry Payne (trp@liv.ac.uk)

# ==============================================================
# COMP329 2024 Programming Assignment
# ==============================================================
# 
# The aim of the assignment is to move the robot around the arena
# in such a way as to generate an occupancy grid map of the arena
# itself.  Full details can be found on CANVAS for COMP329
#
# This copy of the navigator class is the same as that downloaded from
# COMP329 Lab Tutorial 6.  If you have completed the lab, then replace this
# file with your completed version.
#
# You can also develop a completely different implementation of the
# navigator class; however, ensure that it implements the method
#
#      get_real_pose(self):
#
# The easiest thing is to reuse the implementation of this method
# from this class.
# ==============================================================


import math
import pose
import pioneer_simpleproxsensors as psps

class PioneerCLNav:
    """ A custom class to demonstrate a closed loop controller with bug1 """

    WALLFOLLOWING = True
    GOALSEEKING = False
    WF_HIT = 0
    WF_SEARCH = 1
    WF_LEAVE = 2
    WF_ROTATE = 3

    BUG_RADIUS = 0.4;   # For wall following, allow a greater distance to hit/leave points
    GOAL_RADIUS = 0.1   # If the robot is this dist from goal, then success
    GOAL_VEL = 1        # min velocity when robot reaches goal before stopping
    START_VEL = 3       # Initial move velocity
    ACCEL_DIST = 0.4    # Accelerate up to distance away from the start
    DECEL_DIST = 0.8    # Decelerate when this distance away from the goal
    ACCEL_ANGLE = 0.524 # Decelerate rotation when this angle (rads) from goal
    WF_DIST = 0.3       # Distance when wall following
    WF_VEL = 5          # Default velocity when wall following



    # ==================================================================================
    # Constructor
    # ==================================================================================
    def __init__(self, robot, prox_sensors):

        self.robot = robot
        self.robot_node = self.robot.getSelf()    # reference to the robot node
        self.pps = prox_sensors                   # reference to the sensor model
        self.goal = self.get_real_pose()          # assume the goal is our current position
        self.start = self.get_real_pose()         # retain our initial position
        self.state = self.GOALSEEKING

        self.prev_error = 0              # Reset values used by the PID controller
        self.total_error = 0             # Reset values used by the PID controller

        self.pid_counter = 0  # Initialize PID counter
        self.pid_diff = 0.0  # Initialize PID derivative term

      
        self.stuck_rescue_timer = 0  # Self-rescue action timer
        self.RESCUE_DURATION = 70  # Duration of self-rescue action in frames
        self.temp_wallfollow_in_leave = False  # Whether in temporary wall-following phase after rescue
        self.temp_wallfollow_leave_timer = 0  # Temporary wall-following timer
        self.TEMP_LEAVE_WF_DURATION = 300  # Duration of wall-following in frames

        self.stuck_check_count_wf_search = 0  # Wall-stuck count in WF_SEARCH state
        self.last_dist_wf_search = float('inf')  # Last distance in WF_SEARCH state
        self.rescue_timer_wf_search = 0  # Self-rescue timer for WF_SEARCH
        self.WF_SEARCH_RESCUE_DURATION = 30  # Duration of reverse self-rescue in WF_SEARCH

        self.stuck_check_count_gs = 0  # Wall-stuck count in GOALSEEKING state
        self.last_dist_gs = float('inf')  # Last distance in GOALSEEKING state
        self.rescue_timer_gs = 0  # Self-rescue timer for GOALSEEKING
        self.GS_RESCUE_DURATION = 30  # Duration of reverse self-rescue in GOALSEEKING

        # enable motors
        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.stuck_check_count = 0
        self.last_dist_to_leave = float('inf')

        self.max_vel = self.left_motor.getMaxVelocity()

        # Initialise motor velocity
        self.stop()

        #self.left_motor.setVelocity(0.0)
        #self.right_motor.setVelocity(0.0)




    # ==================================================================================
    # Internal (helper) methods
    # ==================================================================================
    def set_velocity(self, lv, rv):
        self.left_motor.setVelocity(lv)
        self.right_motor.setVelocity(rv)

    # returns the range to an obstacle in front of the robot
    # up to max_range. Used to trigger wall following
    def range_to_frontobstacle(self):
        range = min(self.pps.get_maxRange(),
                    self.pps.get_value(3),
                    self.pps.get_value(4),
                    self.pps.get_value(2),

                    self.pps.get_value(5))
        return range

    def range_to_frontcornerobstacle(self):
        range = min(self.pps.get_maxRange(),
                    self.pps.get_value(1),
                    self.pps.get_value(2),
                    self.pps.get_value(5),
                    self.pps.get_value(6))
        return range

    def range_to_leftobstacle(self):
        range = min(self.pps.get_maxRange(),
                    self.pps.get_value(0))
        return range

    # ==================================================================================
    # Internal (navigation) methods
    # ==================================================================================

    def pid(self, error):
        kp = 6.0  # Proportional coefficient
        ki = 0.0  # Integral coefficient
        kd = 40.0  # Derivative coefficient
        prop = error
        self.total_error += error

        # Update derivative term every 4 cycles to reduce noise impact
        if ((self.pid_counter % 4) == 0):
            self.pid_diff = error - self.prev_error
            self.prev_error = error

        self.pid_counter += 1

        # Return control value
        return (kp * prop) + (ki * self.total_error) + (kd * self.pid_diff)

    # ==================================================================================
    def adjust_velocity(self, bearing, velocity):
        # Calculate rotation velocity
        rotate_vel = min(self.max_vel, abs(bearing * self.max_vel / self.ACCEL_ANGLE))

        # Determine rotation direction
        if bearing < 0:
            rotate_vel = -rotate_vel

        # Set left and right wheel velocities
        lv = min(self.max_vel, velocity - rotate_vel)
        rv = min(self.max_vel, velocity + rotate_vel)

        self.set_velocity(lv, rv)

    # ==================================================================================
    def rotate(self, bearing, velocity):
        # Check if facing the correct direction (tolerance 0.1)
        if abs(bearing) < 0.1:
            return True

        # Adjust velocity for rotation
        self.adjust_velocity(bearing, velocity)
        return False

    # ==================================================================================
    def track_leave_point(self, p):
        # Calculate distance from current position to goal
        dist = p.get_range(self.goal)

        # Update leave point if a closer point is found
        if dist < self.leave_dist:
            self.leave_dist = dist
            self.leave_point = p

    # ==================================================================================
    # Main logic for the Bug 1 Algorithm
    # ==================================================================================
    # Note we don't handle unaccessible goals

    def bug1(self, p):


        # State 1: WF_HIT
        if self.wf_state == self.WF_HIT:
            # Start checking for return to hit point only after moving a certain distance
            if hit_dist > self.BUG_RADIUS:
                self.wf_state = self.WF_SEARCH
            self.track_leave_point(p)

        # State 2: WF_SEARCH
        elif self.wf_state == self.WF_SEARCH:
            # Prioritize handling self-rescue state
            if self.rescue_timer_wf_search > 0:
                self.rescue_timer_wf_search -= 1
                # Execute reverse action
                self.set_velocity(-self.WF_VEL * 0.5, -self.WF_VEL * 0.5)
                print(f"[WF_SEARCH][In self-rescue] Reverse remaining {self.rescue_timer_wf_search} frames")
                if self.rescue_timer_wf_search == 0:
                    print(f"[WF_SEARCH][Self-rescue ended] Resume normal search")
                return

            print(f"[WF_SEARCH] Distance to hit point: {hit_dist:.2f} | Current leave point distance to goal: {self.leave_dist:.2f}")

            # Wall-stuck detection: Judged by distance change to hit point
            if abs(hit_dist - self.last_dist_wf_search) < 0.001:
                self.stuck_check_count_wf_search += 1
            else:
                self.stuck_check_count_wf_search = 0
            self.last_dist_wf_search = hit_dist

            # Trigger reverse self-rescue if distance unchanged for 50 consecutive frames
            if self.stuck_check_count_wf_search >= 50:
                print(f"[WF_SEARCH][Wall-stuck triggered] Distance {hit_dist:.3f} unchanged, starting reverse self-rescue")
                self.rescue_timer_wf_search = self.WF_SEARCH_RESCUE_DURATION
                self.stuck_check_count_wf_search = 0
                return

            if hit_dist < self.BUG_RADIUS:
                self.wf_state = self.WF_LEAVE
                print(f"[State switched] Returned to hit point, WF_SEARCH → WF_LEAVE")
            else:
                # Force update leave point
                current_dist_to_goal = p.get_range(self.goal)
                if current_dist_to_goal < self.leave_dist - 0.1:  # Update only if 0.1m closer than current leave point
                    self.leave_dist = current_dist_to_goal
                    self.leave_point = p
                    print(f"[WF_SEARCH] Updated leave point, distance to goal: {self.leave_dist:.2f}")
                self.track_leave_point(p)

        # State 3: WF_LEAVE 
        elif self.wf_state == self.WF_LEAVE:
            dist_to_leave = p.get_range(self.leave_point)
            bearing_to_leave = p.get_bearing(self.leave_point)
            leave_vel = self.WF_VEL * 0.8



            # Prioritize wall-following phase after self-rescue
            if self.temp_wallfollow_in_leave:
                self.temp_wallfollow_leave_timer -= 1
                # Execute wall-following logic
                front_range = min(self.range_to_frontobstacle(), self.range_to_frontcornerobstacle())
                left_range = self.range_to_leftobstacle()

                if front_range <= self.WF_DIST * 0.9:
                    # Obstacle ahead, turn right slowly
                    self.set_velocity(leave_vel * 0.8, -leave_vel * 0.3)
                else:
                    # Maintain distance from left wall
                    if left_range < self.pps.get_maxRange():
                        error = left_range - self.WF_DIST
                        control = self.pid(error) * 0.7 
                        control = min(control, leave_vel)
                        control = max(control, -leave_vel)
                        self.set_velocity(leave_vel * 0.8, leave_vel * 0.8 + control)
                    else:
                        # No wall detected, turn left slowly to find wall
                        self.set_velocity(leave_vel * 0.3, leave_vel * 0.6)

                # End temporary wall-following, resume normal leave logic
                if self.temp_wallfollow_leave_timer <= 0:
                    self.temp_wallfollow_in_leave = False
                    self.stuck_check_count = 0  
                    print(f"[WF_LEAVE] Temporary wall-following ended, resuming to leave point")
                return

            # Handle self-rescue state
            if self.stuck_rescue_timer > 0:
                self.stuck_rescue_timer -= 1
                # Rescue action: First back up a certain distance
                if self.stuck_rescue_timer > self.RESCUE_DURATION * 0.9:
                    # Back up for first 70% of duration
                    self.set_velocity(-leave_vel * 0.6, -leave_vel * 0.6)
                else:
                    # Turn right slightly for last 30% to deviate from current obstacle
                    self.set_velocity(leave_vel * 0.3, -leave_vel * 0.3)

                # Enter temporary wall-following phase directly after rescue
                if self.stuck_rescue_timer == 0:
                    self.temp_wallfollow_in_leave = True
                    self.temp_wallfollow_leave_timer = self.TEMP_LEAVE_WF_DURATION
                    print(f"[WF_LEAVE] Self-rescue completed, starting temporary wall-following to find path")
                return

            # Wall-stuck detection: Consecutive small distance changes
            if abs(dist_to_leave - self.last_dist_to_leave) < 0.01:
                self.stuck_check_count += 1
            else:
                self.stuck_check_count = 0
            self.last_dist_to_leave = dist_to_leave

            # Trigger self-rescue: Start rescue if distance unchanged for 20 consecutive frames
            if self.stuck_check_count >= 20:
                print(f"[WF_LEAVE][Wall-stuck triggered] Distance {dist_to_leave:.3f} unchanged, starting rescue + wall-following")
                self.stuck_rescue_timer = self.RESCUE_DURATION 
                self.stuck_check_count = 0
                return

            # Normal movement logic
            max_rotate_vel = leave_vel * 0.8  # Limit maximum rotation velocity to avoid over-rotation
            rotate_vel = bearing_to_leave * (self.max_vel / self.ACCEL_ANGLE)
            rotate_vel = max(-max_rotate_vel, min(rotate_vel, max_rotate_vel))

            lv = leave_vel - rotate_vel
            rv = leave_vel + rotate_vel

            # Limit velocity range
            lv = max(-self.max_vel, min(lv, self.max_vel))
            rv = max(-self.max_vel, min(rv, self.max_vel))

            self.set_velocity(lv, rv)

            # Leave point arrival judgment (relax threshold to avoid jitter)
            if dist_to_leave < self.BUG_RADIUS * 2:
                self.wf_state = self.WF_ROTATE
                self.stuck_check_count = 0
                self.temp_wallfollow_in_leave = False  
                print(f"[State switched] Arrived at leave point, WF_LEAVE → WF_ROTATE")
        # State 4: WF_ROTATE 
        elif self.wf_state == self.WF_ROTATE:
            bearing = p.get_bearing(self.goal)
            if self.rotate(bearing, self.WF_VEL):
                self.state = self.GOALSEEKING
                self.start = self.get_real_pose()  # Reset start point
                self.wf_state = self.WF_HIT        # Reset wall-following substate
                return

        # Remove original unified wall-following call, execute by state instead
        if self.wf_state in [self.WF_HIT, self.WF_SEARCH]:
            self.wall_following(p)  # Only search and hit states need wall-following

    # ==================================================================================
    def wall_following(self, p):
        # Check for obstacles ahead
        front_range = min(self.range_to_frontobstacle(),
                          self.range_to_frontcornerobstacle())

        # If obstacle ahead, turn right to bypass
        if front_range <= self.WF_DIST:
            self.set_velocity(self.WF_VEL * 1.2, -self.WF_VEL * 0.5) 
        else:
            # Check left distance
            left_range = self.range_to_leftobstacle()

            if left_range < self.pps.get_maxRange():

                error = left_range - self.WF_DIST
                control = self.pid(error)


                control = min(control, self.WF_VEL)
                control = max(control, -self.WF_VEL)

                self.set_velocity(self.WF_VEL, self.WF_VEL + control)
            else:
                # No wall detected, turn left to find wall
                self.set_velocity(self.WF_VEL * 0.25, self.WF_VEL * 0.75)

    # ==================================================================================
    def goal_seeking(self, p):
        # Calculate distance and angle to goal
        goal_dist = p.get_range(self.goal)
        bearing = p.get_bearing(self.goal)

        # Prioritize handling self-rescue state
        if self.rescue_timer_gs > 0:
            self.rescue_timer_gs -= 1
            # Execute reverse action
            self.set_velocity(-self.START_VEL * 0.5, -self.START_VEL * 0.5)
            print(f"[GOALSEEKING][In self-rescue] Reverse remaining {self.rescue_timer_gs} frames")
            if self.rescue_timer_gs == 0:
                print(f"[GOALSEEKING][Self-rescue ended] Resume normal goal tracking")
            return

        # Wall-stuck detection
        if abs(goal_dist - self.last_dist_gs) < 0.001:
            self.stuck_check_count_gs += 1
        else:
            self.stuck_check_count_gs = 0
        self.last_dist_gs = goal_dist

        # Trigger reverse self-rescue if distance unchanged for 50 consecutive frames
        if self.stuck_check_count_gs >= 50:
            print(f"[GOALSEEKING][Wall-stuck triggered] Distance {goal_dist:.3f} unchanged, starting reverse self-rescue")
            self.rescue_timer_gs = self.GS_RESCUE_DURATION
            self.stuck_check_count_gs = 0
            return

        # Check for obstacles ahead
        obstacle_dist = self.range_to_frontobstacle()

        if self.emergency_stop(obstacle_dist):
            # Back up briefly then rotate
            self.set_velocity(-self.START_VEL / 2, -self.START_VEL / 2)

            self.set_velocity(self.START_VEL / 4, -self.START_VEL / 4) 
            return

        base_wf_dist = self.WF_DIST

        # Switch to wall-following if obstacle is too close
        if obstacle_dist <= base_wf_dist:
            self.state = self.WALLFOLLOWING
            self.wf_state = self.WF_HIT
            # Reset PID controller
            self.prev_error = 0
            self.total_error = 0
            self.pid_counter = 0
            self.pid_diff = 0.0
            # Record hit point and leave point
            self.hit_point = p
            self.leave_point = p
            self.leave_dist = goal_dist
        else:
            # Calculate acceleration and deceleration
            accel_rate = (self.max_vel - self.START_VEL) / self.ACCEL_DIST
            decel_rate = (self.max_vel - self.GOAL_VEL) / (self.DECEL_DIST - self.GOAL_RADIUS)
            yintercept = self.max_vel - (decel_rate * self.DECEL_DIST)

            # Calculate distance to start point
            start_dist = p.get_range(self.start)

            # Calculate acceleration velocity (when leaving start)
            accel_vel = min(self.max_vel, (start_dist * accel_rate) + self.START_VEL)

            # Calculate deceleration velocity (when approaching goal)
            decel_vel = min(self.max_vel, (goal_dist * decel_rate) + yintercept)

            # Consider obstacle-based deceleration
            if (obstacle_dist - self.WF_DIST) < self.DECEL_DIST:
                decel_vel = min(decel_vel,
                                ((obstacle_dist - self.WF_DIST) * decel_rate) + yintercept)

            # Final velocity takes the minimum
            final_vel = min(decel_vel, accel_vel)

            # Adjust velocity
            self.adjust_velocity(bearing, final_vel)

    # ==================================================================================
    # External methods (Navigation)
    # ==================================================================================

    def emergency_stop(self, obstacle_dist):
        """Emergency stop to avoid collision"""
        emergency_dist = 0.15  

        if obstacle_dist < emergency_dist:
            self.set_velocity(0, 0)
            return True
        return False

    # ==================================================================================
    def stop(self):
        self.set_velocity(0.0, 0.0)

    # ==================================================================================
    # Get Real Pose - ask the supervisor where the robot is
    # ==================================================================================
    def get_real_pose(self):
        if self.robot_node is None:
            return pose.Pose(0, 0, 0)

        real_pos = self.robot_node.getPosition()
        rot = self.robot_node.getOrientation()
        theta = math.atan2(-rot[0], rot[3])
        halfpi = math.pi / 2
        theta2 = theta + halfpi
        if (theta > halfpi):
            theta2 = -(3 * halfpi) + theta

        return pose.Pose(real_pos[0], real_pos[1], theta2)

    # ==================================================================================
    # Go to some pose
    # ==================================================================================
    # update() updates the current action of the robot in moving to the goal
    # returns false if at goal or true if moving

    def update(self):
        p = self.get_real_pose()
        if (p.get_range(self.goal) < self.GOAL_RADIUS):
            self.stop()
            return True

        if (self.state == self.GOALSEEKING):
            self.goal_seeking(p)
        else:
            self.bug1(p)
        return False    # Not yet found goal

    def set_goal(self, p):
        self.goal = pose.Pose(p.x, p.y, p.theta)
        self.start = self.get_real_pose()
        self.update()