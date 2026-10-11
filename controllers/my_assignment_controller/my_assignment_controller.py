"""my_assignment_controller controller."""

# ==============================================================
# COMP329 2024 Programming Assignment
# ==============================================================
#
# The aim of the assignment is to move the robot around the arena
# in such a way as to generate an occupancy grid map of the arena
# itself.  Full details can be found on CANVAS for COMP329
#
# This controller file simply creates a controller and initialises
# the occupancy grid class.  You will need to complete the code in
# the occupancy grid class to implement the inverse sensor model
# and complete the update method.
#+
# Note that the size of the occupancy grid can be changed (see below)
# as well as the update frequency of the map, and whether or not a
# map is generated. Changing these values may be useful during the
# debugging phase, but ensure that the solution you submit generates
# an occupancy grid map of size 100x100 cells.
#
# Note that this class makes the following assumptions:
#
#	PioneerCLNav
#	============
#	You have a navigator class called PioneerCLNav that implements
#	the following methods (based on Lab Tutorial 6):
#	  - a constructor that takes two arguments, an instance of a Supervisor
#	    robot object and an instance of a PioneerSimpleProxSensors
#	  - set_goal() which defines a destination pose
#	  - update() that makes any changes necessary as part of navigation
#	You can replace these with other calls if you want to use a different
#	class or means of navigation.  Note this is NOT included.
#	Note - if you use a different class you must implement a method called
#	get_real_pose() that returns an object of type Pose representing the true
#	location of the robot.  Examples of this class are used in Lab Tutorials 3 and 6.
#
#	PioneerSimpleProxSensors
#	========================
#	You have a sensor class based on the work in Lab 5.  This is used by
#	navigator class, but could also be used by a navigator class.  Code
#	for the version used in the tutorial is included with the assignment.
#
#	Pose
#	====
#	This is the latest version of the Pose class (it includes some additional
#	methods used in Lab Tutorial 6), and is used throughout the COMP329
#	code base.  A version is included with the assignment.
#
#	OccupancyGrid
#	=============
#	You have an occupancy grid class.  The constructor takes the following
#	arguments:
#	  - an instance of a Supervisor robot object
#	  - the number of cells per meter.  The higher the number, the higher
#	    the resolution of the map.  I recommend between 5-20.
#	  - the name of the display object on the robot where the map appears
#	  - the initial pose of the robot
#	  - an instance of the PioneerSimpleProxSensors object
#
#	To update the occupancy grid, call the map() method with the current Pose
#	To draw the occupancy grid, call the paint() method
#
#
# ==============================================================

from controller import Supervisor
import pioneer_clnav as pn
import pioneer_simpleproxsensors as psps
import occupancy_grid as ogrid
import math
import pose

# ==================================================================================
# Main Methods
# ==================================================================================

def run_robot(robot):

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # ---------------------------------------------------------------------------
    # initialise other classes such as the proximity and occupancy grid classes
    # ---------------------------------------------------------------------------

    pps = psps.PioneerSimpleProxSensors(robot)
    nav = pn.PioneerCLNav(robot, pps)

    # 2nd argument determines how many cells per meter of the arena.
    # Use 10 for testing navigation, but 20 for high-quality map (slow)
    occupancy_grid = ogrid.OccupancyGrid(robot, 10, "display", nav.get_real_pose(), pps)

    # ---------------------------------------------------------------------------
    # Initialise your exploration approach.  Here we just set an initial
    # way point as the goal.
    # INSERT INITIALISATION CODE HERE

    waypoints = [
        # A circle around the outer ring
        pose.Pose(-4.5,  4.5, 0),
        pose.Pose(4.5, 3.5, 0),
        pose.Pose(4.5, -4.5, 0),
        pose.Pose(-4.5, -4.5, 0),
        pose.Pose(-4.0, -4.0, 0),
        # Diagonal line
        pose.Pose(4.2, 3, 0),


    ]

    current_waypoint = 0
    waypoint_reached = False
    exploration_complete = False
    coverage_threshold = 0.90  # 90% coverage rate as completion criterion

    # Distance arrival tolerance
    goal_distance_tolerance = 0.1

    # Set the first waypoint
    nav.set_goal(waypoints[current_waypoint])
    print(f"Initializing waypoint: Current waypoint index {current_waypoint}, target waypoint {waypoints[current_waypoint]}")  


    # ---------------------------------------------------------------------------
    # Main loop:
    while robot.step(timestep) != -1:

        # ---------------------------------------------------------------------------
        # Add code to manage the movement or navigation here.  Remember not to write
        # blocking code, as the robot should loop here and update the map every cycle
        # INSERT NAVIGATION CODE HERE

        # Get current pose
       # 1. Get current pose and distance to waypoint
        current_pose = nav.get_real_pose()
        if 0 <= current_waypoint < len(waypoints):
            dist_to_goal = current_pose.get_range(waypoints[current_waypoint])
            # 2. Determine arrival if distance is less than tolerance
            if dist_to_goal <= goal_distance_tolerance:
                waypoint_reached = True
          
        # Update navigation
        if nav.update():
            waypoint_reached = True

        # If waypoint is reached, proceed to the next one
        if waypoint_reached:
            print(f"[Waypoint reached] Reached waypoint {current_waypoint}: {waypoints[current_waypoint]}")
            current_waypoint += 1
            waypoint_reached = False

            if current_waypoint < len(waypoints):
                nav.set_goal(waypoints[current_waypoint])
                # Force state switch and reset parameters
                nav.state = nav.GOALSEEKING  # Switch back to goal seeking
                nav.wf_state = nav.WF_HIT    # Reset wall-following sub-state
                nav.start = current_pose     # Reset start point (refer to Lab6 Step2 acceleration logic)
                nav.prev_error = 0           # Reset PID error (refer to Lab6 Step3)
                nav.total_error = 0
                nav.pid_counter = 0
                print(f"Proceeding to waypoint {current_waypoint}: {waypoints[current_waypoint]}")
            else:
                # All waypoints completed, check coverage rate
                if occupancy_grid.coverage >= coverage_threshold:
                    exploration_complete = True
                    print(f"Exploration completed! Coverage rate: {occupancy_grid.coverage * 100:.2f}%")
                else:
                    # Insufficient coverage rate, restart exploration
                    print(f"Insufficient coverage rate ({occupancy_grid.coverage * 100:.2f}%), continuing exploration")
                    current_waypoint = 0
                    nav.set_goal(waypoints[current_waypoint])

	# ----------------------------------------------------------------------------
	# the following two lines update the occupancy grid, and then display the grid
	# Comment out the method paint() if you don't want to see the occupancy grid

        occupancy_grid.map(nav.get_real_pose())
        occupancy_grid.paint()
	# ----------------------------------------------------------------------------


    nav.update()	# ensures motors come to a stop

    # Enter here exit cleanup code.


if __name__ == "__main__":
    # ---------------------------------------------------------------------------
    # create the Supervised Robot instance.
    # ---------------------------------------------------------------------------
    my_robot = Supervisor()
    run_robot(my_robot)