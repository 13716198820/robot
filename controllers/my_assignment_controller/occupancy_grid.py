# OccupancyGrid Class Definition
# File: occupancy_grid.py
# Date: 30th Jan 2022
# Description: Occupancy Grid in Python for COMP329 (2022)
# Author: Terry Payne (trp@liv.ac.uk)
#
#          Updated to include pose by the map method
#          Stores the arena value to check if it set set up, and returns if failure

# ==============================================================
# COMP329 2024 Programming Assignment
# ==============================================================
# 
# The aim of the assignment is to move the robot around the arena
# in such a way as to generate an occupancy grid map of the arena
# itself.  Full details can be found on CANVAS for COMP329
#
# This occupancy grid class constructs an array corresponding to the
# size of the arena and the number of cells specified in the constructor.
# This is done by looking for object ARENA from the scene tree.  It also
# checks the size of the display attached to the robot (as specified
# in the constructor arguments) and scales the resulting map onto the
# display.
#
# The constructor is called with the following arguments:
#  - an instance of a Supervisor robot object
#  - the number of cells per meter.  The higher the number, the higher
#    the resolution of the map.  I recommend between 5-20.
#  - the name of the display object on the robot where the map appears
#  - the initial pose of the robot
#  - an instance of the PioneerSimpleProxSensors object
#
# To update the occupancy grid, the map() method is called with the current Pose
# To draw the occupancy grid, the paint() method is called.
#
# If an ARENA object is not defined, then a warning message on the
# console will appear.  You should ensure that the DEF field of the
# Rectangle Arena is filled with this string "ARENA"
#
#
# THIS IS WHAT YOU NEED TO DO
# ===========================
# The one thing missing in this class is the code to generate the
# log-odds value for each cell, depending on whether it is occupied,
# free, or unknown (for this return lprior, which is defined below).
# This should be done in the method
#
#     private double invSensorModel(Pose p, double x, double y);
# 
# This method takes the position of the current pose, and the (x,y)
# location of a cell based on a real coordinate point (i.e. the
# same coordinate system as a pose).
# Currently the method returns the value of the variable logodds; a
# good approach would be to set this to be the appropriate value 
# for the cell.
# 
# The method is called at the end of the map() method, which
# iterates through every cell in the occupancy grid.
#
# Details of the inverse sensor model can be found in Part 3 of
# the lecture notes (pages 51-66).
#
#    COMP329 7 Occupancy Grids and Mapping with Known Poses 2024-25.pdf
# 
# There is no need to implement the more advanced sensor model, but some
# thought should be given as to the choice of values for the occupied and
# empty log odds values.  See the definition of 'lprior' below.
# ==============================================================

import sys
import math
import pioneer_simpleproxsensors as psps

import pose


class OccupancyGrid:
    """ A custom class to model an occupancy grid with optional display """

    # --------------------------------
    # define the display constants
    DARKGRAY = 0x3C3C3C
    GRAY = 0xABABAB
    BLACK = 0x000000
    WHITE = 0xFFFFFF

    # --------------------------------
    # Fixed log odds values (found empirically)
    lprior = math.log(0.5/(1-0.5))
    # Add explicit occupied/free log-odds
    locc = math.log(0.85 / (1.0 - 0.85))
    lfree = math.log(0.35 / (1.0 - 0.35))


    # Constants for the inverse sensor model (values are halved to optimise performance)

    HALFALPHA = 0.02               # Thickness of any wall found
    HALFBETA = math.pi/36.0          # sensor cone opening angle



    # ==================================================================================
    # Constructor
    # ==================================================================================

    def __init__(self, robot, grid_scale, display_name, robot_pose, prox_sensors):

        self.robot = robot
        self.robot_pose = robot_pose
        self.prox_sensors = prox_sensors
        self.radius = self.prox_sensors.get_radius()

        # Store Arena state instance variables
        self.arena = robot.getFromDef("ARENA")
        if self.arena is None:
            print("COMP329 >>>Please define the DEF parameter of the RectangleArena as ARENA in the scene tree.<<<", file=sys.stderr)
            return


        floorSize_field = self.arena.getField("floorSize")
        floorSize = floorSize_field.getSFVec2f()
        self.arena_width = floorSize[0]
        self.arena_height = floorSize[1]

        # ---------------------------------------------------------------------------
        # Initialise grid - grid_scale cells per m
        self.num_row_cells = int(grid_scale * self.arena_width)
        self.num_col_cells = int(grid_scale * self.arena_height)
        print(f"Buidling an Occupancy Grid Map of size {self.num_row_cells} x {self.num_col_cells}")

        self.grid = [self.lprior]*(self.num_row_cells * self.num_col_cells)

        #self.loop_count = 0

        # ------------------------------------------
        # If provided, set up the display
        self.display = robot.getDevice(display_name)
        if self.display is not None:
            self.device_width = self.display.getWidth()
            self.device_height = self.display.getHeight()

            #Determine the rendering scale factor
            wsf = self.device_width / self.arena_width
            hsf = self.device_height / self.arena_height
            self.scalefactor = min(wsf, hsf)

            self.cell_width = int(self.device_width / self.num_row_cells)
            self.cell_height = int(self.device_height / self.num_col_cells)
        else:
            self.device_width = 0
            self.device_height = 0
            self.scalefactor = 0.0

    # ==================================================================================
    # Getters / Setters
    # ==================================================================================
    # The following can be used externally to check the status of the grid map,
    # for example, to develop an exploration strategy.
    def get_num_row_cells(self):
        return self.num_row_cells
    def get_num_col_cells(self):
        return self.num_col_cells
    # UPDATE - Corrected error in arguments for get_grid_size
    def get_grid_size(self):
        return len(self.grid)
    # UPDATE - call to cell_probability not scoped with "self"
    def get_cell_probability(self, i):
        return self.cell_probability(self.grid[i])

    def get_cell_probability_at_pose(self, p):
        # determine the cell at pose p
        # Return prior for now (no direct index provided)
        return self.cell_probability(self.lprior)

    # ==================================================================================
    # helper methods for mapping to the display
    # Map the real coordinates to screen coordinates assuming
    #   the origin is in the center and y axis is inverted

    def scale(self, l):
        return int(l * self.scalefactor)
    def mapx(self, x):
        return int((self.device_width / 2.0) + self.scale(x))
    def mapy(self, y):
        return int((self.device_height / 2.0) - self.scale(y))

    def set_pose(self, p):
        self.robot_pose.set_pose_position(p)

    #Convert log odds into a probability
    #Update - handle overflow
    def cell_probability(self, lodds):
        # Handle overflow properly
        try:
            exp = math.exp(lodds)
        except:
            exp = math.inf
        return 1 - (1 / (1 + exp))


    # ---------------------------------------------------------------------------
    # UPDATE WITH A FULLY WORKING SENSOR MODEL HERE
    # ---------------------------------------------------------------------------
    # Get log odds value for cell x,y given the current pose
    # ---------------------------------------------------------------------------

    def inv_sensor_model(self, p, x, y):
        # Compute from the closest sensor's origin and heading, not robot center
        logodds = self.lprior

        # If the cell is within robot body, ignore 
        dx_r = x - p.x
        dy_r = y - p.y
        r_robot = math.sqrt(dx_r * dx_r + dy_r * dy_r)
        if r_robot <= self.radius:
            return logodds  # inside robot footprint, no update

        max_range = self.prox_sensors.get_maxRange()
        eps = 1e-2

        # Track closest angle diff within any sensor's beam
        best = None 
        for i in range(self.prox_sensors.get_number_of_sensors()):
            sp = self.prox_sensors.get_relative_sensor_pose(i)
            # Sensor global origin by rotating relative offset and translating
            sx = p.x + math.cos(p.theta) * sp.x - math.sin(p.theta) * sp.y
            sy = p.y + math.sin(p.theta) * sp.x + math.cos(p.theta) * sp.y
            # Sensor global heading
            stheta = p.theta + sp.theta

            dx = x - sx
            dy = y - sy
            rs = math.sqrt(dx * dx + dy * dy)  # distance from sensor origin to cell
            phi = math.atan2(dy, dx)           # global bearing from sensor origin to cell
            angle_diff = phi - stheta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            angle_diff_abs = abs(angle_diff)

            # Only consider cells within sensor beam 
            eff_beta = self._effective_halfbeta(rs, max_range)
            if angle_diff_abs <= eff_beta:
                mr = self.prox_sensors.get_value(i)
                if best is None or angle_diff_abs < best[0]:
                    best = (angle_diff_abs, rs, mr, i)

        # If no sensor beam covers the cell, return prior (don't erase mapped walls)
        if best is None:
            return logodds

        angle_diff_abs, rs, mr, si = best

        # Inverse sensor model
        base_alpha = self.HALFALPHA * 2.0
        alpha = self._effective_alpha(rs, base_alpha, max_range)
        eps = 1e-2

        # Downweight free updates at the edge of the cone angle
        eff_beta = self._effective_halfbeta(rs, max_range)
        edge_factor = 1.0 - min(1.0, angle_diff_abs / eff_beta)  # 1 at center ray, ~0 at edge
        # Keep at least 20% strength to avoid no updates at all
        edge_factor = 0.2 + 0.8 * edge_factor

        if mr < max_range - eps:
            # Detection: mark only the area near the measured distance as occupied; mark space before detection as free; keep prior after detection
            if abs(rs - mr) <= (alpha * 0.5):
                logodds = self.locc
            elif rs < mr - (alpha * 0.5):
                # Edge-weighted free update to prevent walls from disappearing quickly after leaving
                logodds = self.lfree * edge_factor
            else:
                logodds = self.lprior
        else:
            # Maximum range: keep prior, do not mark as free
            logodds = self.lprior


        return logodds

    # Adaptive cone angle based on distance from sensor: wider at close range, narrower at far range
    def _effective_halfbeta(self, r, max_range):
        k_near = 1.5
        k_far = 0.6
        t = max(0.0, min(1.0, r / max_range))
        scale = k_near * (1.0 - t) + k_far * t
        return self.HALFBETA * scale

    # Adaptive wall thickness window based on distance: slightly thicker at far range to reduce long-distance missed detections
    def _effective_alpha(self, r, base_alpha, max_range):
        k_far = 1.8
        t = max(0.0, min(1.0, r / max_range))
        return base_alpha * (1.0 + (k_far - 1.0) * t)

    # ==================================================================================
    # External Methods
    # ==================================================================================
    # Update the occupancy grid based on the current pose

    def map(self, p):
        if self.arena is None:
            print("COMP329 >>>Please define the DEF parameter of the RectangleArena as ARENA in the scene tree.<<<", file=sys.stderr)
            return
        x_orig_offset = self.arena_width / 2
        y_orig_offset = self.arena_height / 2

        x_inc = self.arena_width / self.num_row_cells
        y_inc = self.arena_height / self.num_col_cells

        x_cell_offset = x_inc / 2
        y_cell_offset = y_inc / 2
        self.robot_pose.set_pose_position(p)

        for i in range(len(self.grid)):
            # Convert cell into a coordinate.  Recall that the arena is dimensions -n..+n
            x = x_inc * int(i % self.num_row_cells) - x_orig_offset + x_cell_offset
            y = -(y_inc * int(i / self.num_row_cells) - y_orig_offset + y_cell_offset)

            self.grid[i] = self.grid[i] + self.inv_sensor_model(self.robot_pose, x, y) - self.lprior

    # ==================================================================================
    # Update the display using a grayscale to represent the different probabilities
    # Note that a percentage of coverage is generated, based on counting the number of
    # cells have have a probability < 0.1 (empty) or > 0.9 (occupied)

    def paint(self):


        if self.arena is None:
            print("COMP329 >>>Please define the DEF parameter of the RectangleArena as ARENA in the scene tree.<<<", file=sys.stderr)
            return

        if self.display is None:
            return

        # draw a backgound
        self.display.setColor(0xF0F0F0)
        self.display.fillRectangle(0, 0, self.device_width, self.device_height)

        # draw values for occupancy grid
        self.coverage = 0.0
        for i in range(len(self.grid)):
            p = self.cell_probability(self.grid[i])
            x = self.cell_width * int(i % self.num_row_cells)
            y = self.cell_height * int(i / self.num_col_cells)

            if (p < 0.1):
                self.display.setColor(self.WHITE)
            elif (p < 0.2):
                self.display.setColor(0xDDDDDD)
            elif (p < 0.3):
                self.display.setColor(0xBBBBBB)
            elif (p < 0.4):
                self.display.setColor(0x999999)
            elif (p > 0.9):
                self.display.setColor(self.BLACK)
            elif (p > 0.8):
                self.display.setColor(0x222222)
            elif (p > 0.7):
                self.display.setColor(0x444444)
            elif (p > 0.6):
                self.display.setColor(0x666666)
            else:
                self.display.setColor(self.GRAY)

            self.display.fillRectangle(x, y, self.cell_width, self.cell_height)

            if(p < 0.1) or (p > 0.9):
                self.coverage += 1.0

        # normalise coverage
        self.coverage = self.coverage / len(self.grid)

        self.display.setColor(self.GRAY)
        # vertical lines
        x=0
        for i in range(self.num_row_cells):
            self.display.drawLine(x, 0, x, self.device_height)
            x += self.cell_width

        # horizontal lines
        y=0
        for j in range(self.num_row_cells):
            self.display.drawLine(0, y, self.device_width, y)
            y += self.cell_height

        # draw robot body
        self.display.setColor(self.WHITE)
        self.display.fillOval(self.mapx(self.robot_pose.x),
                              self.mapy(self.robot_pose.y),
                              self.scale(self.radius),
                              self.scale(self.radius))

        self.display.setColor(self.DARKGRAY)
        self.display.drawOval(self.mapx(self.robot_pose.x),
                              self.mapy(self.robot_pose.y),
                              self.scale(self.radius),
                              self.scale(self.radius))
        self.display.drawLine(self.mapx(self.robot_pose.x),
                              self.mapy(self.robot_pose.y),
                              self.mapx(self.robot_pose.x + math.cos(self.robot_pose.theta) * self.radius),
                              self.mapy(self.robot_pose.y + math.sin(self.robot_pose.theta) * self.radius))

        # Provide coverage percentage
        self.display.setColor(0xF0F0F0)  # Off White
        self.display.fillRectangle(self.device_width-80, self.device_height-18, self.device_width-20, self.device_height)
        self.display.setColor(0x000000)  # Black
        self.display.drawRectangle(self.device_width-80, self.device_height-18, self.device_width-20, self.device_height)


        self.display.setFont("Arial", 10, True)
        self.display.drawText(f"{self.coverage * 100:.2f}%", self.device_width-60, self.device_height-14);