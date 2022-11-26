
#########################################################################
#                                                                       #
#                        Grabber-Tank AI Script                         #
#                              Caden Perez                              #
#                         CSCE-480 Intro to AI                          #
#                                                                       #
#          Pathfinding algorithm and functions for the robot.           #
#                                                                       #
#########################################################################

import sense
import rpilib.move as move
import time

speed_set = 60

class Path:
    def __init__(self):
        directions = [ [-1,0], [0,1], [1,0], [0,-1] ]
        visited = {}
        times = []
        turns = []

    # Log a new time between turns
    def newTime(self, base_time):
        cur_time = time.time()
        self.times.append(cur_time - base_time)
        base_time = cur_time
        return base_time

    # Log a new turn
    def newTurn(self, t):
        self.turns.append(t)

    # Detect if an object or wall is in view and decide what action to take
    def wallDetected(detected_objects, img):
        dis = sense.ultra() # distance in cm

        if dis < 3 and dis > 1:
            move.motorStop() # stop moving
            if len(detected_objects) > 0:
                max_size = 0
                for x in detected_objects:
                    if x.size() > max_size:
                        closest_object = x
                x = closest_object.x
                w = closest_object.w

                # Object detected, change direction to approach object
                if x > (img.x/2) + w:
                    return 'redirect_left'
                if x < (img.x/2) - w:
                    return 'redirect_right'

                # If object is centered, opt to grab it
                elif x >= (img.x/2) - w and x <= (img.x/2) + w:
                    return 'grab'

            # Wall detected
            else:
                return 'wall'

        # If no object or wall detected, carry on
        return 'none'

    # DFS Pathfinding algorithm
    async def dfs(self, row, col, arrow, detected_objects, img):
        key = str(row) + ',' + str(col)

        if key not in self.visited:
            self.visited.add(key)

            for i in range(4):
                status = self.wallDetected(detected_objects, img)
                if status == 'wall':
                    curDirection = self.directions[arrow]
                    row += curDirection[0]
                    col += curDirection[1]
                    self.dfs(row, col, arrow)
                elif status == 'redirect' or 'grab':
                    return
                move(speed_set, 'no', 'right', 0.25)
                arrow = (arrow+1) % 4

        # Turn robot 180 degrees and move to backward location
        move(speed_set, 'no', 'left', 0.50) # 0.5 = half a rotation
        move(speed_set, 'forward', 'no', 1)
        grab_object_Here = 0

        return (row, col)
