import random
import math
import copy
import numpy as np
# import matplotlib.pyplot as plt # Commented out for ROS environment
import time

# import sys, select, termios, tty # Commented out for ROS environment

class RRT():
    """
    Class for RRT Planning
    """

    def plan(self, start_pt, goal_pt, obstacle_points_2d, obstacle_radius=0.5):
        """Plan a stable local path and collision-check every path segment."""
        self.start = Node(start_pt[0], start_pt[1], 0.0)
        self.obstacleList = [(x, y, obstacle_radius) for (x, y) in obstacle_points_2d]
        self.rrtTargets = [(goal_pt[0], goal_pt[1], 0.5)]

        # Do not generate a random-looking path when the direct route is clear.
        if self._segment_collision_free(start_pt, goal_pt, self.obstacleList):
            return [start_pt, goal_pt]

        dx = goal_pt[0] - start_pt[0]
        dy = goal_pt[1] - start_pt[1]
        distance = max(math.hypot(dx, dy), self.expandDis)
        forward_x = dx / distance
        forward_y = dy / distance
        lateral_x = -forward_y
        lateral_y = forward_x
        lateral_range = min(3.0, max(1.5, distance * 0.75))

        # A fixed seed makes a static scene produce a static path. The path still
        # changes when collision checks reject different branches after obstacles move.
        rng = random.Random(870)
        self.nodeList = [self.start]
        for i in range(min(self.maxIter, 250)):
            if i % 5 == 0:
                rnd = [goal_pt[0], goal_pt[1]]
            else:
                along = rng.uniform(0.0, distance + self.expandDis)
                lateral = rng.uniform(-lateral_range, lateral_range)
                rnd = [start_pt[0] + forward_x * along + lateral_x * lateral,
                       start_pt[1] + forward_y * along + lateral_y * lateral]
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            nearest = self.nodeList[nind]

            theta = math.atan2(rnd[1] - nearest.y, rnd[0] - nearest.x)
            newNode = Node(nearest.x + self.expandDis * math.cos(theta),
                           nearest.y + self.expandDis * math.sin(theta), theta)
            newNode.cost = nearest.cost + self.expandDis
            newNode.parent = nind

            if self._segment_collision_free(
                    (nearest.x, nearest.y), (newNode.x, newNode.y),
                    self.obstacleList):
                self.nodeList.append(newNode)
                dist_to_goal = math.hypot(newNode.x - goal_pt[0], newNode.y - goal_pt[1])
                if (dist_to_goal < self.expandDis * 2
                        and self._segment_collision_free(
                            (newNode.x, newNode.y), goal_pt,
                            self.obstacleList)):
                    # Found path to goal
                    path = [(goal_pt[0], goal_pt[1])]
                    curr = newNode
                    while curr is not None:
                        path.append((curr.x, curr.y))
                        curr = self.nodeList[curr.parent] if curr.parent is not None else None
                    path.reverse()
                    return path

        # A partial path is unsafe for vehicle control because its apparent end
        # can point away from the requested goal. Stop instead of publishing it.
        return None

    @staticmethod
    def _segment_collision_free(start_pt, end_pt, obstacle_list):
        """Return false when an inflated obstacle touches a complete segment."""
        x1, y1 = start_pt
        x2, y2 = end_pt
        vx = x2 - x1
        vy = y2 - y1
        length_sq = vx * vx + vy * vy
        for ox, oy, radius in obstacle_list:
            if length_sq <= 1e-12:
                closest_x, closest_y = x1, y1
            else:
                projection = ((ox - x1) * vx + (oy - y1) * vy) / length_sq
                projection = max(0.0, min(1.0, projection))
                closest_x = x1 + projection * vx
                closest_y = y1 + projection * vy
            obs_dx = ox - closest_x
            obs_dy = oy - closest_y
            if obs_dx * obs_dx + obs_dy * obs_dy <= radius * radius:
                return False
        return True

    def __init__(self, start=(0,0,0), planDistance=10.0, obstacleList=None, expandDis=0.5, turnAngle=30, maxIter=500, rrtTargets=None):


        self.start = Node(start[0], start[1], start[2])
        self.startYaw = start[2]

        self.planDistance = planDistance
        self.expandDis = expandDis
        self.turnAngle = math.radians(turnAngle)

        self.maxDepth = int(planDistance / expandDis)

        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.rrtTargets = rrtTargets

        self.aboveMaxDistance = 0
        self.belowMaxDistance = 0
        self.collisionHit = 0
        self.doubleNodeCount = 0

        self.savedRandoms = []

    def Planning(self, animation=False, interactive=False):
        self.nodeList = [self.start]
        self.leafNodes = []

        for i in range(self.maxIter):
            rnd = self.get_random_point_from_target_list()

            nind = self.GetNearestListIndex(self.nodeList, rnd)

            nearestNode = self.nodeList[nind]

            if (nearestNode.cost >= self.planDistance):
                continue

            newNode = self.steerConstrained(rnd, nind)

            if newNode in self.nodeList:
                continue

            if self.__CollisionCheck(newNode, self.obstacleList):
                self.nodeList.append(newNode)

                if (newNode.cost >= self.planDistance):
                    self.leafNodes.append(newNode)

        return self.nodeList, self.leafNodes

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            return newNode

        newNode.cost = mincost
        newNode.parent = minind

        return newNode

    def steerConstrained(self, rnd, nind):
        nearestNode = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

        angleChange = self.pi_2_pi(theta - nearestNode.yaw)

        angle30degree = math.radians(40) # Original was 30, but main script uses 40

        if angleChange > angle30degree:
            angleChange = self.turnAngle
        elif angleChange >= -angle30degree:
            angleChange = 0
        else:
            angleChange = -self.turnAngle

        newNode = copy.deepcopy(nearestNode)
        newNode.yaw += angleChange
        newNode.x += self.expandDis * math.cos(newNode.yaw)
        newNode.y += self.expandDis * math.sin(newNode.yaw)

        newNode.cost += self.expandDis
        newNode.parent = nind

        return newNode

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi

    def steer(self, rnd, nind):
        nearestNode = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = copy.deepcopy(nearestNode)
        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)

        newNode.cost += self.expandDis
        newNode.parent = nind
        return newNode

    def get_random_point(self):

        randX = random.uniform(0, self.planDistance)
        randY = random.uniform(-self.planDistance, self.planDistance)
        rnd = [randX, randY]

        car_rot_mat = np.array([[math.cos(self.startYaw), -math.sin(self.startYaw)], [math.sin(self.startYaw), math.cos(self.startYaw)]])
        rotatedRnd = np.dot(car_rot_mat, rnd)

        rotatedRnd = [rotatedRnd[0] + self.start.x, rotatedRnd[1] + self.start.y]
        return rotatedRnd

    def get_random_point_from_target_list(self):

        maxTargetAroundDist = 3

        if not self.rrtTargets:
            return self.get_random_point()

        targetId = np.random.randint(len(self.rrtTargets))
        x, y, oSize = self.rrtTargets[targetId]

        randAngle = random.uniform(0, 2 * math.pi)
        randDist = random.uniform(oSize, maxTargetAroundDist)
        finalRnd = [x + randDist * math.cos(randAngle), y + randDist * math.sin(randAngle)]

        return finalRnd

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]

        if len(goalinds) == 0:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = self.expandDis * 3.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta, d):

        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False

        return True

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList):
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d  <= size ** 2:
                return False
        return True

class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.cost = 0.0
        self.parent = None

    def __str__(self):
        return str(round(self.x, 2)) + "," + str(round(self.y,2)) + "," + str(math.degrees(self.yaw)) + "," + str(self.cost)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.yaw == other.yaw and self.cost == other.cost

    def __repr__(self):
        return str(self)

# main function for standalone testing (not a ROS node)
def main():
    print("Start rrt planning!")

    radius = 1
    obstacleList = [
        (1, -3, radius), (1, 3, radius), (6, -3, radius), (6, 3, radius),
        (12.5, -2.5, radius), (12, 4, radius), (20, -1, radius), (18.5, 6.5, radius),
        (24, 1, radius), (22, 8, radius)
    ]

    start = [0.0, 0.0, math.radians(0.0)]
    planDistance = 10
    iterationNumber = 500
    rrtConeTargets = []

    for o in obstacleList:
        coneDist = math.sqrt((start[0] - o[0]) ** 2 + (start[1] - o[1]) ** 2)
        if coneDist > 10 and coneDist < 15:
            rrtConeTargets.append((o[0], o[1], radius))

    startTime = time.time()

    rrt = RRT(start, planDistance, obstacleList=obstacleList, expandDis=1, maxIter=iterationNumber, rrtTargets = rrtConeTargets)
    rrt.Planning(False, False) # No animation or interactive input

    print (f'rrt.Planning(): {(time.time() - startTime) * 1000} ms')
    print (f'nodesNumber/iteration: {len(rrt.nodeList)}/{iterationNumber}')

if __name__ == '__main__':
    main()
