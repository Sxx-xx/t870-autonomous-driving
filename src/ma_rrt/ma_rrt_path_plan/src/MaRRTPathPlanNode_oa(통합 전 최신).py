#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RRT Path Planning with multiple remote goals.

author: Maxim Yastremsky(@MaxMagazin)
based on the work of AtsushiSakai(@Atsushi_twi)

"""
import rospy
import csv

import ma_rrt_oa

from vehicle_msgs.msg import TrackCone, Track, Command, Waypoint, WaypointsArray

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

# Matrix/Array library
import numpy as np
import time, math

class MaRRTPathPlanNode:
    # All variables, placed here are static

    def __init__(self):
        # Get all parameters from launch file
        self.shouldPublishWaypoints = rospy.get_param('~publishWaypoints', True)
        self.shouldPublishPredefined = rospy.get_param('~publishPredefined', False)

        if rospy.has_param('~path'):
            self.path = rospy.get_param('~path')

        if rospy.has_param('~filename'):
            self.filename = rospy.get_param('~filename')

        if rospy.has_param('~odom_topic'):
            self.odometry_topic = rospy.get_param('~odom_topic')
        else:
            self.odometry_topic = "/odometry"

        if rospy.has_param('~world_frame'):
            self.world_frame = rospy.get_param('~world_frame')
        else:
            self.world_frame = "PandarXT-16"

        waypointsFrequency = rospy.get_param('~desiredWaypointsFrequency', 5)
        self.waypointsPublishInterval = 1.0 / waypointsFrequency
        self.lastPublishWaypointsTime = 0

        # All Subs and pubs
        rospy.Subscriber("/track", Track, self.mapCallback)
        #rospy.Subscriber(self.odometry_topic, Odometry, self.odometryCallback)
        #rospy.subscriber("/imu/data_raw", Imu, self.yawCallback)
        #rospy.Subscriber("/dvsim/cmd", Command, self.carSensorsCallback)

        # Create publishers
        self.waypointsPub = rospy.Publisher("/waypoints", WaypointsArray, queue_size=0)
        self.newwaypointsPub = rospy.Publisher("/newwaypoints", WaypointsArray, queue_size=5)
        self.goalMarkerPub = rospy.Publisher("/visual/goal_marker", Marker, queue_size=1)  # 목표 지점 퍼블리셔 추가

        # visuals
        self.treeVisualPub = rospy.Publisher("/visual/tree_marker_array", MarkerArray, queue_size=0)
        self.bestBranchVisualPub = rospy.Publisher("/visual/best_tree_branch", Marker, queue_size=1)
        self.filteredBranchVisualPub = rospy.Publisher("/visual/filtered_tree_branch", Marker, queue_size=1)
        self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)

        self.carPosX = 0.0
        self.carPosY = 0.0
        self.carPosYaw = 0.0

        self.map = []
        self.savedWaypoints = []
        self.preliminaryLoopClosure = False
        self.loopClosure = False

        self.rrt = None

        # if self.shouldPublishPredefined:
        #     with open(self.path + self.filename) as csv_file:
        #         csv_reader = csv.reader(csv_file, delimiter=',')

        #         for row in csv_reader:
        #             self.savedWaypoints.append((float(row[0]), float(row[1])))

        #         # print("predefinedWaypoints:", self.savedWaypoints)
        #     self.preliminaryLoopClosure = True
        #     self.loopClosure = True

        self.filteredBestBranch = []
        self.discardAmount = 0

        print("MaRRTPathPlanNode Constructor has been called")

    def __del__(self):
        print('MaRRTPathPlanNode: Destructor called.')

    def odometryCallback(self, odometry):
        # rospy.loginfo("odometryCallback")

        # start = time.time()

        self.carPosX = odometry.pose.pose.position.x
        self.carPosY = odometry.pose.pose.position.y
        #print "Estimated processing odometry callback: {0} ms".format((time.time() - start)*1000)

    def yawCallback(self, yaw):
        self.carPosYaw = yaw.orientation.vz

    def carSensorsCallback(self, command):
        # rospy.loginfo("carSensorsCallback")

        # start = time.time()
        # The ackermann angle [rad]
        self.steerAngle = math.degrees((command.theta_l + command.theta_r) / 2.0)

        #print "Estimated processing map callback: {0} ms".format((time.time() - start)*1000);

    def mapCallback(self, track):
        self.map = track.cones


    def sampleTree(self):
        if self.loopClosure and len(self.savedWaypoints) > 0:
            self.publishWaypoints()
            return

        if not self.map:    
            return

        frontConesDist = 12
        frontCones = self.getFrontConeObstacles(self.map, frontConesDist)

        coneObstacleSize = 1.5  # 장애물 크기 조정 1.165
        coneObstacleList = [(cone.x, cone.y, coneObstacleSize) for cone in frontCones]

        fixed_goal_distance = 1.5
        MIN_CONE_DISTANCE = 0.7  # 두 콘 사이의 최소 거리

        def cluster_cones(cones, min_distance):
            clusters = []
            for cone in cones:
                added = False
                for cluster in clusters:
                    if any(math.hypot(cone.x - c.x, cone.y - c.y) < min_distance for c in cluster):
                        cluster.append(cone)
                        added = True
                        break
                if not added:
                    clusters.append([cone])
            return clusters

        cone_clusters = cluster_cones(frontCones, MIN_CONE_DISTANCE)

        rospy.loginfo(f"현재 장애물: {len(cone_clusters)}")
        
        if len(cone_clusters) == 1:
            # # 하나의 클러스터만 있는 경우 (단일 장애물 또는 매우 가까운 여러 장애물)
            avg_x = sum(cone.x for cone in cone_clusters[0]) / len(cone_clusters[0])
            avg_y = sum(cone.y for cone in cone_clusters[0]) / len(cone_clusters[0])
            goal_x = avg_x + fixed_goal_distance * math.cos(self.carPosYaw)
            goal_y = avg_y + fixed_goal_distance * math.sin(self.carPosYaw)
        elif len(cone_clusters) >= 2:
            # 여러 클러스터가 있는 경우
            cluster_centers = [(sum(cone.x for cone in cluster) / len(cluster),
                                sum(cone.y for cone in cluster) / len(cluster))
                            for cluster in cone_clusters]
            
            # 가장 가까운 두 클러스터 찾기
            sorted_clusters = sorted(cluster_centers, key=lambda center: math.hypot(center[0] - self.carPosX, center[1] - self.carPosY))
            closest_clusters = sorted_clusters[:2]

            # 두 클러스터 사이의 중간점을 목표점으로 설정
            goal_x = (closest_clusters[0][0] + closest_clusters[1][0]) / 2
            goal_y = (closest_clusters[0][1] + closest_clusters[1][1]) / 2
        else:
            # 감지된 콘이 없는 경우
            goal_x = self.carPosX + fixed_goal_distance * math.cos(self.carPosYaw)
            goal_y = self.carPosY + fixed_goal_distance * math.sin(self.carPosYaw)

        # 목표 지점을 Rviz에 시각화
        self.publishGoalMarker(goal_x, goal_y)

        rrtTargets = [(goal_x, goal_y, 0.0)]

        start = [self.carPosX, self.carPosY, self.carPosYaw]
        iterationNumber = 1000
        planDistance = 6.0
        expandDistance = 0.6  # 확장 거리 축소
        expandAngle = 40

        rrt = ma_rrt_oa.RRT(
            start, 
            planDistance, 
            obstacleList=coneObstacleList, 
            expandDis=expandDistance, 
            turnAngle=expandAngle, 
            maxIter=iterationNumber,
            rrtTargets=rrtTargets
        )

        nodeList, leafNodes = rrt.Planning()

        self.publishTreeVisual(nodeList, leafNodes)

        bestBranch = self.findBestBranch(leafNodes, nodeList, coneObstacleSize, expandDistance, planDistance)

        if bestBranch:
            newWaypoints = [(node.x, node.y) for node in bestBranch]
            self.mergeWaypoints(newWaypoints)
            self.publishWaypoints(newWaypoints)

        



    def publishGoalMarker(self, goal_x, goal_y):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.goalMarkerPub.publish(marker)


    def mergeWaypoints(self, newWaypoints):
        # print "mergeWaypoints:", "len(saved):", len(self.savedWaypoints), "len(new):", len(newWaypoints)
            if not newWaypoints:
                return

            maxDistToSaveWaypoints = 1.8
            maxWaypointAmountToSave = 5
            waypointsDistTollerance = 100

        # check preliminary loopClosure
            if len(self.savedWaypoints) > 15:
                firstSavedWaypoint = self.savedWaypoints[0]

                for waypoint in reversed(newWaypoints):
                    distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypoint[0], waypoint[1])
                    if distDiff < waypointsDistTollerance:
                        self.preliminaryLoopClosure = False
                    #print ("preliminaryLoopClosure = True")
                        break

        # print "savedWaypoints before:", self.savedWaypoints
        # print "newWaypoints:", newWaypoints

            newSavedPoints = []

            for i in range(len(newWaypoints)):
                waypointCandidate = newWaypoints[i]

                carWaypointDist = self.dist(self.carPosX, self.carPosY, waypointCandidate[0], waypointCandidate[1])
            # print "check candidate:", waypointCandidate, "with dist:", carWaypointDist

                if i >= maxWaypointAmountToSave or carWaypointDist > maxDistToSaveWaypoints:
                # print "condition to break:", i, i >= maxWaypointAmountToSave,  "or", (carWaypointDist > maxDistToSaveWaypoints)
                    break
                else:
                    for savedWaypoint in reversed(self.savedWaypoints):
                        waypointsDistDiff = self.dist(savedWaypoint[0], savedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                        if waypointsDistDiff < waypointsDistTollerance:
                            self.savedWaypoints.remove(savedWaypoint) #remove similar
                        # print "remove this point:", savedWaypoint, "with diff:", waypointsDistDiff
                            break

                    if (self.preliminaryLoopClosure):
                        distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                        if distDiff < waypointsDistTollerance:
                            self.loopClosure = False
                            print ("loopClosure = True")
                            break

                # print "add this point:", waypointCandidate
                self.savedWaypoints.append(waypointCandidate)
                newSavedPoints.append(waypointCandidate)

            if newSavedPoints: # make self.savedWaypoints and newWaypoints having no intersection
                for point in newSavedPoints:
                    newWaypoints.remove(point)

        # print "savedWaypoints after:", self.savedWaypoints
        # print "newWaypoints after:", newWaypoints

    
    
    def dist(self, x1, y1, x2, y2, shouldSqrt = True):
        distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
        return math.sqrt(distSq) if shouldSqrt else distSq

    def publishWaypoints(self, newWaypoints = None):
        if (time.time() - self.lastPublishWaypointsTime) < self.waypointsPublishInterval:
            return

        # print "publishWaypoints(): start"
        waypointsArray = WaypointsArray()
        newwaypointsArray = WaypointsArray()
        waypointsArray.header.frame_id = self.world_frame
        waypointsArray.header.stamp = rospy.Time.now()
        newwaypointsArray.header.frame_id = self.world_frame
        newwaypointsArray.header.stamp = rospy.Time.now()



        # if not self.savedWaypoints and newWaypoints:
        #     firstWaypoint = newWaypoints[0]
        #
        #     auxWaypointMaxDist = 2
        #
        #     # auxilary waypoint to start
        #     if self.dist(self.carPosX, self.carPosY, firstWaypoint[0], firstWaypoint[1]) > auxWaypointMaxDist:
        #         waypointsArray.waypoints.append(Waypoint(0, self.carPosX, self.carPosY))
        #         print "add aux point with car pos"

        for i in range(len(self.savedWaypoints)):
            waypoint = self.savedWaypoints[i]
            waypointId = len(waypointsArray.waypoints)
            w = Waypoint(waypoint[0], waypoint[1], waypointId)
            waypointsArray.waypoints.append(w)

        if newWaypoints is not None:
            for i in range(len(newWaypoints)):
                waypoint = newWaypoints[i]
                waypointId = len(waypointsArray.waypoints)
                w = Waypoint(waypoint[0], waypoint[1], waypointId)
                waypointsArray.waypoints.append(w)
                newwaypointsArray.waypoints.append(w)
                # print "added from newWaypoints:", waypointId, waypoint[0], waypoint[1]

        if self.shouldPublishWaypoints:
            # print "publish ros waypoints:", waypointsArray.waypoints
            self.waypointsPub.publish(waypointsArray)
            self.newwaypointsPub.publish(newwaypointsArray)
            self.lastPublishWaypointsTime = time.time()

            self.publishWaypointsVisuals(newWaypoints)

            # print "publishWaypoints(): len(waypointsArray.waypoints):", len(waypointsArray.waypoints)
            # print "------"

    def publishWaypointsVisuals(self, newWaypoints=None):
        markerArray = MarkerArray()

        if newWaypoints:
            newWaypointsMarker = Marker()
            newWaypointsMarker.header.frame_id = self.world_frame
            newWaypointsMarker.header.stamp = rospy.Time.now()
            newWaypointsMarker.lifetime = rospy.Duration(1)
            newWaypointsMarker.ns = "new-publishWaypointsVisuals"
            newWaypointsMarker.id = 2

            newWaypointsMarker.type = newWaypointsMarker.SPHERE_LIST
            newWaypointsMarker.action = newWaypointsMarker.ADD
            newWaypointsMarker.pose.orientation.w = 1
            newWaypointsMarker.scale.x = 0.3
            newWaypointsMarker.scale.y = 0.3
            newWaypointsMarker.scale.z = 0.3

            newWaypointsMarker.color.a = 0.65
            newWaypointsMarker.color.b = 1.0

            for waypoint in newWaypoints:
                p = Point(waypoint[0], waypoint[1], 0.0)
                newWaypointsMarker.points.append(p)

        markerArray.markers.append(newWaypointsMarker)

        self.waypointsVisualPub.publish(markerArray)

    def getLineIntersection(self, a1, a2, b1, b2):
        """
        Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
        a1: [x, y] a point on the first line
        a2: [x, y] another point on the first line
        b1: [x, y] a point on the second line
        b2: [x, y] another point on the second line
        https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
        """
        s = np.vstack([a1,a2,b1,b2])        # s for stacked
        h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
        l1 = np.cross(h[0], h[1])           # get first line
        l2 = np.cross(h[2], h[3])           # get second line
        x, y, z = np.cross(l1, l2)          # point of intersection
        if z == 0:                          # lines are parallel
            return (float('inf'), float('inf'))
        return (x/z, y/z)

    def getLineSegmentIntersection(self, a1, a2, b1, b2):
        # https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
        # Return true if line segments a1a2 and b1b2 intersect
        # return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
        return self.ccw(a1,b1,b2) != self.ccw(a2,b1,b2) and self.ccw(a1,a2,b1) != self.ccw(a1,a2,b2)

    def ccw(self, A, B, C):
        # if three points are listed in a counterclockwise order.
        # return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    def getFilteredBestBranch(self, bestBranch):
        if not bestBranch:
            return

        everyPointDistChangeLimit = 2.0
        newPointFilter = 0.2
        maxDiscardAmountForReset = 2

        if not self.filteredBestBranch:
            self.filteredBestBranch = list(bestBranch)
        else:
            changeRate = 0
            shouldDiscard = False
            for i in range(len(bestBranch)):
                node = bestBranch[i]
                filteredNode = self.filteredBestBranch[i]

                dist = math.sqrt((node.x - filteredNode.x) ** 2 + (node.y - filteredNode.y) ** 2)
                if dist > everyPointDistChangeLimit: # changed too much, skip this branch
                    shouldDiscard = True
                    self.discardAmount += 1
                    # print "above DistChangeLimit:, shouldDiscard!,", "discAmount:", self.discardAmount

                    if self.discardAmount >= maxDiscardAmountForReset:
                        self.discardAmount = 0
                        self.filteredBestBranch = list(bestBranch)
                        # print "broke maxDiscardAmountForReset:, Reset!"
                    break

                changeRate += (everyPointDistChangeLimit - dist)
            # print "branch changeRate: {0}".format(changeRate);

            if not shouldDiscard:
            #     return
            # else:
                for i in range(len(bestBranch)):
                    self.filteredBestBranch[i].x = self.filteredBestBranch[i].x * (1 - newPointFilter) + newPointFilter * bestBranch[i].x
                    self.filteredBestBranch[i].y = self.filteredBestBranch[i].y * (1 - newPointFilter) + newPointFilter * bestBranch[i].y

                self.discardAmount = 0
                # print "reset discardAmount, ", "discAmount:", self.discardAmount

        self.publishFilteredBranchVisual()
        return list(self.filteredBestBranch) # return copy

    

    def findBestBranch(self, leafNodes, nodeList, coneObstacleSize, expandDistance, planDistance):
        if not leafNodes:
            return

        minCost = float('inf')
        bestLeaf = None

    # 가장 비용이 적은 리프 노드 선택
        for leaf in leafNodes:
            if leaf.cost < minCost:
                minCost = leaf.cost
                bestLeaf = leaf

        if not bestLeaf:
            return []

    # 최적의 리프 노드에서 루트까지의 경로 반환
        path = []
        node = bestLeaf
        while node.parent is not None:
            path.append(node)
            node = nodeList[node.parent]
        path.append(node)

        return path[::-1]  # 경로를 뒤집어서 반환


    def isLeftCone(self, node, parentNode, cone):
        # //((b.X - a.X)*(cone.Y - a.Y) - (b.Y - a.Y)*(cone.X - a.X)) > 0;
        return ((node.x - parentNode.x) * (cone.y - parentNode.y) - (node.y - parentNode.y) * (cone.x - parentNode.x)) > 0;

    def publishBestBranchVisual(self, nodeList, leafNode):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "publishBestBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1

        marker.color.a = 0.7
        marker.color.r = 1.0

        node = leafNode

        parentNodeInd = node.parent
        while parentNodeInd is not None:
            parentNode = nodeList[parentNodeInd]
            p = Point(node.x, node.y, 0)
            marker.points.append(p)

            p = Point(parentNode.x, parentNode.y, 0)
            marker.points.append(p)

            parentNodeInd = node.parent
            node = parentNode

        self.bestBranchVisualPub.publish(marker)

    def publishFilteredBranchVisual(self):

        if not self.filteredBestBranch:
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "publisshFilteredBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1

        marker.color.a = 0.7
        marker.color.b = 1.0

        for i in range(len(self.filteredBestBranch)):
            node = self.filteredBestBranch[i]
            p = Point(node.x, node.y, 0)
            if i != 0:
                marker.points.append(p)

            if i != len(self.filteredBestBranch) - 1:
                marker.points.append(p)

        self.filteredBranchVisualPub.publish(marker)

    def publishTreeVisual(self, nodeList, leafNodes):

        if not nodeList and not leafNodes:
            return

        markerArray = MarkerArray()

        # tree lines marker
        treeMarker = Marker()
        treeMarker.header.frame_id = self.world_frame
        treeMarker.header.stamp = rospy.Time.now()
        treeMarker.ns = "rrt"

        treeMarker.type = treeMarker.LINE_LIST
        treeMarker.action = treeMarker.ADD
        treeMarker.scale.x = 0.03

        treeMarker.pose.orientation.w = 1

        treeMarker.color.a = 0.7
        treeMarker.color.g = 0.7

        treeMarker.lifetime = rospy.Duration(0.2)

        for node in nodeList:
            if node.parent is not None:
                p = Point(node.x, node.y, 0)
                treeMarker.points.append(p)

                p = Point(nodeList[node.parent].x, nodeList[node.parent].y, 0)
                treeMarker.points.append(p)

        markerArray.markers.append(treeMarker)

        # leaves nodes marker
        leavesMarker = Marker()
        leavesMarker.header.frame_id = self.world_frame
        leavesMarker.header.stamp = rospy.Time.now()
        leavesMarker.lifetime = rospy.Duration(0.2)
        leavesMarker.ns = "rrt-leaves"

        leavesMarker.type = leavesMarker.SPHERE_LIST
        leavesMarker.action = leavesMarker.ADD
        leavesMarker.pose.orientation.w = 1
        leavesMarker.scale.x = 0.15
        leavesMarker.scale.y = 0.15
        leavesMarker.scale.z = 0.15

        leavesMarker.color.a = 0.5
        leavesMarker.color.b = 0.1

        for node in leafNodes:
            p = Point(node.x, node.y, 0)
            leavesMarker.points.append(p)

        markerArray.markers.append(leavesMarker)

        # publis marker array
        self.treeVisualPub.publish(markerArray)

    def getFrontConeObstacles(self, map, frontDist):
        if not map:
            return []

        headingVector = self.getHeadingVector()
        # print("headingVector:", headingVector)

        headingVectorOrt = [-headingVector[1], headingVector[0]]
        # print("headingVectorOrt:", headingVectorOrt)

        behindDist = 1.0
        carPosBehindPoint = [self.carPosX - behindDist * headingVector[0], self.carPosY - behindDist * headingVector[1]]

        # print "carPos:", [self.carPosX, self.carPosY]
        # print "carPosBehindPoint:", carPosBehindPoint

        frontDistSq = frontDist ** 2

        frontConeList = []
        for cone in map:
            if (headingVectorOrt[0] * (cone.y - carPosBehindPoint[1]) - headingVectorOrt[1] * (cone.x - carPosBehindPoint[0])) < 0:
                if ((cone.x) ** 2 + (cone.y) ** 2) < frontDistSq:
                    frontConeList.append(cone)
        return frontConeList

    def getHeadingVector(self):
        headingVector = [1.0, 0]
        carRotMat = np.array([[math.cos(self.carPosYaw), -math.sin(self.carPosYaw)], [math.sin(self.carPosYaw), math.cos(self.carPosYaw)]])
        #NEU에 맞춰서 수정할 필요 있음!
        headingVector = np.dot(carRotMat, headingVector)
        return headingVector

    def getConesInRadius(self, map, x, y, radius):
        coneList = []
        radiusSq = radius * radius
        for cone in map:
            if ((cone.x - x) ** 2 + (cone.y - y) ** 2) < radiusSq:
                coneList.append(cone)
        return coneList



    def getMiddlePoint(self):
        return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    def length(self):
        return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def getPartsLengthRatio(self):
        import math

        part1Length = math.sqrt((self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
        part2Length = math.sqrt((self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)

    def __eq__(self, other):
        return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
             or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

    def __str__(self):
        return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1,2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2,2)) + ")"

    def __repr__(self):
        return str(self)

if __name__ == '__main__':

    a1 = np.array([0, 0])
    a2 = np.array([5, 0])
    b1 = np.array([0, 5])
    b2 = np.array([5, 0])

    maNode = MaRRTPathPlanNode()

    if maNode.getLineSegmentIntersection(a1, a2, b1, b2):
        print ("intersected")
    else:
        print ("not intersected")