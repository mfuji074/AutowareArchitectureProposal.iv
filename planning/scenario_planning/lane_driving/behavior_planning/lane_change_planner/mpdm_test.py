# -*- coding: utf-8 -*-
#!/usr/bin/env python
import time
import sys
import unique_id
import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from autoware_perception_msgs.msg import Semantic, Shape
from dummy_perception_publisher.msg import Object
from std_msgs.msg import Bool

def main(case=0, no_engage=False):
    rospy.init_node("go")
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    initial_twist_pub = rospy.Publisher('/initialtwist', TwistStamped, queue_size=1)
    goal_pub = rospy.Publisher('/planning/mission_planning/goal', PoseStamped, queue_size=1)
    dummycar_pub = rospy.Publisher('/simulation/dummy_perception_publisher/object_info', Object, queue_size=1)
    engage_pub = rospy.Publisher('/autoware/engage', Bool, queue_size=1)
    time.sleep(1)

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
    initial_pose.pose.pose.position.x = 81553.1
    initial_pose.pose.pose.position.y = 50222
    initial_pose.pose.pose.orientation.z = 0.777928
    initial_pose.pose.pose.orientation.w = 0.628354

    initial_twist = TwistStamped()
    initial_twist.twist.linear.x = 5.0

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.orientation.z = 0.782579
    goal_pose.pose.orientation.w = 0.622552

     ####### object 1 #######
    object_info = Object()
    # header
    object_info.header.frame_id = "map"
    object_info.header.stamp = rospy.Time.now()
    # semantic
    object_info.semantic.type = Semantic.CAR
    object_info.semantic.confidence = 1.0
    # shape
    object_info.shape.type = Shape.BOUNDING_BOX
    width = 1.8
    length = 4.0
    object_info.shape.dimensions.x = length
    object_info.shape.dimensions.y = width
    object_info.shape.dimensions.z = 2.0
    # action
    object_info.action = Object.ADD
    # id
    object_info.id = unique_id.toMsg(unique_id.fromRandom())
    # object initial state
    # pose
    object_info.initial_state.pose_covariance.pose.position.x = 81564.0
    object_info.initial_state.pose_covariance.pose.position.y = 50178.0
    #object_info.initial_state.pose_covariance.pose.position.x = 81558.0
    #object_info.initial_state.pose_covariance.pose.position.y = 50207.5
    object_info.initial_state.pose_covariance.pose.position.z = 0.0
    theta = 1.75
    quat = quaternion_from_euler(0.0, 0.0, theta)
    object_info.initial_state.pose_covariance.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    # twist
    object_info.initial_state.twist_covariance.twist.linear.x = 5.0
    object_info.initial_state.twist_covariance.twist.linear.y = 0.0
    object_info.initial_state.twist_covariance.twist.linear.z = 0.0

    ####### object 2 #######
    object_info2 = Object()
    # header
    object_info2.header.frame_id = "map"
    object_info2.header.stamp = rospy.Time.now()
    # semantic
    object_info2.semantic.type = Semantic.CAR
    object_info2.semantic.confidence = 1.0
    # shape
    object_info2.shape.type = Shape.BOUNDING_BOX
    width = 1.8
    length = 4.0
    object_info2.shape.dimensions.x = length
    object_info2.shape.dimensions.y = width
    object_info2.shape.dimensions.z = 2.0
    # action
    object_info2.action = Object.ADD
    # id
    object_info2.id = unique_id.toMsg(unique_id.fromRandom())
    object_info2.initial_state.pose_covariance.pose.position.x = 81558.0
    object_info2.initial_state.pose_covariance.pose.position.y = 50207.5
    object_info2.initial_state.pose_covariance.pose.position.z = 0.0
    theta = 1.75
    quat = quaternion_from_euler(0.0, 0.0, theta)
    object_info2.initial_state.pose_covariance.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    # twist
    object_info2.initial_state.twist_covariance.twist.linear.x = 5.0
    object_info2.initial_state.twist_covariance.twist.linear.y = 0.0
    object_info2.initial_state.twist_covariance.twist.linear.z = 0.0
    ########################################################

    if case == 0:
        #goal_pose.pose.position.x = 81535.6
        #goal_pose.pose.position.y = 50333.9
        goal_pose.pose.position.x = 81527
        goal_pose.pose.position.y = 50382.4
    elif case == 1:
        goal_pose.pose.position.x = 81527
        goal_pose.pose.position.y = 50382.4

    time.sleep(1)
    initial_pose_pub.publish(initial_pose)
    time.sleep(1)
    goal_pub.publish(goal_pose)
    #time.sleep(1)
    #initial_twist_pub.publish(initial_twist)
    time.sleep(1)
    dummycar_pub.publish(object_info)

    if not no_engage:
        time.sleep(3)
        msg = Bool(data=True)
        engage_pub.publish(msg)



    if case == 1:
        time.sleep(5)
        dummycar_pub.publish(object_info2)

if __name__ == '__main__':
    case = int(sys.argv[sys.argv.index('-r') + 1]) if '-r' in sys.argv else 0
    no_engage = True if '--no-engage' in sys.argv else False

    main(case, no_engage)
