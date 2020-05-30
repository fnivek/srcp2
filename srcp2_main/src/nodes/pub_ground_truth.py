#!/usr/bin/python
import rospy
import tf
from gazebo_msgs.msg import ModelStates


class GtPub(object):
    def __init__(self):
        self._robots = []
        self._tf_bc = tf.TransformBroadcaster()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_cb)

    def get_robots(self):
        topics = rospy.get_published_topics()
        topics = filter(lambda topic: topic[0].find('imu', -3) != -1, topics)
        self._robots = [topic.split('/')[1] for topic, _ in topics]

    def model_states_cb(self, msg):
        for index, name in enumerate(msg.name):
            child_frame = name + '_tf'
            if name in self._robots:
                child_frame += '/base_footprint'
            pose = msg.pose[index]
            self._tf_bc.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                                      (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                                      rospy.Time(),
                                      child_frame,
                                      'gt_world')


if __name__ == '__main__':
    rospy.init_node('pub_ground_truth')

    gt_pub = GtPub()
    gt_pub.get_robots()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        gt_pub.get_robots()
        rate.sleep()
