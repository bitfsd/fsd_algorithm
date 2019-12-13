import rospy
from geometry_msgs.msg import Pose2D

class TemplateException(Exception):
    pass


class Template:
    def __init__(self):
        if not rospy.has_param('template_config'):
            raise TemplateException('No template configuration found')
        self.config = rospy.get_param('template_config')

        # initiate subscribers
        rospy.Subscriber('/localization/pose_in', Pose2D,
                         self.input_pose_callback)

    def input_pose_callback(self, msg):
        
        """
        Callback function for the vision_cone_detector topic.
        """

        rospy.loginfo('Message received: {}'.format(msg))
