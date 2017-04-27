#! /usr/bin/env python                                                                                 
                                                                                                       
import rospy
import tf
import geometry_msgs.msg               
                              
                       
def wait_for_time():   
    """Wait for simulated time to begin.
    """                
    while rospy.Time().now().to_sec() == 0:                                                            
        pass                                                                                           
                                                                                                       
                                                                                                       
def main():                                                                                            
    rospy.init_node('ee_pose_demo')                                                               
    wait_for_time()

    # make a tfListener
    listener = tf.TransformListener()
    rospy.sleep(rospy.Duration.from_sec(1))

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print trans, rot
        rate.sleep()

                      
                      
if __name__ == '__main__':
    main()
