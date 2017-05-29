#!/usr/bin/env python 

from sensor_msgs.msg import PointCloud2
import perception_new
import rospy


def wait_for_time(): 
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

    
def main():
    is_sim = True  # if reading from robot set to False                                                                           
    rospy.init_node('publish_saved_cloud')
    wait_for_time()                                                                     
    argv = rospy.myargv()
    if len(argv) < 2:
        print 'Publishes a saved point cloud to a latched topic.'
        print 'Usage: rosrun applications publish_saved_cloud.py ~/cloud.bag'
        return
    path = argv[1]
    camera = perception_new.MockCamera()
    # cloud = camera.read_cloud(path)

    if is_sim:
        cloud = camera.read_cloud(path)
        # cloud = camera.read_cloud('kitchen.bag')
    else:
        # Think this is reading from actual robot, not our bag files
        cloud = rospy.wait_for_message('head_camera/depth_registered/points', PointCloud2)

    if cloud is None:
        rospy.logerr('Could not load point cloud from {}'.format(path))
        return

    pub = rospy.Publisher('mock_point_cloud', PointCloud2, queue_size=1)       
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        cloud.header.stamp = rospy.Time.now()
        pub.publish(cloud)
        rate.sleep()                                          
    
    
if __name__ == '__main__':
    main()
