#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "perception_new/downsampler.h"

#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception_new {
    Downsampler::Downsampler(const ros::Publisher& pub) : pub_(pub) {}

    void Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {

        // Transform cloud to base_link frame
        tf::TransformListener tf_listener;                                                    
        tf_listener.waitForTransform("base_link", msg.header.frame_id,                     
                                    ros::Time(0), ros::Duration(5.0));                       
        tf::StampedTransform transform;                                                       
        try {                                                                                 
        tf_listener.lookupTransform("base_link", msg.header.frame_id,                    
                                    ros::Time(0), transform);                               
        } catch (tf::LookupException& e) {                                                    
            std::cerr << e.what() << std::endl;                                                 
            //return 1;                                                                           
        } catch (tf::ExtrapolationException& e) {                                             
            std::cerr << e.what() << std::endl;                                                 
            //return 1;                                                                           
        }   

        sensor_msgs::PointCloud2 msg_2;                                                   
        pcl_ros::transformPointCloud("base_link", transform, msg, msg_2);


        PointCloudC::Ptr cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud);
        ros::NodeHandle nh;
        PointCloudC::Ptr downsampled_cloud(new PointCloudC());
        pcl::VoxelGrid<PointC> vox;
        vox.setInputCloud(cloud);
        double voxel_size;
        ros::param::param("voxel_size", voxel_size, 0.01);
        vox.setLeafSize(voxel_size, voxel_size, voxel_size);
        vox.filter(*downsampled_cloud);

        sensor_msgs::PointCloud2 msg_out;
        pcl::toROSMsg(*downsampled_cloud, msg_out);
        pub_.publish(msg_out);
    }

};