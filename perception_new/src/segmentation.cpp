#include "perception_new/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/filter_indices.h"

#include "pcl/common/common.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

#include "pcl/segmentation/extract_clusters.h"

#include "geometry_msgs/Pose.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"

#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"


// I want to test color based segmentation
// #include "pcl/segmentation/region_growing_rgb.h"
// #include "pcl/visualization/cloud_viewer.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception_new {

void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coeff) {
    double distance_above_plane;
    ros::param::param("distance_above_plane", distance_above_plane, 0.005);

    pcl::PointIndices indices_internal;
    pcl::SACSegmentation<PointC> seg;
    seg.setOptimizeCoefficients(true);
    // Search for a plane perpendicular to some axis (specified below).
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    // Set the distance to the plane for a point to be an inlier.
    seg.setDistanceThreshold(distance_above_plane);
    seg.setInputCloud(cloud);

    // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
    Eigen::Vector3f axis;
    axis << 0, 0, 1;
    seg.setAxis(axis);
    seg.setEpsAngle(pcl::deg2rad(10.0));

    // coeff contains the coefficients of the plane:
    // ax + by + cz + d = 0
    //pcl::ModelCoefficients coeff;
    seg.segment(indices_internal, *coeff);

    // Build custom indices that ignores points above the plane.
    for (size_t i = 0; i < cloud->size(); ++i) {
        const PointC& pt = cloud->points[i];
        float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
            coeff->values[2] * pt.z + coeff->values[3];
        if (val <= distance_above_plane) {
            indices->indices.push_back(i);
        }  
        // We could add a "below" cutoff, depending on how we want to box the table
    }

    // If we just want to use the points from the seg.segment method without any special filtering,
    // use this line
    // *indices = indices_internal;

    if (indices->indices.size() == 0) {
        ROS_ERROR("Unable to find surface.");
        return;
    }
}

// Computes the axis-aligned bounding box of a point cloud.
//
// Args:
//  cloud: The point cloud
//  pose: The output pose. Because this is axis-aligned, the orientation is just
//    the identity. The position refers to the center of the box.
//  dimensions: The output dimensions, in meters.
void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
    PointC min_pcl;
    PointC max_pcl;
    pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);
    float centerX = (max_pcl.x + min_pcl.x) / 2;
    float centerY = (max_pcl.y + min_pcl.y) / 2;
    float centerZ = (max_pcl.z + min_pcl.z) / 2;
    float DimX = (max_pcl.x - min_pcl.x);
    float DimY = (max_pcl.y - min_pcl.y);
    float DimZ = (max_pcl.z - min_pcl.z);

    pose->position.x = centerX;
    pose->position.y = centerY;
    pose->position.z = centerZ;

    dimensions->x = DimX;
    dimensions->y = DimY;
    dimensions->z = DimZ;
}

void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices) {

    // These lines filter out the indices above the surface. However, they did not seem to work for some reason.
    // So, we pass in a cloud that is already made up of indices above the surface
                        
    // pcl::ExtractIndices<PointC> extract;
    // pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
    // extract.setInputCloud(cloud);
    // extract.setIndices(surface_indices);
    // extract.setNegative(true);
    // extract.filter(above_surface_indices->indices);

    // ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

    double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
    ros::param::param("ec_min_cluster_size", min_cluster_size, 2000);
    ros::param::param("ec_max_cluster_size", max_cluster_size, 13000);
    
    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(cloud);
    //euclid.setIndices(above_surface_indices);
    euclid.setClusterTolerance(cluster_tolerance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);

    // Remove any NAN points from the cloud, so it can extract without crashing
    std::vector<int> nan_inds;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nan_inds);
    // ROS_INFO("Nan vector size: %ld .", nan_inds.size());

    euclid.extract(*object_indices);

    // Find the size of the smallest and the largest object,
    // where size = number of points in the cluster
    size_t min_size = std::numeric_limits<size_t>::max();
    size_t max_size = std::numeric_limits<size_t>::min();
    for (size_t i = 0; i < object_indices->size(); ++i) {
        size_t cluster_size = object_indices->at(i).indices.size();
        if (cluster_size > max_size) {
            max_size = cluster_size;
        }
        if (cluster_size < min_size) {
            min_size = cluster_size;
        }
    }

    ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
         object_indices->size(), min_size, max_size);

}

Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub, const ros::Publisher& above_surface_pub)
    : surface_points_pub_(surface_points_pub), marker_pub_(marker_pub), above_surface_pub_(above_surface_pub) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {

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

    ROS_INFO("Here3");

    sensor_msgs::PointCloud2 msg_2;                                                   
    pcl_ros::transformPointCloud("base_link", transform, msg, msg_2);

    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud);

    // Get the indices of the surface (table) given the point cloud
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    SegmentSurface(cloud, table_inliers, coeff); 

    // Given these data types:
    PointCloudC::Ptr original_cloud(cloud);
    PointCloudC::Ptr subset_cloud(new PointCloudC);

    // Extract subset of original_cloud into subset_cloud:
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(original_cloud);
    extract.setIndices(table_inliers);
    extract.filter(*subset_cloud);

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*subset_cloud, msg_out);
    surface_points_pub_.publish(msg_out);

    // Create the box marker for the segmented cloud
    visualization_msgs::Marker table_marker;
    table_marker.ns = "table";
    table_marker.header.frame_id = "base_link";
    table_marker.type = visualization_msgs::Marker::CUBE;
    //GetAxisAlignedBoundingBox(subset_cloud, &table_marker.pose, &table_marker.scale);
    table_marker.color.r = 1;
    table_marker.color.a = 0.8;
    //marker_pub_.publish(table_marker);

    // This is another way to get the shape around a point cloud, instead of using a
    // axis aligned box
    PointCloudC::Ptr extract_out(new PointCloudC());
    shape_msgs::SolidPrimitive shape;
    geometry_msgs::Pose table_pose;
    simple_grasping::extractShape(*subset_cloud, coeff, *extract_out, shape,
                                table_pose);
    table_marker.pose = table_pose;
    if (shape.type == shape_msgs::SolidPrimitive::BOX) {
        double y_table_offset;
        ros::param::param("y_table_offset", y_table_offset, 0.0);

        table_marker.scale.x = shape.dimensions[0];
        table_marker.scale.y = shape.dimensions[1] + y_table_offset;
        table_marker.scale.z = shape.dimensions[2];
    }
    table_marker.pose.position.z -= table_marker.scale.z;
    marker_pub_.publish(table_marker);

    // Create a cloud for the objects above the table
    PointCloudC::Ptr cloud_out(new PointCloudC);
    extract.setNegative(true);
    extract.filter(*cloud_out);
    sensor_msgs::PointCloud2 cloud_out_msg;
    pcl::toROSMsg(*cloud_out, cloud_out_msg);
    above_surface_pub_.publish(cloud_out_msg);

    // Segment the objects above the table
    std::vector<pcl::PointIndices> object_indices;
    SegmentSurfaceObjects(cloud_out, table_inliers, &object_indices);

    for (size_t i = 0; i < object_indices.size(); ++i) {
        // Reify indices into a point cloud of the object.
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);

        *indices = object_indices[i];
        PointCloudC::Ptr object_cloud(new PointCloudC());
        // object_cloud using indices
        extract.setInputCloud(cloud_out); // cloud_out
        extract.setNegative(false);
        extract.setIndices(indices);
        extract.filter(*object_cloud);
        // Publish a bounding box around it.
        visualization_msgs::Marker object_marker;
        object_marker.ns = "objects";
        object_marker.id = i;
        object_marker.header.frame_id = "base_link";
        object_marker.type = visualization_msgs::Marker::CUBE;
        GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                                    &object_marker.scale);

        // Getting the pose of where to grab the book on the spine
        geometry_msgs::Pose spine_pose;
        spine_pose.position.x = object_marker.pose.position.x;
        spine_pose.position.y = object_marker.pose.position.y;
        spine_pose.position.z = object_marker.pose.position.z;

        float x_offset = object_marker.scale.x / 2.0;
        spine_pose.position.x -= x_offset;

        visualization_msgs::Marker spine_marker;
        spine_marker.header.frame_id = "base_link";
        spine_marker.type = visualization_msgs::Marker::CUBE;
        spine_marker.pose = spine_pose;

        spine_marker.scale.x = 0.05;
        spine_marker.scale.y = 0.05;
        spine_marker.scale.z = 0.05;

        spine_marker.color.b = 1;
        spine_marker.color.a = 0.3;
        if (i < 1) {
            marker_pub_.publish(spine_marker);

            ROS_INFO("Pose x %f, y %f, and z %f",
            spine_pose.position.x, spine_pose.position.y, spine_pose.position.z);
        }


        object_marker.color.g = 1;
        object_marker.color.a = 0.3;
        marker_pub_.publish(object_marker);
    }

    // Testing color based segmentation
    // pcl::RegionGrowingRGB<PointC> reg;
    // reg.setInputCloud (cloud);
    // reg.setDistanceThreshold (10);
    // reg.setPointColorThreshold (5);
    // reg.setRegionColorThreshold (3);
    // reg.setMinClusterSize (600);

    // std::vector <pcl::PointIndices> clusters;
    // reg.extract (clusters);

    // PointCloudC::Ptr colored_cloud = reg.getColoredCloud ();

    // sensor_msgs::PointCloud2 color_msg;
    // pcl::toROSMsg(*colored_cloud, color_msg);
    // //above_surface_pub_.publish(color_msg);

    // // For looking at the color seperated cloud
    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    // viewer.showCloud (colored_cloud);
    // while (!viewer.wasStopped ())
    // {
    //     boost::this_thread::sleep (boost::posix_time::microseconds (100));
    // }


}
}  // namespace perception_new