#include "ros/ros.h"
#include "perception_new/crop.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception_new/segmentation.h"

// Pcl imports
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "pcl/filters/extract_indices.h"
#include "pcl/filters/filter_indices.h"

// Service imports
#include "perception_new/GetSpineLocations.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

// Global variable for the current_cropped cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud;

void update_cloud(const sensor_msgs::PointCloud2& msg) {
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud);
    current_cloud = cloud;
}

bool segment_and_get_poses(perception_new::GetSpineLocations::Request &req, perception_new::GetSpineLocations::Response &res) {
    PointCloudC::Ptr cloud = current_cloud;

    // Step 1: Segment out the table

    // Get the indices of the surface (table) given the point cloud
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    perception_new::SegmentSurface(cloud, table_inliers, coeff);

    // Given these data types:
    PointCloudC::Ptr original_cloud(cloud);
    PointCloudC::Ptr subset_cloud(new PointCloudC);

    // Extract subset of original_cloud into subset_cloud:
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(original_cloud);
    extract.setIndices(table_inliers);
    extract.filter(*subset_cloud);

    // Get the scale and points of the bounding box around the table
    geometry_msgs::Pose surface_pose;
    geometry_msgs::Vector3 surface_scale;
    perception_new::GetAxisAlignedBoundingBox(subset_cloud, &surface_pose, &surface_scale);

    // Step 2: Get the objects above the table

    // Create a cloud for the objects above the table
    PointCloudC::Ptr cloud_out(new PointCloudC);
    extract.setNegative(true);
    extract.filter(*cloud_out);

    // Step 3: Segment the objects above the table

    // Segment the objects above the table
    std::vector<pcl::PointIndices> object_indices;
    perception_new::SegmentSurfaceObjects(cloud_out, table_inliers, &object_indices);

    // Step 4: While segmenting, replace calculate the spine poses

    std::vector<geometry_msgs::Pose> spine_poses;

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
        geometry_msgs::Pose object_pose;
        geometry_msgs::Vector3 object_scale;
        perception_new::GetAxisAlignedBoundingBox(object_cloud, &object_pose, &object_scale);

        // Getting the pose of where to grab the book on the spine
        geometry_msgs::Pose spine_pose;
        spine_pose.position.x = object_pose.position.x;
        spine_pose.position.y = object_pose.position.y;
        spine_pose.position.z = object_pose.position.z;

        float x_offset = object_scale.x / 2.0;
        spine_pose.position.x -= x_offset;

        ROS_INFO("Spine Pose x %f, y %f, and z %f", spine_pose.position.x, spine_pose.position.y, spine_pose.position.z);

        spine_poses.push_back(spine_pose);
    }

    // Step 5: add the spine poses to the response and return

    res.spine_poses = spine_poses;
    res.surface_pose = surface_pose;
    res.surface_x_size = surface_scale.x;
    res.surface_y_size = surface_scale.y;
    res.surface_z_size = surface_scale.z;
    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "book_pose_finder");
    ros::NodeHandle nh;

    // Set up the cropping so we crop the point cloud
    ros::Publisher crop_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
    perception_new::Cropper cropper(crop_pub);
    ros::Subscriber crop_sub = nh.subscribe("cloud_in", 1, &perception_new::Cropper::Callback, &cropper);

    // Set up a subscriber for the cropped could, to update the current_cloud
    ros::Subscriber current_sub = nh.subscribe("cropped_cloud", 1, update_cloud);

    // Set up service callback
    ros::ServiceServer service = nh.advertiseService("get_spines", segment_and_get_poses);

    ros::spin();
    return 0;
}