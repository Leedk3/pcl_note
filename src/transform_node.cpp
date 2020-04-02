#include <ros/ros.h>
#include <string>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

ros::Publisher transformed_cloud_pub;
ros::Subscriber origin_cloud_sub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Create a container for the data.

    pcl::PointCloud<pcl::PointXYZI> pcl_buf;
    pcl::fromROSMsg(*cloud_msg, pcl_buf);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    // Do data processing here...
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // Define a translation of # meters on the x,y,z axis.
    transform.translation() << +3, 3, 10.0;
    double yaw = 1.57;
    transform.rotate (Eigen::AngleAxisf (yaw, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud (pcl_buf, *transformed_cloud, transform);

    // sensor_msgs::PointCloud2 output;
    // pcl_conversions::fromPCL(transformed_cloud, output);

    // Publish the data.
    transformed_cloud_pub.publish(transformed_cloud);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_example"); //initiate node called gps_waypoint
    ros::NodeHandle nh;
    ROS_INFO("Initiated pcl_example node");
    ros::Rate r(50);

    

    origin_cloud_sub = nh.subscribe ("velodyne_points", 1, cloud_cb);
    while(ros::ok()){

        ros::spinOnce();
        r.sleep();
    }

    // ros::spinOnce();
    return 0;
}