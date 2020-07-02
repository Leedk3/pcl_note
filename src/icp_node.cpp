// Includes listed here
#include <ros/ros.h>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <random>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

//PCL library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

class ICP_EXAMPLE
{
    public:
        ICP_EXAMPLE(ros::NodeHandle& nh);        
        ~ICP_EXAMPLE();
        void allocateMemory();
        void resetParameters();
        void ConvertDataToPointcloud();
        void MapPointcloud();
        void icpMatching();
        void loopClosureThread();
        void run();
        void fixed_frame_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
        
        
    private:        
        ros::NodeHandle nh;
        ros::Publisher pubInputData;
        ros::Publisher pubMapData;
        ros::Publisher pubIcpKeyFrames;

        ros::Subscriber subLaneMap;
        pcl::PointCloud<pcl::PointXYZ>::Ptr InputData; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr MapData;

        const float historyKeyframeFitnessScore = 0.3;

        std::mutex mtx;


};

ICP_EXAMPLE::ICP_EXAMPLE(ros::NodeHandle& nh)
{
  // Create ROS subscriber for fixed_frame topic
  subLaneMap = nh.subscribe("/fake_vector_map", 1, &ICP_EXAMPLE::fixed_frame_cb, this);

  // // Create ROS publisher for transformed pointcloud
  pubInputData = nh.advertise<sensor_msgs::PointCloud2>("/ICP_example/InputData",1);
  pubMapData = nh.advertise<sensor_msgs::PointCloud2>("/ICP_example/MapData",1);
  pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/ICP_example/AfterICP",1);
  allocateMemory();
};

ICP_EXAMPLE::~ICP_EXAMPLE() {    
}

void ICP_EXAMPLE::allocateMemory()
{
  InputData.reset(new pcl::PointCloud<pcl::PointXYZ>());
  InputData->points.resize(10 * 2);

  MapData.reset(new pcl::PointCloud<pcl::PointXYZ>());
  MapData->points.resize(10 * 2);
}

void ICP_EXAMPLE::ConvertDataToPointcloud()
{
  std::vector<geometry_msgs::Point> Data;
  InputData->clear();
  Data.clear();
  geometry_msgs::Point DataBuf;
  for (int i = 0; i < 10; i++)
  {
    DataBuf.x = 3;
    DataBuf.y = i;
    Data.push_back(DataBuf);
    DataBuf.x = -3;
    DataBuf.y = i;
    Data.push_back(DataBuf);
  }

  pcl::PointXYZ PCLPoint;
  size_t cloudSize, index; 
  cloudSize = Data.size();

  for (size_t i = 0; i < cloudSize; ++i)
  {
    PCLPoint.x = Data.at(i).x;
    PCLPoint.y = Data.at(i).y;
    PCLPoint.z = Data.at(i).z;
    index = i;
    InputData->push_back(PCLPoint);
  }


  sensor_msgs::PointCloud2 laserCloudTemp;
  pcl::toROSMsg(*InputData, laserCloudTemp);
  laserCloudTemp.header.stamp = ros::Time::now();
  laserCloudTemp.header.frame_id = "base_link";
  pubInputData.publish(laserCloudTemp);
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void ICP_EXAMPLE::MapPointcloud()
{
  std::vector<geometry_msgs::Point> Data;
  MapData->clear();
  Data.clear();
  geometry_msgs::Point DataBuf;
  // std::cout << "rand : " << fRand(2, -2) << std::endl;
  double rand = fRand(2, -2);
  for (int i = 0; i < 10; i++)
  {
    DataBuf.x = 1 + rand;
    DataBuf.y = i ;
    Data.push_back(DataBuf);
    DataBuf.x = -5 + rand;
    DataBuf.y = i ;
    Data.push_back(DataBuf);
  }

  pcl::PointXYZ PCLPoint;
  size_t cloudSize, index; 
  cloudSize = Data.size();

  for (size_t i = 0; i < cloudSize; ++i)
  {
    PCLPoint.x = Data.at(i).x;
    PCLPoint.y = Data.at(i).y;
    PCLPoint.z = Data.at(i).z;
    index = i;
    MapData->push_back(PCLPoint);
  }

  sensor_msgs::PointCloud2 laserCloudTemp;
  pcl::toROSMsg(*MapData, laserCloudTemp);
  laserCloudTemp.header.stamp = ros::Time::now();
  laserCloudTemp.header.frame_id = "base_link";
  pubMapData.publish(laserCloudTemp);
}

void ICP_EXAMPLE::icpMatching()
{
  std::lock_guard<std::mutex> lock(mtx);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance(20);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  // Align clouds
  icp.setInputSource(InputData);
  icp.setInputTarget(MapData);
  pcl::PointCloud<pcl::PointXYZ>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZ>());
  icp.align(*unused_result);
  
  
  if (icp.hasConverged() || icp.getFitnessScore() < historyKeyframeFitnessScore)
  {
      std::cout << "ICP converged." << std::endl
                << "The score is " << icp.getFitnessScore() << std::endl;
      std::cout << "Transformation matrix:" << std::endl;
      std::cout << icp.getFinalTransformation() << std::endl;
  }
  else 
  {
    std::cout << "ICP did not converge." << std::endl;
    std::cout << "icp score: "<< icp.getFitnessScore() << std::endl;
    return;
  }

  // publish corrected cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr closed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud (*InputData, *closed_cloud, icp.getFinalTransformation());
  sensor_msgs::PointCloud2 cloudMsgTemp;
  pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
  cloudMsgTemp.header.stamp = ros::Time().now();
  cloudMsgTemp.header.frame_id = "/base_link";
  pubIcpKeyFrames.publish(cloudMsgTemp);

  // mtx.unlock();


}

void ICP_EXAMPLE::loopClosureThread(){

    // if (loopClosureEnableFlag == false)
    //     return;
    ros::Rate rate(5);
    while (ros::ok()){
        rate.sleep();
        ICP_EXAMPLE::icpMatching();
    }
}

void ICP_EXAMPLE::resetParameters()
{
  InputData->clear();
  MapData->clear();
}

void ICP_EXAMPLE::run()
{
  // init();
  ConvertDataToPointcloud();
  MapPointcloud();
  // icpMatching();
  // resetParameters();
  
}

void ICP_EXAMPLE::fixed_frame_cb (const sensor_msgs::PointCloud2ConstPtr& msg) 
{
 
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_in(
  //                          new pcl::PointCloud<pcl::PointXYZ>
  //                          );
  // pcl::fromROSMsg ( *next_pc2_msg, *cloud2_in );

  // // Perform ICP
  // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // icp.setInputCloud(cloud2_in);
  // icp.setInputTarget(cloud_in);
  // pcl::PointCloud<pcl::PointXYZ> Final;
  // icp.align(Final);

  // // Convert the pcl/PointCloud to sensor_msgs/PointCloud2
  // sensor_msgs::PointCloud2 output;
  // pcl::toROSMsg( *cloud2_in, output );
  // // Publish the results
  // _pub.publish( output );
  // ConvertDataToPointcloud();
  // resetParameters();
}




int main (int argc, char** argv) {

  // Initialize ROS
  ros::init (argc, argv, "icp_example");
  ros::NodeHandle nh("~");  
  ROS_INFO("start");
  ros::Rate loop_rate(1);
  ICP_EXAMPLE test(nh);

  std::thread loopthread(&ICP_EXAMPLE::loopClosureThread, &test);

  while(ros::ok()){
    test.run();
    loop_rate.sleep();
    ros::spinOnce();
  }

  loopthread.join();
  return 0;
  
}