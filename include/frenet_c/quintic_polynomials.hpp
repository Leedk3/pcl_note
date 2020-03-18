#include <ros/ros.h>
// #include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <math.h>
#include <iostream>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <opencv/cv.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
// #include <gtsam/nonlinear/Values.h>

using namespace cv;
using namespace std;

class POLY_FIT
{
    public:
        POLY_FIT(ros::NodeHandle& n);        
        ~POLY_FIT();
        void main_seqeunce();

        // ROS_CALLBACK
        void wpt_poses_callback(const geometry_msgs::PoseArrayConstPtr& msg);
        void ekf_odom_callback(const nav_msgs::OdometryConstPtr& msg);
        
    private:        
        // initialize variables
        ros::NodeHandle nh;
        ros::Subscriber wpt_samples_sub;
        ros::Subscriber robot_pose_sub;
        ros::Publisher fitting_local_path_pub;
        // geometry_msgs::PoseArray wpt_poses;

        // initialize variables
        std::vector <std::pair<double, double>> waypointVect;
        std::vector <std::pair<double, double>> Local_waypoint;
        std::vector <std::pair<double, double>> Global_waypoint;
        std::vector<std::pair < double, double> > ::iterator iter; //init. iterator

        geometry_msgs::PoseArray waypoints;
        uint32_t current_index;
        int number_of_total_index;
        double robot_x;
        double robot_y;
        geometry_msgs::Quaternion robot_heading;
        visualization_msgs::Marker fitting_local_path_msg;

        cv::Mat polyfit(std::vector<cv::Point2f>& in_point, int n);
        cv::Mat poly_coeffi;

        void get_current_index(std::vector <std::pair<double, double>> &ref_waypoint);
        void reference_path_fitting(std::vector <std::pair<double, double>> &waypointVect);
        std::vector <std::pair<double, double>> global_to_local_conversion(std::vector <std::pair<double, double>> &GlobalVect, 
                                                                           double &robot_x, double &robot_y, geometry_msgs::Quaternion &robot_heading);
        double first_derivative_of_cubic_poly(cv::Mat& input_poly_coeffi, double &X);
        double second_derivative_of_cubic_poly(cv::Mat& input_poly_coeffi, double &X);

        void calculate_steering_cmd();

};

POLY_FIT::POLY_FIT(ros::NodeHandle& n) 
{
    ROS_DEBUG("POLY_FIT object created");
    wpt_samples_sub = n.subscribe("/PoseArray/wpt", 10, &POLY_FIT::wpt_poses_callback, this);
    robot_pose_sub = n.subscribe("/outdoor_nav/odometry/EKF_estimated", 10, &POLY_FIT::ekf_odom_callback, this);
    fitting_local_path_pub = n.advertise<visualization_msgs::Marker>("/fitting_local_path", 10);
};

POLY_FIT::~POLY_FIT() 
{    
    ROS_INFO("POLY_FIT destructor.");
}

void POLY_FIT::main_seqeunce(){

    POLY_FIT::get_current_index(Global_waypoint);
    Local_waypoint = POLY_FIT::global_to_local_conversion(Global_waypoint, robot_x, robot_y, robot_heading);

    if (Local_waypoint.size() != 0){
        POLY_FIT::reference_path_fitting(Local_waypoint);
    }   
}

void POLY_FIT::get_current_index(std::vector <std::pair<double, double>> &ref_waypoint){
    // get index number of closest distance
    std::vector<double> dist_array;
    dist_array.clear();
    double minimum_dist = 100000.0;
    uint32_t cnt = 0;
    for(std::vector < std::pair < double, double >> ::iterator iterDisp = ref_waypoint.begin(); iterDisp != ref_waypoint.end(); iterDisp++)
    {
        // ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
        double dist= sqrt(pow(robot_x - iterDisp->first, 2) + pow(robot_y - iterDisp->second, 2));
        dist_array.push_back(dist);
        
        if (dist_array[cnt] < minimum_dist){
            minimum_dist = dist_array[cnt];
            current_index = cnt;
        }
        cnt++;
    }
    ROS_INFO("Get closest index in the path -> current_index = %d, min_dist = %f, %d", current_index, minimum_dist, number_of_total_index);
    if (minimum_dist > 100){
        current_index = 0;
        // ROS_ERROR("CHECK THE LOCALIZATION. It's too far from the path");
    }
}

std::vector <std::pair<double, double>> POLY_FIT::global_to_local_conversion(std::vector <std::pair<double, double>> &GlobalVect, 
                                                                             double &robot_x, double &robot_y, geometry_msgs::Quaternion &robot_heading){
    // Global to local conversion
    std::vector <std::pair<double, double>> LocalVect;
    double Local_X = 0;
    double Local_Y = 0;
    double robot_roll, robot_pitch, robot_yaw;
    tf2::Quaternion tf_quat;
    tf2::convert(robot_heading, tf_quat);
    tf2::Matrix3x3 mat(tf_quat);
    mat.getRPY(robot_roll, robot_pitch, robot_yaw);

    if ( GlobalVect.size() != 0){
        // for (uint32_t i =0; i < GlobalVect.size(); i++){
        for(std::vector < std::pair < double, double >> ::iterator iterDisp = GlobalVect.begin(); iterDisp != GlobalVect.end(); iterDisp++){
            Local_X = (iterDisp->first - robot_x)*cos(robot_yaw) + (iterDisp->second - robot_y)*sin(robot_yaw);
            Local_Y = -(iterDisp->first - robot_x)*sin(robot_yaw) + (iterDisp->second - robot_y)*cos(robot_yaw);
            LocalVect.push_back(std::make_pair(Local_X,Local_Y));        
        }
        return LocalVect;
    }
}

void POLY_FIT::reference_path_fitting(std::vector <std::pair<double, double>> &waypointVect){
    // quintic spliner

    cv::Point2f sampled_xy;
    vector<Point2f> sampled_xy_array;
    uint32_t cut_dist_forward = 15;
    uint32_t cut_dist_back = 5;

    if (waypointVect.size() != 0){
        sampled_xy_array.clear();
        if (current_index < cut_dist_forward){
            for (uint32_t i = 0; i < cut_dist_forward ; ++i){
                sampled_xy.x = waypointVect[current_index + i].first;
                sampled_xy.y = waypointVect[current_index + i].second;
                sampled_xy_array.push_back(sampled_xy);
            }
        }
        else if(current_index >= cut_dist_back && current_index < number_of_total_index - cut_dist_forward){
            for (uint32_t i = 0; i <= cut_dist_forward ; ++i){
                sampled_xy.x = waypointVect[current_index + i - cut_dist_back].first;
                sampled_xy.y = waypointVect[current_index + i - cut_dist_back].second;
                sampled_xy_array.push_back(sampled_xy);
            }
        }
        else if(current_index >= number_of_total_index - cut_dist_forward){
            for (uint32_t i = 0; i < number_of_total_index - cut_dist_forward ; ++i){
                sampled_xy.x = waypointVect[current_index + i -1].first;
                sampled_xy.y = waypointVect[current_index + i -1].second;
                sampled_xy_array.push_back(sampled_xy);
            }
        }
    }
    int poly_order = 3;
    poly_coeffi = POLY_FIT::polyfit(sampled_xy_array, poly_order);

    geometry_msgs::Point Point_buf;
    std_msgs::ColorRGBA color_buf;
    fitting_local_path_msg.header.frame_id = "/base_footprint";
    fitting_local_path_msg.action = visualization_msgs::Marker::ADD;
    fitting_local_path_msg.type = visualization_msgs::Marker::LINE_STRIP;
    fitting_local_path_msg.scale.x = 0.3;
    fitting_local_path_msg.scale.y = 0.3;
    fitting_local_path_msg.scale.z = 0.3;
    fitting_local_path_msg.points.clear();

    fitting_local_path_msg.color.r = 0.0f;
    fitting_local_path_msg.color.g = 1.0f;
    fitting_local_path_msg.color.b = 1.0f;
    fitting_local_path_msg.color.a = 1.0f;
    for (uint32_t i = 0; i < sampled_xy_array.size(); i++){
        Point_buf.x = (double)0.5 * i - 0.5 * cut_dist_back;
        Point_buf.y = poly_coeffi.at<double>(poly_order, 0)*pow(Point_buf.x,poly_order)+poly_coeffi.at<double>(poly_order-1, 0)*pow(Point_buf.x,poly_order-1)
                      + poly_coeffi.at<double>(poly_order-2, 0)*pow(Point_buf.x,poly_order-2)+ poly_coeffi.at<double>(poly_order-3, 0); //Cubic
        color_buf.g = 1.0f;
        color_buf.a = 1.0f;
        fitting_local_path_msg.points.push_back(Point_buf);
        fitting_local_path_msg.colors.push_back(color_buf);
    }
    fitting_local_path_pub.publish(fitting_local_path_msg);
     
}

double POLY_FIT::first_derivative_of_cubic_poly(cv::Mat& input_poly_coeffi, double &X){
    double Y_dot = 3 * poly_coeffi.at<double>(3, 0)*X*X + 2 * poly_coeffi.at<double>(2, 0)* X + poly_coeffi.at<double>(1, 0);
    return Y_dot;
}

double POLY_FIT::second_derivative_of_cubic_poly(cv::Mat& input_poly_coeffi, double &X){
    double Y_dot = 6 * poly_coeffi.at<double>(3, 0)*X + 2 * poly_coeffi.at<double>(2, 0);
    return Y_dot;
}

void POLY_FIT::calculate_steering_cmd(){

}


void POLY_FIT::wpt_poses_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    waypointVect.clear();
    number_of_total_index = msg->poses.size();
    // ROS_INFO("%d", number_of_total_index);
    for (int i = 0; i < number_of_total_index ; i++){
        double x_utm_i = msg->poses.at(i).position.x;
        double y_utm_i = msg->poses.at(i).position.y;
        waypointVect.push_back(std::make_pair(x_utm_i, y_utm_i));
    }

    Global_waypoint.clear();
	Global_waypoint.assign( waypointVect.begin(), waypointVect.end());    

}

void POLY_FIT::ekf_odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    //data subscribe
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    robot_heading = msg ->pose.pose.orientation;
}

cv::Mat POLY_FIT::polyfit(std::vector<cv::Point2f>& in_point, int n)
{

    int size = in_point.size();

    int x_num = n + 1;

    Mat mat_u(size, x_num, CV_64F);
    Mat mat_y(size, 1, CV_64F);
    Mat mat_k(x_num, 1, CV_64F);

    if (size == 0){
        for (int i = 0; i < mat_k.rows; ++i){
            mat_k.at<double>(i,0) = 0;
        }
        return mat_k;
    }
    else{
        for (int i = 0; i < mat_u.rows; ++i)
                for (int j = 0; j < mat_u.cols; ++j)
                {
                        mat_u.at<double>(i, j) = pow(in_point[i].x, j);
                }

        for (int i = 0; i < mat_y.rows; ++i)
        {
                mat_y.at<double>(i, 0) = in_point[i].y;
        }

        mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
        
        return mat_k;
    }

}