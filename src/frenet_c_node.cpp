#include <ros/ros.h>
// #include "frenet_c/quintic_polynomials.hpp"
#include "frenet_c/QuarticPolynomial.hpp"
#include "frenet_c/FrenetPath.hpp"
#include "frenet_c/cubic_spliner.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "frenet_c"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ROS_INFO("Initiated frenet_c node");
    ros::Rate r(50);

    //test input
    std::vector <double> x_vec;
    std::vector <double> y_vec;
    Spline spline(n, x_vec, y_vec);
    while(ros::ok()){

        // path_follower.main_seqeunce();
        // path_follower.reference_path_fitting();
        ros::spinOnce();
        r.sleep();
    }

    // ros::spinOnce();
    return 0;
}