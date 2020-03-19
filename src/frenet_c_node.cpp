#include <ros/ros.h>
// #include "frenet_c/quintic_polynomials.hpp"
#include "frenet_c/QuarticPolynomial.hpp"
#include "frenet_c/FrenetPath.hpp"
#include "frenet_c/cubic_spliner.hpp"
#include "frenet_c/cubic_spliner_2D.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frenet_c"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ROS_INFO("Initiated frenet_c node");
    ros::Rate r(50);

    //test input
    std::vector <double> x_vec{-2.5, 2.0, 4.5, 2.0, 7.5, 3.0, 1.0};
    std::vector <double> y_vec{0.7, -6, 5, 2.5, 1.0, 5.0, -2.0};

    // Spline spline(x_vec, y_vec);
    Spline2D spline_2d(x_vec, y_vec);

    double result_yaw = spline_2d.calc_yaw(1.0);
    double result_curv = spline_2d.calc_curvature(1.0);    
    std::pair<double, double> result_pair = spline_2d.calc_position(1.0);
    cout << "------------------------ result : ------------------------" << endl;
    cout << result_yaw << "  " << result_curv <<" " << result_pair.first << " " << result_pair.second << endl;
    
    while(ros::ok()){

        ros::spinOnce();
        r.sleep();
    }

    // ros::spinOnce();
    return 0;
}