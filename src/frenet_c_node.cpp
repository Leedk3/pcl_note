#include <ros/ros.h>
#include <tuple>
#include <typeinfo>
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
    // std::vector <double> x_vec{-2.5, 2.0, 4.5, 2.0, 7.5, 3.0, 1.0};
    // std::vector <double> y_vec{0.7, -6, 5, 2.5, 1.0, 5.0, -2.0};

    // // Spline spline(x_vec, y_vec);
    // Spline2D spline_2d(x_vec, y_vec);

    // double result_yaw = spline_2d.calc_yaw(1.0);
    // double result_curv = spline_2d.calc_curvature(1.0);    
    // std::pair<double, double> result_pair = spline_2d.calc_position(1.0);
    // cout << "------------------------ result : ------------------------" << endl;
    // cout << result_yaw << "  " << result_curv <<" " << result_pair.first << " " << result_pair.second << endl;

    // QuinticPolynomial test_obj(1,2,3,4,5,6,7);
    // double four = test_obj.calc_point(4);
    // double five = test_obj.calc_first_derivative(5);
    // double six = test_obj.calc_second_derivative(6);
    // double seven = test_obj.calc_third_derivative(7);

    // cout << four << endl;
    // cout << five << endl;
    // cout << six << endl;
    // cout << seven << endl;




    vector<double> wx,wy;
    wx.push_back(0.0);wx.push_back(10.0);wx.push_back(20.5);wx.push_back(35.0);wx.push_back(70.5);
    wy.push_back(0.0);wy.push_back(-6.0);wy.push_back(5.0);wy.push_back(6.5);wy.push_back(0.0);

    vector<vector<double> > ob{ {20.0, 10.0}, 
                               {30.0, 6.0}, 
                               {30.0, 8.0},
                               {35.0, 8.0},
                               {50.0, 3.0} };

    vector<double> tx;
    vector<double> ty;
    vector<double> tyaw;
    vector<double> tc;
    Spline2D csp;

    std::tie(tx,ty,tyaw,tc,csp) = generate_target_course(wx,wy);

    double c_speed = 10.0 / 3.6;
    double c_d = 2.0;
    double c_d_d = 0.0;
    double c_d_dd = 0.0;
    double s0 = 0.0;

    double area = 20.0;

    for(int sim = 0; sim < 3; sim++) {
        auto path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob);

        s0 = path.s[1];
        c_d = path.d[1];
        c_d_d = path.d_d[1];
        c_d_dd = path.d_dd[1];
        c_speed = path.s_d[1];

        cout << s0 << " " << c_d << " " << c_d_d << " " << c_d_dd << " " << c_speed << " " << endl;
    }

    

    while(ros::ok()){

        ros::spinOnce();
        r.sleep();
    }

    // ros::spinOnce();
    return 0;
}