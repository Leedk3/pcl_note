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
    ros::Rate r(5);
    vector<double> wx,wy;
    wx.push_back(0.0);wx.push_back(10.0);wx.push_back(20.5);wx.push_back(30.0);wx.push_back(70.5);
    wy.push_back(0.0);wy.push_back(-6.0);wy.push_back(5.0);wy.push_back(4.5);wy.push_back(0.0);

    vector<vector<double> > ob{ {20.0, 20.0}, 
                               {30.0, 5.0}, 
                               {30.0, 4.0},
                               {35.0, 3.0},
                               {50.0, 2.0} };

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

    

    while(ros::ok()){

        for(int sim = 0; sim < 20; sim++) {
            auto path = frenet_optimal_planning(n, csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob);
            for (int j = 0; j < path.s.size(); j++){
                cout << "path:" <<path.s.at(j) << endl;
            }

            if(path.s[0] == -1 && path.d[0] == -1.1){
                cout << "All path are canceled, no progress" << endl;
            }
            else{
                cout << "Optimal path is found, further progress" << endl;
                s0 = path.s[1];
                c_d = path.d[1];
                c_d_d = path.d_d[1];
                c_d_dd = path.d_dd[1];
                c_speed = path.s_d[1];
            }

            
            cout <<"order : " << sim << " " << s0 << " " << c_d << " " << c_d_d << " " << c_d_dd << " " << c_speed << " " << endl;
        }

        ros::spinOnce();
        r.sleep();
    }

    // ros::spinOnce();
    return 0;
}