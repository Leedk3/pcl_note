#ifndef FrenetPath_H
#define FrenetPath_H

#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "QuarticPolynomial.hpp"
#include "QuinticPolynomial.hpp"
#include "cubic_spliner_2D.hpp"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

using namespace cv;
using namespace std;

// Parameter
#define MAX_SPEED 5.0 / 3.6      // maximum speed [m/s]
#define MAX_ACCEL 2.0             // maximum acceleration [m/ss]
#define MAX_CURVATURE 1.0         // maximum curvature [1/m]
#define MAX_ROAD_WIDTH 5.0        // maximum road width [m]
#define D_ROAD_W 1.0              // road width sampling length [m]
#define DT 0.2                    // time tick [s]
#define MAX_T 8.1                 // max prediction time [m] ----------------> python code에서 floating point가 잘 못되서 +- 0.1을 해줌
#define MIN_T 3.9                 // min prediction time [m] ----------------> python code에서 floating point가 잘 못되서 +- 0.1을 해줌
#define TARGET_SPEED 10.0 / 3.6   // target speed [m/s]
#define D_T_S 5.0 / 3.6           // target speed sampling length [m/s]
#define N_S_SAMPLE 1              // sampling number of target speed
#define ROBOT_RADIUS 2.0          // robot radius [m]

// cost weights
#define K_J 0.1
#define K_T 0.1
#define K_D 1.0
#define K_LAT 1.0
#define K_LON 1.0

// math
#define PI 3.1415926535897

class FrenetPath 
{
public:
    vector<double> t;

    vector<double> d;
    vector<double> d_d;
    vector<double> d_dd;
    vector<double> d_ddd;

    vector<double> s;
    vector<double> s_d;
    vector<double> s_dd;
    vector<double> s_ddd;

    double cd = 0.0;
    double cv = 0.0;
    double cf = 0.0;

    vector<double> x;
    vector<double> y;
    vector<double> yaw;
    vector<double> ds;
    vector<double> c;
};

vector<FrenetPath> calc_frenet_paths(double _c_speed, double _c_d, double _c_d_d, double _c_d_dd, double _s0) {
    vector<FrenetPath> frenet_paths;

    for(double di = -MAX_ROAD_WIDTH; di < MAX_ROAD_WIDTH;) {

        for(double Ti = MIN_T; Ti < MAX_T; ) {

            FrenetPath fp;

            QuinticPolynomial lat_qp(_c_d, _c_d_d, _c_d_dd, di, 0.0, 0.0, Ti);
            
            double t;
            for(t = 0.0; t <=Ti;) {
                fp.t.push_back(t);
                t += DT;
            }

            for(auto i : fp.t) {            // https://www.geeksforgeeks.org/range-based-loop-c/

                fp.d.push_back(lat_qp.calc_point(double(i)));
                fp.d_d.push_back(lat_qp.calc_first_derivative(i));
                fp.d_dd.push_back(lat_qp.calc_second_derivative(i));
                fp.d_ddd.push_back(lat_qp.calc_third_derivative(i));
            }

            for(double tv = TARGET_SPEED-D_T_S*N_S_SAMPLE; tv <= TARGET_SPEED+D_T_S*N_S_SAMPLE;) {

                FrenetPath tfp;
                tfp = fp;
                QuarticPolynomial lon_qp(_s0, _c_speed, 0.0, tv, 0.0, Ti);

                fp.s.clear();
                fp.s_d.clear();
                fp.s_dd.clear();
                fp.s_ddd.clear();

                for(auto i : fp.t) {            // https://www.geeksforgeeks.org/range-based-loop-c/
                    fp.s.push_back(lon_qp.calc_point(i));
                    fp.s_d.push_back(lon_qp.calc_first_derivative(i));
                    fp.s_dd.push_back(lon_qp.calc_second_derivative(i));
                    fp.s_ddd.push_back(lon_qp.calc_third_derivative(i));
                }

                double Jp = 0;
                double Js = 0;

                for(auto p : fp.d_ddd) {
                    Jp += pow(p,2);
                }

                for(auto s : fp.s_ddd) {
                    Js += pow(s,2);
                }

                double ds = pow(TARGET_SPEED-fp.s_d.back(),2);

                fp.cd = K_J * Jp + K_T * Ti + K_D * pow(fp.d.back(),2);
                fp.cv = K_J * Js + K_T * Ti + K_D * ds;
                fp.cf = K_LAT * fp.cd + K_LON * fp.cv;

                frenet_paths.push_back(fp);

                tv += D_T_S;
            }

            Ti += DT;
        }
        di += D_ROAD_W;
    }

    return frenet_paths;
}

vector<FrenetPath> calc_global_paths(ros::NodeHandle& _n, vector<FrenetPath> _fplist, Spline2D _csp) {
    vector<FrenetPath> return_fplist;
    return_fplist = _fplist;
    ros::Publisher position_markers_pub  = _n.advertise<visualization_msgs::MarkerArray>("/frenet_paths", 1);
    // ros::Publisher position_marker_pub  = _n.advertise<visualization_msgs::Marker>("/frenet_single_path", 1);
    visualization_msgs::MarkerArray markers;
    markers.markers.clear();
    std_msgs::ColorRGBA colors;
    int count = 0;
    for(int fp_idx = 0; fp_idx < int(return_fplist.size()); fp_idx++) {
        count++;
        // calc global positions
        visualization_msgs::Marker marker1;
        marker1.header.frame_id = "map";
        marker1.header.stamp = ros::Time();
        marker1.id = count;
        marker1.type = visualization_msgs::Marker::LINE_STRIP;
        marker1.action = visualization_msgs::Marker::ADD;
        marker1.pose.position.x = 0;
        marker1.pose.position.y = 0;
        marker1.pose.orientation.w = 1.0;
        marker1.scale.x = 0.5;
        marker1.scale.y = 0.5;
        marker1.scale.z = 0.5;
        marker1.color.a = 1.0; // Don't forget to set the alpha!
        marker1.color.r = 0.0;
        marker1.color.g = 1.0;
        marker1.color.b = 0.0;

        for(int i = 0; i < int(return_fplist[fp_idx].s.size()); i++){
            // cout << return_fplist[fp_idx].s.size() << endl;
            auto p_rxry = _csp.calc_position(return_fplist[fp_idx].s[i]);
            // if ix is None: --> Not implemeted part
            //     break
                    // visualization_msgs::Marker marker1;
            geometry_msgs::Point point_xy;
            point_xy.x = p_rxry.first;
            point_xy.y = p_rxry.second;
            marker1.points.push_back(point_xy);
            colors.g = 1;
            colors.a = 1;
        
            marker1.colors.push_back(colors);
            auto i_ryaw = _csp.calc_yaw(return_fplist[fp_idx].s[i]);
            double di = return_fplist[fp_idx].d[i];

            double fx = p_rxry.first + di * cos(i_ryaw + PI / 2.0);
            double fy = p_rxry.second + di * sin(i_ryaw + PI / 2.0);

            return_fplist[fp_idx].x.push_back(fx);
            return_fplist[fp_idx].y.push_back(fy);
        }
        markers.markers.push_back(marker1);
        // calc yaw and ds
        for(int i = 0; i < int(return_fplist[fp_idx].x.size()-1); i++) {
            double dx = return_fplist[fp_idx].x[i+1] - return_fplist[fp_idx].x[i];
            double dy = return_fplist[fp_idx].y[i+1] - return_fplist[fp_idx].y[i];
            
            return_fplist[fp_idx].yaw.push_back(atan2(dy,dx));
            return_fplist[fp_idx].ds.push_back(sqrt(dx*dx + dy*dy));
        }
        return_fplist[fp_idx].yaw.push_back(return_fplist[fp_idx].yaw.back());
        return_fplist[fp_idx].ds.push_back(return_fplist[fp_idx].ds.back());

        // calc curvature
        for(int i = 0; i < int(return_fplist[fp_idx].yaw.size()-1); i++) {
            return_fplist[fp_idx].c.push_back((return_fplist[fp_idx].yaw[i+1] - return_fplist[fp_idx].yaw[i]) / return_fplist[fp_idx].ds[i]);
        }
    }
    position_markers_pub.publish(markers);

    return return_fplist;
}

// ob init example : 
// vector<vector<int> > ob{ { 1, 2 }, 
//                                { 4, 5, 6 }, 
//                                { 7, 8, 9, 10 } }; 

bool check_collision(FrenetPath _fp, vector<vector<double>> _ob) {

    bool collision_flg = false;

    for(int i = 0; i < int(_ob.size()); i++) {
        for(int j = 0; j < int(_fp.x.size()); j++) {
            double d = pow(_fp.x[j] - _ob[i][0],2) + pow(_fp.y[j] - _ob[i][1],2);
            if(d < pow(ROBOT_RADIUS,2)) {
                collision_flg = true;
                i = int(_ob.size());
                j = int(_fp.x.size());
                break;
            }
                
        }
    }
    return collision_flg;
}


vector<FrenetPath> check_paths(vector<FrenetPath> _fplist, vector<vector<double>> _ob) {
    vector<int> ok_ind;
    vector<FrenetPath> return_fp;

    for(int i = 0; i < int(_fplist.size()); i++) {
        if(std::any_of(_fplist[i].s_d.begin(), _fplist[i].s_d.end(), [](double v){return v > MAX_SPEED; })){
            continue;
        }
        else if(std::any_of(_fplist[i].s_dd.begin(), _fplist[i].s_dd.end(), [](double a){return abs(a) > MAX_ACCEL; })){
            continue;
        }
        else if(std::any_of(_fplist[i].c.begin(), _fplist[i].c.end(), [](double c){return abs(c) > MAX_CURVATURE; })){
            continue;
        }
        else if(check_collision(_fplist[i], _ob)){
            continue;
        } 
        ok_ind.push_back(i);
        return_fp.push_back(_fplist[i]);
    }

    return return_fp;
}

FrenetPath frenet_optimal_planning(ros::NodeHandle& _n, Spline2D _csp, double _s0, double _c_speed, double _c_d, double _c_d_d, double _c_d_dd, vector<vector<double>> _ob) {
    auto fplist = calc_frenet_paths(_c_speed, _c_d, _c_d_d, _c_d_dd, _s0);
    fplist = calc_global_paths(_n, fplist, _csp);
    fplist = check_paths(fplist, _ob);

    cout << fplist.size() << endl;

    if(fplist.size() != 0) {
        FrenetPath best_path;

        // find minimum cost path
        double min_cost = 1000000000000000.0;
        for(int i = 0; i < int(fplist.size()); i++) {
            if(min_cost >= fplist[i].cf) {
                min_cost = fplist[i].cf;
                best_path = fplist[i];
            }
        }

        return best_path;
    }
    else {
        FrenetPath empty_path;
        empty_path.s.push_back(-1);
        empty_path.d.push_back(-1.1);
        return empty_path;
    }
}

tuple<vector<double>,vector<double>,vector<double>,vector<double>,Spline2D> generate_target_course(vector<double> _x, vector<double> _y) {
    Spline2D csp(_x,_y);

    vector<double> rx,ry,ryaw,rk;
    for(double i_s = 0.0; i_s < csp.s.back();) {   
        auto p_rxry = csp.calc_position(i_s);
        auto i_ryaw = csp.calc_yaw(i_s);
        auto i_rk = csp.calc_curvature(i_s);

        rx.push_back(p_rxry.first);
        ry.push_back(p_rxry.second);
        ryaw.push_back(i_ryaw);
        rk.push_back(i_rk);

        i_s += 0.1;
    }

    return std::make_tuple(rx, ry, ryaw, rk, csp);
}



#endif