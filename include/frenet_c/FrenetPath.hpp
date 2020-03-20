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

using namespace cv;
using namespace std;

// Parameter
#define MAX_SPEED 50.0 / 3.6      // maximum speed [m/s]
#define MAX_ACCEL 2.0             // maximum acceleration [m/ss]
#define MAX_CURVATURE 1.0         // maximum curvature [1/m]
#define MAX_ROAD_WIDTH 7.0        // maximum road width [m]
#define D_ROAD_W 1.0              // road width sampling length [m]
#define DT 0.2                    // time tick [s]
#define MAX_T 5.0                 // max prediction time [m]
#define MIN_T 4.0                 // min prediction time [m]
#define TARGET_SPEED 30.0 / 3.6   // target speed [m/s]
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

        // cout << di << endl;
        for(double Ti = MIN_T; Ti < MAX_T; ) {

            // cout << Ti << endl;
            FrenetPath fp;

            QuinticPolynomial lat_qp(_c_d, _c_d_d, _c_d_dd, di, 0.0, 0.0, Ti);
            
            for(double t = 0.0; t <Ti;) {
                fp.t.push_back(t);
                t += DT;
            }

            for(auto i : fp.t) {            // https://www.geeksforgeeks.org/range-based-loop-c/
                fp.d.push_back(lat_qp.calc_point(i));
                fp.d_d.push_back(lat_qp.calc_first_derivative(i));
                fp.d_dd.push_back(lat_qp.calc_second_derivative(i));
                fp.d_ddd.push_back(lat_qp.calc_third_derivative(i));
            }

            cout  << "---------------------------PRINT D ---------------------------" << endl;
            for(auto i : fp.t) {
                cout << fp.d[i] << " " << fp.d_d[i] << " " << fp.d_dd[i] << " " << fp.d_ddd[i] << " " << endl;
            }

            

            for(double tv = TARGET_SPEED-D_T_S*N_S_SAMPLE; tv <= TARGET_SPEED+D_T_S*N_S_SAMPLE;) {

                // cout << tv << endl;

                FrenetPath tfp;
                tfp = fp;
                QuarticPolynomial lon_qp(_s0, _c_speed, 0.0, tv, 0.0, Ti);

                for(auto i : fp.t) {            // https://www.geeksforgeeks.org/range-based-loop-c/
                    tfp.s.push_back(lon_qp.calc_point(i));
                    tfp.s_d.push_back(lon_qp.calc_first_derivative(i));
                    tfp.s_dd.push_back(lon_qp.calc_second_derivative(i));
                    tfp.s_ddd.push_back(lon_qp.calc_third_derivative(i));
                }

                // cout  << "---------------------------PRINT S ---------------------------" << endl;

                // for(auto i : tfp.s) {
                //     cout << tfp.s[i] << " " << tfp.s_d[i] << " " << tfp.s_dd[i] << " " << tfp.s_ddd[i] << " "  << endl;
                // }

                double Jp = 0;
                double Js = 0;

                for(auto p : tfp.d_ddd) {
                    Jp += pow(tfp.d_ddd[p],2);
                }

                for(auto s : tfp.s_ddd) {
                    Js += pow(tfp.s_ddd[s],2);
                }

                double ds = pow(TARGET_SPEED-tfp.s_d[-1],2);

                tfp.cd = K_J * Jp + K_T * Ti + K_D * pow(tfp.d[-1],2);
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds;
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv;

                frenet_paths.push_back(tfp);

                tv += D_T_S;
            }

            Ti += DT;
        }
        di += D_ROAD_W;
    }

    return frenet_paths;
}

vector<FrenetPath> calc_global_paths(vector<FrenetPath> _fplist, Spline2D _csp) {
    vector<FrenetPath> return_fplist;
    return_fplist = _fplist;

    for(int fp_idx = 0; fp_idx < int(return_fplist.size()); fp_idx++) {

        // calc global positions
        for(int i = 0; i < int(return_fplist[fp_idx].s.size()); i++){
            auto p_rxry = _csp.calc_position(return_fplist[fp_idx].s[i]);
            // if ix is None: --> Not implemeted part
            //     break

            auto i_ryaw = _csp.calc_yaw(return_fplist[fp_idx].s[i]);
            double di = return_fplist[fp_idx].d[i];

            double fx = p_rxry.first + di * cos(i_ryaw + PI / 2.0);
            double fy = p_rxry.second + di * sin(i_ryaw + PI / 2.0);

            return_fplist[fp_idx].x.push_back(fx);
            return_fplist[fp_idx].y.push_back(fy);
        }

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
        if(std::any_of(_fplist[i].s_d.begin(), _fplist[i].s_d.end(), [](double v){return v > MAX_SPEED; }))
            continue;
        else if(std::any_of(_fplist[i].s_dd.begin(), _fplist[i].s_dd.end(), [](double a){return abs(a) > MAX_ACCEL; }))
            continue;
        else if(std::any_of(_fplist[i].c.begin(), _fplist[i].c.end(), [](double c){return abs(c) > MAX_CURVATURE; }))\
            continue;
        else if(check_collision(_fplist[i], _ob))
            continue;
        
        ok_ind.push_back(i);
        return_fp.push_back(_fplist[i]);
    }

    return return_fp;
}

FrenetPath frenet_optimal_planning(Spline2D _csp, double _s0, double _c_speed, double _c_d, double _c_d_d, double _c_d_dd, vector<vector<double>> _ob) {
    auto fplist = calc_frenet_paths(_c_speed, _c_d, _c_d_d, _c_d_dd, _s0);

    cout << fplist.size() << endl;

    fplist = calc_global_paths(fplist, _csp);
    fplist = check_paths(fplist, _ob);

    FrenetPath best_path;

    // find minimum cost path
    double min_cost = 1000000.0;
    for(int i = 0; i < int(fplist.size()); i++) {
        if(min_cost >= fplist[i].cf) {
            min_cost = fplist[i].cf;
            best_path = fplist[i];
        }
    }

    return best_path;
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