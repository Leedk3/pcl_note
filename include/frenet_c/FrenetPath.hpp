#ifndef FrenetPath_H
#define FrenetPath_H

#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "QuarticPolynomial.hpp"
#include "QuinticPolynomial.hpp"

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

            for(double tv = TARGET_SPEED-D_T_S*N_S_SAMPLE; tv < TARGET_SPEED+D_T_S*N_S_SAMPLE;) {
                FrenetPath tfp;
                tfp = fp;
                QuarticPolynomial lon_qp(_s0, _c_speed, 0.0, tv, 0.0, Ti);

                for(auto i : fp.t) {            // https://www.geeksforgeeks.org/range-based-loop-c/
                    tfp.s.push_back(lon_qp.calc_point(i));
                    tfp.s_d.push_back(lon_qp.calc_first_derivative(i));
                    tfp.s_dd.push_back(lon_qp.calc_second_derivative(i));
                    tfp.s_ddd.push_back(lon_qp.calc_third_derivative(i));
                }

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

// // vector<FrenetPath> calc_global_paths(vector<FrenetPath> _fplist, ) {

// // }




#endif