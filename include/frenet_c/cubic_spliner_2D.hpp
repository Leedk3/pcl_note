#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <numeric>

// #include <opencv/cv.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <stdio.h>
// #include <gtsam/nonlinear/Values.h>

// #include "frenet_c/cubic_spliner.hpp"

using namespace cv;
using namespace std;

class Spline2D
{
    public:
        Spline2D(ros::NodeHandle& _n, std::vector<double> _x, std::vector<double> _y);        
        ~Spline2D();
        vector<double> __calc_s(std::vector<double> _x, std::vector<double> _y);
        vector<pair<double, double>> calc_position(std::vector<double> _s);
        vector<double> calc_curvature(std::vector<double> _s);
        vector<double> calc_yaw(std::vector<double> _s);

    public:        
        // initialize variables
        vector <double> s, ds, x, y;

    private:
        ros::NodeHandle nh;
};

Spline2D::Spline2D(ros::NodeHandle& _n, std::vector<double> _x, std::vector<double> _y) 
{
    ROS_DEBUG("Object of Spline2D Class was created");
    s.clear();
    ds.clear();
    x.clear();
    y.clear();

    this->s = Spline2D::__calc_s(_x, _y);
    this->x.assign(_x.begin(), _x.end());
    this->y.assign(_y.begin(), _y.end());
};

Spline2D::~Spline2D() 
{   
    ROS_INFO("Spline2D Class destructor.");

}

vector<double> Spline2D::__calc_s(std::vector<double> _x, std::vector<double> _y){
    vector<double> dx, dy, s;
    vector<double>::iterator iter;
    int i = 0;
    double ds_i, s_i;
    
    dx.clear();
    dy.clear();
    std::adjacent_difference(_x.begin(), _x.end(), dx.begin());
    std::adjacent_difference(_x.begin(), _x.end(), dy.begin());

    for (iter=_x.begin();iter < _x.end() - 1; iter++, i++) {
        ds_i = hypot(dx.at(i), dy.at(i));
        this->ds.push_back(ds_i);
        if (i == 0){
            s_i = 0;
        }
        else {
            s_i = s_i + ds_i;
        }
        s.push_back(s_i);
    }
    return s;
}

vector<pair<double, double>> Spline2D::calc_position(std::vector<double> _s){
    Spline* sx = new Spline(nh, _s, this->x);
    Spline* sy = new Spline(nh, _s, this->y);
    vector <double> x,y;
    vector <pair <double, double> > xy_pair;
    vector<double>::iterator iter;
    double x_i, y_i;
    for (iter=_s.begin();iter!=_s.end(); iter++) {
        x_i = sx->calc(*iter);
        y_i = sy->calc(*iter);
        xy_pair.push_back(std::make_pair(x_i,y_i));
    }
    delete sx;
    delete sy;

    return xy_pair;
}

vector<double> Spline2D::calc_curvature(std::vector<double> _s){
    Spline* sx = new Spline(nh, _s, this->x);
    Spline* sy = new Spline(nh, _s, this->y);
    vector <double> k;
    vector<double>::iterator iter;
    double dx_i, ddx_i, dy_i, ddy_i;
    double k_i;
    for (iter=_s.begin();iter!=_s.end(); iter++) {
        dx_i = sx->calcd(*iter);
        ddx_i = sx->calcdd(*iter);
        dy_i = sy->calcd(*iter);
        ddy_i = sy->calcdd(*iter);
        k_i = (ddy_i * dx_i - ddx_i * dy_i) / pow((dx_i*dx_i + dy_i*dy_i),1.5);
        k.push_back(k_i);
    }
    delete sx;
    delete sy;

    return k;
}

vector<double> Spline2D::calc_yaw(std::vector<double> _s){
    Spline* sx = new Spline(nh, _s, this->x);
    Spline* sy = new Spline(nh, _s, this->y);
    vector <double> yaw;
    vector<double>::iterator iter;
    double dx_i, dy_i;
    double yaw_i;
    for (iter=_s.begin();iter!=_s.end(); iter++) {
        dx_i = sx->calcd(*iter);
        dy_i = sy->calcd(*iter);
        yaw_i = atan2(dy_i, dx_i);
        yaw.push_back(yaw_i);
    }
    delete sx;
    delete sy;

    return yaw;
}
