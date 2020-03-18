#ifndef Spline2D_H
#define Spline2D_H

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

#include "frenet_c/cubic_spliner.hpp"

using namespace cv;
using namespace std;

class Spline2D
{
    public:
        Spline2D(std::vector<double> _x, std::vector<double> _y);        
        ~Spline2D();
        vector<double> __calc_s(std::vector<double> _x, std::vector<double> _y);
        pair< double, double> calc_position(double _s);
        double calc_curvature(double _s);
        double calc_yaw(double _s);

    public:        
        // initialize variables
        vector <double> s, ds, x, y;

    private:
        ros::NodeHandle nh;
};

Spline2D::Spline2D(std::vector<double> _x, std::vector<double> _y) 
{
    ROS_DEBUG("Object of Spline2D Class was created");
    s.clear();
    ds.clear();
    x.clear();
    y.clear();

    this->s = Spline2D::__calc_s(_x, _y);
    this->x.assign(_x.begin(), _x.end());
    this->y.assign(_y.begin(), _y.end());
}

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

std::pair< double, double> Spline2D::calc_position(double _s){
    Spline* sx = new Spline(this->s, this->x);
    Spline* sy = new Spline(this->s, this->y);
    
    double x, y;
    
    x = sx->calc(_s);
    y = sy->calc(_s);
    
    delete sx;
    delete sy;

    return std::make_pair(x,y);
}

double Spline2D::calc_curvature(double _s){
    Spline* sx = new Spline(this->s, this->x);
    Spline* sy = new Spline(this->s, this->y);

    double dx, ddx, dy, ddy;
    double k;

    dx = sx->calcd(_s);
    ddx = sx->calcdd(_s);
    dy = sy->calcd(_s);
    ddy = sy->calcdd(_s);
    k = (ddy * dx - ddx * dy) / pow((dx*dx + dy*dy),1.5);

    delete sx;
    delete sy;

    return k;
}

double Spline2D::calc_yaw(double _s){
    Spline* sx = new Spline(this->s, this->x);
    Spline* sy = new Spline(this->s, this->y);

    double dx, dy;
    double yaw;
    dx = sx->calcd(_s);
    dy = sy->calcd(_s);
    yaw = atan2(dy, dx);

    delete sx;
    delete sy;

    return yaw;
}


#endif