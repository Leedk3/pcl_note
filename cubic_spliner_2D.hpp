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
        Spline2D(){};
        Spline2D(std::vector<double> _x, std::vector<double> _y);        
        ~Spline2D();
        vector<double> __calc_s(std::vector<double> _x, std::vector<double> _y);
        pair< double, double> calc_position(double _s);
        double calc_curvature(double _s);
        double calc_yaw(double _s);

    public:        
        // initialize variables
        vector <double> s, ds, x, y;
        Spline sx;
        Spline sy;
    private:
        ros::NodeHandle nh;
};

Spline2D::Spline2D(std::vector<double> _x, std::vector<double> _y) 
{
    ROS_DEBUG("Object of Spline2D Class was created");
    this->s.clear();
    this->ds.clear();
    this->x.clear();
    this->y.clear();

    this->s = Spline2D::__calc_s(_x, _y);
    this->x.assign(_x.begin(), _x.end());
    this->y.assign(_y.begin(), _y.end());

    this->sx.init(this->s,_x);
    this->sy.init(this->s,_y);

    /* debug */
    // vector <double>::iterator iter3;
    // for (iter3=this->s.begin(); iter3 != this->s.end(); iter3++){
    //     cout << "s" << *iter3 << endl;
    // }
    // 

}

Spline2D::~Spline2D() 
{   
    // ROS_INFO("Spline2D Class destructor.");

}

vector<double> Spline2D::__calc_s(std::vector<double> _x, std::vector<double> _y){
    vector<double> dx(_x.size());
    vector<double> dy(_y.size());
    vector<double> s;
    vector<double>::iterator iter;
    int i = 0;
    double ds_i, s_i;

    std::adjacent_difference(_x.begin(), _x.end(), dx.begin() );
    std::adjacent_difference(_y.begin(), _y.end(), dy.begin() );
    dx.erase(dx.begin());
    dy.erase(dy.begin());

    s.push_back(0);
    for (iter=dx.begin();iter < dx.end(); iter++, i++) {
        ds_i = hypot(dx.at(i), dy.at(i));
        this->ds.push_back(ds_i);
        s_i = s_i + ds_i;        
        s.push_back(s_i);
    }
    return s;
}


std::pair< double, double> Spline2D::calc_position(double _s){
    double x, y;
    
    x = this->sx.calc(_s);
    y = this->sy.calc(_s);

    return std::make_pair(x,y);
}

double Spline2D::calc_curvature(double _s){
    double dx, ddx, dy, ddy;
    double k;

    dx = this->sx.calcd(_s);
    ddx = this->sx.calcdd(_s);
    dy = this->sy.calcd(_s);
    ddy = this->sy.calcdd(_s);
    k = (ddy * dx - ddx * dy) / pow((dx*dx + dy*dy),1.5);

    return k;
}

double Spline2D::calc_yaw(double _s){
    double dx, dy;
    double yaw;
    dx = this->sx.calcd(_s);
    dy = this->sy.calcd(_s);

    yaw = atan2(dy, dx);

    return yaw;
}

#endif