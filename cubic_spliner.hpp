#ifndef Spline_H
#define Spline_H

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

using namespace cv;
using namespace std;


class Spline
{
    public:
        Spline() {};      
        ~Spline();
        double calc(double _t);
        double calcd(double _t);
        double calcdd(double _t);
        int __search_index(double _x);
        Mat __calc_A(std::vector<double> h);
        Mat __calc_B(std::vector<double> h);
        void init(std::vector<double> _x, std::vector<double> _y);
    public:        
        // initialize variables
        vector <double> x,y,a,b,c,d,w;
        int nx;

    private:
        // initialize variables
        ros::NodeHandle nh;
};

void Spline::init(std::vector<double> _x, std::vector<double> _y) {
    ROS_DEBUG("Object of Spline Class was created");

    x.clear();
    y.clear();
    
    b.clear();

    this->x.assign(_x.begin(), _x.end()); 
    this->y.assign(_y.begin(), _y.end()); 

    this -> nx = _x.size();

    vector <double> h(_x.size());
    // h.clear();
    std::adjacent_difference(_x.begin(), _x.end(), h.begin() );
    h.erase(h.begin());
    
    a.clear();
    this->a.assign(_y.begin(), _y.end());    


    Mat A = __calc_A(h);
    Mat B = __calc_B(h);
    Mat C = A.inv() * B;
    if (C.isContinuous())
        C.col(0).copyTo(this->c);

    vector<double>::iterator iter;
    int i = 0;
    double d_i = 0;
    double b_i = 0;
    for (iter=_x.begin();iter < _x.end() - 1; iter++, i++) {
        d_i = (this->c.at(i+1) - this->c.at(i)) / (3.0*h.at(i));
        b_i = (this->a.at(i+1) - this->a.at(i)) / h.at(i) - h.at(i) * (this->c.at(i+1) + 2.0 * this->c.at(i)) / 3;
        this->d.push_back(d_i);
        this->b.push_back(b_i);
    }
}

Spline::~Spline() 
{    
    // ROS_INFO("Spline Class destructor.");
}

double Spline::calc(double _t){
    if ( _t < this->x.at(0)){
        return -1;
    }
    else if ( _t > this->x.back() ){
        return -1;
    }
    int found_index;
    double dx, result;
    found_index = Spline::__search_index(_t);
    dx = _t - this->x.at(found_index);
    result = this->a.at(found_index) + this->b.at(found_index) * dx + 
             this->c.at(found_index) * pow(dx,2) + this->d.at(found_index) * pow(dx,3);

    return result;
}

double Spline::calcd(double _t){
    if ( _t < this->x.at(0)){
        return -1;
    }
    else if ( _t > this->x.back() ){
        return 0;
    }
    int found_index;
    double dx, result;
    found_index = Spline::__search_index(_t);
    
    dx = _t - this->x.at(found_index);
    result = this->b.at(found_index) + this->c.at(found_index)* dx *2 + this->d.at(found_index)* pow(dx,2) * 3;

    return result;
}

double Spline::calcdd(double _t){
    if ( _t < this->x.at(0)){
        return -1;
    }
    else if ( _t > this->x.back() ){
        return 0;
    }
    int found_index;
    double dx, result;
    found_index = Spline::__search_index(_t);
    dx = _t - this->x.at(found_index);
    result = this->c.at(found_index)* 2 + this->d.at(found_index)* dx * 6;

    return result;
}

int Spline::__search_index(double _x){
    auto iter=lower_bound(this-> x.begin(), this->x.end(), _x) - this->x.begin();
    int result = int(iter) - 1;
    if (result < 0) {
        result = 0;
    }
    return result;
}

Mat Spline::__calc_A(std::vector<double> h){
    Mat A;
    A = Mat::zeros(this->nx,this->nx,CV_64F);
    A.at<double>(0,0) = 1.0;
    vector<double>::iterator iter;
    int i = 0; 
    for (iter=h.begin();iter!=h.end();iter++,i++) {
        if (i != this->nx){
            iter++;
            double iter_next = *iter;
            iter--;
            A.at<double>(i+1, i+1) = 2.0 * (*iter + iter_next);    //I am not sure it is right or not
        }
        A.at<double>(i+1, i) = *iter;
        A.at<double>(i, i+1) = *iter; 
    }
    A.at<double>(0,1) = 0.0;
    A.at<double>(this->nx - 1, this->nx - 2) = 0.0;
    A.at<double>(this->nx - 1, this->nx - 1) = 1.0;
    
    return A;
}

Mat Spline::__calc_B(std::vector<double> h){
    Mat B;
    B = Mat::zeros(this->nx, 1, CV_64F);
    for (int i =0; i < this->nx - 2; ++i) {
        B.at<double>(i, 1) = 3.0 * (this->a.at(i+2) - this->a.at(i+1)) / h.at(i+1) - 
                               3.0 * (this->a.at(i+1) - this->a.at(i)) / h.at(i);
    }
    return B;
}

#endif