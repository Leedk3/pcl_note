#ifndef QuarticPolynomial_H
#define QuarticPolynomial_H

#include <iostream>
#include <iomanip>
#include <vector>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class QuarticPolynomial
{
    double a0,a1,a2,a3,a4;
    Mat A;    // row,col
    Mat b;    // row,col
public:
    QuarticPolynomial(double _xs,double _vxs,double _axs, double _vxe,double _axe,double _time);
    double calc_point(double _t);
    double calc_first_derivative(double _t);
    double calc_second_derivative(double _t);
    double calc_third_derivative(double _t);
};

QuarticPolynomial::QuarticPolynomial(double _xs,double _vxs,double _axs,double _vxe,double _axe,double _time) {

    this->A = Mat(2,2,CV_64F,Scalar(0));    // row,col
    this->b = Mat(2,1,CV_64F,Scalar(0));    // row,col

    this->a0 = _xs;
    this->a1 = _vxs;
    this->a2 = _axs / 2.0;

    // this->A << 3*pow(_time,2), 4*pow(_time,3),
    //             6*_time, 12*pow(_time,2);

    this->A.at<double>(0,0) = 3*pow(_time,2);
    this->A.at<double>(0,1) = 4*pow(_time,3);
    this->A.at<double>(1,0) = 6*_time;
    this->A.at<double>(1,1) = 12*pow(_time,2);


    // this->b << _vxe-this->a1-2*this->a2*_time,
    //             _axe-2*this->a2;

    this->b.at<double>(0,0) = _vxe-this->a1-2*this->a2*_time;
    this->b.at<double>(1,0) = _axe-2*this->a2;

    Mat x = A.inv() * b;

    this->a3 = x.at<double>(0,0);
    this->a4 = x.at<double>(1,0);
}

double QuarticPolynomial::calc_point(double _t) {
    return this->a0 + this->a1*_t + this->a2*pow(_t,2) + this->a3*pow(_t,3) + this->a4*pow(_t,4);
}

double QuarticPolynomial::calc_first_derivative(double _t) {
    return this->a1 + 2*this->a2*_t + 3*this->a3*pow(_t,2) + 4*this->a4*pow(_t,3);
}

double QuarticPolynomial::calc_second_derivative(double _t) {
    return 2*this->a2 + 6*this->a3*_t + 12*this->a4*pow(_t,2);
}

double QuarticPolynomial::calc_third_derivative(double _t) {
    return 6*this->a3 + 24*this->a4*_t;
}

#endif