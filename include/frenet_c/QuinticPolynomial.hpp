#ifndef QuinticPolynomial_H
#define QuinticPolynomial_H

#include <iostream>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class QuinticPolynomial
{
    double a0,a1,a2,a3,a4,a5;
    Mat A; // rows,cols
    Mat b; // rows,cols

public:
    QuinticPolynomial(double _xs,double _vxs, double _axs, double _xe, double _vxe, double _axe, double _time);
    double calc_point(double _t);
    double calc_first_derivative(double _t);
    double calc_second_derivative(double _t);
    double calc_third_derivative(double _t);
};



QuinticPolynomial::QuinticPolynomial(double _xs,double _vxs, double _axs, double _xe, double _vxe, double _axe, double _time) {

    this->A = Mat(3,3,CV_64F,Scalar(0));    // row,col
    this->b = Mat(3,1,CV_64F,Scalar(0));    // row,col

    this->a0 = _xs;
    this->a1 = _vxe;
    this->a2 = _axs / 2.0;

    this->A.at<double>(0,0) = pow(_time,3);
    this->A.at<double>(0,1) = pow(_time,4);
    this->A.at<double>(0,2) = pow(_time,5);
    this->A.at<double>(1,0) = 3*pow(_time,2);
    this->A.at<double>(1,1) = 4*pow(_time,3);
    this->A.at<double>(1,2) = 5*pow(_time,4);
    this->A.at<double>(2,0) = 6*_time;
    this->A.at<double>(2,1) = 12*pow(_time,2);
    this->A.at<double>(2,2) = 20*pow(_time,3);

    this->b.at<double>(0,0) = _xe - this->a0 - this->a1 * _time - this->a2 * pow(_time,2);
    this->b.at<double>(1,0) = _vxe - this->a1 -2 * this->a2 * _time;
    this->b.at<double>(2,0) = _axe - 2 * this->a2;

    Mat x = A.inv() * b;

    this->a3 = x.at<double>(0,0);
    this->a4 = x.at<double>(0,1);
    this->a5 = x.at<double>(0,2);
}



double QuinticPolynomial::calc_point(double _t) {
    return this->a0 + this->a1 * _t + this->a2 * pow(_t,2) + this->a3 * pow(_t,3) + this->a4 * pow(_t,4) + this->a5 * pow(_t,5);
}

double QuinticPolynomial::calc_first_derivative(double _t) {
    return this->a1 + 2 * this->a2 * _t + 3 * this->a3 * pow(_t,2) + 4 * this->a4 * pow(_t,3) + 5 * this->a5 * pow(_t,4);
}

double QuinticPolynomial::calc_second_derivative(double _t) {
    return 2 * this->a2 + 6 * this->a3 * _t + 12 * this->a4 * pow(_t,2) + 20 * this->a5 * pow(_t,3);
}

double QuinticPolynomial::calc_third_derivative(double _t) {
    return 6 * this->a3 + 24 * this->a4 * _t + 60 * this->a5 * pow(_t,2);
}

#endif