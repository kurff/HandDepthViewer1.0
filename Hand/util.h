#pragma once
#include <cmath>
#include <Eigen/Core>
void normalize(double axis[3]){
   double sum = sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
   axis[0] /= (sum + 1e-20);
   axis[1] /= (sum + 1e-20);
   axis[2] /= (sum + 1e-20);
}

void cross_product(double axis_a[3], double axis_b[3], double axis_c[3]){
	axis_c[0] = axis_a[1]*axis_b[2] - axis_a[2]*axis_b[1];
	axis_c[1] = axis_a[2]*axis_b[0] - axis_a[0]*axis_b[2];
	axis_c[2] = axis_a[0]*axis_b[1] - axis_a[1]*axis_b[0];
}

void set_axis(double axis_a[3], double axis_b[3], double axis_c[3], double position[3], Eigen::MatrixXd& rot){
	rot = Eigen::MatrixXd::Zero(4,4);
	rot(0,0) = axis_a[0]; rot(1,0) = axis_a[1]; rot(2,0) = axis_a[2];
	rot(0,1) = axis_b[0]; rot(1,1) = axis_b[1]; rot(2,1) = axis_b[2];
	rot(0,2) = axis_c[0]; rot(1,2) = axis_c[1]; rot(2,2) = axis_c[2];
	rot(0,3) = position[0]; rot(1,3) = position[1]; rot(2,3) = position[2];
	rot(3,3) = 1.0;
}

