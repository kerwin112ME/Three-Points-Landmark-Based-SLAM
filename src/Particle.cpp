#include "Particle.h"
#include <cstdlib>
#include <iostream>
#include <cmath>

#define PI 3.1416

const double Particle::singularPoint = 1e-6;
//const double Particle::mean = 0.0;
//const double Particle::stddev = 0.03;

inline double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

Particle::Particle(double x, double y, double w): _x(x), _y(y), _w(w) {
	_num_updates = 0;
	//dist = normal_distribution<double>(mean, stddev);
}

Particle::Particle(double x, double y, double w, Eigen::Matrix<double, 3, 2> rgbPos, Eigen::Matrix<double, 3, 3> rgbCov): _x(x), _y(y), _w(w){
	_rgbPos = rgbPos;
	_rgbCov = rgbCov;
	_num_updates = 0;
}

void Particle::move(double dx, double dy) {
	double x_noise = fRand(-0.03, 0.03);
	double y_noise = fRand(-0.03, 0.03);
	_x += (dx + x_noise);
	_y += (dy + y_noise);
}

void Particle::addNewLm(Eigen::Vector2d U, Eigen::Vector3d Z) {
	Eigen::Vector3d lmY = _rgbPos.block<3,1>(0,1); // y-coor of landmarks
	Eigen::Vector3d dX2 = Z.array().pow(2).matrix() - (lmY.array() - _y).pow(2).matrix();
	for(int i=0; i<3; i++) 
		dX2(i) = (dX2(i) < 0)? 0: dX2(i);

	Eigen::Vector3d dX = dX2.array().sqrt().matrix();
	if(_num_updates < 1) {
		_rgbPos.block<3,1>(0,0) = _rgbPos.block<3,1>(0,0) + dX;

		Eigen::Vector3d diag;
		for(int i=0; i<3; i++) 
			diag(i) = (dX(i,0) == 0)? 1.2: 0.2;
		_rgbCov = diag.asDiagonal();
	}
}

Eigen::MatrixXd Particle::computeJacobian() {
	Eigen::ArrayXd dx = _rgbPos.block<3,1>(0,0).array() - _x;
	Eigen::ArrayXd dy = _rgbPos.block<3,1>(0,1).array() - _y;
	Eigen::Vector3d diag = (dx / (dx.pow(2) + dy.pow(2)).sqrt()).matrix();

	return diag.asDiagonal();
}

void Particle::updateLandMarks(Eigen::Vector3d Z, Eigen::MatrixXd R) {
	// update by EKF
	Eigen::ArrayXd dx = _rgbPos.block<3,1>(0,0).array() - _x;
	Eigen::ArrayXd dy = _rgbPos.block<3,1>(0,1).array() - _y;
	Eigen::Matrix3d H = this->computeJacobian();

	Eigen::MatrixXd Qz = H*_rgbCov*H.transpose() + R;
	Eigen::MatrixXd QzInv;
	if(Qz.determinant() < singularPoint) 
		QzInv = Qz.completeOrthogonalDecomposition().pseudoInverse();
	else QzInv = Qz.inverse();
	
	Eigen::MatrixXd K = _rgbCov*H.transpose()*QzInv;
	Eigen::ArrayXd Z_hat = (dx.pow(2) + dy.pow(2)).sqrt();
	Eigen::Vector3d dZ = Z - Z_hat.matrix();

	// update the states and the covariance
	_rgbPos.block<3,1>(0,0) += K*dZ;
	_rgbCov = (Eigen::MatrixXd::Identity(3,3) - K*H)*_rgbCov;
}

double Particle::computeWeight(Eigen::Vector3d Z, Eigen::MatrixXd R) {
	Eigen::ArrayXd dx = _rgbPos.block<3,1>(0,0).array() - _x;
	Eigen::ArrayXd dy = _rgbPos.block<3,1>(0,1).array() - _y;
	Eigen::Matrix3d H = this->computeJacobian();
	Eigen::MatrixXd Qz = H*_rgbCov*H.transpose() + R;
	Eigen::MatrixXd QzInv = Qz.inverse();

	if(Qz.determinant() < singularPoint) return 1.0;

	Eigen::ArrayXd Z_hat = (dx.pow(2) + dy.pow(2)).sqrt();
	Eigen::Vector3d dZ = Z - Z_hat.matrix();

	double num = -0.5*dZ.transpose()*QzInv*dZ;
	double den = 2*PI*sqrt(Qz.determinant());

	double w = exp(num)/den;
	return w;
}

