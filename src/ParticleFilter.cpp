#include "ParticleFilter.h"
#include <iostream>
#include <cstdlib>

using namespace std;

inline double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

ParticleFilter::ParticleFilter(double mapLength, int num_particles, Eigen::MatrixXd lmY, Eigen::Matrix3d Q_init, Eigen::Matrix3d R_init):
	_mapLength(mapLength),  _num_particles(num_particles), _lmY(lmY), _Q(Q_init), _R(R_init)
{
	//_particles.resize(num_particles);
	_lmX_est = Eigen::MatrixXd::Zero(3,1);

	Eigen::Matrix<double, 3, 2> rgbPos;
	Eigen::Matrix<double, 3, 3> rgbCov;
	rgbPos.block<3,1>(0,0) << 2.0, 3.0, 4.0;
	rgbPos.block<3,1>(0,1) = lmY;
	for(int i=0; i<num_particles; i++) {
		double x = fRand(0, mapLength);
		double y = fRand(0, mapLength);
		double w = 1.0 / num_particles;
		_particles.push_back(Particle(x, y, w, rgbPos, rgbCov));
	}
}

void ParticleFilter::predictParticles(Eigen::Vector2d U) {
	// move according to the motion of the robot
	// U = [dx; dy], motion data (2*1 vector)
	for(int i=0; i<_num_particles; i++)
		_particles[i].move(U(0), U(1));
}

void ParticleFilter::updateParticles(Eigen::Vector2d U, Eigen::Vector3d Z) {
	// Z: rgb measurement data (3*1 matrix)
	double sumWeight = 0.0;
	for(int i=0; i<_num_particles; i++) {
		// first time
		if(_particles[i]._num_updates < 1)
			_particles[i].addNewLm(U, Z);
		else {
			_particles[i]._w = _particles[i].computeWeight(Z, _R);
			_particles[i].updateLandMarks(Z, _R);
		}
		_particles[i]._num_updates ++;
		sumWeight += _particles[i]._w;
	}

	_lmX_est = Eigen::Vector3d(0, 0, 0);
	for(int i=0; i<_num_particles; i++) {
		_particles[i]._w /= sumWeight;
		_lmX_est += _particles[i]._w*_particles[i]._rgbPos.block<3,1>(0,0);
	}
}

void ParticleFilter::resampling() {
	double sumWeight = 0.0;
	for(int i=0; i<_num_particles; i++)
		sumWeight += _particles[i]._w;

	for(int i=0; i<_num_particles; i++)
		_particles[i]._w /= sumWeight;

	// Systematic Resampling
	vector<Particle> oldParticles = _particles;
	double r = fRand(0, 1); 
	double w = oldParticles[0]._w; // initial weight
	int i = 0;
	for(int pos=0; pos<_num_particles; pos++) {
		double idx = r/_num_particles + (double)pos/_num_particles;

		while(idx > w) 
			w += oldParticles[++ i]._w;
		
		_particles[pos] = oldParticles[i];
	}

}


Eigen::Vector2d ParticleFilter::computeLocation() {
	double x = 0.0, y = 0.0, sum_w = 0.0;
	for(int i=0; i<_num_particles; i++) {
		x += _particles[i]._x * _particles[i]._w;
		y += _particles[i]._y * _particles[i]._w;
		sum_w += _particles[i]._w;
	}

	x /= sum_w;
	y /= sum_w;
	Eigen::Vector2d location(x, y);

	return location;
}
