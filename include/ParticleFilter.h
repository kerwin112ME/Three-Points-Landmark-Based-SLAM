#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include "Particle.h"
#include <Eigen/Dense>
#include <vector>

class ParticleFilter {
public:
    typedef shared_ptr<ParticleFilter> Ptr;
    double _mapLength;
    int _num_particles;
    vector<Particle> _particles;
    Eigen::MatrixXd _lmX_est; // estimation of landmarks' x-position
    Eigen::MatrixXd _lmY;
    Eigen::Matrix3d _Q; // lm process noise
    Eigen::Matrix3d _R; // lm measurement noise
    
    ParticleFilter(double mapLength, int num_particles, Eigen::MatrixXd lmY, Eigen::Matrix3d Q_init, Eigen::Matrix3d R_init);
    void predictParticles(Eigen::Vector2d U);
    void updateParticles(Eigen::Vector2d U, Eigen::Vector3d Z);
    void resampling();

    Eigen::Vector2d computeLocation();

};


#endif