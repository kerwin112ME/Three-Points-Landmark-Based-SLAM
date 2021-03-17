#ifndef Particle_H
#define Particle_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <random>
#include <memory>

using namespace std;

class Particle {
public:
	typedef shared_ptr<Particle> Ptr;
	static const double singularPoint;
	//static const double mean;
	//static const double stddev;
	//default_random_engine generator;
	//normal_distribution<double> dist;

    double _x;
    double _y;
    double _w; // weight
    int _num_updates;

    Eigen::Matrix<double, 3, 2> _rgbPos;
    Eigen::Matrix<double, 3, 3> _rgbCov; // lm x's positions covariance
	
    Particle(double x, double y, double w);
    Particle(double x, double y, double w, Eigen::Matrix<double, 3, 2> rgbPos, Eigen::Matrix<double, 3, 3> rgbCov);

    void move(double dx, double dy);
    void addNewLm(Eigen::Vector2d U, Eigen::Vector3d Z);
    void updateLandMarks(Eigen::Vector3d Z, Eigen::MatrixXd R);

    Eigen::MatrixXd computeJacobian(); // compute the H matrix in EKF
    double computeWeight(Eigen::Vector3d Z, Eigen::MatrixXd R);

};

#endif