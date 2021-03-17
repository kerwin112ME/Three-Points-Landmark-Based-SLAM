#ifndef VIEWER_H
#define VIEWER_H

#include <vector>
#include "Particle.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class Viewer {
public:
	int _num_particles;
	Eigen::Vector3d _lmY;
	vector<cv::Point2d> path;

	double scale = 1.0;
	double min_x = -0.5;
	double min_y = -0.5;
	double max_x = 0.5;
	double max_y = 0.5;
	const double frameScale = 0.0;
	double imageSize = 500.0;

	Viewer(int num_particles, Eigen::Vector3d lmY, Eigen::Vector2d initLoc);
	cv::Mat drawParticles(const std::vector<Particle> &particles, Eigen::Vector3d lmX);
	cv::Point2i convertToImageCoordinates(const cv::Point2d pointInMeters);
	void drawTrajectory(cv::Mat &frame);
	void show(Eigen::Vector2d location, std::vector<Particle> &particles, Eigen::Vector3d lmX);
};


#endif