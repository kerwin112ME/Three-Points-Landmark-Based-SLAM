#include "viewer.h"
#include <algorithm>

using namespace std;

Viewer::Viewer(int num_particles, Eigen::Vector3d lmY, Eigen::Vector2d initLoc)
	:_num_particles(num_particles), _lmY(lmY) {
	path.push_back(cv::Point2d(initLoc(0), initLoc(1)));
	double largestLmY = max(lmY(0), max(lmY(1), lmY(2)));
	double smallestLmY = min(lmY(0), min(lmY(1), lmY(2)));

	min_y = min(min_y, smallestLmY - frameScale);
	max_y = max(max_y, largestLmY + frameScale);
}

void Viewer::drawTrajectory(cv::Mat &frame) {
	cv::Point2i p1, p2;
	for(int i=0; i<path.size()-1; i++) {
		p1 = convertToImageCoordinates(path[i]);
		p2 = convertToImageCoordinates(path[i+1]);

		cv::line(frame, p1, p2, cv::Scalar(255,255,255), 1.5);
	}
	cv::circle(frame, p2, 3, cv::Scalar(255,255,255), -1);
}

cv::Point2i Viewer::convertToImageCoordinates(const cv::Point2d pointInMeters) {
	cv::Point2d pt = (pointInMeters - cv::Point2d(min_x, min_y)) * scale;
	return cv::Point2i(pt.x, imageSize - pt.y); // reverse y
}

cv::Mat Viewer::drawParticles(const std::vector<Particle> &particles, Eigen::Vector3d lmX) {
	for(auto &par: particles) {
		double x = par._x, y = par._y;
		min_x = min(min_x, x - frameScale);
		min_y = min(min_y, y - frameScale);
		max_x = max(max_x, x + frameScale);
		max_y = max(max_y, y + frameScale);
	}

	cv::Mat frame(imageSize, imageSize, CV_8UC3, cv::Scalar::all(0));
	scale = min(imageSize / (max_x - min_x), imageSize / (max_y - min_y));

	// scatter particles
	for(auto &par: particles) {
		cv::Point2i p = convertToImageCoordinates(cv::Point2d(par._x, par._y));
		cv::circle(frame, p, 1, CV_RGB(255,69,0), -1);
	}

	// scatter landmarks
	cv::Point2d red(lmX(0), _lmY(0)), green(lmX(1), _lmY(1)), blue(lmX(2), _lmY(2));
	cv::Point2i red_i = convertToImageCoordinates(red);
	cv::Point2i green_i = convertToImageCoordinates(green);
	cv::Point2i blue_i = convertToImageCoordinates(blue);

	cv::circle(frame, red_i, 5, CV_RGB(255,0,0), -1);
	cv::circle(frame, green_i, 5, CV_RGB(0,255,0), -1);
	cv::circle(frame, blue_i, 5, CV_RGB(0,0,255), -1);

	return frame;
}

void Viewer::show(Eigen::Vector2d location, std::vector<Particle> &particles, Eigen::Vector3d lmX) {
	double x = location(0), y = location(1);
	min_x = min(min_x, x - frameScale);
	min_y = min(min_y, y - frameScale);
	max_x = max(max_x, x + frameScale);
	max_y = max(max_y, y + frameScale);

	path.push_back(cv::Point2d(x, y));

	cv::Mat frame = drawParticles(particles, lmX);
	drawTrajectory(frame);

	cv::imshow("frame", frame);
	cv::waitKey(1);
}