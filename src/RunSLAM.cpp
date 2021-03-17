#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include "Particle.h"
#include "ParticleFilter.h"
#include "viewer.h"

using namespace std;

void readData(string path, vector<vector<double>> &U, vector<vector<double>> &Z) {
	// U: motion data, Z: measurement data
	ifstream file(path);
	string lineStr;
	if(file.fail())
		cout << "fail to read the file." << endl;
	while(getline(file, lineStr)) {
		stringstream ss(lineStr);
		string str;
		vector<double> curr(5);
		int i = 0;
		while(getline(ss, str, ',')) {
			curr[i ++] = stod(str);
		}
		U.push_back({curr[0], curr[1]});
		Z.push_back({curr[2], curr[3], curr[4]});
	}
}

int main() {
	string inputFilePath = "../Input/test_input.csv";
	vector<vector<double>> U, Z;
	readData(inputFilePath, U, Z);

	int tspan = U.size();
    Eigen::Matrix3d Q, R; // prcoess and measurement noise
    Eigen::Vector3d lmY;
    Q << 0,0,0,0,0,0,0,0,0;
    R << 1,0,0,0,1,0,0,0,1;
    lmY << 3, 3, 3;

    double mapLength = 6.0;
    int num_particles = 1500;
   	ParticleFilter::Ptr PF(new ParticleFilter(mapLength, num_particles, lmY, Q, R));
   	Viewer viewer(num_particles, lmY, PF->computeLocation());

   	cout << "start SLAM!" << endl;
   	vector<vector<double>> trajectory(tspan, vector<double>(2));
   	for(int i=0; i<tspan; i++) {
   		Eigen::Vector2d Ucurr;
   		Eigen::Vector3d Zcurr;
   		Ucurr << U[i][0], U[i][1];
   		Zcurr << Z[i][0], Z[i][1], Z[i][2];

   		PF->predictParticles(Ucurr);
   		PF->updateParticles(Ucurr, Zcurr);
   		PF->resampling();

   		Eigen::Vector2d location = PF->computeLocation();
   		trajectory[i][0] = location(0);
   		trajectory[i][1] = location(1);

   		viewer.show(location, PF->_particles, PF->_lmX_est);
   	}

    return 0;
}