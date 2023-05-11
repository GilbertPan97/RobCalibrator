/**
@file : common.h
@package : common library
@brief CPP common example functionality.
@copyright (c) 2023, Shanghai Fanuc Ltd.
@version 16.03.2023, SFR: initial version
*/
#include "data_processor.h"
#include "calibrator.h"
#include "algorithm.h"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <Eigen/Core>

using namespace std;

void viewer_3D(std::vector<cv::Point3f> pnt_cloud);

std::string ImgSaveDir = "../../../data/ScanData/";

int main(int argc, char* argv[])
{
	// Read scan lines data
	int snap_cnt = 10;
	std::string file_path;
	std::vector<std::vector<cv::Point3f>> scan_lines;
	for (size_t i = 0; i < snap_cnt; i++){
		// read from yml file
		file_path = ImgSaveDir + std::to_string(i + 1) +"_3C_line.yml";

		std::vector<cv::Point3f> scan_line;
		cv::FileStorage fs(file_path, cv::FileStorage::READ);
		fs["scan_line"] >> scan_line;

		scan_lines.push_back(scan_line);
		fs.release();

		// check view
        // viewer_3D(scan_line);
	}
	// process scan data
	DataProc proc(scan_lines, Calibrator::CalibObj::SPHERE);
	float rad_sphere = 25.4 / 2.0;
	std::vector<cv::Point3f> ctr_pnts = proc.CalcSphereCtrs(rad_sphere);

	// read robot pose data
	std::string file_path1;
	file_path1 = ImgSaveDir + "robpose.txt";
	ifstream infile(file_path1);
	std::vector<Eigen::Vector<float, 6>> vec_rob_pose;

	std::string txt_line;
	while (std::getline(infile, txt_line)){
		// read as xyzwpr
		Eigen::Vector<float, 6> xyzwpr;
		
		istringstream iss(txt_line);

		for (int i = 0; i < 6; ++i)
			iss >> xyzwpr(i);

		vec_rob_pose.push_back(xyzwpr);
	}

	// build calibration
	Calibrator::LineScanner::HandEyeCalib hec;

	hec.SetRobPose(vec_rob_pose);

	hec.SetObjData(ctr_pnts, Calibrator::CalibObj::SPHERE);

	hec.run(Calibrator::LineScanner::SolveMethod::ITERATION);

	float err = hec.CalcCalibError();

	return 0;
}

void viewer_3D(std::vector<cv::Point3f> pnt_cloud){
    // visulize polyline point
    cv::viz::Viz3d viz("Point Cloud");
    cv::viz::WCoordinateSystem worldCsys(10.0);
    cv::viz::WCloud cloudWidget(pnt_cloud, cv::viz::Color::green());
    viz.showWidget("CoordinateSystem", worldCsys);
    viz.showWidget("Cloud", cloudWidget);
    viz.spin();
}