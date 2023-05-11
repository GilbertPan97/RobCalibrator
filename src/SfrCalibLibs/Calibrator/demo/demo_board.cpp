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
#include <opencv2/flann.hpp>
#include <opencv2/viz.hpp>
#include <Eigen/Core>

using namespace std;

std::string ImgSaveDir = "../../../data/ScanData/";

std::vector<cv::Point3f> radiusOutlierRemoval(const std::vector<cv::Point3f>& points, int k, float radius);
void dbscan(const std::vector<cv::Point3f>& points, float eps, int minPts, std::vector<int>& labels);
std::vector<cv::Point3f> getLargestCluster(const std::vector<std::vector<cv::Point3f>>& clusters);
std::vector<cv::Point3f> getCornerPoints(const std::vector<cv::Point3f>& polyline, float tolerance);
std::vector<cv::Point3f> detectPolylineCorners(const std::vector<cv::Point3f>& points, float threshold_angle, float simplification_epsilon);
std::vector<cv::Point2f> to2D(std::vector<cv::Point3f> point_3d);
std::vector<cv::Point3f> to3D(std::vector<cv::Point2f> points_2d);
void viewer_3D(std::vector<cv::Point3f> pnt_cloud);

int main(int argc, char* argv[])
{
    // read robot pose data
	std::string file_path1;
	file_path1 = ImgSaveDir + "rob2board.txt";
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

	// Read scan lines data
	int snap_cnt = 12;
	std::string file_path;
	std::vector<std::vector<cv::Point3f>> scan_lines;
	for (size_t i = 0; i < snap_cnt; i++){
		// read from yml files
		file_path = ImgSaveDir + std::to_string(i + 1) +"_3C_line.yml";

		std::vector<cv::Point3f> scan_line;
		cv::FileStorage fs(file_path, cv::FileStorage::READ);
		fs["scan_line"] >> scan_line;

		scan_lines.push_back(scan_line);
		fs.release();

        // check view
        // viewer_3D(scan_line);
	}

    DataProc proc(scan_lines, Calibrator::CalibObj::TRIANGLE_BOARD);

    std::vector<std::vector<cv::Point3f>> tri_edges_cam = proc.CalcTriEdgePntsInCamera();

    Eigen::VectorXf xyzwpr_board(6);
    xyzwpr_board << 493.6f, 366.9f, -305.0f, 0.4f, 0.0f, 179.6f;
    Eigen::Vector3f tri_vert(60.0f, 54.0f, 0.0f);
    float tri_ang = 38;
    std::vector<std::vector<cv::Point3f>> tri_edges_rob = proc.CalcTriEdgePntsInRobase(xyzwpr_board, tri_vert, tri_ang);

    Calibrator::LineScanner::HandEyeCalib hec;

    hec.SetRobPose(vec_rob_pose, xyzwpr_board);
    hec.SetObjData(tri_edges_cam, tri_edges_rob, Calibrator::CalibObj::TRIANGLE_BOARD);

    Eigen::VectorXf sol(12);
    ifstream sol_file(std::string(ImgSaveDir +"solution.txt"));
    std::getline(sol_file, txt_line);
    istringstream iss_sol(txt_line);
    for (size_t i = 0; i < 12; i++)
        iss_sol >> sol(i);
    std::cout << "INFO: Read solution is: \n" << sol.transpose() << std::endl;

    hec.linerFit(sol);

	return 0;
}

std::vector<cv::Point3f> radiusOutlierRemoval(const std::vector<cv::Point3f>& points, int k, float radius) {
    std::vector<cv::Point3f> filteredPoints;

    for (const auto& p : points) {
        std::vector<float> distances;
        for (const auto& q : points) {
            if (p != q) {
                float distance = cv::norm(p - q);
                distances.push_back(distance);
            }
        }
        std::sort(distances.begin(), distances.end());
        if (distances[k] < radius) {
            filteredPoints.push_back(p);
        }
    }

    return filteredPoints;
}

// Function to get the largest cluster from a set of clusters


std::vector<cv::Point3f> getCornerPoints(const std::vector<cv::Point3f>& polyline, float tolerance) {
    /* finish with cv::approxPolyDP */
    // Simplify the polyline using the Douglas-Peucker algorithm
    std::vector<cv::Point2f> simplifiedPolyline;
    cv::approxPolyDP(to2D(polyline), simplifiedPolyline, tolerance, false);

    return to3D(simplifiedPolyline);
}

std::vector<cv::Point3f> detectPolylineCorners(const std::vector<cv::Point3f>& points, float threshold_angle, float simplification_epsilon) {
    std::vector<cv::Point2f> simplified_points;
    std::vector<cv::Point2f> corners_2d;
    std::vector<cv::Point3f> corners;

    std::vector<cv::Point2f> points_2d;
    for(const cv::Point3f & pnt: points)
        points_2d.push_back({pnt.x, pnt.z});
    
    cv::approxPolyDP(points_2d, simplified_points, simplification_epsilon, true);

    for (int i = 1; i < simplified_points.size() - 1; i++) {
        cv::Point2f p1 = simplified_points[i - 1];
        cv::Point2f p2 = simplified_points[i];
        cv::Point2f p3 = simplified_points[i + 1];

        cv::Point2f v1 = p1 - p2;
        cv::Point2f v2 = p3 - p2;

        float dot_product = v1.dot(v2);
        float det = v1.x * v2.y - v1.y * v2.x;
        float angle = atan2(det, dot_product) * 180.0 / CV_PI;

        if (abs(angle) > threshold_angle)
            corners_2d.push_back(p2);
    }


    for (cv::Point2f& corner : corners_2d) {
        // find the closest point in the original polyline to the corner point
        float min_distance = std::numeric_limits<float>::max();
        cv::Point2f closest_point;
        for (const cv::Point2f& point : points_2d) {
            float distance = cv::norm(point - corner);
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = point;
            }
        }
        corner = closest_point;
    }

    return to3D(simplified_points);
}

std::vector<cv::Point3f> to3D(std::vector<cv::Point2f> points_2d){
    std::vector<cv::Point3f> points_3d;

    for(const cv::Point2f pnt2d: points_2d)
        points_3d.push_back({pnt2d.x, 0, pnt2d.y});

    return points_3d;
}

std::vector<cv::Point2f> to2D(std::vector<cv::Point3f> point_3d){
    std::vector<cv::Point2f> points_2d;

    for(const cv::Point3f pnt3d: point_3d)
        points_2d.push_back({pnt3d.x, pnt3d.z});

    return points_2d;
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