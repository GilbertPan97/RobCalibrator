/**
@file : data_processor.cpp
@package : data_processor library
@brief CPP common example functionality.
@copyright (c) 2023, Shanghai Fanuc Ltd.
@version 16.03.2023, SFR: initial version
*/
#include "data_processor.h"
#include "utils.h"

#include <Eigen/Dense>
#include <opencv2/viz.hpp>

DataProc::DataProc(){
    
}

DataProc::DataProc(std::vector<std::vector<cv::Point3f>> scan_lines, CalibObj obj){
    scan_lines_ = scan_lines;
    calib_obj_ = obj;
}

void DataProc::SetScanData(std::vector<std::vector<cv::Point3f>> scan_lines, CalibObj obj){
    scan_lines_ = scan_lines;
    calib_obj_ = obj;
}

std::vector<cv::Point3f> DataProc::CalcSphereCtrs(float rad_sphere){
    // check calibrate object
    if(calib_obj_ != CalibObj::SPHERE){
        std::cout << "ERROR: Calibration object is not set or fault.\n";
        exit(-3);
    }

    // check scan_lines_
    if(scan_lines_.size() == 0){
        std::cout << "ERROR: No scan data, please set scan lines first.\n";
        exit(-3);
    }

    // filter background pnts and noise (get circle_lines_)
    for(const std::vector<cv::Point3f> & scan_line: scan_lines_){
        // filter noise data
        std::vector<cv::Point3f> filter_line = radiusOutlierRemoval(scan_line, 3, 1.0);

        // filter background points
        std::cout << "INFO: Filter background by cluster. \n";
        std::vector<cv::Point3f> circle_line = FilterBackground(filter_line, 2.0, 10);
        circle_lines_.push_back(circle_line);

        // view procession point cloud
        // viewer_3D(circle_line, circle_line);
    }

    // calculate sphere centers (camera frame) under different robot pose
    std::vector<cv::Point3f> ctr_pnts;
    cv::Point2f ctr_arc;    // center of tangential arc of a sphere
    cv::Point3f ctr_sphere;
    float rad_arc;          // radius of tangential arc of a sphere
    float dis_ctr;          // distance between arc center and sphere center
    for (size_t i = 0; i < circle_lines_.size(); i++)
    {
        std::vector<cv::Point2f> circle_line_2d;
        for(const cv::Point3f & pnt: circle_lines_[i]){
            cv::Point2f pnt_2d;
            pnt_2d.x = pnt.x;
            pnt_2d.y = pnt.z;   // pnt.y = 0
            circle_line_2d.push_back(pnt_2d);
        }

        fitCircle(circle_line_2d, ctr_arc, rad_arc, false);

        // calculate distance of arc and sphere center
        if(rad_sphere >= rad_arc)
            dis_ctr = sqrt(rad_sphere * rad_sphere - rad_arc * rad_arc);
        else{
            std::cout << "ERROR: rad_sphere < rad_arc at data: " << i+1 << std::endl;
            exit(-3);
        }
        
        // @FIXME: what is sphere center direction
        ctr_sphere = cv::Point3f(ctr_arc.x, dis_ctr, ctr_arc.y);
        ctr_pnts_.push_back(ctr_sphere);
    }
    
    return ctr_pnts_;
}

std::vector<std::vector<cv::Point3f>> DataProc::CalcTriEdgePntsInCamera(){
    // check calibrate object
    if(calib_obj_ != CalibObj::TRIANGLE_BOARD){
        std::cout << "ERROR: Calibration object is not set or fault.\n";
        exit(-3);
    }

    // check scan_lines_
    if(scan_lines_.size() == 0){
        std::cout << "ERROR: No scan data, please set scan lines first.\n";
        exit(-3);
    }

    for(const std::vector<cv::Point3f> & scan_line: scan_lines_){
        // filter noise data
        std::vector<cv::Point3f> filter_line = radiusOutlierRemoval(scan_line, 3, 1.0);

        // filter background points
        std::cout << "INFO: Filter background by cluster. \n";
        std::vector<cv::Point3f> polyline = FilterBackground(filter_line, 1.0, 10);

        // Extract the corner points from polyline
        std::vector<cv::Point3f> corners = getCornerPoints(polyline, 0.5);
        
        // check view
        // viewer_3D(polyline, corners);

        if(corners.size() == 10){
            tri_edge0_.push_back(corners[4]);
            tri_edge1_.push_back(corners[5]);
        }
    }

    return {tri_edge0_, tri_edge1_};
}

std::vector<std::vector<cv::Point3f>> DataProc::CalcTriEdgePntsInRobase(Eigen::Vector<float, 6> vec_pose_board, Eigen::Vector3f tri_vert, float tri_angle){
    // calculate transform of robot to board
    auto htm_board = CalibUtils::XYZWPRVecToHTM(vec_pose_board);
    tri_angle = tri_angle * M_PI / 180;     // transfer to rad

    // calculate tirangle edge points in robot base frame
    std::vector<cv::Point3f> tri_edge0_rob;
    std::vector<cv::Point3f> tri_edge1_rob;
    for (size_t i = 0; i < tri_edge0_.size(); i++){
        float t = cv::norm(tri_edge0_[i] - tri_edge1_[i])/2;
        Eigen::Vector4f x0 = {tri_vert[0] + t/tan(tri_angle/2), tri_vert[1] + t, 0, 1};
        Eigen::Vector4f x1 = {tri_vert[0] + t/tan(tri_angle/2), tri_vert[1] - t, 0, 1};

        Eigen::Vector3f x0_rob = (htm_board * x0).head(3);
        Eigen::Vector3f x1_rob = (htm_board * x1).head(3);

        tri_edge0_rob.push_back(cv::Point3f(x0_rob[0], x0_rob[1], x0_rob[2]));
        tri_edge1_rob.push_back(cv::Point3f(x1_rob[0], x1_rob[1], x1_rob[2]));
    }
    
    // allocate result
    std::vector<std::vector<cv::Point3f>> tri_edge_in_rob(2);
    tri_edge_in_rob.push_back(tri_edge0_rob);
    tri_edge_in_rob.push_back(tri_edge1_rob);
    
    return {tri_edge0_rob, tri_edge1_rob};
}

/**********************************************************************/

/************************** private function **************************/

/**********************************************************************/
std::vector<cv::Point3f> DataProc::FilterBackground(std::vector<cv::Point3f> filter_line, float eps, int minPts){

    // Cluster the points using DBSCAN
    std::vector<int> labels(filter_line.size(), -1);
    dbscan(filter_line, eps, minPts, labels);
    // Separate the points into clusters
    std::vector<std::vector<cv::Point3f>> clusters;
    for (int i = 0; i < labels.size(); i++) {
        int label = labels[i];
        if (label >= clusters.size()) {
            clusters.resize(label + 1);
        }
        clusters[label].push_back(filter_line[i]);
    }

    // Get the largest cluster
    std::vector<cv::Point3f> circle_line = getLargestCluster(clusters);

    return circle_line;
}

void DataProc::dbscan(const std::vector<cv::Point3f>& points, float eps, int minPts, std::vector<int>& labels) {
    cv::Mat data(points.size(), 3, CV_32F);
    for (int i = 0; i < points.size(); i++) {
        data.at<float>(i, 0) = points[i].x;
        data.at<float>(i, 1) = points[i].y;
        data.at<float>(i, 2) = points[i].z;
    }
    cv::flann::Index index(data, cv::flann::KDTreeIndexParams());
    int clusterIndex = 0;
    for (int i = 0; i < data.rows; i++) {
        if (labels[i] != -1) {
            continue;
        }
        std::vector<int> neighbors;
        std::vector<float> distances;
        index.radiusSearch(data.row(i), neighbors, distances, eps, data.rows, cv::flann::SearchParams());
        if (neighbors.size() < minPts) {
            labels[i] = 0;
            continue;
        }
        labels[i] = ++clusterIndex;
        std::queue<int> expansionQueue;
        for (int j = 0; j < neighbors.size(); j++) {
            if (labels[neighbors[j]] == -1) {
                expansionQueue.push(neighbors[j]);
                labels[neighbors[j]] = clusterIndex;
            }
        }
        while (!expansionQueue.empty()) {
            int k = expansionQueue.front();
            expansionQueue.pop();
            std::vector<int> neighbors2;
            std::vector<float> distances2;
            index.radiusSearch(data.row(k), neighbors2, distances2, eps, data.rows, cv::flann::SearchParams());
            if (neighbors2.size() >= minPts) {
                for (int j = 0; j < neighbors2.size(); j++) {
                    if (labels[neighbors2[j]] == -1) {
                        expansionQueue.push(neighbors2[j]);
                        labels[neighbors2[j]] = clusterIndex;
                    }
                }
            }
        }
    }
}

std::vector<cv::Point3f> DataProc::getLargestCluster(const std::vector<std::vector<cv::Point3f>>& clusters) {
    int maxClusterSize = 0;
    int maxClusterIndex = -1;
    for (int i = 0; i < clusters.size(); i++) {
        if (clusters[i].size() > maxClusterSize) {
            maxClusterSize = clusters[i].size();
            maxClusterIndex = i;
        }
    }
    if (maxClusterIndex == -1) {
        // No clusters found
        return std::vector<cv::Point3f>();
    }
    else {
        // Return the largest cluster
        return clusters[maxClusterIndex];
    }
}

bool DataProc::is_circle_PntCloud(std::vector<cv::Point3f> cluster, cv::Point3f center, float rmse_thresh){
    // calculate the length from points to center
    std::vector<float> lens;
    for (const cv::Point3f &pnt : cluster){
        float len = sqrt((pnt.x - center.x) * (pnt.x - center.x) + 
                         (pnt.y - center.y) * (pnt.y - center.y) +
                         (pnt.z - center.z) * (pnt.z - center.z));
        lens.push_back(len);
    }

    // calculate RMSE (Root Mean Square Error, RMSE)
    float mean = 0;
    for (int i = 0; i < lens.size(); ++i)
        mean += lens[i];
    mean /= lens.size();

    float sum_squared_diff = 0;
    for (int i = 0; i < lens.size(); ++i) {
        float diff = lens[i] - mean;
        sum_squared_diff += diff * diff;
    }
    float rmse = std::sqrt(sum_squared_diff / lens.size());

    if(rmse > rmse_thresh)
        return false;
    else
        return true;
}

std::vector<cv::Point3f> DataProc::radiusOutlierRemoval(const std::vector<cv::Point3f>& points, int k, float radius) {

    // filter invalid points
    std::vector<cv::Point3f> valid_line;
    for(const cv::Point3f & pnt: points){
        if(!isnan(pnt.x) && !isnan(pnt.y) && !isnan(pnt.z))
            valid_line.push_back(pnt);
        else
            continue;
    }

    // filter noise data
    std::vector<cv::Point3f> filteredPoints;

    for (const auto& p : valid_line) {
        std::vector<float> distances;
        for (const auto& q : valid_line) {
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


void DataProc::fitCircle(const std::vector<cv::Point2f>& points, cv::Point2f& center, float& radius, bool display)
{
    int n = points.size();

    // Construct the coefficient matrix A and the right-hand side vector b
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; i++) {
        double x = static_cast<double>(points[i].x);
        double y = static_cast<double>(points[i].y);
        A(i, 0) = 2.0 * x;
        A(i, 1) = 2.0 * y;
        A(i, 2) = 1.0;
        b(i) = x * x + y * y;
    }

    // Solve the linear system using SVD
    Eigen::VectorXd x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    // Compute the center and radius of the circle
    center.x = static_cast<float>(x(0));
    center.y = static_cast<float>(x(1));
    radius = std::sqrt(x(0) * x(0) + x(1) * x(1) + x(2));

    // sidplay check
    if(display)
        fit_display(points, center, radius);
}

void DataProc::fit_display(const std::vector<cv::Point2f> points, cv::Point2f ctr_circle, float rad_circle){
    int width = 600, height = 600;
    cv::Point2f axis_ctr(width / 2.0f, height / 2.0f);    // axis center
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    // draw axis of X and Y
    cv::line(image, cv::Point2f(0, axis_ctr.y), cv::Point2f(width, axis_ctr.y), cv::Scalar(255, 255, 255), 1);
    cv::line(image, cv::Point2f(axis_ctr.x, 0), cv::Point2f(axis_ctr.x, height), cv::Scalar(255, 255, 255), 1);

    // draw original data
    for (const cv::Point2f & p: points) 
        cv::circle(image, p + axis_ctr, 2, cv::Scalar(0, 255, 0), -1);

    // draw fit circle
    cv::circle(image, ctr_circle + axis_ctr, 1, cv::Scalar(255, 0, 0), 2);
    cv::circle(image, ctr_circle + axis_ctr, rad_circle, cv::Scalar(255, 0, 0), 2);

    // show
    cv::imshow("image", image);
    cv::waitKey(0);
}

std::vector<cv::Point3f> DataProc::getCornerPoints(const std::vector<cv::Point3f>& polyline, float tolerance) {
    // Simplify the polyline using the Douglas-Peucker algorithm
    std::vector<cv::Point2f> simplifiedPolyline;
    cv::approxPolyDP(to2D(polyline), simplifiedPolyline, tolerance, false);

    return to3D(simplifiedPolyline);
}

std::vector<cv::Point3f> DataProc::to3D(std::vector<cv::Point2f> points_2d){
    
    std::vector<cv::Point3f> points_3d;

    for(const cv::Point2f pnt2d: points_2d)
        points_3d.push_back({pnt2d.x, 0, pnt2d.y});

    return points_3d;
}

std::vector<cv::Point2f> DataProc::to2D(std::vector<cv::Point3f> point_3d){
    
    std::vector<cv::Point2f> points_2d;

    for(const cv::Point3f pnt3d: point_3d)
        points_2d.push_back({pnt3d.x, pnt3d.z});

    return points_2d;
}

void DataProc::viewer_3D(std::vector<cv::Point3f> pnt_cloud){
    // visulize polyline point
    cv::viz::Viz3d viz("Point Cloud");
    cv::viz::WCoordinateSystem worldCsys(10.0);
    cv::viz::WCloud cloudWidget(pnt_cloud, cv::viz::Color::green());
    viz.showWidget("CoordinateSystem", worldCsys);
    viz.showWidget("Cloud", cloudWidget);
    viz.spin();
}

void DataProc::viewer_3D(std::vector<cv::Point3f> pnt_cloud, std::vector<cv::Point3f> mark_pnts){
    // View point cloud
    cv::viz::Viz3d viz("Point Cloud");
    cv::viz::WCoordinateSystem worldCsys(10.0);
    cv::viz::WCloud cloudWidget(pnt_cloud, cv::viz::Color::green());
    cv::viz::WCloud markPntsWidget(mark_pnts, cv::viz::Color::red());
    markPntsWidget.setRenderingProperty(cv::viz::POINT_SIZE, 5.0);

    viz.showWidget("Cloud", cloudWidget);
    viz.showWidget("MarkPnts", markPntsWidget);
    viz.showWidget("CoordinateSystem", worldCsys);
    viz.spin();
}