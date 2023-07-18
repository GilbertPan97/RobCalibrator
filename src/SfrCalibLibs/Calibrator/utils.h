/**
@file : utils.cpp
@package : utils library
@brief CPP common example functionality.
@copyright (c) 2023, Shanghai Fanuc Ltd.
@version 16.03.2023, SFR: initial version
*/
#ifndef UTILS_H
#define UTILS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


namespace CalibUtils
{

    Eigen::Matrix4f RT2HomogeneousMatrix(const Eigen::Matrix3f& R, const Eigen::Vector3f& T);

    void HomogeneousMtr2RT(const Eigen::Matrix4f& homoMtr, Eigen::Matrix3f& R, Eigen::Vector3f& T);

    Eigen::Matrix4f XYZWPRVecToHTM(const Eigen::VectorXf XYZWPR);

    Eigen::VectorXf HTMToXYZWPRVec(const Eigen::Matrix4f HTM);
    
    Eigen::VectorXf HTMToXYZQUAVec(const Eigen::Matrix4f HTM);

    cv::Mat toCvMat(const Eigen::MatrixXf& mtr_eigen);

    Eigen::MatrixXf toEigenMatrix(const cv::Mat& mtr_cv);

    Eigen::VectorXf toEigenVector(const cv::Mat& vec_cv);

    bool isRotatedMatrix(const Eigen::Matrix3f& R);

    bool isInVector(const int idx, std::vector<int> idxVector);

    Eigen::Matrix3f skew(const Eigen::Vector3f& v);

    Eigen::Matrix3f rodrigues(const Eigen::Vector3f& v);

    std::vector<Eigen::Vector3f> toEigenPoints(const std::vector<cv::Point3f>& points);

    std::vector<cv::Point3f> to3D(std::vector<cv::Point2f> points_2d);

    std::vector<cv::Point2f> to2D(std::vector<cv::Point3f> point_3d);
}

#endif