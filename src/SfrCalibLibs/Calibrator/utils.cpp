/**
@file : utils.cpp
@package : utils library
@brief CPP common example functionality.
@copyright (c) 2023, Shanghai Fanuc Ltd.
@version 16.03.2023, SFR: initial version
*/
#include "utils.h"

#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


namespace Calibrator 
{
    namespace Utils
    {       
        Eigen::Matrix4f RT2HomogeneousMatrix(const Eigen::Matrix3f& R, const Eigen::Vector3f& T){
            Eigen::Matrix4f HomoMtr;
            HomoMtr.setIdentity();
            HomoMtr.block(0, 0, 3, 3) = R;
            HomoMtr.block(0, 3, 3, 1) = T;
	        return HomoMtr;
        }

        void HomogeneousMtr2RT(const Eigen::Matrix4f& homoMtr, Eigen::Matrix3f& R, Eigen::Vector3f& T){
            R = homoMtr.block(0, 0, 3, 3);
	        T = homoMtr.block(0, 3, 3, 1);
        }

        Eigen::Matrix4f XYZWPRVecToHTM(const Eigen::VectorXf XYZWPR){
            if (XYZWPR.rows() != 6){
                std::cout << "The XYZWPR is not 6-dimention vector (ERROR:Function HandEyeCalib::XYZWPRVecToHTM call).";
                exit(1);
            }
            Eigen::Matrix4f HTM;			// Homogeneous transformation matrix
            Eigen::Vector3f XYZ = XYZWPR.head(3);
            Eigen::Vector3f WPR = XYZWPR.tail(3);
            Eigen::Matrix3f R;
            R = Eigen::AngleAxisf(WPR(2) * M_PI / 180, Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(WPR(1) * M_PI / 180, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(WPR(0) * M_PI / 180, Eigen::Vector3f::UnitX());

            HTM.setIdentity();
            HTM.block(0, 0, 3, 3) = R;
            HTM.block(0, 3, 3, 1) = XYZ;

            return HTM;
        }

        Eigen::VectorXf HTMToXYZWPRVec(const Eigen::Matrix4f HTM) {
            Eigen::Matrix3f R;
            Eigen::Vector3f T;
            HomogeneousMtr2RT(HTM, R, T);

            if(!isRotatedMatrix(R)){
                std::cout << "HTMToXYZWPRVec input error (not HTM).";
                exit(-1);
            }
            
            Eigen::Vector3f RPW_euler = R.eulerAngles(2, 1, 0) * 180 / M_PI;	// degree

            Eigen::VectorXf xyzwpr(6);
            xyzwpr << T[0], T[1], T[2], RPW_euler[2], RPW_euler[1], RPW_euler[0];
            return xyzwpr;
        }

        cv::Mat toCvMat(const Eigen::MatrixXf& mtr_eigen){
            cv::Mat mtr_cv;
	        cv::eigen2cv(mtr_eigen, mtr_cv);
	        return mtr_cv;
        }

        Eigen::MatrixXf toEigenMatrix(const cv::Mat& mtr_cv){
            Eigen::MatrixXf mtr_eigen;
	        cv::cv2eigen(mtr_cv, mtr_eigen);
	        return mtr_eigen;
        }

        Eigen::VectorXf toEigenVector(const cv::Mat& vec_cv){
            Eigen::VectorXf vec_eigen;
	        // cv::cv2eigen(vec_cv, vec_eigen);		// Only column vectors are allowed
	        return vec_eigen;
        }

        bool isRotatedMatrix(const Eigen::Matrix3f& R){
            if ((R * R.transpose()).isIdentity() &&
            abs(R.determinant() - 1) < 1e-6)
                return true;
            else
                return false;
        }

        bool isInVector(const int idx, std::vector<int> idxVector){
            bool sta = false;

            if (idxVector.empty())
                return sta;

            for (size_t i = 0; i < idxVector.size(); i++)
            {
                if (idxVector[i] == idx) {
                    sta = true;
                    break;
                }
                else {
                    sta = false;
                    continue;
                }  
            }
            return sta;
        }

        Eigen::Matrix3f skew(const Eigen::Vector3f& v) {
            Eigen::Matrix3f skew_mat;
            skew_mat << 0, -v(2), v(1),
                        v(2), 0, -v(0),
                        -v(1), v(0), 0;
            return skew_mat;
        }

        Eigen::Matrix3f rodrigues(const Eigen::Vector3f& v) {
            Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
            float theta = v.norm();
            if (theta < 1e-8) {
                return I;
            }
            Eigen::Vector3f k = v / theta;
            Eigen::Matrix3f K = skew(k);
            Eigen::Matrix3f out_mat = std::cos(theta) * I + (1 - std::cos(theta)) * 
                k * k.transpose() + std::sin(theta) * K;
            return out_mat;
        }

        std::vector<Eigen::Vector3f> toEigenPoints(const std::vector<cv::Point3f>& points){
            std::vector<Eigen::Vector3f> result;
            result.reserve(points.size());

            for (const auto& p : points){
                Eigen::Vector3f v(p.x, p.y, p.z);
                result.push_back(v);
            }

            return result;
        }
    }

}