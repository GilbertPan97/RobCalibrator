/**
@file : calibrator.cpp
@package : calibrator library
@brief CPP common example functionality.
@copyright (c) 2023, Shanghai Fanuc Ltd.
@version 16.03.2023, SFR: initial version
*/
#include "calibrator.h"
#include "common.h"
#include "utils.h"
#include "algorithm.h"

#include <iostream>
#include <cmath>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>

// Utils is visible for all Calibrator space
using namespace CalibUtils; 

namespace LineScanner
{
    HandEyeCalib::HandEyeCalib(){

    }

    HandEyeCalib::HandEyeCalib(CalibType type){
        type_ = type;
    }

    HandEyeCalib::~HandEyeCalib(){

    }

    bool HandEyeCalib::SetRobPose(const std::vector<Eigen::Vector<float, 6>> rob_pose){
        if(rob_pose.size()==0){
            std::cout << "ERROR: robot pose data is empty. \n";
            return false;
        }
        nbr_data_ = rob_pose.size();
        vec_end2base_ = rob_pose;

        // generate mtr_end2base_
        Eigen::Matrix4f tmp_HTM;
        for (const Eigen::Vector<float, 6>& xyzwpr: vec_end2base_){
            tmp_HTM = CalibUtils::XYZWPRVecToHTM(xyzwpr);
            mtr_end2base_.push_back(tmp_HTM);
        }
        return true;
    }

    bool HandEyeCalib::SetRobPose(const std::vector<Eigen::Vector<float, 6>> rob_pose,
                                    Eigen::Vector<float, 6> ref_frame){
        if(rob_pose.size()==0){
            std::cout << "ERROR: robot pose data is empty. \n";
            return false;
        }
        nbr_data_ = rob_pose.size();
        vec_end2base_ = rob_pose;

        // generate mtr_end2base_
        Eigen::Matrix4f tmp_HTM;
        for (const Eigen::Vector<float, 6>& xyzwpr: vec_end2base_){
            tmp_HTM = CalibUtils::XYZWPRVecToHTM(ref_frame) * 
                      CalibUtils::XYZWPRVecToHTM(xyzwpr);
            mtr_end2base_.push_back(tmp_HTM);
        }
        return true;
    }

    bool HandEyeCalib::SetObjData(std::vector<cv::Point3f> pnts_data, CalibObj obj){
        // check the volume of the dataset
        if (nbr_data_ != pnts_data.size()){
            std::cout << "WARNING: Number of lines not match nbr_data_. \n";
            nbr_data_ = pnts_data.size();
        }

        calib_obj_ = obj;

        if (obj == CalibObj::SPHERE)
            ctr_pnts_ = pnts_data;
        else if(obj == CalibObj::BLOCK)
            edge_pnts_ = pnts_data;
        else{
            std::cout << "ERROR: calibration objection is false.\n";
            return false;
        }
        
        return true;
    }

    bool HandEyeCalib::SetObjData(std::vector<std::vector<cv::Point3f>> tri_edges_cam, 
                                  std::vector<std::vector<cv::Point3f>> tri_edges_rob, 
                                  CalibObj obj){
        // check calib object type
        if (obj != CalibObj::TRIANGLE_BOARD){
            std::cout << "ERROR: Calibration object set false.\n";
            return false;
        }

        calib_obj_ = obj;

        tri_edges_cam_ = tri_edges_cam;
        tri_edges_rob_ = tri_edges_rob;

        return true;
    }

    bool HandEyeCalib::run(SolveMethod method){
        // check calibration dataset
        if(nbr_data_ != mtr_end2base_.size() || nbr_data_ != ctr_pnts_.size()){
            std::cout << "ERROR: Error in data number";
            return false;
        }

        // construct algorithm parser and calculate result
        Eigen::Matrix3f Rx;
        Eigen::Vector3f tx;
        if (calib_obj_ == CalibObj::SPHERE){
            algorithm algor(Constructor::PNT_CONSTANT, method);
            algor.Simultaneous_Calib(mtr_end2base_, toEigenPoints(ctr_pnts_), Rx, tx);
        } else if(calib_obj_ == CalibObj::BLOCK){
            algorithm algor(Constructor::LINE_PARALLEL, method);
            algor.Simultaneous_Calib(mtr_end2base_, toEigenPoints(ctr_pnts_), Rx, tx);
        }

        // save calib result to mtr_cam2rob_ and vec_cam2rob_
        mtr_cam2rob_ = CalibUtils::RT2HomogeneousMatrix(Rx, tx);
        vec_cam2rob_ = CalibUtils::HTMToXYZWPRVec(mtr_cam2rob_);

        std::cout << "INFO: Calibration result in matrix formate: \n"
                    << mtr_cam2rob_ << std::endl;
        std::cout << "INFO: Calibration result in XYZWPR formate: \n"
                    << vec_cam2rob_.transpose() << std::endl;

        return true;
    }

    bool HandEyeCalib::linerFit(Eigen::VectorXf x){
        Eigen::MatrixXf F(2 * 3 * nbr_data_, 12);
        Eigen::VectorXf q(2 * 3 * nbr_data_);
        
        for (size_t i = 0; i < tri_edges_rob_.size(); i++)
        {
            for (size_t j = 0; j < mtr_end2base_.size(); j++)
            {
                Eigen::Vector3f p_cam(tri_edges_cam_[i][j].x,
                                        tri_edges_cam_[i][j].y,
                                        tri_edges_cam_[i][j].z);

                Eigen::Vector3f p_rob(tri_edges_rob_[i][j].x,
                                        tri_edges_rob_[i][j].y,
                                        tri_edges_rob_[i][j].z);

                Eigen::Matrix3f Ra = mtr_end2base_[j].block<3, 3>(0, 0);
                Eigen::Vector3f ta = mtr_end2base_[j].block<3, 1>(0, 3);
                auto kro = Eigen::kroneckerProduct(p_cam.transpose(), Ra);     // 3 * 9 matrix
                
                // std::cout << "Kro is: \n" << kro << std::endl;
                F.block<3, 12>(3 * j + 3 * i * mtr_end2base_.size(), 0) << kro, Ra;
                q.block<3, 1>(3 * j + 3 * i * mtr_end2base_.size(), 0) << p_rob - ta;
            }
        }

        // linear fit to solve equation
        Eigen::MatrixXf F_pinv = F.completeOrthogonalDecomposition().pseudoInverse();
        auto solutions = F_pinv * q;    // solutions = [vec(Rx); tx], size: 12, 1
        std::cout << "INFO: Regression solution is:\n " << solutions.transpose() << std::endl;

        Eigen::Vector3f R_col1 = solutions.block<3, 1>(0, 0);
        Eigen::Vector3f R_col3 = solutions.block<3, 1>(6, 0);
        R_col1.normalize();
        R_col3.normalize();

        Eigen::VectorXf sol(12);
        Eigen::Vector3f R_col2 = R_col1.cross(R_col3);
        std::cout << "INFO: true col2 is: " << R_col2.transpose() << std::endl;
        sol << R_col1, R_col2, R_col3, solutions.block<3, 1>(9, 0);
        std::cout << "INFO: Regression solution is:\n " << sol.transpose() << std::endl;

        // calculate regression error
        Eigen::VectorXf vec_err = q - F * x;
        float error = vec_err.norm()/nbr_data_;
        std::cout << "INFO: Regression error is: \n" << vec_err.transpose() << std::endl;

        return true;
    }

    bool HandEyeCalib::linerFit(){
        
        Eigen::MatrixXf A(3, tri_edges_rob_.size() * mtr_end2base_.size());
        Eigen::MatrixXf B(3, tri_edges_rob_.size() * mtr_end2base_.size());

        for (size_t i = 0; i < tri_edges_rob_.size(); i++)
        {
            for (size_t j = 0; j < mtr_end2base_.size(); j++)
            {
                Eigen::Vector3f hp_cam(tri_edges_cam_[i][j].x,
                                        tri_edges_cam_[i][j].z,
                                        1);

                Eigen::Vector4f hp_rob(tri_edges_rob_[i][j].x,
                                        tri_edges_rob_[i][j].y,
                                        tri_edges_rob_[i][j].z,
                                        1);

                Eigen::Vector3f p_flange = (mtr_end2base_[j].inverse() * hp_rob).head(3);
                B.block<3, 1>(0, i * mtr_end2base_.size() + j) = p_flange;
                A.block<3, 1>(0, i * mtr_end2base_.size() + j) = hp_cam;
            }
        }

        Eigen::Matrix3f X = B * (A.transpose() * A).inverse() * A.transpose();

        return true;
    }

    Eigen::Matrix4f HandEyeCalib::GetCalcResult(){
        return mtr_cam2rob_;
    }

    Eigen::Vector<float, 6> HandEyeCalib::GetCalcResultXYZWPR(){
        return vec_cam2rob_;
    }

    Eigen::Vector<float, 7> HandEyeCalib::GetCalcResultXYZQUA(){
        // get XYZ Qua from HTM
        return CalibUtils::HTMToXYZQUAVec(mtr_cam2rob_);
    }

    bool HandEyeCalib::CalcCalibError(std::vector<float>& dist_ctr_pnts, float& calib_error){
        // check calibration restlt
        if (mtr_cam2rob_.hasNaN()){
            std::cout << "ERROR: Calib result is empty." << std::endl;
            exit(-3);
        }

        // calculate sphere center points (in rob base frame)
        Eigen::Vector4f CtrPnt_base_h;
        Eigen::Vector4f CtrPnt_cam_h;
        std::cout << "INFO: Sphere center coordination in robot base frame is: \n";
        for (size_t i = 0; i < mtr_end2base_.size(); i++)
        {
            CtrPnt_cam_h << ctr_pnts_[i].x, 
                            ctr_pnts_[i].y, 
                            ctr_pnts_[i].z, 
                            1;
            CtrPnt_base_h = mtr_end2base_[i] * mtr_cam2rob_ * CtrPnt_cam_h;
            cv::Point3f pt(CtrPnt_base_h[0], CtrPnt_base_h[1], CtrPnt_base_h[2]);
            std::cout << "  " << pt << std::endl;
            ctr_pnts_base_.push_back(pt);
        }

        // calculate the standard deviation of the distance from robot base to spherecenter point
        std::cout << "INFO: The distance of center point in robot base frame is: \n";
        // std::vector<float> dist_ctr_pnts;
        for (const auto& point : ctr_pnts_base_) {
            float dist = std::sqrt(point.dot(point));
            dist_ctr_pnts.push_back(dist);
            std::cout << "  " << dist;
        }
        std::cout << std::endl;
        float mean = std::accumulate(dist_ctr_pnts.begin(), dist_ctr_pnts.end(), 0.0) / dist_ctr_pnts.size();
        float variance = 0.0;
        for (auto x : dist_ctr_pnts)
            variance += std::pow(x - mean, 2);
        calib_error_ = std::sqrt(variance / (dist_ctr_pnts.size() - 1));
        calib_error = calib_error_;
        std::cout << "INFO: Standard deviation is: " << calib_error_ << std::endl;

        return true;
    }

    void HandEyeCalib::drawClusters(cv::Mat& img, std::vector<cv::Point2f>& centers, std::vector<std::vector<cv::Point2f>>& points) {
        // draw cluster center
        for (int i = 0; i < centers.size(); i++) {
            circle(img, centers[i], 5, cv::Scalar(0, 0, 255), -1);
        }

        // draw point cloud of circle
        for (int i = 0; i < points.size(); i++) {
            cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);
            for (int j = 0; j < points[i].size(); j++)
                circle(img, points[i][j], 2, color, -1);
        }
    }

    
} // namespace LineScanner

namespace Camera
{

} // namespace Camera
