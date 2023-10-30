/**
@file : calibrator.h
@package : calibrator library
@brief CPP common example functionality.
@copyright (c) 2023, Shanghai Fanuc Ltd.
@version 16.03.2023, SFR: initial version
*/
#ifndef CALIBRATOR_H
#define CALIBRATOR_H
#include "common.h"
#include "algorithm.h"

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

namespace LineScanner
{
    class HandEyeCalib {
    private:
        CalibObj calib_obj_;
        CalibType type_;

        int nbr_data_; 
        std::vector<Eigen::Vector<float, 6>> vec_end2base_;
        std::vector<Eigen::Matrix4f> mtr_end2base_;
        
        std::vector<cv::Point3f> ctr_pnts_;         // sphere center points (camera frame)
        std::vector<cv::Point3f> edge_pnts_;        // block edge points (camera frame) 
        std::vector<std::vector<cv::Point3f>> tri_edges_cam_;       // triangle two edge points (camera frame)
        std::vector<std::vector<cv::Point3f>> tri_edges_rob_;       // triangle two edge points (robot base frame)

        std::vector<cv::Point3f> ctr_pnts_base_;    // sphere center points (robot base frame)
        std::vector<cv::Point3f> edge_pnts_base_;   // block edge points (robot base frame) 

        Eigen::Matrix4f mtr_cam2rob_;
        Eigen::Vector<float, 6> vec_cam2rob_;

        float calib_error_;
    
    public:
        HandEyeCalib();

        HandEyeCalib(CalibType type);

        ~HandEyeCalib();

        bool SetRobPose(const std::vector<Eigen::Vector<float, 6>> rob_pose);

        bool SetRobPose(const std::vector<Eigen::Vector<float, 6>> rob_pose,
                        Eigen::Vector<float, 6> ref_frame);

        bool SetObjData(std::vector<cv::Point3f> pnts_data, CalibObj obj);

        bool SetObjData(std::vector<std::vector<cv::Point3f>> tri_edges_cam, 
                        std::vector<std::vector<cv::Point3f>> tri_edges_rob, 
                        CalibObj obj);

        bool run(SolveMethod method);
        
        bool linerFit(Eigen::VectorXf x);
        
        bool linerFit();

        Eigen::Matrix4f GetCalcResult();

        Eigen::Vector<float, 6> GetCalcResultXYZWPR();

        Eigen::Vector<float, 7> GetCalcResultXYZQUA();

        bool CalcCalibError(std::vector<float>& dist_ctr_pnts, float& calib_error);

    private:
        void drawClusters(cv::Mat& img, 
                            std::vector<cv::Point2f>& centers, 
                            std::vector<std::vector<cv::Point2f>>& points);
    };
}

#endif
