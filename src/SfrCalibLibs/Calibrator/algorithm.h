/**
@file : algorithm.h
@package : algorithm library
@brief CPP common example functionality.
@copyright (c) 2023, Shanghai Fanuc Ltd.
@version 16.03.2023, SFR: initial version
*/
#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <iostream>
#include <vector>

#include <Eigen/Core>

namespace LineScanner
{
    enum Constructor {          // Ways to construct linear equations
        PNT_CONSTANT = 0,       // constant point in world (robot base) frame 
        LINE_PARALLEL = 1,      // parallel line in world (robot base) frame
    };

    enum SolveMethod {          // Methods to solve calibration linear equations
        ITERATION = 0,          // based on Rodriguez 
        REGRESSION = 1          // based on kronecker product
    };

    class algorithm{
    private:
        Eigen::Matrix3f R_solution_;
        Eigen::Vector3f t_solution_;
        Constructor construct_type_;          // label to imply equation constructe type
        SolveMethod solve_method_;

        // dataset when calibration is simultaneous
        std::vector<Eigen::Matrix4f> mtr_end2base_;
        std::vector<Eigen::Vector3f> ctr_pnts_;
        int nbr_data_;

        // dataset when calibration is separate

    public:
        algorithm(Constructor type, SolveMethod method_);
        
        /* @brief Simultaneous Calibration: Calculate rotational component and
            translational component (hand-eye relation) simultaneously, 
            typically used for calibration when calib_obj_ is sphere.

        @param htm_end2base: The pose series of robot end (flange) to base, represent
            as HTM ([R, t; 0, 1]) of flange frame in base frame
        @param p_inCam: The coordination of dataset (points) in camera frame, if (Constructor)
            type == PNT_CONSTANT, the point (sphere center) is constant in robot base frame, 
            while if (Constructor) type == LINE_PARALLEL, the points (block edge) is collinear.
        @param R_solution: The calibration result of rotational component of hand-eye relation
        @param t_solution: The calibration result of translational component of hand-eye relation

        @return bool: The status of calibrate procession
        */
        bool Simultaneous_Calib(std::vector<Eigen::Matrix4f> htm_end2base, 
                                std::vector<Eigen::Vector3f> p_cam, 
                                Eigen::Matrix3f & R_solution,
                                Eigen::Vector3f & t_solution);
        
        bool Separate_Calib(std::vector<Eigen::Matrix4f> htm_end2base_r,
                            std::vector<Eigen::Vector3f> p_cam_r, 
                            std::vector<Eigen::Matrix4f> htm_end2base_t,
                            std::vector<Eigen::Vector3f> p_cam_t,
                            Eigen::Matrix3f & R_solution,
                            Eigen::Vector3f & t_solution);

    private:
        void construct_linear_equation_iter(Eigen::Matrix3f Rx_init, 
                                            Eigen::Vector3f tx_init,
                                            Eigen::MatrixXf & F,
                                            Eigen::VectorXf & q);
        
        void construct_linear_equation_kron(Eigen::MatrixXf & F, Eigen::VectorXf & q);

        Eigen::Matrix3f rota_schmidt_orth(Eigen::Matrix3f r_mat);

    };

} // namespace LineScanner

// namespace Camera
// {

// } // namespace Camera

#endif