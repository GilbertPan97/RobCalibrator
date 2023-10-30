/**
@file : algorithm.cpp
@package : algorithm library
@brief CPP common example functionality.
@copyright (c) 2023, Shanghai Fanuc Ltd.
@version 16.03.2023, SFR: initial version
*/
#include "algorithm.h"
#include "utils.h"

#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <unsupported/Eigen/KroneckerProduct>

namespace LineScanner
{

    algorithm::algorithm(Constructor type, SolveMethod method_){
        construct_type_ = type;
        solve_method_ = method_;
    }

    bool algorithm::Simultaneous_Calib(std::vector<Eigen::Matrix4f> htm_end2base, 
                                        std::vector<Eigen::Vector3f> p_cam, 
                                        Eigen::Matrix3f &R_solution,
                                        Eigen::Vector3f &t_solution){
        // get calib dataset info
        if (htm_end2base.size() == p_cam.size()){
            nbr_data_ = htm_end2base.size();
            mtr_end2base_ = htm_end2base, ctr_pnts_ = p_cam;
        } else {
            std::cout << "ERROR: calib dataset is not match. \n";
            return false;
        }

        // calculate execution
        if (solve_method_ == SolveMethod::ITERATION){
            // 1. set iterative initial value
            Eigen::Matrix3f Rx_init = Eigen::Matrix3f::Identity();
            Eigen::Vector3f tx_init(1.0, 1.0, 1.0);

            // 2. iterative calibrate
            float error;
            int iter = 0;
            do{
                // construct linear equation: F * delta = q
                // calculate delta: delta_R & delta_t
                // delta: deviation between initial iterative and true value
                // n data can only build n-1 equation, which can be stack to F & q
                Eigen::MatrixXf F(3 * (nbr_data_ - 1), 6);
                Eigen::VectorXf q(3 * (nbr_data_ - 1));
                construct_linear_equation_iter(Rx_init, tx_init, F, q);

                // linear fit to solve equation
                Eigen::MatrixXf F_pinv = F.completeOrthogonalDecomposition().pseudoInverse();
                Eigen::VectorXf delta;
                delta = F_pinv * q;

                // iterative initial value
                Eigen::Vector3f delta_R = delta.block<3, 1>(0, 0);
                Eigen::Vector3f delta_t = delta.block<3, 1>(3, 0);
                Rx_init = CalibUtils::rodrigues(delta_R) * Rx_init;
                tx_init = tx_init + delta_t;

                error = delta.norm();
                iter++;
            }while(error >= 1.0e-4 && iter <= 300);

            // 3. output solve result
            R_solution = Rx_init, t_solution = tx_init;
            std::cout << "INFO: Iterative solution is: \n" << R_solution << std::endl
                        << t_solution.transpose() << std::endl;
        } 

        if(solve_method_ == SolveMethod::REGRESSION){
            Eigen::MatrixXf F(3 * (nbr_data_ - 1), 12);
            Eigen::VectorXf q(3 * (nbr_data_ - 1));
            construct_linear_equation_kron(F, q);

            // linear fit to solve equation
            Eigen::MatrixXf F_pinv = F.completeOrthogonalDecomposition().pseudoInverse();
            auto solutions = F_pinv * q;        // solutions = [vec(Rx); tx], size: 12, 1
            std::cout << "INFO: Regression solution is: \n" << solutions.transpose() << std::endl;

            // calculate regression error
            Eigen::VectorXf vec_err = q - F * solutions;
            float error = vec_err.norm()/nbr_data_;
            std::cout << "INFO: Regression error is: \n" << vec_err.transpose() << std::endl;

            // restore Rx and tx
            R_solution << solutions.block<3,1>(0, 0),
                            solutions.block<3,1>(3, 0),
                            solutions.block<3,1>(6, 0);
            R_solution = rota_schmidt_orth(R_solution);
            t_solution << solutions.block<3,1>(9, 0);

            std::cout << "INFO: R_ort solution is: \n" << R_solution << std::endl; 
        }

        return true;
    }

    bool algorithm::Separate_Calib(std::vector<Eigen::Matrix4f> htm_end2base_r,
                                    std::vector<Eigen::Vector3f> p_cam_r, 
                                    std::vector<Eigen::Matrix4f> htm_end2base_t,
                                    std::vector<Eigen::Vector3f> p_cam_t,
                                    Eigen::Matrix3f & R_solution,
                                    Eigen::Vector3f & t_solution){
        // get calib dataset info
        return true;
    }

    void algorithm::construct_linear_equation_iter(Eigen::Matrix3f Rx_init, Eigen::Vector3f tx_init,
                                                    Eigen::MatrixXf & F, Eigen::VectorXf & q){
        // check F & q size
        if(F.rows() != 3 * (nbr_data_ - 1) || F.cols() != 6){
            F.resize(3 * (nbr_data_ - 1), 6);
            std::cout << "WARNING: The size of 'F' is illegal, which is adjusted. \n";
        };
        if(q.size() != 3 * (nbr_data_ - 1)){
            q.resize(3 * (nbr_data_ - 1));
            std::cout << "WARNING: The size of 'q' is illegal, which is adjusted. \n";
        };

        // construct equation
        Eigen::Matrix3f Ra_i, Ra_j;     // j = i-1
        Eigen::Vector3f ta_i, ta_j, tsc_i, tsc_j;
        for (size_t i = 1; i < nbr_data_; i++)
        {
            // allocate data
            CalibUtils::HomogeneousMtr2RT(mtr_end2base_[i], Ra_i, ta_i);
            CalibUtils::HomogeneousMtr2RT(mtr_end2base_[i-1], Ra_j, ta_j);
            tsc_i = ctr_pnts_[i];
            tsc_j = ctr_pnts_[i-1];

            // calculate linear equation coefficient (Fi)
            Eigen::Matrix3f F_Ri;    // coefficient of delta_R
            Eigen::Matrix3f F_ti;    // coefficient of delta_t
            F_Ri = -Ra_i * CalibUtils::skew(Rx_init * tsc_i)+ 
                    Ra_j * CalibUtils::skew(Rx_init * tsc_j);
            F_ti = Ra_i - Ra_j;

            Eigen::MatrixXf Fi(3, 6);
            Fi.block<3, 3>(0, 0) = F_Ri;
            Fi.block<3, 3>(0, 3) = F_ti;

            // calculate linear value (qi) of equation 
            Eigen::Vector3f qi;
            qi = -(Ra_i * Rx_init * tsc_i + Ra_i * tx_init + ta_i) +
                    (Ra_j * Rx_init * tsc_j + Ra_j * tx_init + ta_j);

            // stack Fi and qi back to F and q (following to row)
            F.block<3, 6>(3 * (i-1), 0) = Fi;
            q.block<3, 1>(3 * (i-1), 0) = qi;
        }
        // std::cout << "F block is: \n" << F << std::endl;
        // std::cout << "q block is: \n" << q << std::endl;
    }

    void algorithm::construct_linear_equation_kron(Eigen::MatrixXf & F, Eigen::VectorXf & q){
        // check F & q size
        if(F.rows() != 3 * (nbr_data_ - 1) || F.cols() != 12){
            F.resize(3 * (nbr_data_ - 1), 12);
            std::cout << "WARNING: The size of 'F' is illegal, which is adjusted. \n";
        };
        if(q.size() != 3 * (nbr_data_ - 1)){
            q.resize(3 * (nbr_data_ - 1));
            std::cout << "WARNING: The size of 'q' is illegal, which is adjusted. \n";
        };

        // construct equation
        Eigen::Matrix3f Ra_i, Ra_j;     // j = i-1
        Eigen::Vector3f ta_i, ta_j, tsc_i, tsc_j;
        for (size_t i = 1; i < nbr_data_; i++)
        {
            // allocate data
            CalibUtils::HomogeneousMtr2RT(mtr_end2base_[i], Ra_i, ta_i);
            CalibUtils::HomogeneousMtr2RT(mtr_end2base_[i-1], Ra_j, ta_j);
            tsc_i = ctr_pnts_[i];
            tsc_j = ctr_pnts_[i-1];

            // calculate linear equation coefficient (Fi)
            auto kro_i = Eigen::kroneckerProduct(tsc_i.transpose(), Ra_i);
            auto kro_j = Eigen::kroneckerProduct(tsc_j.transpose(), Ra_j);

            Eigen::MatrixXf Fi(3, 12);
            Fi.block<3, 9>(0, 0) << kro_j - kro_i;
            Fi.block<3, 3>(0, 9) << Ra_j - Ra_i;

            // std::cout << "Kro_j - Kro_i is: \n" << kro_j - kro_i << std::endl;
            // std::cout << "Ra_j - Ra_i is: \n" << Ra_j - Ra_i << std::endl;
            // std::cout << "F_i is: \n" << Fi << std::endl;

            // calculate linear value (qi) of equation 
            Eigen::Vector3f qi = -(ta_j - ta_i);

            // stack Fi and qi back to F and q (following to row)
            F.block<3, 12>(3 * (i-1), 0) = Fi;
            q.block<3, 1>(3 * (i-1), 0) = qi;
        }
        // std::cout << "F block is: \n" << F << std::endl;
        // std::cout << "q block is: \n" << q << std::endl;
    }

    Eigen::Matrix3f algorithm::rota_schmidt_orth(Eigen::Matrix3f r_mat){
        Eigen::Vector3f alpha1 = r_mat.col(0);
        Eigen::Vector3f alpha2 = r_mat.col(1);
        Eigen::Vector3f alpha3 = r_mat.col(2);

        Eigen::Vector3f beta1 = alpha1;
        Eigen::Vector3f beta2 = alpha2 - alpha2.dot(beta1)/beta1.dot(beta1) * beta1;
        Eigen::Vector3f beta3 = alpha3 - alpha3.dot(beta1)/beta1.dot(beta1) * beta1 - 
                                alpha3.dot(beta2)/beta2.dot(beta2) * beta2;
        
        beta1.normalize(), beta2.normalize(), beta3.normalize();

        Eigen::Matrix3f r_orth_mat;
        r_orth_mat << beta1, beta2, beta3;

        if (!CalibUtils::isRotatedMatrix(r_orth_mat))
            exit(-1);
        
        return r_orth_mat;
    }

} // namespace LineScanner

// namespace Camera
// {

// } // namespace Camera