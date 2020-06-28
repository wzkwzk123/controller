//
// Created by cjh on 5/12/20.
// code from baidu apollo
//

#ifndef CONTROLLER_MPC_SOLVER_H
#define CONTROLLER_MPC_SOLVER_H

#include <vector>
#include "eigen3/Eigen/Core"
#include <memory>
#include <qp_solver/qp_solver.h>
#include "active_set_qp_solver.h"
#include <iostream>

/**
 * @brief Solver for discrete-time model predictive control problem.
 * @param matrix_a The system dynamic matrix
 * @param matrix_b The control matrix
 * @param matrix_c The disturbance matrix
 * @param matrix_q The cost matrix for control state
 * @param matrix_lower The lower bound control constrain matrix
 * @param matrix_upper The upper bound control constrain matrix
 * @param matrix_initial_state The initial state matrix
 * @param reference The control reference vector with respect to time
 * @param eps The control convergence tolerance
 * @param max_iter The maximum iterations for solving ARE
 * @param control The feedback control matrix (pointer)
 */


namespace control{

    bool SolveLinearMPC(
            const Eigen::MatrixXd &matrix_a, const Eigen::MatrixXd &matrix_b,
            const Eigen::MatrixXd &matrix_c, const Eigen::MatrixXd &matrix_q,
            const Eigen::MatrixXd &matrix_r, const Eigen::MatrixXd &matrix_lower,
            const Eigen::MatrixXd &matrix_upper,
            const Eigen::MatrixXd &matrix_initial_state,
            const std::vector<Eigen::MatrixXd> &reference,
            const double eps,
            const int max_iter,
            std::vector<Eigen::MatrixXd> &control
            );

}

#endif //CONTROLLER_MPC_SOLVER_H
