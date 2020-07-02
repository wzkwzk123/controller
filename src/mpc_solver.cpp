//
// Created by cjh on 5/12/20.
//
#include "mpc_solver.h"

namespace control{
    using Matrix = Eigen::MatrixXd;
    bool SolveLinearMPC(
            const Eigen::MatrixXd &matrix_a, const Eigen::MatrixXd &matrix_b,
            const Eigen::MatrixXd &matrix_c, const Eigen::MatrixXd &matrix_q,
            const Eigen::MatrixXd &matrix_r,
            const Eigen::MatrixXd &matrix_lower,
            const Eigen::MatrixXd &matrix_upper,
            const Eigen::MatrixXd &matrix_initial_state,
            const std::vector<Eigen::MatrixXd> &reference,
            const double eps,
            const int max_iter,
            std::vector<Eigen::MatrixXd> &control
            ){

        size_t horizon = reference.size();
        // update augment reference matrix_t_
        Matrix matrix_t = Matrix::Zero(matrix_b.rows() * horizon, 1);
        for(int i=0; i<horizon; ++i){
            // sub matrix operate
            matrix_t.block(i * reference[0].size(), 0, reference[0].size(), 1) = reference[i];
        }

        // update augment control
        Matrix matrix_v = Matrix::Zero(control[0].rows()*horizon, 1);
        for(int i=0; i<horizon; ++i){
            matrix_v.block(i * control[0].rows(), 0, control[0].rows(), 1) = control[i];
        }

        std::vector<Matrix> matrix_a_power(horizon);
        /*
         *   【A
         *     A*A
         *     ***
         *     A^horizon-1
         *   】
         */
        matrix_a_power[0] = matrix_a;
        for(size_t i=0; i<matrix_a_power.size(); ++i){
            matrix_a_power[i] = matrix_a * matrix_a_power[i-1];
        }

        /*
         * [B   0 ...    0
         *  AB  B ...    0
         *  AAB AB B ... 0
         *  AAAB ...     B
         * ]
         */
        Matrix matrix_k = Matrix::Zero(matrix_b.rows() * horizon, matrix_b.cols() * horizon);
        // Matrix matrix_k = Matrix::Zero(matrix_b.row() * horizon, matrix_b.cols() * horizon);
        matrix_k.block(0, 0, matrix_b.rows(), matrix_b.cols()) = matrix_b;
        for(size_t i = 1; i<horizon; i++){
            for(size_t j=0; j<i; j++){
                matrix_k.block(i*matrix_b.rows(), j*matrix_b.cols(), matrix_b.rows(), matrix_b.cols()) =
                        matrix_a_power[i-j-1] * matrix_b;
            }
            matrix_k.block(i*matrix_b.rows(), i*matrix_b.cols(), matrix_b.rows(), matrix_b.cols())= matrix_b;
        }

        Matrix matrix_m = Matrix::Zero(matrix_b.rows()*horizon, 1);
        Matrix matrix_qq = Matrix::Zero(matrix_k.rows(), matrix_k.rows());
        Matrix matrix_rr = Matrix::Zero(matrix_k.cols()*horizon, matrix_k.cols()*horizon);

        Matrix matrix_ll = Matrix::Zero(horizon*matrix_lower.rows(), 1);
        Matrix matrix_uu = Matrix::Zero(horizon*matrix_lower.rows(), 1);

        // TODO find the function of the matrix_cc
        Matrix matrix_cc = Matrix::Zero(horizon * matrix_c.rows(), 1);
        Matrix matrix_aa = Matrix::Zero(horizon * matrix_a.rows(), matrix_a.cols());
        matrix_aa.block(0, 0, matrix_a.rows(), matrix_a.cols()) = matrix_a;

        for(size_t i=1; i<horizon; i++){
            matrix_aa.block(i*matrix_a.rows(), 0, matrix_a.rows(), matrix_a.cols()) =
                    matrix_a * matrix_aa.block((i-1)*matrix_a.rows(), 0, matrix_a.rows(), matrix_a.cols());
        }

        matrix_m.block(0, 0, matrix_a.rows(), 1) = matrix_a * matrix_initial_state;
        for(size_t i=1; i<horizon; i++){
            matrix_m.block(i*matrix_a.rows(), 0, matrix_a.rows(), 1) =
                    matrix_a * matrix_m.block((i-1)*matrix_a.rows(), 0, matrix_a.rows(), 1);
        }

        matrix_cc.block(0, 0, matrix_c.rows(), 1) = matrix_c;
        for (size_t i = 1; i < horizon; ++i) {
            matrix_cc.block(i * matrix_c.rows(), 0, matrix_c.rows(), 1) =
                    matrix_cc.block((i - 1) * matrix_c.rows(), 0, matrix_c.rows(), 1) +
                    matrix_aa.block((i - 1) * matrix_a.rows(), 0, matrix_a.rows(),
                                    matrix_a.cols()) *
                    matrix_c;
        }

        // compute
        for (size_t i = 0; i < horizon; ++i) {
            matrix_ll.block(i * (control)[0].rows(), 0, (control)[0].rows(), 1) =
                    matrix_lower;
            matrix_uu.block(i * (control)[0].rows(), 0, (control)[0].rows(), 1) =
                    matrix_upper;
            matrix_qq.block(i * matrix_q.rows(), i * matrix_q.rows(), matrix_q.rows(),
                            matrix_q.rows()) = matrix_q;
            matrix_rr.block(i * matrix_r.rows(), i * matrix_r.rows(), matrix_r.cols(),
                            matrix_r.cols()) = matrix_r;
        }

        //

        Matrix matrix_m1 = matrix_k.transpose() * matrix_qq * matrix_k + matrix_rr;
        Matrix matrix_m2 = matrix_k.transpose() * matrix_qq * (matrix_m + matrix_cc - matrix_t);

        // Format in qp_solver
        /**
         * *           min_x  : q(x) = 0.5 * x^T * Q * x  + x^T c
         * *           with respect to:  A * x = b (equality constraint)
         * *                             C * x >= d (inequality constraint)
         * **/

        // TODO(QiL) : change qp solver to box constraint or substitute QPOASES
        // Method 1: QPOASES
        Matrix matrix_inquality_constraint_ll = Matrix::Identity(matrix_ll.rows(), matrix_ll.rows());
        Matrix matrix_inquality_constraint_uu = Matrix::Identity(matrix_uu.rows(), matrix_uu.rows());
        Matrix matrix_inquality_constraint = Matrix::Zero(matrix_ll.rows()+matrix_uu.rows(), matrix_ll.cols());
        matrix_inquality_constraint << matrix_inquality_constraint_ll, -matrix_inquality_constraint_uu;
        Matrix matrix_inequality_boundary = Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.cols());
        matrix_inequality_boundary << matrix_ll, -matrix_uu;

        Matrix matrix_equality_constrain = Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.rows());
        Matrix matrix_equality_boundary = Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.cols());

        std::unique_ptr<QpSolver> qp_solver(new ActiveSetQpSolver(matrix_m1, matrix_m2, matrix_inquality_constraint, matrix_inequality_boundary,matrix_equality_constrain, matrix_equality_boundary));

        // TODO no match function
        // std::unique_ptr<ActiveSetQpSolver> qp_solver;
        bool result = qp_solver->Solve();
        if(!result){
            std::cout << "Linear MPC solver failed when solve the qp problem";
        }
        matrix_v = qp_solver->params();

        for(size_t i=0; i<horizon; i++){
            control[i] = matrix_v.block(i*control[0].rows(), 0, control[0].rows(), 1);
        }

        return true;
    }
}
