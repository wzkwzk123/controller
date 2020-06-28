//
// Created by cjh on 5/9/20.
//
#include "mpc_controller.h"
namespace control{
    bool MPCController::LoadControlConf() {
        ts_ = mpcParam_.ts;
        cf_ = vehParam_.cf;
        cr_ = vehParam_.cr;
        wheelBase_ = vehParam_.wheelBase;
        massFront_ = vehParam_.massFront;
        massRear_ = vehParam_.massRear;
        mass_ = massFront_ + massRear_;
        lf_ = wheelBase_ * (1.0 - massFront_ / mass_);
        lr_ = wheelBase_ * (1.0 - massRear_ / mass_);
        iz_ = lf_ * lf_ * massFront_ + lr_ * lr_ * massRear_;

        maxIteration_ = mpcParam_.maxIteration;
        stateSize_ = mpcParam_.stateSize;
        controlSize_ = mpcParam_.controlSize;
        controlHorizon_ = mpcParam_.controlHorizon;
        return true;
    }

    bool MPCController::Init() {
        if(!LoadControlConf()){
            std::cout << "load control configuration failed";
        }
        // Matrix init -- initialize the model matrix
        matrix_a_ = Matrix::Zero(stateSize_, stateSize_);
        matrix_ad_ = Matrix::Zero(controlSize_, controlSize_);
        //TODO specify the model A B
        // note -- matrix_c in apollo is the

        matrix_state_ = Matrix::Zero(stateSize_, 1);
        control_matrix_ = Matrix::Zero(controlSize_, 1);

        matrix_q_ = mpcParam_.matrix_q;
        matrix_r_ = mpcParam_.matrix_r;

        max_acceleration_ = vehParam_.max_acceleration;

        return true;
    }

    void MPCController::UpdateState(){
        // state from simulator X, Y, yaw

    }

    // TODO change matrix A B according to new state from simulator
    void MPCController::UpdateMatrix(){

    }

    bool MPCController::ComputeControlCommand() {
        // update state and weight matrix
        UpdateState();
        UpdateMatrix();

        Matrix refState(stateSize_, 1);
        std::vector<Matrix> reference(controlHorizon_, refState);

        Matrix lower_bound(mpcParam_.controlSize, 1);
        lower_bound << -steer_single_direction_max_degree_, max_deceleration_;

        Matrix upper_bound(mpcParam_.controlSize, 1);
        upper_bound << steer_single_direction_max_degree_, max_acceleration_;

        conLowBound_ = mpcParam_.conLowBound;
        conUpperBound_ = mpcParam_.conUpperBound;
        std::vector<Eigen::MatrixXd> control(controlHorizon_, control_matrix_);


        if(!SolveLinearMPC(matrix_ad_, matrix_bd_, matrix_cd_,matrix_q_updated_,
                matrix_r_updated_, lower_bound, upper_bound, matrix_state_, reference,
                mpc_eps_, mpc_max_iteration_, control)){
            std::cout << "mpc failed, when mpc_controller call mpc_solver";
        }

        // mpc steer result
        // double steer_angle = control[0](0, 0) * / M_PI * steer_transmission_ratio_ / steer_single_direction_max_degree_ * 100;
        // steer_angle = Clamp(steer_angle, -100.0, 100); // output of MPC controller

        // TODO figure out the steer_limited


    }



}