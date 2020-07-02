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

    Status MPCController::Init(const ControllerParam& control_conf) {
        if(!LoadControlConf()){
            std::cout << "load control configuration failed";
        }
        // Matrix init -- initialize the model matrix
        // matrix_a_ * x +  matrix_ad_
        matrix_a_ = Matrix::Zero(stateSize_, stateSize_);
        matrix_ad_ = Matrix::Zero(stateSize_, stateSize_);
        //TODO specify the model A B
        // note -- matrix_c in apollo is the

        matrix_state_ = Matrix::Zero(stateSize_, 1);
        control_matrix_ = Matrix::Zero(controlSize_, 1);

        matrix_a_coeff_ = Matrix ::Zero(stateSize_, stateSize_);

        matrix_b_ = Matrix::Zero(stateSize_, controlSize_);
        matrix_bd_ = Matrix::Zero(stateSize_, controlSize_);

        // todo understand the function
        matrix_c_ = Matrix::Zero(stateSize_, 1);
        matrix_c_(5, 0) = 1.0;

        matrix_q_ = mpcParam_.matrix_q;
        matrix_r_ = mpcParam_.matrix_r;

        // update
        matrix_q_updated_ = matrix_q_;
        matrix_r_updated_ = matrix_r_;

        max_acceleration_ = vehParam_.max_acceleration;

        AINFO << "MPC Controller is initialized";
        return Status::ok();
    }

    void MPCController::UpdateState(){
        // state from simulator X, Y, yaw

    }

    // TODO change matrix A B according to new state from simulator
    void MPCController::UpdateMatrix(){

    }

    Status MPCController::computeControlCommand(
            const Localization& localization,
            const Chassis& chassis,
            const Trajectory& trajectory,
            ControlCommand& controlCommand) {
        // update state and weight matrix
        UpdateState();
        UpdateMatrix();

        Matrix refState(stateSize_, 1);
        refState << 0., 0., 0., 0., 0. ,0.;
        std::vector<Matrix> reference(controlHorizon_, refState);

        Matrix lower_bound(mpcParam_.controlSize, 1);
        lower_bound << -steer_single_direction_max_degree_, max_deceleration_;

        Matrix upper_bound(mpcParam_.controlSize, 1);
        upper_bound << steer_single_direction_max_degree_, max_acceleration_;

        conLowBound_ = mpcParam_.conLowBound;
        conUpperBound_ = mpcParam_.conUpperBound;
        std::vector<Eigen::MatrixXd> control(controlHorizon_, control_matrix_);

        // \sum x^T matrix_q_updated_ x + u^T matrix_r_updated_ u
        // s.t. x = matrix_ad_ x + matrix_bd_ u + matrix_cd_
        //      lower_bound <= u < = upper_bound
        if(!SolveLinearMPC(matrix_ad_, matrix_bd_, matrix_cd_,matrix_q_updated_,
                matrix_r_updated_, lower_bound, upper_bound, matrix_state_, reference,
                mpc_eps_, mpc_max_iteration_, control)){
            std::cout << "mpc failed, when mpc_controller call mpc_solver";
        }

        // mpc steer result
        // double steer_angle = control[0](0, 0) * / M_PI * steer_transmission_ratio_ / steer_single_direction_max_degree_ * 100;
        // steer_angle = Clamp(steer_angle, -100.0, 100); // output of MPC controller

        // TODO figure out the steer_limited

        return Status::ok();
    }

    std::string MPCController::Name() const { return name_; }



}