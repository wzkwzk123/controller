//
// Created by cjh on 5/9/20.
//

#ifndef CONTROLLER_MPC_CONTROLLER_H
#define CONTROLLER_MPC_CONTROLLER_H

#include "controller.h"
#include "parameters.h"
#include <glog/logging.h>
#include <string.h>
#include <iostream>
#include "mpc_solver.h"
#include <time.h>
#include "math_utils.h"

namespace control{
    // Input --
    // Output -- Acceleration, steer angle.
    class MPCController : public Controller{
    public:
        MPCController(const VehicleParam& vehParam, const MPCControllerParam& mpcParam){
            vehParam_ = vehParam;
            mpcParam_ = mpcParam;
        };
        virtual ~MPCController();

        bool LoadControlConf();
        bool Init() override ;
        void UpdateState();
        void UpdateMatrix();
        bool ComputeControlCommand();

    protected:
        VehicleParam vehParam_;
        MPCControllerParam mpcParam_;

        double ts_{0.};

        // vehicle conf
        double cf_{0.};
        double cr_{0.};
        double wheelBase_{0.};
        double massFront_{0.0};
        double massRear_{0.0};
        double mass_{0.};

        // distance from wheel center to COM
        double lf_{0.};
        double lr_{0.};

        // rotational inertia
        double iz_{0.};
        // ratio between the turn of the steering wheel and the turn of the wheel
        double steer_transmission_ratio_{0.};
        // the maximum of steer
        double steer_single_direction_max_degree_{0.};

        double max_acceleration_{0.};
        double max_deceleration_{0.};

        int stateSize_{0};
        int controlSize_{0};
        int controlHorizon_{0};

        // mpc algorithm parameters
        int maxIteration_{0};

        // vehicle state matrix
        Eigen::MatrixXd matrix_a_;
        // discrete vehicle state matrix
        Eigen::MatrixXd matrix_ad_;

        Eigen::MatrixXd matrix_a_coeff_;
        // 4 by 1 matrix; state matrix
        Eigen::MatrixXd matrix_state_;
        Eigen::MatrixXd control_matrix_;
        Eigen::MatrixXd conLowBound_;
        Eigen::MatrixXd conUpperBound_;

        // control matrix
        Eigen::MatrixXd matrix_b_;
        Eigen::MatrixXd matrix_bd_;

        // offset matrix
        Eigen::MatrixXd matrix_c_;
        Eigen::MatrixXd matrix_cd_;

        // control weighting matrix
        Eigen::MatrixXd matrix_r_;
        // update control weighting matix
        Eigen::MatrixXd matrix_r_updated_;
        // state weighting matrix
        Eigen::MatrixXd matrix_q_;
        // update control weighting matrix
        Eigen::MatrixXd matrix_q_updated_;

        // heading error of the last control cycle
        double previous_heading_error_{0.};
        double previous_lateral_error_{0.};

        // number of iterations
        int mpc_max_iteration_{0};
        // threshold for computation
        double mpc_eps_{0.0};


    };

}



#endif //CONTROLLER_MPC_CONTROLLER_H
