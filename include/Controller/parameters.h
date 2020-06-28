//
// Created by cjh on 5/11/20.
//

#ifndef CONTROLLER_PARAMETERS_H
#define CONTROLLER_PARAMETERS_H

namespace control{
    using Matrix = Eigen::MatrixXd;

    struct VehicleParam{
        double cf{155495.};
        double cr{155495};
        double wheelBase{1.7}; //
        int steerTransmissionRatio{1};
        double massFront{0.4}; // kg
        double massRear{0.5};
        double max_acceleration{3.0};
        double max_deceleration{-2.0};
    };

    struct MPCControllerParam{
        double ts{0.1};
        int stateSize{6};
        int controlSize{2};

        double maxSteerAngle{180.};
        double maxLatAcc{1.};
        double maxLonAcc{3.};
        double maxLonDec{-3.};

        int maxIteration{100};
        int controlHorizon{10};

        Matrix matrix_q = Matrix::Zero(stateSize, stateSize);
        Matrix matrix_r = Matrix::Identity(controlSize, controlSize);
        Matrix conLowBound = Matrix::Zero(controlSize, 1);
        Matrix conUpperBound = Matrix::Zero(controlSize, 1);


    };
}


#endif //CONTROLLER_PARAMETERS_H
