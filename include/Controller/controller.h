//
// Created by cjh on 5/11/20.
//

#ifndef CONTROLLER_CONTROLLER_H
#define CONTROLLER_CONTROLLER_H

#include "eigen3/Eigen/Core"
#include "status.h"
#include "parameters.h"
#include "log.h"

namespace control{

    class Controller{
        // basic class for all controllers
    public:
        Controller() = default;
        virtual ~Controller() = default;

        virtual Status Init(const ControllerParam& control_conf) = 0;

        virtual Status computeControlCommand(
                const Localization& localization,
                const Chassis& chassis,
                const Trajectory& trajectory,
                ControlCommand& controlCommand) = 0;

        virtual std::string Name() const = 0;

        // reset controller
        virtual bool reset();

        // stop controller
        virtual void stop();
    };

}

#endif //CONTROLLER_CONTROLLER_H
