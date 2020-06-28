//
// Created by cjh on 5/11/20.
//

#ifndef CONTROLLER_CONTROLLER_H
#define CONTROLLER_CONTROLLER_H

#include "eigen3/Eigen/Core"

namespace control{

    class Controller{
        // basic class for all controllers
    public:
        Controller() = default;
        virtual ~Controller() = default;

        virtual bool Init();

        virtual bool computeControlCommand();

        // reset controller
        virtual bool reset();

        // stop controller
        virtual void stop();
    };

}

#endif //CONTROLLER_CONTROLLER_H
