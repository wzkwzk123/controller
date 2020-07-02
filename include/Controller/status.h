//
// Created by zwu on 2020/6/30.
//

#ifndef CONTROLLER_STATUS_H
#define CONTROLLER_STATUS_H

namespace control{

    /**
     * @class Status
     *
     *
     */
    class Status{
    public:
        Status(){}
        ~Status() = default;

        static Status ok() { return Status(); }
    };

}

#endif //CONTROLLER_STATUS_H
