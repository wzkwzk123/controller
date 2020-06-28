//
// Created by cjh on 5/12/20.
//

#ifndef CONTROLLER_QP_SOLVER_GFLAGS_H
#define CONTROLLER_QP_SOLVER_GFLAGS_H

#include <gflags/gflags.h>

namespace control{
    DECLARE_double(default_active_set_eps_num);
    DECLARE_double(default_active_set_eps_den);
    DECLARE_double(default_active_set_eps_iter_ref);
    DECLARE_double(default_qp_smoothing_eps_num);
    DECLARE_double(default_qp_smoothing_eps_den);
    DECLARE_double(default_qp_smoothing_eps_iter_ref);
    DECLARE_bool(default_enable_active_set_debug_info);
    DECLARE_int32(default_qp_iteration_num);
}




#endif //CONTROLLER_QP_SOLVER_GFLAGS_H
