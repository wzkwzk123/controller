cmake_minimum_required(VERSION 3.15)
project(Controller)

set(CMAKE_CXX_STANDARD 11)

find_package(Threads)
add_executable(Controller main.cpp include/Controller/mpc_controller.h src/mpc_controller.cpp include/Controller/controller.h src/controller.cpp include/Controller/parameters.h test/test1.cpp include/Controller/mpc_solver.h src/mpc_solver.cpp include/Controller/qp_solver/qp_solver.h src/qp_solver/qp_solver.cpp include/Controller/qp_solver/active_set_qp_solver.h src/qp_solver/active_set_qp_solver.cpp include/Controller/qp_solver/qp_solver_gflags.h src/qp_solver/qp_solver_gflags.cpp include/Controller/math/math_utils.h include/Controller/status.h include/log.h)

include_directories(include/)
include_directories(include/Controller/)
include_directories(include/Controller/math)
include_directories(include/Controller/qp_solver/)
include_directories(/home/zwu/software/)
include_directories(/home/zwu/software/qpOASES-3.2.1/include)

target_link_libraries(Controller qpOASES)
target_link_libraries(Controller libgflags.a)
target_link_libraries(Controller libglog.a ${CMAKE_THREAD_LIBS_INIT})
