cmake_minimum_required(VERSION 3.18)
project(sr4_rnea LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加 Pinocchio 的安装路径，供 find_package 查找。
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(pd_controller STATIC src/PD_con.cpp)
set_target_properties(pd_controller PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(pd_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(pd_controller PUBLIC Eigen3::Eigen)

add_executable(sr4_rnea src/rnea.cpp)

# pinocchio::pinocchio 会自动提供 Pinocchio 的头文件路径。
target_link_libraries(sr4_rnea
    PRIVATE
    Eigen3::Eigen
    pinocchio::pinocchio
)

# 将同一份 C++ PD 控制器封装为 Python 动态库。
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(rokae_pd Python_code/python_bindings.cpp)
target_link_libraries(rokae_pd PRIVATE pd_controller)
set_target_properties(rokae_pd PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Python_code"
)
