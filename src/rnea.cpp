#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <Eigen/Core>
#include <iostream>

int main()
{
    // 相对路径以运行程序时的工作目录为准，请在项目根目录运行。
    pinocchio::Model model;
    pinocchio::urdf::buildModel("../urdf/ROKAE_SR4.urdf", model);
    pinocchio::Data data(model);

    // 世界坐标系下的重力加速度，单位：m/s^2。
    model.gravity.linear() << 0.0, 0.0, -9.81;

    // SR4 的 6 个关节按 joint_1 到 joint_6 排列。
    // 修改下面的 q、dq、ddq 即可计算对应状态所需的关节力矩。
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);     // 角度，rad
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model.nv);    // 角速度，rad/s
    Eigen::VectorXd ddq = Eigen::VectorXd::Zero(model.nv);   // 角加速度，rad/s^2

    // RNEA：tau = M(q) * ddq + C(q, dq) * dq + g(q)。
    // dq、ddq 均为零时，结果为重力补偿力矩。
    const Eigen::VectorXd tau = pinocchio::rnea(model, data, q, dq, ddq);

    std::cout << "Joint torques (N*m): " << tau.transpose() << '\n';
    return 0;
}
