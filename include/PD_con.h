#ifndef PD_CON
#define PD_CON

#include <eigen3/Eigen/Dense>

class PD_con{
private:
    int DOF;

public:

    //控制器输入为期望位置、真实位置、期望速度、真实速度
    Eigen::VectorXd PD_controller(
        const Eigen::VectorXd& q_real,
        const Eigen::VectorXd& dq_real,
        const Eigen::VectorXd& q_ideal,
        const Eigen::VectorXd& dq_ideal
    );

};

#endif