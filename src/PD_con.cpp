#include "PD_con.h"

Eigen::VectorXd PD_con::PD_controller(
    const Eigen::VectorXd& q_real,
    const Eigen::VectorXd& dq_real,
    const Eigen::VectorXd& q_ideal,
    const Eigen::VectorXd& dq_ideal
)
{
    // 六个关节分别对应一组 PD 增益。
    Eigen::VectorXd Kp(6);
    Eigen::VectorXd Kd(6);
    Eigen::VectorXd tau_limit(6);
    Kp << 100.0, 100.0, 100.0, 10.0, 3.0, 0.1;
    Kd <<  20.0,  20.0,  20.0,  20.0,  1.0,  0.1;
    tau_limit << 300.0, 300.0, 300.0, 300.0, 300.0, 300.0;

    const Eigen::VectorXd position_error = q_ideal - q_real;
    const Eigen::VectorXd velocity_error = dq_ideal - dq_real;

    // 分别相乘计算
    Eigen::VectorXd tau = Kp.cwiseProduct(position_error)
                        + Kd.cwiseProduct(velocity_error);

    // 将六个关节的输出力矩限制在 [-tau_limit, tau_limit]
    tau = tau.cwiseMax(-tau_limit).cwiseMin(tau_limit);
    return tau;
}
