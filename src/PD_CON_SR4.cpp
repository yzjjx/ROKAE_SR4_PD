#include "PD_con.h"
#include "rokae/robot.h"

#include <array>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace rokae;

struct TrajectoryPoint
{
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(6);
};

std::vector<TrajectoryPoint> loadTrajectory(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("无法打开轨迹文件：" + path);

    std::string header;
    std::getline(file, header);

    std::vector<TrajectoryPoint> trajectory;
    double time;
    while (file >> time)
    {
        TrajectoryPoint point;
        for (int i = 0; i < 6; ++i)
            file >> point.q(i);
        for (int i = 0; i < 6; ++i)
            file >> point.dq(i);
        trajectory.push_back(point);
    }

    if (trajectory.empty())
        throw std::runtime_error("轨迹文件中没有数据");

    return trajectory;
}

int main()
{
    try
    {
        const std::string trajectory_path =
            "data_in/circle_R200_joint_trajectory_SR4_V50.txt";
        const auto trajectory = loadTrajectory(trajectory_path);

        const std::string ip = "192.168.2.160";
        const std::string local_ip = "192.168.2.2";
        std::error_code ec;

        xMateRobot robot;
        robot.connectToRobot(ip, local_ip);
        std::cout << "机器人连接成功" << std::endl;

        robot.setOperateMode(OperateMode::automatic, ec);
        if (ec)
            throw std::runtime_error("设置操作模式失败：" + ec.message());

        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        if (ec)
            throw std::runtime_error("设置实时控制模式失败：" + ec.message());

        robot.setRtNetworkTolerance(20, ec);
        robot.setPowerState(true, ec);
        if (ec)
            throw std::runtime_error("机器人上电失败：" + ec.message());

        auto rt_controller = robot.getRtMotionController().lock();
        if (!rt_controller)
            throw std::runtime_error("获取实时控制器失败");

        // 先低速运动到轨迹起点，避免进入力矩模式时出现较大的位置误差。
        std::array<double, 6> q_start{};
        for (int i = 0; i < 6; ++i)
            q_start[i] = trajectory.front().q(i);
        rt_controller->MoveJ(0.1, robot.jointPos(ec), q_start);

        // SDK在每个1 ms控制回调前更新真实关节位置和速度。
        robot.startReceiveRobotState(
            std::chrono::milliseconds(1),
            {RtSupportedFields::jointPos_m, RtSupportedFields::jointVel_m});

        PD_con pd_controller;
        Torque command(6);
        std::array<double, 6> q_real{};
        std::array<double, 6> dq_real{};
        std::size_t step = 0;

        std::function<Torque(void)> callback = [&]() {
            robot.getStateData(RtSupportedFields::jointPos_m, q_real);
            robot.getStateData(RtSupportedFields::jointVel_m, dq_real);

            const Eigen::Map<const Eigen::VectorXd> q(q_real.data(), 6);
            const Eigen::Map<const Eigen::VectorXd> dq(dq_real.data(), 6);
            const Eigen::VectorXd tau = pd_controller.PD_controller(
                q, dq, trajectory[step].q, trajectory[step].dq);

            for (int i = 0; i < 6; ++i)
                command.tau[i] = tau(i);

            ++step;
            if (step == trajectory.size())
                command.setFinished();

            return command;
        };

        rt_controller->startMove(RtControllerMode::torque);
        rt_controller->setControlLoop(callback, 0, true);
        rt_controller->startLoop(true);

        robot.stopReceiveRobotState();
        std::cout << "PD轨迹跟踪结束" << std::endl;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << std::endl;
        return -1;
    }

    return 0;
}
