"""使用 C++ PD 控制器在 MuJoCo 中跟踪给定关节轨迹。"""

from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

from rokae_pd import PDController


ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT.parent/ "xml" / "ROKAE_SR4.XML"
TRAJECTORY_PATH = ROOT.parent / "data_in" / "circle_R200_joint_trajectory_SR4_V50.txt"
OUTPUT_PATH = ROOT.parent/ "data_out" / "actual_joint_positions.csv"


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    data = mujoco.MjData(model)
    controller = PDController()

    trajectory = np.loadtxt(TRAJECTORY_PATH, skiprows=1)
    q_reference = trajectory[:, 1:7]
    dq_reference = trajectory[:, 7:13]

    data.qpos[:] = q_reference[0]
    data.qvel[:] = dq_reference[0]
    mujoco.mj_forward(model, data)
    actual_positions = [np.r_[data.time, data.qpos.copy()]]

    with mujoco.viewer.launch_passive(model, data) as viewer:
        for q_ideal, dq_ideal in zip(q_reference[:-1], dq_reference[:-1]):
            if not viewer.is_running():
                break

            step_start = time.perf_counter()
            data.ctrl[:] = controller.compute_control(
                data.qpos, data.qvel, q_ideal, dq_ideal
            )

            mujoco.mj_step(model, data)
            actual_positions.append(np.r_[data.time, data.qpos.copy()])
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep
                           - (time.perf_counter() - step_start)))

    np.savetxt(
        OUTPUT_PATH,
        np.asarray(actual_positions),
        delimiter=",",
        header="time_s,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad",
        comments="",
    )

    q_final_ideal = q_reference[len(actual_positions) - 1]
    error = np.rad2deg(q_final_ideal - data.qpos)
    print("最终关节误差 (deg):", np.round(error, 4))
    print("真实关节位置：", OUTPUT_PATH)


if __name__ == "__main__":
    main()
