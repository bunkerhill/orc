# This script follows the example of reactive_control/tsid/ex_2_biped.py


import time
import os
import rospkg
import pinocchio as pin
from orc.utils.robot_wrapper import RobotWrapper
from orc.utils.robot_simulator import RobotSimulator
import limx_conf as conf
from orc.reactive_control.tsid.tsid_biped import TsidBiped
import numpy as np
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt
import orc.utils.plot_utils as plut



print("Load robot model")

# Add limx tron1 robot description to ROS package path
current_path = os.environ.get("ROS_PACKAGE_PATH", "")
new_path = os.environ.get('HOME') + "/catkin_ws/src/robot_description"
os.environ["ROS_PACKAGE_PATH"] = f"{new_path}:{current_path}"
current_path = os.environ.get("ROS_PACKAGE_PATH", "")
# confirm robot_description can be find as ros package.
rp = rospkg.RosPack()
rp_path_found=rp.get_path("robot_description")
print(f"Package found at: {rp_path_found}") 

conf.srdf = os.environ.get('HOME') + "/orc/kuo_test/SF_TRON1A/SF_TRON1.srdf"
conf.urdf = os.environ.get('HOME') + "/catkin_ws/src/robot_description/pointfoot/SF_TRON1A/urdf/robot.urdf"
conf.path = os.environ.get('HOME') + "/catkin_ws/src/robot_description/pointfoot/SF_TRON1A/"

tsid = TsidBiped(conf)

# robot_simu = RobotWrapper(tsid.robot_display.model, tsid.robot_display.collision_model, tsid.robot_display.visual_model)
# simu = RobotSimulator(conf, robot_simu)

N = conf.N_SIMULATION
com_pos = np.empty((3, N)) * nan
com_vel = np.empty((3, N)) * nan
com_acc = np.empty((3, N)) * nan

com_pos_ref = np.empty((3, N)) * nan
com_vel_ref = np.empty((3, N)) * nan
com_acc_ref = np.empty((3, N)) * nan
com_acc_des = np.empty((3, N)) * nan # acc_des = acc_ref - Kp*pos_err - Kd*vel_err
left_foot_force = np.empty((12, N)) * nan # f_x f_y f_z times 4 contact points
left_foot_total_force = np.empty((3, N)) * nan # some of 4 point's forces
right_foot_force = np.empty((12, N)) * nan
right_foot_total_force = np.empty((3, N)) * nan



offset = tsid.robot.com(tsid.formulation.data())
amp = np.array([0.0, 0.0, 0.05])
two_pi_f = 2 * np.pi * np.array([0.0, 0.0, 0.5])
two_pi_f_amp = two_pi_f * amp
two_pi_f_squared_amp = two_pi_f * two_pi_f_amp
sampleCom = tsid.trajCom.computeNext()
samplePosture = tsid.trajPosture.computeNext()

t = 0.0
q, v = tsid.q, tsid.v

for i in range(0, N):
    time_start = time.time()

    sampleCom.value(offset + amp * np.sin(two_pi_f * t))
    sampleCom.derivative(two_pi_f_amp * np.cos(two_pi_f * t))
    sampleCom.second_derivative(-two_pi_f_squared_amp * np.sin(two_pi_f * t))

    tsid.comTask.setReference(sampleCom)

    # print(samplePosture.value())
    # print(sampleCom.value())
    tsid.postureTask.setReference(samplePosture)

    HQPData = tsid.formulation.computeProblemData(t, q, v)
    if i == 0: HQPData.print_all()

    sol = tsid.solver.solve(HQPData)
    if sol.status != 0:
        print("QP problem could not be solved! Error code:", sol.status)
        break

    tau = tsid.formulation.getActuatorForces(sol)
    dv = tsid.formulation.getAccelerations(sol)

    com_pos[:, i] = tsid.robot.com(tsid.formulation.data())
    com_vel[:, i] = tsid.robot.com_vel(tsid.formulation.data())
    com_acc[:, i] = tsid.comTask.getAcceleration(dv)
    com_pos_ref[:, i] = sampleCom.value()
    com_vel_ref[:, i] = sampleCom.derivative()
    com_acc_ref[:, i] = sampleCom.second_derivative()
    com_acc_des[:, i] = tsid.comTask.getDesiredAcceleration

    if tsid.formulation.checkContact(tsid.contactRF.name, sol):
        right_foot_force[:, i] = tsid.formulation.getContactForce(tsid.contactRF.name, sol)
        right_foot_total_force[0, i] = right_foot_force[0, i] + right_foot_force[3, i] + right_foot_force[6, i] + right_foot_force[9, i]
        right_foot_total_force[1, i] = right_foot_force[1, i] + right_foot_force[4, i] + right_foot_force[7, i] + right_foot_force[10, i]
        right_foot_total_force[2, i] = right_foot_force[2, i] + right_foot_force[5, i] + right_foot_force[8, i] + right_foot_force[11, i]
    if tsid.formulation.checkContact(tsid.contactLF.name, sol):
        left_foot_force[:, i] = tsid.formulation.getContactForce(tsid.contactLF.name, sol)
        left_foot_total_force[0, i] = left_foot_force[0, i] + left_foot_force[3, i] + left_foot_force[6, i] + left_foot_force[9, i]
        left_foot_total_force[1, i] = left_foot_force[1, i] + left_foot_force[4, i] + left_foot_force[7, i] + left_foot_force[10, i]
        left_foot_total_force[2, i] = left_foot_force[2, i] + left_foot_force[5, i] + left_foot_force[8, i] + left_foot_force[11, i]
    if i % conf.PRINT_N == 0:
        print("Time %.3f" % (t))
        if tsid.formulation.checkContact(tsid.contactRF.name, sol):
            f = tsid.formulation.getContactForce(tsid.contactRF.name, sol)
            print(
                "\tnormal force %s: %.1f"
                % (tsid.contactRF.name.ljust(20, "."), tsid.contactRF.getNormalForce(f))
            )

        if tsid.formulation.checkContact(tsid.contactLF.name, sol):
            f = tsid.formulation.getContactForce(tsid.contactLF.name, sol)
            print(
                "\tnormal force %s: %.1f"
                % (tsid.contactLF.name.ljust(20, "."), tsid.contactLF.getNormalForce(f))
            )

        print(
            "\ttracking err %s: %.3f"
            % (tsid.comTask.name.ljust(20, "."), norm(tsid.comTask.position_error, 2))
        )
        print("\t||v||: %.3f\t ||dv||: %.3f" % (norm(v, 2), norm(dv)))

    q, v = tsid.integrate_dv(q, v, dv, conf.dt)
    t += conf.dt

    if i % conf.DISPLAY_N == 0:
        tsid.display(q)

    time_spent = time.time() - time_start
    if time_spent < conf.dt:
        time.sleep(conf.dt - time_spent)


print("test")
time = np.arange(0.0, N * conf.dt, conf.dt)

(f, ax) = plut.create_empty_figure(3, 1)
for i in range(3):
    ax[i].plot(time, com_pos[i, :], label="CoM " + str(i))
    ax[i].plot(time, com_pos_ref[i, :], "r:", label="CoM Ref " + str(i))
    ax[i].set_xlabel("Time [s]")
    ax[i].set_ylabel("CoM [m]")
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

(f, ax) = plut.create_empty_figure(3, 1)
for i in range(3):
    ax[i].plot(time, com_vel[i, :], label="CoM Vel " + str(i))
    ax[i].plot(time, com_vel_ref[i, :], "r:", label="CoM Vel Ref " + str(i))
    ax[i].set_xlabel("Time [s]")
    ax[i].set_ylabel("CoM Vel [m/s]")
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

(f, ax) = plut.create_empty_figure(3, 1)
for i in range(3):
    ax[i].plot(time, com_acc[i, :], label="CoM Acc " + str(i))
    ax[i].plot(time, com_acc_ref[i, :], "r:", label="CoM Acc Ref " + str(i))
    ax[i].plot(time, com_acc_des[i, :], "g--", label="CoM Acc Des " + str(i))
    ax[i].set_xlabel("Time [s]")
    ax[i].set_ylabel("CoM Acc [m/s^2]")
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

(f, ax) = plut.create_empty_figure(3, 1)
for i in range(3):
    ax[i].plot(time, left_foot_total_force[i, :], label="Left foot " + str(i))
    ax[i].plot(time, right_foot_total_force[i, :], label="Right foot " + str(i))
    ax[i].set_xlabel("Time [s]")
    ax[i].set_ylabel("Ground reaction force [N]")
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

plt.show()
print("test end")

