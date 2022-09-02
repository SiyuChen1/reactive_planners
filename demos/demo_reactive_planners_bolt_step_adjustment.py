""" @namespace Demos of Bolt step adjustment
@file
@copyright Copyright (c) 2017-2019,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example
"""
from distutils.log import debug
import numpy as np
import pybullet as p
from robot_properties_bolt.config import BoltConfig
from robot_properties_bolt.bolt_wrapper import BoltRobot
from mim_control.robot_centroidal_controller import RobotCentroidalController
from mim_control.robot_impedance_controller import RobotImpedanceController
from reactive_planners.lipm_simulator import LipmSimpulator
from reactive_planners_cpp import DcmReactiveStepper
import pinocchio as se3
from scipy.spatial.transform import Rotation as R
from bullet_utils.env import BulletEnvWithGround

import logging
logging.basicConfig(level=logging.DEBUG)
import yaml
from time import sleep

# pin_robot.data definition and properties see below
# ~/pinocchio/bindings/python/multibody/data.hpp

def zero_cnt_gain(kp, cnt_array):
    gain = np.array(kp).copy()
    for i, v in enumerate(cnt_array):
        if v == 1:
            gain[3 * i : 3 * (i + 1)] = 0.0
    return gain


def yaw(q):
    return np.array(
        R.from_quat([np.array(q)[3:7]]).as_euler("xyz", degrees=False)
    )[0, 2]


if __name__ == "__main__":
    # Create a robot instance. This initializes the simulator as well.
    env = BulletEnvWithGround()
    robot = env.add_robot(BoltRobot(is_passive=False))
    tau = np.zeros(6)

    time = 0
    sim_freq = 10000  # Hz
    ctrl_freq = 1000
    plan_freq = 1000

    p.resetDebugVisualizerCamera(1.6, 50, -35, (0.0, 0.0, 0.0))
    p.setTimeStep(1.0 / sim_freq)
    p.setRealTimeSimulation(0)
    for ji in range(8):
        p.changeDynamics(
            robot.robotId,
            ji,
            linearDamping=0.04,
            angularDamping=0.04,
            restitution=0.0,
            lateralFriction=4.0,
            spinningFriction=5.6,
        )
    
    logging.debug("#joints = %s", p.getNumJoints(robot.robotId))
    # for jointId in range(p.getNumJoints(robot.robotId)):
    #     jointInfo = p.getJointInfo(robot.robotId, jointId)
    #     print("jointName:", jointInfo[1])

    logging.debug("joint_names = %s", robot.joint_names)
    logging.debug("end_effector_names = %s", robot.end_effector_names)
    logging.debug("robot_type = %s", type(robot))

    logging.debug("robot.pinocchio_joint_ids = %s", robot.pinocchio_joint_ids)
    # DEBUG:root:robot.pinocchio_joint_ids = [2 3 4 5 6 7]

    # bullet_joint_map is mapping from joint name to joint id (in pybullet convention)
    # bullet_joint_map = {}
    #     for ji in range(pybullet.getNumJoints(robot_id)):
    #         bullet_joint_map[
    #             pybullet.getJointInfo(robot_id, ji)[1].decode("UTF-8")
    #         ] = ji

    # joint_names are joints that we want to control, in this case #joint_names = 6
    # self.bullet_joint_ids = np.array(
    #         [bullet_joint_map[name] for name in joint_names]
    #     )
    # self.bullet_joint_ids: pybullet joint id for joints that we want to control

    logging.debug("robot.bullet_joint_ids = %s", robot.bullet_joint_ids)
    # DEBUG:root:robot.bullet_joint_ids = [0 1 2 4 5 6]

    # self.pinocchio_joint_ids = np.array(
    #         [pinocchio_robot.model.getJointId(name) for name in joint_names]
    #     )
    # self.pinocchio_joint_ids: joints id that we want to control (in pinocchio convention)

    # note: q and dq are followed the convention of pinocchio
    # initial configuration of bolt with len(q) = 13
    q = np.matrix(BoltConfig.initial_configuration).T
    # initial velocity configuration of bolt with len(qdot) = 12
    qdot = np.matrix(BoltConfig.initial_velocity).T

    robot.reset_state(q, qdot)
    # definition of def reset_state(self, q, dq): # in src/robot_properties_bolt/src/robot_properties_bolt/bolt_wrapper.py
    # def reset_state(self, q, dq): # within class BoltRobot(PinBulletWrapper):
    #     if self.is_passive:
    #         q_simu = np.concatenate([q[0:10], [0.0], q[10:13], [0.0]])
    #         dq_simu = np.concatenate([dq[0:9], [0.0], dq[9:12], [0.0]])
    #         super(BoltRobot, self).reset_state(q_simu, dq_simu)
    #     else:
    #         super(BoltRobot, self).reset_state(q, dq)
    
    logging.debug("robot_is_passive = %s", str(robot.is_passive))
    # Since robot is passive, call PinBulletWrapper.reset_state(q, dq)
    # class PinBulletWrapper is defined in src/bullet_utils/src/bullet_utils/wrapper.py

    # we have to check the parameter robot.useFixedBase because it will have an influnce of the decoding of q and dq
    logging.debug("robot_useFixedBase = %s", str(robot.useFixedBase))
    # DEBUG:root:robot_useFixedBase = False 
    # in our case: DEBUG:root:robot_useFixedBase = False

    # so the definitions are: 
    # base_inertia_pos, base_inertia_quat = pybullet.getBasePositionAndOrientation(self.robot_id)
    # # Get transform between inertial frame and link frame in base
    # base_stat = pybullet.getDynamicsInfo(self.robot_id, -1)
    # base_inertia_link_pos, base_inertia_link_quat = pybullet.invertTransform(base_stat[3], base_stat[4])
    # pos, orn = pybullet.multiplyTransforms(base_inertia_pos, base_inertia_quat, base_inertia_link_pos, base_inertia_link_quat)
    # moving base pos = q[0:3] = (pos) 
    # moving base orientation = q[3:7] = (orn)
    # vel, orn = pybullet.getBaseVelocity(self.robot_id)
    # dq[:3] = vel
    # dq[3:6] = orn
    # # Pinocchio assumes the base velocity to be in the body frame -> rotate.
    # rot = np.array(pybullet.getMatrixFromQuaternion(q[3:7])).reshape((3, 3))
    # dq[0:3] = rot.T.dot(dq[0:3])
    # dq[3:6] = rot.T.dot(dq[3:6])

    # # until now, we get the base position and orientation in the world coordinate system

    # # Query the joint readings.
    # joint_states = pybullet.getJointStates(self.robot_id, self.bullet_joint_ids)

    # if not self.useFixedBase:
    #     for i in range(self.nj):
    #         q[5 + self.pinocchio_joint_ids[i]] = joint_states[i][0]
    #         dq[4 + self.pinocchio_joint_ids[i]] = joint_states[i][1]


    # # finally we reset the joint state
    # for i, bullet_joint_id in enumerate(self.bullet_joint_ids):
    #             pybullet.resetJointState(
    #                 self.robot_id,
    #                 bullet_joint_id,
    #                 q[5 + self.pinocchio_joint_ids[i]],
    #                 dq[4 + self.pinocchio_joint_ids[i]],
    #             )

    # in briefly
    # first three elements q[0:3] are floating base position
    # first three elements dq[0:3] are floating base velocity
    # q[3:7] are floating base orientation in quaternion
    # q[3:7] are floating base angular velocity
    # the other left elements are joint generalised position and generalised velocity

    logging.debug("length of q = " + str(len(q)))
    logging.debug("type(q) = " + str(type(q)))
    logging.debug("length of dq = %s", str(len(qdot)))
    # fun fact: len(q) = len(dq) + 1
    vec2list = lambda m: np.array(m.T).reshape(-1).tolist()

    # position of inertial frame in local coordinates of the joint frame
    # -1 means base
    base_stat = p.getDynamicsInfo(robot.robot_id, -1)

    # base_stat[3]: position of inertial frame in local coordinates of the joint frame, here not joint frame, but base frame
    # base_stat[4]: orientation of inertial frame in local coordinates of joint frame, but base frame
    # vec2list(q[:3]): floating base pos
    # vec2list(q[3:7]): floating base orientation
    base_pos, base_quat = p.multiplyTransforms(vec2list(q[:3]), vec2list(q[3:7]), base_stat[3], base_stat[4])
    logging.debug('base_pose = %s', str(base_stat[3]))
    logging.debug('base_quat = %s', str(base_stat[4]))
    logging.debug('base_pose_transformed = %s', str(base_pos))
    logging.debug('base_quaternion_transformed = %s', str(base_quat))

    # rot is the transformation matrix or (jacobian matrix) 
    rot = np.array(p.getMatrixFromQuaternion(q[3:7])).reshape((3, 3))
    p.resetBaseVelocity(
        robot.robot_id, vec2list(rot.dot(qdot[:3])), vec2list(rot.dot(qdot[3:6]))
    )
    logging.debug("rot = %s", str(rot))

    # question here
    total_mass = sum([i.mass for i in robot.pin_robot.model.inertias[1:]])
    logging.debug("total mass = %s", total_mass)
    logging.debug("robot.pin_robot.model.inertias = %s", type(robot.pin_robot.model.inertias))

    # definition of pinocchio::Model src/multibody/model.hpp
    logging.debug("robot.pin_robot.model = %s", type(robot.pin_robot.model))
    
    for i, iner in enumerate(robot.pin_robot.model.inertias):
        logging.debug("%d-th inertias = %s", i, iner.mass)


    warmup = 5
    kp = np.array([150.0, 150.0, 150.0, 150.0, 150.0, 150.0])
    kd = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
    robot_config = BoltConfig()
    config_file = robot_config.ctrl_path
    logging.debug('config_file_path = %s', config_file)
    bolt_leg_ctrl = RobotImpedanceController(robot, config_file)

    with open(config_file) as config:
        data_in = yaml.safe_load(config)

    num_eef = 0
    for ctrls in data_in["impedance_controllers"]:
        if int(data_in["impedance_controllers"][ctrls]["is_eeff"]):
            num_eef += 1

    # definition of center_controller
    centr_controller = RobotCentroidalController(
        robot_config,
        mu=1,
        kc=[0, 0, 100],
        dc=[0, 0, 10],
        kb=[100, 100, 100],
        db=[10.0, 10, 10],
        qp_penalty_lin=[1, 1, 1e6],
        qp_penalty_ang=[1e6, 1e6, 1],
    )

    is_left_leg_in_contact = True
    l_min = -0.1
    l_max = 0.1
    w_min = -0.08
    w_max = 0.2
    t_min = 0.1
    t_max = 0.8
    l_p = 0.1035  # Pelvis width
    com_height = 0.36487417
    weight = [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
    mid_air_foot_height = 0.05
    control_period = 1 / ctrl_freq
    planner_loop = 0.010
    
    # get this value in rviz
    x_des_local = [
        q[0].item(),
        q[1].item() + 0.02,
        0.0,
        q[0].item(),
        q[1].item() - 0.02,
        0.0,
    ]

    past_x = x_des_local.copy()
    v_des = [0.5, 0.3, 0.0]
    sim = LipmSimpulator(com_height)

    # definition of class DcmReactiveStepper()
    # src/reactive_planners/python/reactive_planners/dcm_reactive_stepper.py
    dcm_reactive_stepper = DcmReactiveStepper()
    dcm_reactive_stepper.initialize(
        is_left_leg_in_contact,
        l_min,
        l_max,
        w_min,
        w_max,
        t_min,
        t_max,
        l_p,
        com_height,
        weight,
        mid_air_foot_height,
        control_period,
        planner_loop, # not used in polynomial swing foot trajectory generation
        x_des_local[:3], # left foot position
        x_des_local[3:], # right foot position
    )

    dcm_reactive_stepper.set_desired_com_velocity(v_des)

    x_com = [[0.0], [0.0], [com_height]]
    cnt_array = [1, 1]
    time = 0
    control_time = 0
    open_loop = True
    dcm_force = [0.0, 0.0, 0.0]

    # the meaning of the variable offset is not clear
    offset = 0.0171  # foot radius
    
    dcm_reactive_stepper.start()

    # for i in range(9):
    for i in range(1):
        last_qdot = qdot
        q, qdot = robot.get_state()

        # output of robot.pin_robot.com(q, qdot) is:
        # pin.centerOfMass(self.model, self.data, q, v)
        # where model is robot model
        # data is a struct which defined datas for further use
        robot.pin_robot.com(q, qdot)

        # x_com is the position of com (center of mass) in (x, y, z)
        x_com = robot.pin_robot.com(q, qdot)[0]

        # xd_com is the velocity of com (center of mass) in (x, y, z)
        xd_com = robot.pin_robot.com(q, qdot)[1]

        logging.info("iterations = %d ", i)
        # print(x_com, q)
        # print(xd_com, qdot)
        # print(last_qdot, qdot)
        # print(robot.pin_robot.com(q, qdot))
        # print(x_com, xd_com)

        if warmup <= i:
            print("--------------------------------------------------------")
            left = bolt_leg_ctrl.imp_ctrl_array[0]
            right = bolt_leg_ctrl.imp_ctrl_array[1]
            left_foot_location = np.array(
                left.pin_robot.data.oMf[left.frame_end_idx].translation
            ).reshape(-1)
            right_foot_location = np.array(
                right.pin_robot.data.oMf[right.frame_end_idx].translation
            ).reshape(-1)
            left_foot_vel = np.array(
                se3.SE3(
                    left.pin_robot.data.oMf[left.frame_end_idx].rotation,
                    np.zeros((3, 1)),
                )
                * se3.computeFrameJacobian(
                    robot.pin_robot.model,
                    robot.pin_robot.data,
                    q,
                    left.frame_end_idx,
                ).dot(qdot)[0:3]
            )
            right_foot_vel = np.array(
                se3.SE3(
                    right.pin_robot.data.oMf[right.frame_end_idx].rotation,
                    np.zeros((3, 1)),
                )
                * se3.computeFrameJacobian(
                    robot.pin_robot.model,
                    robot.pin_robot.data,
                    q,
                    right.frame_end_idx,
                ).dot(qdot)[0:3]
            )
            # at first is left leg in contact
            print("dcm_reactive_stepper.get_is_left_leg_in_contact() = ", dcm_reactive_stepper.get_is_left_leg_in_contact())
            if dcm_reactive_stepper.get_is_left_leg_in_contact():
                pos_for_plotter = (
                    dcm_reactive_stepper.get_right_foot_position().copy()
                )
                vel_for_plotter = (
                    dcm_reactive_stepper.get_right_foot_velocity().copy()
                )
            else:
                pos_for_plotter = (
                    dcm_reactive_stepper.get_left_foot_position().copy()
                )
                vel_for_plotter = (
                    dcm_reactive_stepper.get_left_foot_velocity().copy()
                )

            print("left_foot_location = ", left_foot_location)
            print("right_foot_location = ", right_foot_location)
            print("offset = ", offset)
            logging.info("current time = %f", time)
            dcm_reactive_stepper.run(
                time,
                [
                    left_foot_location[0],
                    left_foot_location[1],
                    left_foot_location[2] - offset,
                ],
                [
                    right_foot_location[0],
                    right_foot_location[1],
                    right_foot_location[2] - offset,
                ],
                left_foot_vel,
                right_foot_vel,
                x_com,
                xd_com,
                yaw(q),
                not open_loop,
            )
            dcm_force = dcm_reactive_stepper.get_forces().copy()
            print("dcm_force = ", dcm_force)

            x_des_local = []
            x_des_local.extend(
                dcm_reactive_stepper.get_left_foot_position().copy()
            )
            x_des_local.extend(
                dcm_reactive_stepper.get_right_foot_position().copy()
            )

            x_des_local[2] += offset
            x_des_local[5] += offset

            if dcm_reactive_stepper.get_is_left_leg_in_contact():
                cnt_array = [1, 0]
            else:
                cnt_array = [0, 1]

            time += 1/plan_freq

        for j in range(2):
            imp = bolt_leg_ctrl.imp_ctrl_array[j]

            # frame_root_name(joint): The root frame name where the spring starts(Ex. Hip)
            # frame_end_name(link): the second frame name where the spring ends(Ex. end effector)
            logging.debug("imp.frame_root_name = %s", imp.frame_root_name)
            logging.debug("imp.frame_end_name = %s", imp.frame_end_name)
            logging.debug("imp_controller_type = %s ", type(imp))
            
            # oMf, "frames absolute placement (wrt world)"
            print(imp.pin_robot.data.oMf[imp.frame_root_idx].translation)
            
            # modify center of mass location with respect to its own frame
            x_des_local[3 * j : 3 * (j + 1)] -= imp.pin_robot.data.oMf[
                imp.frame_root_idx
            ].translation
        
        # src/mim_control/python/mim_control/robot_centroidal_controller.py
        # def compute_com_wrench(self, q, dq, des_pos, des_vel, des_ori, des_angvel):
        # """Compute the desired COM wrench (equation 1).

        # Args:
        #     des_pos: desired center of mass position at time t
        #     des_vel: desired center of mass velocity at time t
        #     des_ori: desired base orientation at time t (quaternions)
        #     des_angvel: desired base angular velocity at time t
        # Returns:
        #     Computed w_com
        # """
        w_com = centr_controller.compute_com_wrench(
            q.copy(),
            qdot.copy(),
            [0.0, 0.0, com_height],
            [0.0, 0.0, 0.0],
            [0, 0.0, 0, 1.0],
            [0.0, 0.0, 0.0],
        )
        w_com[0] = 0.0
        w_com[1] = 0.0

        print("w_com = ", w_com)

        # src/mim_control/python/mim_control/robot_centroidal_controller.py
        # def compute_force_qp(self, q, dq, cnt_array, w_com):
        # """Computes the forces needed to generated a desired centroidal wrench.
        # Args:
        #     q: Generalized robot position configuration.
        #     dq: Generalized robot velocity configuration.
        #     cnt_array: Array with {0, 1} of #endeffector size indicating if
        #         an endeffector is in contact with the ground or not. Forces are
        #         only computed for active endeffectors.
        #     w_com: Desired centroidal wrench to achieve given forces.
        # Returns:
        #     Computed forces as a plain array of size 3 * num_endeffectors.
        # """

        print("cnt_array = ", cnt_array)
        F = centr_controller.compute_force_qp(q, qdot, cnt_array, w_com)

        # [qdot[0].item(), qdot[1].item(), qdot[2].item()] is com velocity
        des_vel = np.concatenate(
            (
                dcm_reactive_stepper.get_left_foot_velocity()
                - [qdot[0].item(), qdot[1].item(), qdot[2].item()],
                dcm_reactive_stepper.get_right_foot_velocity()
                - [qdot[0].item(), qdot[1].item(), qdot[2].item()],
            )
        )
        
        print("des_vel = ", des_vel)

        # if cnt_array[0] == 1 and cnt_array[1] == 0:
        #     F[3:] = -dcm_force[:3]
        # elif cnt_array[0] == 0 and cnt_array[1] == 1:
        #     F[:3] = -dcm_force[:3]
        
        """
        Returns the joint torques at the current timestep

        Input:
            q : current joint positions
            dq : current joint velocities
            kp : Proportional gain
            kd : derivative gain
            x_des : desired lengths with respect to the root frame for each
                    controller (3*number_of_springs)
            xd_des : desired velocities with respect to the root frame
            f : feed forward forces
        """
        tau = bolt_leg_ctrl.return_joint_torques(
            q.copy(),
            qdot.copy(),
            zero_cnt_gain(kp, cnt_array),
            zero_cnt_gain(kd, cnt_array),
            x_des_local,
            des_vel,
            F,
        )
        control_time += 1 / ctrl_freq

        for j in range(10):
            robot.send_joint_command(tau)
            p.stepSimulation()
        
        print(type(robot))

    dcm_reactive_stepper.stop()
