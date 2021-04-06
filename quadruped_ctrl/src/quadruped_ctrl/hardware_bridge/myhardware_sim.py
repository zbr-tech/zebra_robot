#!/usr/bin/env python
import pybullet as p
import pybullet_data
from quadruped_ctrl.hardware_bridge.myhardware_base import MyHardwareBase

# from pybullet_utils import gazebo_world_parser


class MyHardwareSim(MyHardwareBase):
    def __init__(self, sim_freq, communication_freq, position_control_mode):
        self._freq = sim_freq
        self._position_control_mode = position_control_mode
        self._skip_num = int(self._freq / communication_freq)
        robot_start_pos = [0, 0, 0.42]
        p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.resetSimulation()
        p.setTimeStep(1.0 / self._freq)
        p.setGravity(0, 0, -9.8)
        self._reset = p.addUserDebugParameter("reset", 1, 0, 0)
        self._low_energy_mode = p.addUserDebugParameter("low_energy_mode", 1, 0, 0)
        self._high_performance_mode = p.addUserDebugParameter(
            "high_performance_mode", 1, 0, 0
        )
        self._reset_flag = 0  # p.readUserDebugParameter(self._reset)
        # p.readUserDebugParameter(self._low_energy_mode)
        self._low_energy_flag = 0
        self._high_performance_flag = 0
        p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])

        # heightPerturbationRange = 0.06
        # numHeightfieldRows = 256
        # numHeightfieldColumns = 256
        # if terrain == "plane":
        planeShape = p.createCollisionShape(shapeType=p.GEOM_PLANE)
        ground_id = p.createMultiBody(0, planeShape)
        p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
        p.changeDynamics(ground_id, -1, lateralFriction=1)

        boxId = p.loadURDF(
            "mini_cheetah/mini_cheetah.urdf", robot_start_pos, useFixedBase=False
        )
        spinningFriction = 0.0065
        p.changeDynamics(boxId, 3, spinningFriction=spinningFriction)
        p.changeDynamics(boxId, 7, spinningFriction=spinningFriction)
        p.changeDynamics(boxId, 11, spinningFriction=spinningFriction)
        p.changeDynamics(boxId, 15, spinningFriction=spinningFriction)
        self._boxId = boxId
        jointIds = []
        for j in range(p.getNumJoints(boxId)):
            p.getJointInfo(boxId, j)
            jointIds.append(j)
        self._motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
        self._get_last_vel = [0] * 3

    def reset_robot(self):
        # if terrain == "racetrack":
        #     robot_z = 0.4
        # else:
        robot_z = 0.3
        boxId = self._boxId
        motor_id_list = self._motor_id_list
        init_new_pos = [
            0.0,
            -0.8,
            1.6,
            0.0,
            -0.8,
            1.6,
            0.0,
            -0.8,
            1.6,
            0.0,
            -0.8,
            1.6,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        p.resetBasePositionAndOrientation(boxId, [0, 0, robot_z], [0, 0, 0, 1])
        p.resetBaseVelocity(boxId, [0, 0, 0], [0, 0, 0])
        for j in range(12):
            p.resetJointState(
                boxId, motor_id_list[j], init_new_pos[j], init_new_pos[j + 12]
            )

        for j in range(16):
            force = 0
            p.setJointMotorControl2(boxId, j, p.VELOCITY_CONTROL, force=force)

    def get_data(self):
        get_last_vel = self._get_last_vel
        boxId = self._boxId
        motor_id_list = self._motor_id_list
        freq = self._freq
        get_orientation = []
        get_matrix = []
        get_velocity = []
        get_invert = []
        imu_data = [0] * 10
        leg_data = [0] * 24

        pose_orn = p.getBasePositionAndOrientation(boxId)

        for i in range(4):
            get_orientation.append(pose_orn[1][i])
        # get_euler = p.getEulerFromQuaternion(get_orientation)
        get_velocity = p.getBaseVelocity(boxId)
        get_invert = p.invertTransform(pose_orn[0], pose_orn[1])
        get_matrix = p.getMatrixFromQuaternion(get_invert[1])

        # IMU data
        imu_data[3] = pose_orn[1][0]
        imu_data[4] = pose_orn[1][1]
        imu_data[5] = pose_orn[1][2]
        imu_data[6] = pose_orn[1][3]

        imu_data[7] = (
            get_matrix[0] * get_velocity[1][0]
            + get_matrix[1] * get_velocity[1][1]
            + get_matrix[2] * get_velocity[1][2]
        )
        imu_data[8] = (
            get_matrix[3] * get_velocity[1][0]
            + get_matrix[4] * get_velocity[1][1]
            + get_matrix[5] * get_velocity[1][2]
        )
        imu_data[9] = (
            get_matrix[6] * get_velocity[1][0]
            + get_matrix[7] * get_velocity[1][1]
            + get_matrix[8] * get_velocity[1][2]
        )

        # calculate the acceleration of the robot
        linear_X = (get_velocity[0][0] - get_last_vel[0]) * freq
        linear_Y = (get_velocity[0][1] - get_last_vel[1]) * freq
        linear_Z = 9.8 + (get_velocity[0][2] - get_last_vel[2]) * freq
        imu_data[0] = (
            get_matrix[0] * linear_X
            + get_matrix[1] * linear_Y
            + get_matrix[2] * linear_Z
        )
        imu_data[1] = (
            get_matrix[3] * linear_X
            + get_matrix[4] * linear_Y
            + get_matrix[5] * linear_Z
        )
        imu_data[2] = (
            get_matrix[6] * linear_X
            + get_matrix[7] * linear_Y
            + get_matrix[8] * linear_Z
        )

        # joint data
        joint_state = p.getJointStates(boxId, motor_id_list)
        leg_data[0:12] = [
            joint_state[0][0],
            joint_state[1][0],
            joint_state[2][0],
            joint_state[3][0],
            joint_state[4][0],
            joint_state[5][0],
            joint_state[6][0],
            joint_state[7][0],
            joint_state[8][0],
            joint_state[9][0],
            joint_state[10][0],
            joint_state[11][0],
        ]

        leg_data[12:24] = [
            joint_state[0][1],
            joint_state[1][1],
            joint_state[2][1],
            joint_state[3][1],
            joint_state[4][1],
            joint_state[5][1],
            joint_state[6][1],
            joint_state[7][1],
            joint_state[8][1],
            joint_state[9][1],
            joint_state[10][1],
            joint_state[11][1],
        ]
        com_velocity = [get_velocity[0][0], get_velocity[0][1], get_velocity[0][2]]
        # get_last_vel.clear()
        get_last_vel = []
        self._get_last_vel = com_velocity
        self._leg_data = leg_data
        return imu_data, leg_data  # , pose_orn[0]

    # def set_joint_control(self, joint_control):
    #     self._joint_control = joint_control

    def check_mode(self):
        reset = self._reset
        low_energy_mode = self._low_energy_mode
        high_performance_mode = self._high_performance_mode
        ret = {"reset": False, "low_flag": False, "high_flag": False}
        if self._reset_flag < p.readUserDebugParameter(reset):
            self._reset_flag = p.readUserDebugParameter(reset)
            # rospy.logwarn("reset the robot")
            # reset_robot()
            ret["reset"] = True
        if self._low_energy_flag < p.readUserDebugParameter(low_energy_mode):
            self._low_energy_flag = p.readUserDebugParameter(low_energy_mode)
            rospy.logwarn("set robot to low energy mode")
            # cpp_gait_ctrller.set_robot_mode(convert_type(1))
            ret["low_flag"] = True
        if self._high_performance_flag < p.readUserDebugParameter(
            high_performance_mode
        ):
            self._high_performance_flag = p.readUserDebugParameter(
                high_performance_mode
            )
            rospy.logwarn("set robot to high performance mode")
            # cpp_gait_ctrller.set_robot_mode(convert_type(0))
            ret["high_flag"] = True

        return ret

    def communicate(self, joint_control):

        boxId = self._boxId
        motor_id_list = self._motor_id_list
        N_Motors = 12
        # joint_control = self._joint_control
        for _ in range(self._skip_num):
            self.get_data()
            leg_data = self._leg_data
            mcp_force = [
                joint_control.kp[i] * (joint_control.position[i] - leg_data[i])
                + joint_control.kd[i]
                * (joint_control.velocity[i] - leg_data[i + N_Motors])
                + joint_control.effort[i]
                for i in range(N_Motors)
            ]
            # print(leg_data, joint_control)
            # print(mcp_force)
            # set tau to simulator
            if self._position_control_mode:
                p.setJointMotorControlArray(
                    bodyUniqueId=boxId,
                    jointIndices=motor_id_list,
                    controlMode=p.TORQUE_CONTROL,
                    forces=mcp_force,
                )
            else:  # [TODO] get tau
                p.setJointMotorControlArray(
                    bodyUniqueId=boxId,
                    jointIndices=motor_id_list,
                    controlMode=p.TORQUE_CONTROL,
                    forces=tau.contents.eff,
                )
            p.stepSimulation()

    def __del__(self):
        p.disconnect()
