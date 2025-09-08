#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_cr5af_follower import CR5AFFOLLOWERConfig
from .dobot_api_bak import DobotApiDashboard, DobotApiFeedBack
logger = logging.getLogger(__name__)


class CR5AFFollower(Robot):
    """
    SO-101 Follower Arm designed by TheRobotStudio and Hugging Face.
    """

    config_class = CR5AFFOLLOWERConfig
    name = "cr5af_follower"

    def __init__(self, config: CR5AFFOLLOWERConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "joint_1": Motor(1, "sts3215", norm_mode_body),
                "joint_2": Motor(2, "sts3215", norm_mode_body),
                "joint_3": Motor(3, "sts3215", norm_mode_body),
                "joint_4": Motor(4, "sts3215", norm_mode_body),
                "joint_5": Motor(5, "sts3215", norm_mode_body),
                "joint_6": Motor(6, "sts3215", norm_mode_body),
                # "gripper": Motor(7, "sts3215", MotorNormMode.RANGE_0_100),

            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

        # for cr5af
        self.ip = "192.168.5.1"
        self.dashboardPort = 29999
        self.feedPortFour = 30004
        self.dashboard = DobotApiDashboard(self.ip, self.dashboardPort)
        self.feedFour = DobotApiFeedBack(self.ip, self.feedPortFour)

        class item:
            def __init__(self):
                # 自定义添加所需反馈数据
                self.robotMode = -1     #
                self.robotCurrentCommandID = 0
                self.MessageSize = -1
                self.DigitalInputs =-1
                self.DigitalOutputs = -1
                self.robotCurrentCommandID = -1
                self.QActual = [-1,-1,-1,-1,-1,-1]   # 关节实际角度

        self.feedData = item()  # 定义结构对象

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    # @property
    # def is_connected(self) -> bool:
    #     return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())
    def parseResultId(self, valueRecv):
        import re
        if "Not Tcp" in valueRecv:
            print("Control Mode Is Not Tcp")
            return [1]
        return [int(num) for num in re.findall(r'-?\d+', valueRecv)] or [2]
    @property
    def is_connected(self) -> bool:
        if self.parseResultId(self.dashboard.EnableRobot())[0] != 0:
            print("使能失败: 检查29999端口是否被占用")
            return
        # print("使能成功")
        # sock = self.dashboard.reConnect(self.ip, self.dashboardPort)
        # ok = (sock.connect_ex((self.ip, self.dashboardPort)) == 0)
        # return ok and all(cam.is_connected for cam in self.cameras.values())
        return True


    # def connect(self, calibrate: bool = True) -> None:
    #     """
    #     We assume that at connection time, arm is in a rest position,
    #     and torque can be safely disabled to run calibration.
    #     """
    #     if self.is_connected:
    #         raise DeviceAlreadyConnectedError(f"{self} already connected")

    #     self.bus.connect()
    #     if not self.is_calibrated and calibrate:
    #         logger.info(
    #             "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
    #         )
    #         self.calibrate()

    #     for cam in self.cameras.values():
    #         cam.connect()

    #     self.configure()
    #     logger.info(f"{self} connected.")

    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """
        # if self.is_connected:
        #     raise DeviceAlreadyConnectedError(f"{self} already connected")

        # self.bus.connect()
        # if not self.is_calibrated and calibrate:
        #     logger.info(
        #         "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
        #     )
        # self.calibrate()


        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        # return self.bus.is_calibrated
        return True

    def calibrate(self) -> None:
        if self.calibration:
            # self.calibration is not empty here
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            # 标定
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                # self.bus.write_calibration(self.calibration)
                return

        # 写入operating mode
        # logger.info(f"\nRunning calibration of {self}")
        # self.bus.disable_torque()
        # for motor in self.bus.motors:
        #     self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        # 设置电机的Homing_Offset
        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        # homing_offsets = self.bus.set_half_turn_homings()

        # 记录电机的min和max位置
        print(
            "Move all joints sequentially through their entire ranges "
            "of motion.\nRecording positions. Press ENTER to stop..."
        )
        # range_mins, range_maxes = self.bus.record_ranges_of_motion()

        # 保存电机的校准数据
        # self.calibration = {}
        # for motor, m in self.bus.motors.items():
        #     self.calibration[motor] = MotorCalibration(
        #         id=m.id,
        #         drive_mode=0,
        #         homing_offset=homing_offsets[motor],
        #         range_min=range_mins[motor],
        #         range_max=range_maxes[motor],
        #     )

        # fake calibration
        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=100,
                range_min=10,
                range_max=10,
            )

        # self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)
    # 配置电机Operating_Mode
    def configure(self) -> None:
        # with self.bus.torque_disabled():
        #     self.bus.configure_motors()
        #     for motor in self.bus.motors:
        #         self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        #         # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
        #         self.bus.write("P_Coefficient", motor, 16)
        #         # Set I_Coefficient and D_Coefficient to default value 0 and 32
        #         self.bus.write("I_Coefficient", motor, 0)
        #         self.bus.write("D_Coefficient", motor, 32)
        pass

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()
        # obs_dict = self.bus.sync_read("Present_Position")  # ！！！！！！！！！！！！！！！
        # obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        # 读取feedbackdata
        feedInfo = self.feedFour.feedBackData()
        if feedInfo is not None:   
            if hex((feedInfo['TestValue'][0])) == '0x123456789abcdef':
                # 基础字段
                self.feedData.MessageSize = feedInfo['len'][0]
                self.feedData.robotMode = feedInfo['RobotMode'][0]
                self.feedData.DigitalInputs = feedInfo['DigitalInputs'][0]
                self.feedData.DigitalOutputs = feedInfo['DigitalOutputs'][0]
                self.feedData.robotCurrentCommandID = feedInfo['CurrentCommandId'][0]
                self.feedData.QActual = feedInfo['QActual'][0]  # 关节实际角度

                obs_dict = {
                    "joint_1.pos": self.feedData.QActual[0],
                    "joint_2.pos": self.feedData.QActual[1],
                    "joint_3.pos": self.feedData.QActual[2],
                    "joint_4.pos": self.feedData.QActual[3],
                    "joint_5.pos": self.feedData.QActual[4],
                    "joint_6.pos": self.feedData.QActual[5],
                    # "gripper.pos": 0.0,  # 夹爪没有反馈
                }
                # 自定义添加所需反馈数据
                '''
                self.feedData.DigitalOutputs = int(feedInfo['DigitalOutputs'][0])
                self.feedData.RobotMode = int(feedInfo['RobotMode'][0])
                self.feedData.TimeStamp = int(feedInfo['TimeStamp'][0])
                '''
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            the action sent to the motors, potentially clipped.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send goal position to the arm
        # self.bus.sync_write("Goal_Position", goal_pos)
        # print("目标位置:", goal_pos)
        recvmovemess = self.dashboard.MovJ(*list(goal_pos.values()), 1)
        # print("发送 MovJ:", recvmovemess)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def disconnect(self):
        # if not self.is_connected:
        #     raise DeviceNotConnectedError(f"{self} is not connected.")

        # self.bus.disconnect(self.config.disable_torque_on_disconnect)
        self.dashboard.DisableRobot()
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
