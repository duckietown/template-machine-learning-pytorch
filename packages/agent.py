#!/usr/bin/env python3

from dt_computer_vision.camera import CameraModel
from typing import Union, Dict, Optional
from model import PyTorchModel
import signal
import numpy as np
import asyncio
from dtps import context, ContextConfig, DTPSContext
from dtps_http import RawData
from dt_robot_utils import get_robot_name
from duckietown_messages.actuators.differential_pwm import DifferentialPWM
from duckietown_messages.calibrations.camera_intrinsic import CameraIntrinsicCalibration
from duckietown_messages.utils.exceptions import DataDecodingError
from duckietown_messages.sensors.camera import Camera
from duckietown_messages.sensors.compressed_image import CompressedImage
from turbojpeg import TurboJPEG


class PyTorchAgent:
    def __init__(self):
        self._shutdown = False
        self._robot_name = get_robot_name()
        self.pwm_publisher: Optional[DTPSContext] = None
        self.camera_intrinsics: Optional[CameraIntrinsicCalibration] = None
        self.camera_info: Optional[Camera] = None
        self.camera: Optional[CameraModel] = None
        self.model = PyTorchModel()
        # register sigint handler
        signal.signal(signal.SIGINT, self._sigint_handler)
        self._jpeg = TurboJPEG()

    async def save_camera_intrinsics(self, rdata: RawData):
        try:
            camera: CameraIntrinsicCalibration = CameraIntrinsicCalibration.from_rawdata(rdata)
        except DataDecodingError as e:
            print(f"Failed to decode an incoming message: {e.message}")
            print("Camera parameters not available yet.")
            return

        if self.camera_intrinsics is None:
            print("Received camera parameters.")

        self.camera_intrinsics = camera

    async def save_camera_info(self, rdata: RawData):
            """
            Get the camera specification and save it to a variable.
            """
            try:
                camera: Camera = Camera.from_rawdata(rdata)
            except DataDecodingError as e:
                print(f"Failed to decode an incoming message: {e.message}")
                print("Camera information not available yet.")
                return

            if self.camera_info is None:
                print("Received camera information.")

            self.camera_info = camera

    async def img_cb(self, data: RawData):

        if self.camera is None:
            if self.camera_info is not None and self.camera_intrinsics is not None:
                print("Camera info and intrinsics received, initializing camera model")
                self.camera = CameraModel(
                    width=self.camera_info.width,
                    height=self.camera_info.height,
                    K=np.reshape(self.camera_intrinsics.K, (3,3)),
                    D=np.reshape(self.camera_intrinsics.D, (5,)),
                    R=np.reshape(self.camera_intrinsics.R, (3,3)),
                    P=np.reshape(self.camera_intrinsics.P, (3,4)),
                )
            else:
                print("Still waiting for camera info or intrinsics")
                return

        try:
            jpeg_data: CompressedImage = CompressedImage.from_rawdata(data).data
        except DataDecodingError as e:
            print(f"Failed to decode an incoming message: {e.message}")
            return

        image_array = (np.frombuffer(jpeg_data,np.uint8))
        decoded_image = self._jpeg.decode(image_array)
        rectified_img = self.camera.rectifier.rectify(decoded_image)

        pwm = self.model.get_wheel_velocities_from_image(rectified_img)

        try:
            await self.pwm_publisher.publish(pwm.to_rawdata())
        except Exception:
            print("Error publishing wheels data")

    async def worker(self):
        switchboard = (await context("switchboard")).navigate(self._robot_name)
        jpeg = await (switchboard / "sensor" / "camera" / "front_center" / "jpeg").until_ready()
        params = await (switchboard / "sensor" / "camera" / "front_center" / "parameters").until_ready()
        info = await (switchboard / "sensor" / "camera" / "front_center" / "info").until_ready()

        self.pwm_publisher = await (switchboard / "actuator" / "wheels" / "base" / "pwm").until_ready()

        jpeg = jpeg.configure(ContextConfig(patient=True))
        params = params.configure(ContextConfig(patient=True))
        info = info.configure(ContextConfig(patient=True))

        await params.subscribe(self.save_camera_intrinsics)
        await info.subscribe(self.save_camera_info)
        await jpeg.subscribe(self.img_cb)


        await self.join()

    async def join(self):
        while not self._shutdown:
            await asyncio.sleep(1)

    def _sigint_handler(self, _, __):
        self._shutdown = True

    @property
    def is_shutdown(self):
        return self._shutdown


    def spin(self):
        try:
            asyncio.run(self.worker())
        except RuntimeError:
            if not self.is_shutdown:
                print(f"An error occurred while running the event loop: {RuntimeError}")
                raise


if __name__ == "__main__":
    node = PyTorchAgent()

    node.spin()