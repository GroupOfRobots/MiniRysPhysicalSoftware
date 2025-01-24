import cv2 as cv
import numpy as np
from pid import PID
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import get_logger
from enum import Enum
from typing import Optional

class LineFollowerModuleError(RuntimeError):
    pass

class LineFollowerModule:
    class InputMode(Enum):
        NONE = 0
        RAW = 1
        PROCESSED = 2

    image_ = None
    input_mode_ = InputMode.NONE
    image_binary_ = None
    image_debug_ = None
    pid_ = None
    #control_points_ = [(220, 270),(220, 230),(220, 200),(220, 170)]
    control_points_ = [(220, 200)]
    leftT_ = (435, 270)
    rightT_ = (5, 270)

    def __init__(
            self,
            step_time: float,
            pid_k: float,
            pid_ti: float,
            pid_td: float,
            min_u: float,
            max_u: float,
            turn_offset: float,
            logger: Optional[RcutilsLogger]
        ) -> None:
        self.pid_ = PID(step_time, pid_k, pid_ti, pid_td)
        self.min_u_ = min_u
        self.max_u_ = max_u
        self.turn_offset_ = turn_offset
        self.logger_ = logger if logger is not None else get_logger("LineFollowerModule")

    def set_image(self, image: np.ndarray, mode: InputMode) -> None:
        """
        Image should be either binary or encoded in RGB
        """
        self.image_ = image
        self.input_mode_ = mode

    def get_image_binary(self) -> np.ndarray:
        """
        Call after `compute_angular_velocity`
        """
        if self.image_binary_ is not None:
            return self.image_binary_
        raise LineFollowerModuleError("Binary image has not yet been created")

    def get_image_debug(self) -> np.ndarray:
        """
        Call after `compute_angular_velocity`
        """
        if self.image_debug_ is not None:
            return self.image_debug_
        raise LineFollowerModuleError("Debug image has not yet been created")

    def compute_angular_velocity(self) -> Optional[float]:
        try:
            self.__process_image()
            contour = self.__compute_max_contour()

        except LineFollowerModuleError as e:
            self.logger_.warning(f"Encountered error \"{e}\"."
                                 + " Errors at this stage can be raised due to the input image."
                                 + " Skipping this update!")
            return None

        self.image_debug_ = self.image_
        cv.drawContours(image=self.image_debug_, contours=[contour], contourIdx=-1, color=(0, 255, 0), thickness=3, lineType=cv.LINE_AA)

        turn_offset = 0
        error_sum = 0
        for control_point in self.control_points_:
            matching_sum = 0
            matching_count = 0
            for contour_point in contour:
                contour_point = contour_point[0]  # This is necessary due to the way OpenCV works

                # Increment when vertical coordinate matches
                if contour_point[1] == control_point[1]:
                    matching_sum += contour_point[0]
                    matching_count += 1

            if matching_count != 0:
                matching_mean = matching_sum / matching_count
                error_sum += matching_mean - control_point[0]

                cv.circle(self.image_debug_, control_point,                          radius=5, color=(0, 0, 255), thickness=-1)
                cv.circle(self.image_debug_, (int(matching_mean), control_point[1]), radius=5, color=(255, 0, 0), thickness=-1)

        error_mean = error_sum / len(self.control_points_)

        # recognizing 90 degree turn
        if self.__is_in_polygon(contour, self.leftT_) and not self.__is_in_polygon(contour, self.rightT_):
            turn_offset = -self.turn_offset_#-1.0 #-1

        elif not self.__is_in_polygon(contour, self.leftT_) and self.__is_in_polygon(contour, self.rightT_):
            turn_offset = self.turn_offset_#1.0 #1

        u = self.pid_.pid(error_mean, 0) + turn_offset
        u = np.clip(u, self.min_u_, self.max_u_)
        return u

    def __is_in_polygon(self, polygon: np.ndarray, point: tuple) -> bool:
        return cv.pointPolygonTest(polygon, point, False) >= 0

    def __process_image(self) -> None:
        if self.image_ is None:
            raise LineFollowerModuleError('Cannot process image when the image is not set')

        if self.InputMode.PROCESSED == self.input_mode_:
            self.image_binary_ = self.image_
            return

        elif self.InputMode.RAW == self.input_mode_:
            gray = cv.cvtColor(self.image_, cv.COLOR_BGR2GRAY)
            kernel = np.ones((5, 5), np.uint8)
            img_erosion = cv.erode(gray, kernel, iterations=1)

            _, thresh = cv.threshold(img_erosion, 127, 255, cv.THRESH_BINARY_INV)
            self.image_binary_ = thresh
            return

        raise LineFollowerModuleError('Cannot process image when input mode is not set')

    def __compute_max_contour(self) -> np.ndarray:
        if self.image_binary_ is None:
            raise LineFollowerModuleError('Cannot compute max contour when the image is not set')

        contours, _ = cv.findContours(image=self.image_binary_, mode=cv.RETR_EXTERNAL, method=cv.CHAIN_APPROX_NONE)
        areas = [cv.contourArea(c) for c in contours]

        if 0 == np.size(areas):
            raise LineFollowerModuleError('No contour areas were found. Unable to compute the max contour')

        max_idx = np.argmax(areas)
        return contours[max_idx]
