# -- coding: utf-8 -
import pyrealsense2 as rs2
import numpy as np
from Common import singleton, Timeout, Queue

RS2_WIDTH = 848
RS2_HEIGHT = 480
RS2_FPS = 30
RS2_WIDTH_COMPENSATE = 0
RS2_HEIGHT_COMPENSATE = 40

RS2_TEMP_FILTER_ALPHA = 0.4
RS2_TEMP_FILTER_DELTA = 20
RS2_SPAT_FILTER_MAGNITUDE = 2
RS2_HF_FILTER_MODE = 1


@singleton
class D435Camera(object):
    def __init__(self):
        self.pipeline = rs2.pipeline()
        self.config = rs2.config()
        self.config.enable_stream(rs2.stream.color, RS2_WIDTH, RS2_HEIGHT, rs2.format.bgr8, RS2_FPS)
        self.config.enable_stream(rs2.stream.depth, RS2_WIDTH, RS2_HEIGHT, rs2.format.z16, RS2_FPS)
        # self.align = rs2.align(rs2.stream.color)

        # filter of time
        temporal_filter = rs2.temporal_filter()
        temporal_filter.set_option(rs2.option.filter_smooth_alpha, RS2_TEMP_FILTER_ALPHA)
        temporal_filter.set_option(rs2.option.filter_smooth_delta, RS2_TEMP_FILTER_DELTA)

        # filter of space
        spatial_filter = rs2.spatial_filter()
        spatial_filter.set_option(rs2.option.filter_magnitude, RS2_SPAT_FILTER_MAGNITUDE)

        hole_filling_filter = rs2.hole_filling_filter()
        hole_filling_filter.set_option(rs2.option.holes_fill, RS2_HF_FILTER_MODE)
        self.profile = None

        self.depth_frame_queue = [rs2.depth_frame for _ in range(4096)]
        # depth_frame_queue = Queue(2)
        # color_frame_queue = Queue(2)

    def Open(self):
        self.profile = self.pipeline.start(self.config)
        if not self.profile:
            return False
        return True

    def Read(self):
        if not self.profile:
            return None

        try:
            frame_set = self.pipeline.wait_for_frames()
            color_frame = frame_set.get_color_frame()
            depth_frame = frame_set.get_depth_frame()

            return color_frame, depth_frame
        except:
            return None, None

    def Close(self):
        self.pipeline.stop()
        self.profile = None
