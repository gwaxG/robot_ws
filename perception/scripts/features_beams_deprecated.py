#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray, Header
from skimage.feature import hog
from skimage import data, exposure
import torch.nn as nn
import torchvision
import torch
import copy

"""
Python2 Node that transforms the depth image into the HOG feature vector.
"""


class ResNet18(nn.Module):
    def __init__(self):
        super(ResNet18, self).__init__()
        original_model = torchvision.models.resnet18(pretrained=True)
        modules = list(original_model.children())[:-1]  # delete the last fc layer.
        self.features = nn.Sequential(*modules)

    def forward(self, x):
        x = self.features(x)
        return x


class FeaturesSolid:
    def __init__(self):
        rospy.init_node("FeaturesSolid")
        callback_postfix = rospy.get_param("feature_type")
        callback_name = [el for el in dir(self) if callback_postfix in el]
        assert len(callback_name) == 1, "Not only one callback {}".format(len(callback_name))
        self.pub_image = rospy.Publisher("/camera/depth/image/hog", Image, queue_size=1)
        self.pub_image_test = rospy.Publisher("/camera/depth/image/hog/test", Image, queue_size=1)
        self.pub_array = rospy.Publisher("/camera/depth/features/hog", Float32MultiArray, queue_size=1)
        self.ros_img_id = 0
        self.H = int(rospy.get_param("image_height"))
        self.W = int(rospy.get_param("image_width"))
        resolution_x = 1  # degree
        resolution_y = 1  # degree
        # Number of beams for each dimension
        self.N_x = int(12.0 / resolution_x)
        self.N_y = int(60.0 / resolution_x)
        # Size of beams
        self.step_x = self.W / self.N_x
        self.step_y = int(self.H / self.N_y)
        cam_rot_ang = 0.0
        FOV05 = 30.0
        self.H_horizontal_scan = int(self.H/2 - cam_rot_ang / 3.14 * 180.0 / FOV05 * self.H*0.5)
        self.bridge = CvBridge()
        # if-based initialization
        if "resnet" in callback_postfix.lower():
            self.net = ResNet18()
        rospy.Subscriber("/camera/depth/image_raw", Image, getattr(self, callback_name[0]), queue_size=1)
        rospy.spin()

    def apply_filter(self, depth):
        """
        Convert the depth image to 0-255 format.
        Infs and nans are detected and replaced by 0.
        Then, 0-3 is scalled to 0-255.
        :param depth:
        :return:
        """
        depth = np.nan_to_num(depth)
        if depth.max() != depth.min():
            depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255
        return depth

    def simulate_noise(self):
        """
        Add real noise patterns to the depth image
        :return:
        """
        raise NotImplemented()

    def slice_features_from_bands(self, x_band, y_band):
        """
        Transforms bands to a feature vector
        :param x_band:
        :param y_band:
        :return features:
        """
        # Averaging slices
        mean_x = self.average_slice(x_band)
        mean_y = self.average_slice(y_band)

        # Grouping up averaged slices into beams
        grouped_x, nan_x = self.grouping_slices(mean_x, self.step_x, self.N_x)
        grouped_y, nan_y = self.grouping_slices(mean_y, self.step_y, self.N_y)

        # Checking for nans
        grouped_x = self.checking_for_nans(grouped_x, nan_x, self.N_x)
        grouped_y = self.checking_for_nans(grouped_y, nan_y, self.N_y)

        # Horizontal and vertical features
        # features = np.concatenate((grouped_x, grouped_y), axis=0)
        # Only vertical features
        # test
        features = grouped_x + grouped_y
        return features

    def grouping_slices(self, mean, step, N):
        """
        Group averaged slices into beams
        :param mean:
        :param step:
        :param N:
        :return:
        """
        nans = []
        grouped = []
        for i in range(N):
            xi = step / 2 + i * step
            slice_x = mean[int(xi - step / 2):int(xi + step / 2)]
            slice_mean = np.nanmean(slice_x)
            if np.isnan(slice_mean):
                nans.append(i)
            grouped.append(slice_mean)
        return grouped, nans

    def average_slice(self, band):
        """
        Average slices in a band
        :param band:
        :return:
        """
        mean = []
        for slice in band:
            slice_mean = np.nan
            try:
                slice_mean = np.nanmean(slice)
            except Exception as e:
                pass
            mean.append(slice_mean)
        return mean

    def checking_for_nans(self, group, nans, N):
        """
        Replace nans by closest non nan values
        :param group:
        :param nans:
        :param N:
        :return:
        """

        if len(nans) == 0:
            return group
        if float(len(nans))/N > 0.5:
            group = np.nan_to_num(group)
        else:
            notnans = list(set(range(N))-set(nans))
            for nan in nans:
                notnans_with_nan = sorted(notnans+[nan])
                nan_index = notnans_with_nan.index(nan)
                if nan_index == 0:
                    replace_indexes = [notnans_with_nan[nan_index+1]]
                elif nan_index == len(notnans_with_nan)-1:
                    replace_indexes = [notnans_with_nan[nan_index-1]]
                else:
                    replace_indexes = [
                        notnans_with_nan[nan_index - 1],
                        notnans_with_nan[nan_index + 1]
                    ]
                try:
                    group[nan] = np.mean([group[index] for index in replace_indexes])
                except Exception as e:
                    print("Exception")
                    print("len group {}".format(len(group)))
                    print("nan".format(nan))
                    print("replace_indexes {}".format(replace_indexes))
                    print("nan_index {}".format(nan_index))
                    print("len(notnans_with_nan) {}".format(len(notnans_with_nan)))
                    print("N {}".format(N))
        return group

    def callback_FeatureSliceVertical(self, depth_msg):
        """
        :param depth:
        :return:
        """
        # Pre-process image
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth = copy.deepcopy(depth)
        # NOISE
        # depth = self.apply_noise(depth)
        # depth_ = cv2.line(depth, (0, self.H_horizontal_scan), (self.W - 1, self.H_horizontal_scan), (0, 0, 0), 5)
        # self.publish_image_test(depth_)
        # Slice bands
        half_band_width = 5
        x = depth[self.H_horizontal_scan-half_band_width:self.H_horizontal_scan+half_band_width, :].T  # (10, 640)->(640, 10)
        y = depth[:, int(self.W/2)-half_band_width:int(self.W/2)+half_band_width]
        # get features
        features = self.slice_features_from_bands(x, y)
        depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255
        depth = self.draw_features(depth, features)
        self.publish_image_test(depth)
        # Last chance
        features = np.nan_to_num(features)
        self.pub_array.publish(Float32MultiArray(data=features))

    def draw_features(self, depth, features):
        """
        Draw feature distances for debug purposes
        :param depth:
        :param features:
        :return:
        """
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.3
        fontColor = (255, 255, 255)
        lineType = 1
        xfs = features[0:self.N_x]
        yfs = features[self.N_x:self.N_x+self.N_y]
        xx = [int(self.step_x/2 + i * self.step_x) for i in range(self.N_x)]
        xy = [self.H_horizontal_scan for i in range(self.N_x)]
        yx = [int(self.W/2) for i in range(self.N_y)]
        yy = [int(self.step_y/2 + i * self.step_y) for i in range(self.N_y)]
        max_value = max([max(xfs), max(yfs)])
        for i, f in enumerate(xfs):
            clr = int((1.0 - f / max_value) * 255)
            depth = cv2.circle(depth, (xx[i], xy[i]), radius=2, color=(clr, clr, clr), thickness=2)
            if i % 3 == 0 and False:
                bottomLeftCornerOfText = (xx[i], xy[i]+10)
                depth = cv2.putText(
                    depth,
                    str(round(f, 2)),
                    bottomLeftCornerOfText,
                    font,
                    fontScale,
                    fontColor,
                    lineType)
        for i, f in enumerate(yfs):
            center = (yx[i], yy[i])
            clr = int((1.0 - f / max_value) * 255)
            depth = cv2.circle(depth, (yx[i]+10, yy[i]), radius=2, color=(clr, clr, clr), thickness=2)
            if i % 3 == 0 and False:
                bottomLeftCornerOfText = (yx[i] + 15, yy[i])
                depth = cv2.putText(
                    depth,
                    str(round(f, 2)),
                    bottomLeftCornerOfText,
                    font,
                    fontScale,
                    fontColor,
                    lineType)
        return depth

    def callback_FeaturizedResNet(self, depth_img_msg):
        """
        Receive a ROS depth image and publish float array composed of height, width,... values
        :param depth_img_msg: ROS Image
        :return:
        """
        # to_save_2 = lambda depth: (depth - depth.min()) / (depth.max() - depth.min()) * 255
        # to_save = lambda depth: to_save_2(np.nan_to_num(depth))
        depth_img = self.bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding='passthrough')
        # The prev. depth_img is read-only.
        depth_img = copy.deepcopy(depth_img)
        # cv2.imwrite("1no_noise.jpg", to_save(depth_img))
        depth_img = self.apply_noise(depth_img)
        # cv2.imwrite("2noise.jpg", to_save(depth_img))
        depth_img = cv2.resize(depth_img, (128, 96))
        # cv2.imwrite("3noise_resized.jpg", to_save(depth_img))
        depth_img = self.apply_filter(depth_img)
        # cv2.imwrite("4filtered.jpg", depth_img)
        # self.publish_image_test(depth_img)
        color_img = cv2.cvtColor(depth_img, cv2.COLOR_GRAY2RGB)
        # cv2.imwrite("5colored.jpg", depth_img)
        # self.publish_image(color_img)
        color_img = np.moveaxis(color_img, 2, 0)
        observations = torch.Tensor(color_img).unsqueeze(0)
        features = self.net(observations)
        features = features.detach().numpy().flatten()
        # Artificial filtering for OpenAI Box obs space np.count_nonzero(~np.isnan(data)) <- non nan
        features[features > 1.5] = 1.5
        self.pub_array.publish(Float32MultiArray(data=list(features)))

    def callback_CompressedVector(self, depth_img_msg):
        """
        Receive a ROS depth image and publish float array composed of height, width,... values
        :param depth_img_msg: ROS Image
        :return:
        """
        img = self.bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding='passthrough')
        img = cv2.resize(img, (32, 32))
        where_are_NaNs = np.isnan(img)
        try:
            img[where_are_NaNs] = 0
        except Exception:
            pass
        fd = img.flatten()
        self.pub_array.publish(Float32MultiArray(data=fd))
        self.publish_image(img)

    def apply_noise(self, depth):
        """
        Apply noise
        :param depth:
        :return:
        """
        # Noise percentage
        npercent = 0.01  # 10 %
        mu = 0.0
        sigma = 0.01
        noise = np.random.normal(mu, sigma, (self.H, self.W))
        nans = np.array([0 if np.random.random() > npercent else np.nan for i in range(self.H * self.W)]).reshape((self.H, self.W))
        # apply noise and nans
        depth += noise + nans
        return depth

    def publish_image_test(self, img):
        """
        Get an array and, then, publish a grey image
        :param depth_img_msg: float32 array composed of height, width,... values
        :return:
        """
        msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        msg.encoding = '32FC1'
        msg.header.frame_id = "camera_depth_frame"
        self.pub_image_test.publish(msg)

    def publish_image(self, img):
        """
        Get an array and, then, publish a grey image
        :param depth_img_msg: float32 array composed of height, width,... values
        :return:
        """
        msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        msg.encoding = '32FC1'
        self.pub_image.publish(msg)

if __name__ == '__main__':
    FeaturesSolid()