#!/usr/bin/env python

import os
import sys
import argparse
from PIL import Image
import numpy as np
import cv2

import torch
from torch.backends import cudnn
import torchvision.transforms as transforms

sys.path.append("/home/neofelis/seg_mapping_ws/src/segmentation-ros-wrapper/scripts/semantic-segmentation/")
import network
from optimizer import restore_snapshot
from datasets import cityscapes
from datasets import kitti
from datasets import rellis3d
from config import assert_and_infer_cfg

# ROS
import rospy
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32, PointCloud2
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from copy import deepcopy

class SegmentationNode:
    def __init__(self):
        torch.cuda.empty_cache()
        self.set_up()
        self.get_net()
        rospy.init_node("segmentation_node_test", anonymous=True)
        #self.img_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size = 10, buff_size = 2**24)
        self.color_img_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_img_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        ts = message_filters.ApproximateTimeSynchronizer([self.color_img_sub, self.depth_img_sub], 1, 0.5)
        ts.registerCallback(self.callback)
        self.semantic_pub = rospy.Publisher("semantic_seg", Image, queue_size = 1)
        self.trav_pub = rospy.Publisher("trav_seg", Image, queue_size = 1)
        #self.labeled_pc_pub = rospy.Publisher("labeled_pointcloud", PointCloud, queue_size = 1)
        # self.labeled_pcl2_pub = rospy.Publisher("labeled_pcl2", PointCloud2, queue_size = 1)
        self.labeled_pc_array_pub = rospy.Publisher("labeled_array", Float32MultiArray, queue_size=1)
        rospy.loginfo("Initialization Done. Running Inference...")

    def set_up(self):
        parser = argparse.ArgumentParser(description='demo')
        #parser.add_argument('--demo-image', type=str, default='', help='path to demo image', required=True)
        parser.add_argument('--snapshot', type=str, default='/home/neofelis/seg_mapping_ws/src/segmentation-ros-wrapper/scripts/semantic-segmentation/pretrained_models/kitti_best.pth', help='pre-trained checkpoint')
        parser.add_argument('--arch', type=str, default='network.deepv3.DeepWV3Plus', help='network architecture used for inference')
        #parser.add_argument('--save-dir', type=str, default='./save', help='path to save your results')
        self.args = parser.parse_args()
        assert_and_infer_cfg(self.args, train_mode=False)
        cudnn.benchmark = False
        torch.cuda.empty_cache()
        self.args.dataset_cls = kitti
        self.cx = 318.24
        self.cy = 246.72
        self.fx = 383.14
        self.fy = 382.91
        self.depth_scaling = 1000

    def get_net(self):
        net = network.get_net(self.args, criterion=None)
        net = torch.nn.DataParallel(net).cuda()
        rospy.loginfo("Net built.")
        self.net, _ = restore_snapshot(net, optimizer=None, snapshot=self.args.snapshot, restore_optimizer_bool=False)
        self.net.eval()
        rospy.loginfo("Net restored.")

    def callback(self, img_msg, depth_msg):
        br = CvBridge()
        img = br.imgmsg_to_cv2(img_msg , desired_encoding="rgb8") #TODO: RGB or BGR?
        #print(img)
        mean_std = ([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        img_transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize(*mean_std)])
        img_tensor = img_transform(img)
        with torch.no_grad():
            img = img_tensor.unsqueeze(0).cuda()
            pred = self.net(img)
            rospy.loginfo("Inference done.")
        pred = pred.cpu().numpy().squeeze()
        pred = np.argmax(pred, axis=0)
        #print(pred)
        colorized = self.args.dataset_cls.colorize_mask(pred)
        #colorized.save('color_mask.png')
        print(colorized)
        # Publish colorized pred
        colorized_msg = img_msg
        colorized_msg.data = np.array(colorized.convert('RGB')).tobytes()
        self.semantic_pub.publish(colorized_msg)
        self.get_labeled_pc(pred, colorized.convert('RGB'), depth_msg)
       
    def get_labeled_pc(self, pred, colorized, depth_msg):
        # to_publish = PointCloud()
        # to_publish.header = depth_msg.header
        # label_channel = ChannelFloat32()
        # label_channel.name = "labels"
        # label_channel.values = []
        br = CvBridge()
        depth_img = br.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        
        ux = np.arange(depth_img.shape[1])
        uy = np.arange(depth_img.shape[0])
        ux, uy =  np.meshgrid(ux, uy)
        
        pix_depth = depth_img / self.depth_scaling     
        pix_depth.astype('float')
        
        px = (ux - self.cx) * (1.0 / self.fx) * pix_depth
        py = (uy - self.cy) * (1.0 / self.fy) * pix_depth
        pz = pix_depth
        
        px = px.flatten()
        py = py.flatten()
        pz = pz.flatten()
        pred = pred.flatten()
        
        # sjy edit: add pointcloud2
        # points = np.array([px,py,pz,pred]).T # 4*N -> N*4
        # print(points.shape)
        # points.dtype = [('x', np.float64), ('y', np.float64), ('z', np.float64), ('pred', np.float64)]
        # pcl2_msg = ros_numpy.msgify(PointCloud2, points)
        # pcl2_msg.header = depth_msg.header
        # self.labeled_pcl2_pub.publish(pcl2_msg)

        # sjy change pointcloud2 to Float32MultiArray
        points = np.array([px,py,pz,pred]).T # 4*N -> N*4
        points_msg = Float32MultiArray()
        points_msg.data = points.reshape([points.shape[0]*points.shape[1]]).tolist()
        points_msg.layout.data_offset = 0
        points_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        points_msg.layout.dim[0].label = "index"
        points_msg.layout.dim[0].size = points.shape[0]
        points_msg.layout.dim[0].stride = points.shape[0]*points.shape[1]
        points_msg.layout.dim[1].label = "position"
        points_msg.layout.dim[1].size = points.shape[1]
        points_msg.layout.dim[1].stride = points.shape[1]
        self.labeled_pc_array_pub.publish(points_msg)

        return

        # for i in range(len(px)):
        #     p = Point32()
        #     p.x, p.y, p.z = px[i], py[i], pz[i]
        #     to_publish.points.append(p)
        
        # print(pred.shape)
        print(np.max(pred))
        label_channel.values = pred.flatten()
        to_publish.channels = [label_channel]
        self.labeled_pc_pub.publish(to_publish)
        return 
        # for i in range(depth_img.shape[0] * depth_img.shape[1]):
        #    ux = int(i % depth_img.shape[1])
        #    uy = int(i / depth_img.shape[1])
        #    #print(ux, uy)
        #    pix_depth = float(depth_img[uy,ux] / self.depth_scaling);
        #    if pix_depth > 8:
        #        continue
        #    p = Point32()
        #    p.x = (ux-self.cx) * (1.0 / self.fx) * pix_depth
        #    p.y = (uy-self.cy) * (1.0 / self.fy) * pix_depth
        #    p.z = pix_depth
        #    to_publish.points.append(p)
        #    label_channel.values.append(pred[uy][ux])
        label_channel.values = pred[:][0]
        to_publish.channels = [label_channel]
        p = Point32()
        #for i in range(pcl.shape[0]):
        #    p.x, p.y, p.z = pcl[i][0], pcl[i][1], pcl[i][2]
        #    to_publish.points.append(p)
            # to_publish.points.append(Point32(pcl[i][0], pcl[i][1], pcl[i][2]))
        # to_publish.points = pcl.tolist()
        for it in pcl:
            # print(it)
            p.x, p.y, p.z = it[0], it[1], it[2]
            to_publish.points.append(p)
            # print(p.x, p.y, p.z)
        self.labeled_pc_pub.publish(to_publish)

if __name__ == '__main__':
    sys.aargv = rospy.myargv() # to enable roslaunch
    node = SegmentationNode()
    rospy.spin()

if not os.path.exists(args.save_dir):
    os.makedirs(args.save_dir)

colorized = args.dataset_cls.colorize_mask(pred)
colorized.save(os.path.join(args.save_dir, 'color_mask.png'))

label_out = np.zeros_like(pred)
for label_id, train_id in args.dataset_cls.id_to_trainid.items():
    label_out[np.where(pred == train_id)] = label_id
    cv2.imwrite(os.path.join(args.save_dir, 'pred_mask.png'), label_out)
print('Results saved.')
