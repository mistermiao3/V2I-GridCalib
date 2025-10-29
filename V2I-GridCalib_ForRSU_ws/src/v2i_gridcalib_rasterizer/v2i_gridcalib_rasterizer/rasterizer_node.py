# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, Image
from ros2_numpy import numpify, msgify
from v2i_gridcalib_interfaces.srv import RasterizePointCloud

class VirtualCamParams:
    """
    A simple structure to hold virtual camera intrinsic parameters.
    一个用于存储虚拟相机内参的简单数据结构。
    """
    def __init__(self, fx, fy, cx, cy, width, height):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.width = width
        self.height = height

class RasterizerNode(Node):
    """
    This ROS2 node provides a service to implement the 'Distance-Adaptive Rasterized View Generation'
    method from the V2I-GridCalib paper. It converts a 3D point cloud sector into a 
    4-channel pseudo-image based on an approximate distance.

    这个ROS2节点提供了一个服务，用于实现V2I-GridCalib论文中的“距离自适应的光栅化视图生成”方法。
    它根据一个大致的距离，将一个三维点云扇区转换为一个四通道的伪图像。
    """
    def __init__(self):
        super().__init__('rasterizer_node')
        
        # Create a service that other nodes can call to request the rasterization process.
        # 创建一个服务，其他节点可以调用此服务来请求光栅化处理。
        self.srv = self.create_service(
            RasterizePointCloud, 
            '/v2i_gridcalib/rasterize_pointcloud', 
            self.rasterize_callback
        )
        self.get_logger().info('Rasterizer service is ready. Waiting for requests...')

        # This library holds different sets of camera intrinsics for various distance ranges.
        # This is the core component of the "distance-adaptive" strategy described in the paper.
        # 这个库为不同的距离范围存储了不同的相机内参。
        # 这是论文中描述的“距离自适应”策略的核心组成部分。
        self.cam_param_library = [
            # (max_distance_threshold, camera_parameters)
            (15.0, VirtualCamParams(fx=400.0, fy=400.0, cx=480.0, cy=270.0, width=960, height=540)),  # Wide-angle for close ranges | 广角，用于近距离
            (40.0, VirtualCamParams(fx=800.0, fy=800.0, cx=480.0, cy=270.0, width=960, height=540)),  # Standard lens for mid-ranges | 标准镜头，用于中距离
            (float('inf'), VirtualCamParams(fx=1200.0, fy=1200.0, cx=480.0, cy=270.0, width=960, height=540)) # Telephoto for far ranges | 长焦，用于远距离
        ]

    def select_cam_params(self, distance: float) -> VirtualCamParams:
        """
        Selects the appropriate virtual camera parameters from the library based on the provided distance.
        根据提供的距离，从参数库中选择合适的虚拟相机参数。
        """
        for max_dist, params in self.cam_param_library:
            if distance <= max_dist:
                self.get_logger().info(f'Distance {distance:.2f}m is within range <= {max_dist}m. Using fx={params.fx}.')
                return params
        return self.cam_param_library[-1][1]

    def rasterize_callback(self, request: RasterizePointCloud.Request, response: RasterizePointCloud.Response) -> RasterizePointCloud.Response:
        """
        The main callback function for the service. It executes the full pipeline:
        1. Converts ROS PointCloud2 to NumPy.
        2. Performs histogram equalization on intensity.
        3. Selects adaptive camera parameters.
        4. Projects 3D points to a 2D plane, creating a 4-channel image.
        5. Returns the image as a ROS Image message.

        服务的主要回调函数。它执行完整的处理流程：
        1. 将ROS PointCloud2消息转换为NumPy数组。
        2. 对强度值进行直方图均衡化。
        3. 选择自适应的相机参数。
        4. 将3D点投影到2D平面，创建一个四通道图像。
        5. 将图像作为ROS Image消息返回。
        """
        self.get_logger().info(f'Received rasterization request for a point cloud with {request.pointcloud_sector.width} points at distance {request.approximate_distance:.2f}m.')

        # Step 1: Convert ROS PointCloud2 message to a NumPy structured array.
        # 步骤 1: 将ROS PointCloud2消息转换为NumPy结构化数组。
        try:
            points_structured = numpify(request.pointcloud_sector)
            # Reshape if the point cloud is not organized
            if points_structured.ndim > 1:
                points_structured = points_structured.reshape(-1)
            points = np.stack([points_structured['x'], points_structured['y'], points_structured['z'], points_structured['intensity']], axis=1)
        except Exception as e:
            self.get_logger().error(f'Failed to convert PointCloud2 to NumPy array: {e}')
            return response # Return an empty response on failure

        # Step 2: Perform Histogram Equalization on intensity values for better visual contrast.
        # 步骤 2: 对强度值进行直方图均衡化，以获得更好的视觉对比度。
        intensities = points[:, 3].copy()
        if np.max(intensities) > 0:
            intensities_norm = (intensities / np.max(intensities) * 255).astype(np.uint8)
        else:
            intensities_norm = intensities.astype(np.uint8)
        
        equalized_intensities_norm = cv2.equalizeHist(intensities_norm)
        equalized_intensities = equalized_intensities_norm.astype(np.float32) / 255.0

        # Step 3: Select virtual camera parameters based on the provided distance.
        # 步骤 3: 根据提供的距离选择虚拟相机参数。
        cam_params = self.select_cam_params(request.approximate_distance)
        
        # Step 4: Project points onto the 2D image plane and build the 4-channel view.
        # 步骤 4: 将点投影到2D图像平面，并构建四通道视图。
        raster_view = np.zeros((cam_params.height, cam_params.width, 4), dtype=np.float32)

        # Filter out points that are behind the virtual camera (z <= 0).
        # 过滤掉虚拟相机后方 (z <= 0) 的点。
        valid_indices = points[:, 2] > 0
        points_fwd = points[valid_indices]
        equalized_intensities_fwd = equalized_intensities[valid_indices]

        # Apply the pinhole camera projection model (vectorized for efficiency).
        # 应用针孔相机投影模型（为提高效率进行了矢量化操作）。
        u = (cam_params.fx * points_fwd[:, 0] / points_fwd[:, 2] + cam_params.cx).astype(int)
        v = (cam_params.fy * points_fwd[:, 1] / points_fwd[:, 2] + cam_params.cy).astype(int)

        # Filter out points that project outside the image boundaries.
        # 过滤掉投影到图像边界之外的点。
        in_bounds_indices = (u >= 0) & (u < cam_params.width) & (v >= 0) & (v < cam_params.height)
        u_in = u[in_bounds_indices]
        v_in = v[in_bounds_indices]
        points_in = points_fwd[in_bounds_indices]
        intensities_in = equalized_intensities_fwd[in_bounds_indices]

        # Handle occlusions by sorting points by depth from far to near.
        # When projected, nearer points will overwrite farther ones at the same pixel location.
        # 通过将点按深度从远到近排序来处理遮挡问题。
        # 投影时，较近的点会自然地覆盖同一像素位置上较远的点。
        depths = points_in[:, 2]
        sorted_indices = np.argsort(depths)[::-1] # Sort in descending order (far to near)

        # Reorder all relevant arrays based on the sorted depth.
        # 根据排序后的深度重新排列所有相关数组。
        u_sorted = u_in[sorted_indices]
        v_sorted = v_in[sorted_indices]
        points_sorted = points_in[sorted_indices]
        intensities_sorted = intensities_in[sorted_indices]

        # Prepare the 4-channel data packet for each point: [Intensity, X, Y, Z].
        # 为每个点准备四通道数据包：[强度, X, Y, Z]。
        four_channel_data = np.stack([
            intensities_sorted,
            points_sorted[:, 0],
            points_sorted[:, 1],
            points_sorted[:, 2]
        ], axis=1)

        # Efficiently assign the data to the raster view using NumPy's advanced indexing.
        # 使用NumPy的高级索引，高效地将数据赋值给光栅视图。
        raster_view[v_sorted, u_sorted] = four_channel_data

        # Step 5: Convert the final NumPy array back to a ROS Image message.
        # 步骤 5: 将最终的NumPy数组转换回ROS Image消息。
        # The '32FC4' encoding is crucial as it represents 4 channels of 32-bit floats.
        # '32FC4' 编码至关重要，它代表了4个32位浮点数通道。
        response.raster_view = msgify(Image, raster_view, encoding='32FC4')
        response.raster_view.header.frame_id = request.pointcloud_sector.header.frame_id

        self.get_logger().info(f'Rasterization complete. Returning a {cam_params.width}x{cam_params.height} image.')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RasterizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()