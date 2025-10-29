# V2I-GridCalib: Distance-Adaptive Rasterizer (ROS2)

This ROS2 workspace implements the core RSU-side algorithm, **"Distance-Adaptive Rasterized View Generation"**, from the paper **"V2I-GridCalib: A Real-time Online Calibration for Vehicle-to-Infrastructure Systems"**.

The `v2i_gridcalib_rasterizer` node provides a service that takes a sector of a 3D point cloud and an approximate vehicle distance as input. It then outputs a 4-channel pseudo-image where each pixel contains the LiDAR intensity and the precise 3D coordinates `(X, Y, Z)` of the corresponding point. This process is crucial for solving the observational scale mismatch problem in V2I calibration.

---

这个ROS2工作空间实现了论文 **"V2I-GridCalib: A Real-time Online Calibration for Vehicle-to-Infrastructure Systems"** 中的路侧单元(RSU)核心算法——**“距离自适应的光栅化视图生成”**。

`v2i_gridcalib_rasterizer` 节点提供了一个ROS2服务，该服务接收一个扇形3D点云和车辆的大致距离作为输入，然后输出一个四通道的伪图像。在这个伪图像中，每个像素都包含了激光雷达的强度值以及对应点的精确三维坐标 `(X, Y, Z)`。这个过程对于解决V2I标定中的观测尺度失配问题至关重要。

## Dependencies / 依赖项

*   **ROS2 Humble Hawksbill**
*   **Python 3.8+**
*   **Python Libraries (Python 库):**
    *   `numpy`
    *   `opencv-python`
    *   `open3d` (用于测试客户端读取 .pcd 文件)
    *   `ros2-numpy` (用于ROS消息与NumPy数组之间的高效转换)

## Installation / 安装

1.  **Create your ROS2 workspace.** (创建您的ROS2工作区)
    ```bash
    mkdir -p ~/V2I-GridCalib_ws/src
    cd ~/V2I-GridCalib_ws/src
    ```

2.  **Clone or copy the packages** (`v2i_gridcalib_interfaces` and `v2i_gridcalib_rasterizer`) into the `src` folder. (将 `v2i_gridcalib_interfaces` 和 `v2i_gridcalib_rasterizer` 两个功能包克隆或复制到 `src` 文件夹中)

3.  **Install Python dependencies.** (安装Python依赖项)
    ```bash
    pip install numpy opencv-python open3d ros2-numpy
    ```

4.  **Build the workspace.** (编译工作区)
    ```bash
    cd ~/V2I-GridCalib_ws
    colcon build
    ```

5.  **Source the workspace environment.** (Source工作区环境)
    ```bash
    source ~/V2I-GridCalib_ws/install/setup.bash
    ```
    *Tip: Add this line to your `~/.bashrc` to avoid sourcing it for every new terminal.*
    *提示：将此行添加到您的 `~/.bashrc` 文件中，以避免每次打开新终端时都要手动执行。*

## Usage / 使用方法

The system operates as a ROS2 Service. You first need to launch the service node, then call it from a client application (e.g., a test script) with your point cloud data.

本系统以ROS2服务的形式运行。您需要首先启动服务节点，然后从一个客户端应用（例如，一个测试脚本）使用您的点云数据来调用它。

### 1. Launch the Rasterizer Service Node / 启动光栅化服务节点

Open a terminal, source your workspace, and use the provided launch file:
打开一个新终端，source您的工作区，然后使用我们提供的launch文件：

```bash
source ~/V2I-GridCalib_ws/install/setup.bash
ros2 launch v2i_gridcalib_rasterizer rasterizer.launch.py
```
You should see the output: `[rasterizer_node-1] [INFO] [16...]: Rasterizer service is ready. Waiting for requests...`
您应该能看到类似输出：`[rasterizer_node-1] [INFO] [16...]: Rasterizer service is ready. Waiting for requests...`

### 2. Call the Service with a Test Client / 使用测试客户端调用服务

Passing a large point cloud via the command line is not feasible. We recommend using the following Python script as a client to test the service.

通过命令行传递大型点云是不可行的。我们建议使用下面的Python脚本作为客户端来测试服务。

**a. Prepare your data / 准备您的数据**

Save a point cloud file in `.pcd` format. For this example, let's assume you save it at `/home/user/data.pcd`. **The PCD file must contain intensity data.**

将一个点云文件保存为 `.pcd` 格式。在本示例中，我们假设您将其保存在 `/home/user/data.pcd`。 **PCD文件必须包含强度(intensity)数据。**

**b. Create and run the test script / 创建并运行测试脚本**

Create a Python file named `test_client.py` anywhere on your system:
在您系统的任何位置创建一个名为 `test_client.py` 的Python文件：

```python
# test_client.py
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from v2i_gridcalib_interfaces.srv import RasterizePointCloud
import open3d as o3d
import cv2
from ros2_numpy import numpify

def pcd_to_ros_msg(pcd_path, frame_id="rsu_lidar_frame"):
    """ Reads a .pcd file and converts it to a ROS2 PointCloud2 message. """
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    
    # This example assumes the PCD file has XYZ and Intensity.
    # If not, you might need to handle it differently.
    # We will assume a dummy intensity if it's not present.
    if pcd.has_colors(): # Using colors as intensity for this example
        intensities = np.asarray(pcd.colors)[:,0] # Use R channel as intensity
    else:
        intensities = np.zeros(len(points))

    # Create a structured NumPy array that matches ROS2 PointField conventions
    dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)]
    structured_points = np.zeros(points.shape, dtype=dtype)
    structured_points['x'] = points[:, 0]
    structured_points['y'] = points[:, 1]
    structured_points['z'] = points[:, 2]
    structured_points['intensity'] = intensities

    # Manually create the PointCloud2 message
    msg = PointCloud2()
    msg.header = Header(stamp=Node('pcd_loader').get_clock().now().to_msg(), frame_id=frame_id)
    msg.height = 1
    msg.width = len(structured_points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.data = structured_points.tobytes()
    msg.is_dense = True
    return msg

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('rasterizer_client_node')
    cli = node.create_client(RasterizePointCloud, '/v2i_gridcalib/rasterize_pointcloud')

    while not cli.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('Service not available, waiting...')

    req = RasterizePointCloud.Request()
    
    # --- CONFIGURE YOUR TEST HERE ---
    # IMPORTANT: Change this path to your actual PCD file!
    # 重要：请将此路径修改为您的PCD文件实际路径！
    pcd_file_path = '/home/user/data.pcd'
    test_distance = 25.0 # Test with a distance of 25 meters | 使用25米的距离进行测试
    # --------------------------------

    try:
        req.pointcloud_sector = pcd_to_ros_msg(pcd_file_path)
        req.approximate_distance = test_distance
    except Exception as e:
        node.get_logger().error(f"Failed to read PCD file: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Successfully called service!')
        
        # Visualize the result
        result_img_msg = future.result().raster_view
        raster_view_np = numpify(result_img_msg)
        
        # Display the intensity channel (Channel 0)
        intensity_channel = raster_view_np[:, :, 0]
        # Normalize for display
        intensity_display = cv2.normalize(intensity_channel, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        
        cv2.imshow("Resulting Intensity View", intensity_display)
        print("Press any key in the image window to exit.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        node.get_logger().error(f'Exception while calling service: {future.exception()}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**c. Run the test client / 运行测试客户端**

1.  **IMPORTANT**: Open `test_client.py` and change `/home/user/data.pcd` to the actual path of your PCD file.
2.  Open a **new terminal** (the service node should still be running in the first one), source your workspace, and run the script:

    打开一个 **新终端** （服务节点应仍在第一个终端中运行），source您的工作区，然后运行脚本：

    ```bash
    source ~/V2I-GridCalib_ws/install/setup.bash
    python3 test_client.py
    ```

You will see logs in both terminals. If successful, a window will pop up showing the rasterized intensity view of your point cloud.

您将在两个终端中看到日志。如果成功，一个窗口将会弹出，显示您的点云经过光栅化后的强度视图。