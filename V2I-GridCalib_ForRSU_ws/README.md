V2I-GridCalib: Distance-Adaptive Rasterizer (ROS2)
This ROS2 workspace implements the core RSU-side algorithm, "Distance-Adaptive Rasterized View Generation", from the paper "V2I-GridCalib: A Real-time Online Calibration for Vehicle-to-Infrastructure Systems".
The v2i_gridcalib_rasterizer node provides a ROS2 service that takes a sector of a 3D point cloud and an approximate vehicle distance as input. It then outputs a 4-channel pseudo-image where each pixel contains the LiDAR intensity and the precise 3D coordinates (X, Y, Z) of the corresponding point. This process is crucial for solving the observational scale mismatch problem in V2I calibration.
Dependencies
ROS2 Humble Hawksbill
Python 3.8+
Python Libraries:
numpy
opencv-python
open3d (used by the test client to read .pcd files)
ros2-numpy (for efficient conversion between ROS messages and NumPy arrays)
Installation
Create your ROS2 workspace.
code
Bash
mkdir -p ~/V2I-GridCalib_ws/src
cd ~/V2I-GridCalib_ws/src
Clone or copy the packages (v2i_gridcalib_interfaces and v2i_gridcalib_rasterizer) into the src folder.
Install Python dependencies.
code
Bash
pip install numpy opencv-python open3d ros2-numpy
Build the workspace.
code
Bash
cd ~/V2I-GridCalib_ws
colcon build
Source the workspace environment.
code
Bash
source ~/V2I-GridCalib_ws/install/setup.bash
Tip: Add this line to your ~/.bashrc to avoid sourcing it for every new terminal.
Usage
The system operates as a ROS2 Service. You first need to launch the service node, then call it from a client application (e.g., a test script) with your point cloud data.
1. Launch the Rasterizer Service Node
Open a terminal, source your workspace, and use the provided launch file:
code
Bash
source ~/V2I-GridCalib_ws/install/setup.bash
ros2 launch v2i_gridcalib_rasterizer rasterizer.launch.py
You should see the output: [rasterizer_node-1] [INFO] [...]: Rasterizer service is ready. Waiting for requests...
2. Call the Service with a Test Client
Passing a large point cloud via the command line is not feasible. We recommend using the following Python script as a client to test the service.
a. Prepare your data
Save a point cloud file in .pcd format. For this example, let's assume you save it at /home/user/data.pcd. The PCD file must contain intensity data for best results.
b. Create and run the test script
Create a Python file named test_client.py anywhere on your system:
code
Python
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
    
    # This example checks for colors and uses them as intensity.
    # If your PCD has a dedicated intensity field, you may need to adapt the reader.
    if pcd.has_colors(): 
        # Using the Red channel as a stand-in for intensity
        intensities = np.asarray(pcd.colors)[:,0] 
    else:
        # Create dummy intensity if none is available
        intensities = np.zeros(len(points))

    # Create a structured NumPy array that matches ROS2 PointField conventions
    dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)]
    structured_points = np.zeros(points.shape[0], dtype=dtype)
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
    pcd_file_path = '/home/user/data.pcd'
    test_distance = 25.0 # Test with a distance of 25 meters
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
        
        # Visualize the resulting intensity channel
        result_img_msg = future.result().raster_view
        raster_view_np = numpify(result_img_msg)
        
        intensity_channel = raster_view_np[:, :, 0]
        # Normalize for display purposes
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
c. Run the test client
IMPORTANT: Open test_client.py and change /home/user/data.pcd to the actual path of your PCD file.
Open a new terminal (the service node should still be running in the first one), source your workspace, and run the script:
code
Bash
source ~/V2I-GridCalib_ws/install/setup.bash
python3 test_client.py
You will see logs in both terminals. If successful, a window will pop up showing the rasterized intensity view of your point cloud.
Service Details
Service: /v2i_gridcalib/rasterize_pointcloud
Type: v2i_gridcalib_interfaces/srv/RasterizePointCloud
Request:
sensor_msgs/msg/PointCloud2 pointcloud_sector: The input point cloud sector. Must contain x, y, z, and intensity fields.
float32 approximate_distance: The approximate distance to the vehicle in meters. This determines which virtual camera intrinsics are used for rendering.
Response:
sensor_msgs/msg/Image raster_view: The resulting 4-channel, 32-bit float image. The encoding is 32FC4. The channels are ordered as (Intensity, X, Y, Z).
