# ROS CropBox Filter Node

## 📦 Overview

This ROS node provides a versatile way to filter incoming 2D LaserScan or 3D PointCloud2 data. It uses the Point Cloud Library (PCL) to apply a `CropBox` filter, retaining only the points that fall within a specified 3D bounding box.

The key feature of this node is its ability to be configured **dynamically** at runtime using `rqt_reconfigure`. Any adjustments made to the crop box dimensions are automatically saved to a YAML file, ensuring your settings persist across sessions.

This is particularly useful for:
-   Isolating a specific region of interest in a sensor's view.
-   Removing noise or irrelevant data from the environment (e.g., parts of the robot's own chassis).
-   Creating simple, rectangular virtual safety zones.

---

## ⚙️ How It Works

1.  **Input:** The node subscribes to either a `sensor_msgs/LaserScan` topic or a `sensor_msgs/PointCloud2` topic, based on the `type_input` parameter.
2.  **Conversion:** If the input is a LaserScan, it's first converted into a PointCloud2 message.
3.  **Filtering:** A PCL `CropBox` filter is applied to the PointCloud2 data using the configured `x`, `y`, and `z` min/max boundaries.
4.  **Output:**
    * The filtered PointCloud2 is published.
    * A boolean message is published indicating if the box contains a minimum number of points.
    * A visualization marker (`jsk_recognition_msgs/BoundingBox`) is published to show the crop box's position and size in tools like RViz.



---

## 🚀 Usage

The easiest way to run this node is by using the provided launch file.

```bash
roslaunch common_cropbox common_cropbox.launch
