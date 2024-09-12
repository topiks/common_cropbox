#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_geometry/laser_geometry.h"
#include <jsk_recognition_msgs/BoundingBox.h>
#include <common_cropbox/CropboxConfig.h>

#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

// ------------------------------------------------------

int cropbox_routine_init();
void cllbck_tim_10_hz(const ros::TimerEvent& event);
void cllbck_laser_input(const sensor_msgs::LaserScan::ConstPtr& msg);
void update_bounding_cropbox(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
void cropbox_callback(common_cropbox::CropboxConfig &config, uint32_t level);

// ------------------------------------------------------

ros::Timer tim_10_hz;

ros::Subscriber sub_laser_input;

ros::Publisher pub_pcl_output;
ros::Publisher pub_cropbox_params;

dynamic_reconfigure::Server<common_cropbox::CropboxConfig> *cropbox_reconfigure_server;
dynamic_reconfigure::Server<common_cropbox::CropboxConfig>::CallbackType cropbox_reconfigure_callback_f;

std::string topic_laser_input;
std::string topic_pcl_output;
std::string topic_pcl_output_bounding_box;
std::string laser_frame_name;

double cropbox_value[6];

laser_geometry::LaserProjection projector;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
PointCloud::Ptr cloud_original(new PointCloud);
PointCloud::Ptr cloud_cropbox(new PointCloud);

bool already_init = false;

// ======================================================
// ------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "common_cropbox");

    ros::NodeHandle NH;
    ros::NodeHandle NH_private("~");
    ros::AsyncSpinner AS(1);

    // --------------

    if(!NH_private.getParam("topic_laser_input", topic_laser_input))
        topic_laser_input = "/laser_input_common_cropbox";
    if(!NH_private.getParam("topic_pcl_output", topic_pcl_output))
        topic_pcl_output = "/laser_output_common_cropbox";
    if(!NH_private.getParam("laser_frame_name", laser_frame_name))
        laser_frame_name = "base_link";

    topic_pcl_output_bounding_box = topic_pcl_output + "_bounding_box";
    
    NH_private.getParam("x_min", cropbox_value[0]);
    NH_private.getParam("x_max", cropbox_value[1]);
    NH_private.getParam("y_min", cropbox_value[2]);
    NH_private.getParam("y_max", cropbox_value[3]);
    NH_private.getParam("z_min", cropbox_value[4]);
    NH_private.getParam("z_max", cropbox_value[5]);
    
    // --------------

    sub_laser_input = NH.subscribe(topic_laser_input, 1, cllbck_laser_input);

    pub_pcl_output = NH.advertise<sensor_msgs::PointCloud2>(topic_pcl_output, 1);
    pub_cropbox_params = NH.advertise<jsk_recognition_msgs::BoundingBox>(topic_pcl_output_bounding_box, 1);

    tim_10_hz = NH.createTimer(ros::Duration(0.1), cllbck_tim_10_hz);

    // --------------

    if(cropbox_routine_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();

    return 0;
}

// ======================================================

void cllbck_laser_input(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*msg, cloud);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_original);

    pcl::CropBox<PointT> cropbox_filter;
    cropbox_filter.setInputCloud(cloud_original);
    cropbox_filter.setMin(Eigen::Vector4f(cropbox_value[0], cropbox_value[2], cropbox_value[4], 1.0));
    cropbox_filter.setMax(Eigen::Vector4f(cropbox_value[1], cropbox_value[3], cropbox_value[5], 1.0));
    cropbox_filter.filter(*cloud_cropbox);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_cropbox, output);
    output.header.frame_id = laser_frame_name;
    pub_pcl_output.publish(output);
}

// ------------------------------------------------------

void update_bounding_cropbox(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
    if(x_min < x_max && y_min < y_max && z_min < z_max)
    {
        jsk_recognition_msgs::BoundingBox bounding_box;
        bounding_box.header.frame_id = laser_frame_name;
        bounding_box.header.stamp = ros::Time::now();
        bounding_box.pose.position.x = (x_max + x_min) / 2;
        bounding_box.pose.position.y = (y_max + y_min) / 2;
        bounding_box.pose.position.z = (z_max + z_min) / 2;
        bounding_box.dimensions.x = x_max - x_min;
        bounding_box.dimensions.y = y_max - y_min;
        bounding_box.dimensions.z = z_max - z_min;

        pub_cropbox_params.publish(bounding_box);
    }
}

// ------------------------------------------------------

int cropbox_routine_init()
{   
    cropbox_reconfigure_server = new dynamic_reconfigure::Server<common_cropbox::CropboxConfig>(ros::NodeHandle(topic_pcl_output_bounding_box));
    cropbox_reconfigure_callback_f = boost::bind(&cropbox_callback, _1, _2);
    cropbox_reconfigure_server->setCallback(cropbox_reconfigure_callback_f);

    already_init = true;

    return 0;
}

// ------------------------------------------------------

void cropbox_callback(common_cropbox::CropboxConfig &config, uint32_t level)
{
    if(already_init)
    {
        cropbox_value[0] = config.x_min;
        cropbox_value[1] = config.x_max;
        cropbox_value[2] = config.y_min;
        cropbox_value[3] = config.y_max;
        cropbox_value[4] = config.z_min;
        cropbox_value[5] = config.z_max;
    }
}

// ------------------------------------------------------

void cllbck_tim_10_hz(const ros::TimerEvent& event)
{
    update_bounding_cropbox(cropbox_value[0], cropbox_value[1], cropbox_value[2], cropbox_value[3], cropbox_value[4], cropbox_value[5]);
}