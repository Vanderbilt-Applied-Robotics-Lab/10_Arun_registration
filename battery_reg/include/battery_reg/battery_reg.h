#ifndef BATTERY_REG_H
#define BATTERY_REG_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/static_transform_broadcaster.h"

/**
 * Starter files for take-home assignment 10
 * Applied Robotics
 */
class BatteryReg : public rclcpp::Node
{
public:
    /**
     * Constructor.
     */
    BatteryReg();

    /**
     * Constructor.
     */
    ~BatteryReg() = default;

    // DECLARE YOUR PUBLIC METHODS HERE

    /**
     * Publish battery mesh
     */
    void publishBatteryMesh();

private:
    // DECLARE YOUR PRIVATE METHODS HERE

    /**
     * Publishes the battery transform
     * @param R The rotation matrix
     * @param p the translation vector
     */
    void publishBatteryTF(const Eigen::Matrix3d & R, const Eigen::Vector3d & p);

    // DECLARE YOUR MEMBER VARIABLES HERE

    // STUFF FOR PUBLISHING BATTERY 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr battery_pub_;
    visualization_msgs::msg::Marker battery_msg_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

#endif // BATTERY_REG_H