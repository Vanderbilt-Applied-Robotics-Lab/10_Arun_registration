#ifndef BATTERY_REG_H
#define ARUN_REG_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/static_transform_broadcaster.h"

/**
 * Solution to take-home assignment 10
 * Applied Robotics
 * Author: Garrison Johnston
 */
class BatteryReg : public rclcpp::Node
{
    static constexpr uint8_t N_ = 5; // Number of points 
public:
    /**
     * Constructor.
     */
    BatteryReg();

    /**
     * Constructor.
     */
    ~BatteryReg() = default;

    /**
     * Computes the Arun registration
     */
    void registerPoints();

    /**
     * Publish battery mesh
     */
    void publishBatteryMesh();

private:
    /**
     * Computes the centroid of a list of points
     * @param points Array of points to compute centroid of
     * @return The centroid 
     */
    Eigen::Vector3d computeCentroid(std::array<Eigen::Vector3d,N_> & points);

    /**
     * Computes the H matrix from Arun registration
     */
    void createH();

    /**
     * Computes the Fiducial Registration Error (FRE)
     * @return The FRE 
     */
    double computeFRE();

    /**
     * Computes the Target Registration Error (TRE)
     * @return The TRE 
     */
    double computeTRE();

    /**
     * Print the rotation matrix, TRE, and FRE
     */
    void printResults();

    /**
     * Publishes the battery transform
     * @param R The rotation matrix
     * @param p the translation vector
     */
    void publishBatteryTF(const Eigen::Matrix3d & R, const Eigen::Vector3d & p);

    std::array<Eigen::Vector3d,N_> p_w_; // points in workpiece frame
    std::array<Eigen::Vector3d,N_> p_r_; // points in robot frame

    std::array<Eigen::Vector3d,N_> q_w_; // points in workpiece frame
    std::array<Eigen::Vector3d,N_> q_r_; // points in robot frame

    Eigen::Matrix3d H_ = Eigen::Matrix3d::Zero(); // H Matrix from Arun Reg
    Eigen::Matrix3d R_rw_ = Eigen::Matrix3d::Zero(); // the rotation matrix of {w} wrt {r}

    Eigen::Vector3d p_rw_; // The position of {w} wrt {r}
    Eigen::Vector3d p_r_new_; // New point for TRE in {r}
    Eigen::Vector3d p_w_new_; // New point for TRE in {w}

    // STUFF FOR PUBLISHING BATTERY 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr battery_pub_;
    visualization_msgs::msg::Marker battery_msg_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

#endif // EIGEN_EXAMPLE_H