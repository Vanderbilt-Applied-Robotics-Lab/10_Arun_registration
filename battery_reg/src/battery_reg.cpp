#include <battery_reg/battery_reg.h>

BatteryReg::BatteryReg() : Node("battery_reg")
{
    // ADD YOUR CODE HERE

    // setup marker publisher
    battery_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/workpiece", 10);
    tf_broadcaster_ =  std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
}

// IMPLEMENT THE REST OF THE CLASS HERE

void BatteryReg::publishBatteryMesh()
{
    // setup message
    battery_msg_.header.frame_id = "workpiece";
    battery_msg_.id = 0;
    battery_msg_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    battery_msg_.action = visualization_msgs::msg::Marker::ADD;

    // Mesh file
    battery_msg_.mesh_resource =
    "package://battery_reg/meshes/9V_battery.dae";
    battery_msg_.mesh_use_embedded_materials = true;

    // Identity pose relative to the workpiece frame
    battery_msg_.pose.orientation.w = 1.0;

    // Scale
    battery_msg_.scale.x = 1.0;
    battery_msg_.scale.y = 1.0;
    battery_msg_.scale.z = 1.0;

    // publish
    battery_msg_.header.stamp = this->get_clock()->now();
    battery_pub_->publish(battery_msg_);
}

void BatteryReg::publishBatteryTF(const Eigen::Matrix3d & R, const Eigen::Vector3d & p)
{
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "world";
    tf.child_frame_id = "workpiece";

    tf.transform.translation.x = p.x();
    tf.transform.translation.y = p.y();
    tf.transform.translation.z = p.z();

    Eigen::Quaterniond q(R);
    q.normalize();

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
}


int main(int argc, char * argv[])
{
    // initialize the node
    rclcpp::init(argc, argv);

    // create instance of class
    auto node = std::make_shared<BatteryReg>();

    node->publishBatteryMesh();
    rclcpp::spin_some(node); // updates publishers and subscribers

    // ADD YOUR CODE HERE

    rclcpp::shutdown();
    return 0;
}