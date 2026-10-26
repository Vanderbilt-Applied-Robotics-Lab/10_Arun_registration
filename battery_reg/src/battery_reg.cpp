#include <battery_reg/battery_reg.h>

BatteryReg::BatteryReg() : Node("battery_reg")
{
    // setup robot points
    p_r_[0] << 0, -0.0178142,  0.0943163;
    p_r_[1] << -0.0120208,  -0.0059760,  0.0964037;
    p_r_[2] << -0.0305470, -0.0242207, 0.0931867;
    p_r_[3] << -0.0305470, -0.0164066, 0.0488704;
    p_r_[4] << -0.0120208,  0.0018382, 0.0520874;
    
    // setup workpiece points
    p_w_[0] << 0, 0, 0.045;
    p_w_[1] << 0, 0.017, 0.045;
    p_w_[2] << -0.0262, 0.017, 0.045;
    p_w_[3] << -0.0262, 0.017, 0;
    p_w_[4] << 0, 0.017, 0;

    // setup new point
    p_r_new_ << -0.0185262, -0.0282447, 0.0467830;
    p_w_new_ << -0.0262, 0, 0;

    // setup marker publisher
    battery_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/workpiece", 10);
    tf_broadcaster_ =  std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
}

void BatteryReg::registerPoints()
{
    Eigen::Vector3d w_centroid = computeCentroid(p_w_);
    Eigen::Vector3d r_centroid = computeCentroid(p_r_);

    for (int i = 0; i< N_; ++i)
    {
        q_w_[i] = p_w_[i] -  w_centroid;
        q_r_[i] = p_r_[i] -  r_centroid;
    }

    createH();

    // compute the SVD of H
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d U_transpose = svd.matrixU().transpose();
    
    R_rw_ = V*U_transpose;

    // compute determinant of R
    double det_R = R_rw_.determinant();

    if (std::abs(det_R- 1) < 0.001) // det(R) = +1
    {
        // position of {w} wrt {r}
        p_rw_ = r_centroid - R_rw_*w_centroid;

        printResults(); 
        publishBatteryTF(R_rw_, p_rw_);
    }
    else if (std::abs(det_R + 1) < 0.001) // det(R) = -1
    {
        // Invert last column of V
        V.col(2) = -V.col(2);

        // Reconstruct R
        R_rw_ = V*U_transpose;

        // position of {w} wrt {r}
        p_rw_ = r_centroid - R_rw_*w_centroid;

        RCLCPP_WARN(this->get_logger(), "Reflection Detected!");

        printResults(); 
        publishBatteryTF(R_rw_, p_rw_);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Determinate is incorrect: %0.4f", det_R);
    }
}

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


Eigen::Vector3d BatteryReg::computeCentroid(std::array<Eigen::Vector3d,BatteryReg::N_> & points)
{
    // int N = points.length();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int i = 0; i< N_; ++i)
    {
        centroid = centroid + points[i];
    }
    return centroid/N_;
}

void BatteryReg::createH()
{
    // int N = points.length();
    for (int i = 0; i< N_; ++i)
    {
        H_ = H_ + q_w_[i]*(q_r_[i].transpose());
    }
}

double BatteryReg::computeFRE()
{
    // int N = points.length();
    double FRE = 0;
    for (int i = 0; i< N_; ++i)
    {
        FRE = FRE + std::pow((p_r_[i] - (R_rw_*p_w_[i] + p_rw_)).norm(), 2);
    }
    return std::sqrt(FRE/N_);
}

double BatteryReg::computeTRE()
{
    // int N = points.length();
    return (p_r_new_ - (R_rw_*p_w_new_ + p_rw_)).norm();
}

void BatteryReg::printResults()
{
    std::ostringstream oss_r;
    oss_r << R_rw_;

    std::ostringstream oss_p;
    oss_p << p_rw_;

    RCLCPP_INFO(this->get_logger(), "R_rw:\n%s", oss_r.str().c_str());
    RCLCPP_INFO(this->get_logger(), "p_rw:\n%s", oss_p.str().c_str());
    RCLCPP_INFO(this->get_logger(), "FRE: %0.4f, TRE: %0.4f", computeFRE(), computeTRE());
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

    node->registerPoints();
    
    rclcpp::spin_some(node); // updates publishers and subscribers
    rclcpp::shutdown();
    return 0;
}