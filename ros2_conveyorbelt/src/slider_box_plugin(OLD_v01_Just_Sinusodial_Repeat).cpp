#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

namespace gazebo
{
  class SliderBoxPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      model_ = model;
      link_ = model_->GetLink("link");

      node_ = gazebo_ros::Node::Get(sdf);
      RCLCPP_INFO(node_->get_logger(), "SliderBoxPlugin loaded.");

      update_connection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SliderBoxPlugin::OnUpdate, this));

      start_time_ = node_->get_clock()->now();
    }

    void OnUpdate()
    {
      double t = (node_->get_clock()->now() - start_time_).seconds();
      double offset = 0.2 * sin(2 * M_PI * 0.6 * t); // 0.1m amplitude, 진동수: 0.6 Hz

      ignition::math::Pose3d pose;
      pose.Pos().Set(-0.2 + offset, 1.0, 0.79); // Set final location
      pose.Rot().Set(1, 0, 0, 0);

      link_->SetWorldPose(pose);
    }

  private:
    physics::ModelPtr model_;
    physics::LinkPtr link_;
    gazebo_ros::Node::SharedPtr node_;
    rclcpp::Time start_time_;
    event::ConnectionPtr update_connection_;
  };

  GZ_REGISTER_MODEL_PLUGIN(SliderBoxPlugin)
}
