#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

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
      RCLCPP_INFO(node_->get_logger(), "✅ SliderBoxPlugin loaded.");

      update_connection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SliderBoxPlugin::OnUpdate, this));

      start_time_ = node_->get_clock()->now();
      initialized_ = false;
    }

    void OnUpdate()
    {
      double t = (node_->get_clock()->now() - start_time_).seconds();

      // 🔒 최초 1회: 위치를 공중에 고정
      if (!initialized_)
      {
        ignition::math::Pose3d pose;
        pose.Pos().Set(-0.266, 1.0, 0.82); // 초기 위치 (공중)
        pose.Rot().Set(1, 0, 0, 0);
        link_->SetWorldPose(pose);
        initialized_ = true;
      }

      // ⏱️ 시간 기반 속도 설정
      ignition::math::Vector3d vel(0, 0, 0);

      double start_gazebo = 10.0; // gazebo에서 start_gazebo만큼의 시간이 지나면, cup_spawner 동작시켜서 딱 맞게 하자.
      
      // 20초 이후부터는 계속 +X, -X 이동 반복
      if (t >= start_gazebo + 20.0) 
      {
        // 짝수 초에서는 +X 방향으로 이동, 홀수 초에서는 -X 방향으로 이동
        if (static_cast<int>(t) % 2 == 0) 
        {
          vel.X() = 0.3;  // +X 방향으로 전진
        }
        else 
        {
          vel.X() = -0.3; // -X 방향으로 후진
        }
      }

      link_->SetLinearVel(vel); // 실제 속도 적용

      // 회전 속도를 0으로 설정하여 회전 방지
      ignition::math::Vector3d ang_vel(0, 0, 0); // 각속도 0으로 설정
      link_->SetAngularVel(ang_vel);      
    }

  private:
    physics::ModelPtr model_;
    physics::LinkPtr link_;
    gazebo_ros::Node::SharedPtr node_;
    rclcpp::Time start_time_;
    bool initialized_;
    event::ConnectionPtr update_connection_;
  };

  GZ_REGISTER_MODEL_PLUGIN(SliderBoxPlugin)
}
