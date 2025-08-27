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

      //여기 수정 _ 20250713
      if (!link_) {
        RCLCPP_ERROR(node_->get_logger(), "❌ link_ is NULL! Check your link name in model.sdf");
        return;
      }

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

      // 최초 1회: 위치를 공중에 고정
      if (!initialized_)
      {
        ignition::math::Pose3d pose;
        pose.Pos().Set(-0.266, 1.4, 0.82); // 초기 위치 (공중)
        pose.Rot().Set(1, 0, 0, 0);
        link_->SetWorldPose(pose);
        initialized_ = true;
      }

      // 슬라이더 박스 이동 패턴: 1초동안 +X, 1초동안 -X, 3초동안 대기 반복
      ignition::math::Vector3d vel(0, 0, 0);
      
      double start_delay = 43.7;  // start_delay 이후부터 이동 시작 // cup spawn은 20.2xx초부터 하기
      
      if (t >= start_delay) 
      {
        double period = 8.0;  // 한 주기의 길이 (0.5초 + 0.5초 + 7초)
        double time_in_cycle = fmod(t - start_delay, period);  // start_delay 후부터 반복 시작

        if (time_in_cycle < 0.5) {
          vel.X() = 0.7;  // +X로 0.5초 이동
        } 
        else if (time_in_cycle >= 0.5 && time_in_cycle < 1.0) {
          vel.X() = -0.7;  // -X로 0.5초 이동
        } 
        else if (time_in_cycle >= 1.0 && time_in_cycle < 8.0) {
          vel.X() = 0.0;  // 7초 대기
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
