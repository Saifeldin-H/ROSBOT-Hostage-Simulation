#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <sdf/Element.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/sim/Actor.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <ignition/plugin/Register.hh>

namespace hostage_sim
{
namespace sim = ignition::gazebo;

class HostageActorController
    : public sim::System,
      public sim::ISystemConfigure,
      public sim::ISystemPreUpdate
{
public:
  void Configure(
      const sim::Entity &,
      const std::shared_ptr<const sdf::Element> &_sdf,
      sim::EntityComponentManager &,
      sim::EventManager &) override
  {
    this->actorName = this->SdfString(_sdf, "actor_name", this->actorName);
    this->cmdVelTopic = this->SdfString(_sdf, "cmd_vel_topic", this->cmdVelTopic);
    this->poseTopic = this->SdfString(_sdf, "pose_topic", this->poseTopic);
    this->maxLinearSpeed = this->SdfDouble(_sdf, "max_linear_speed", this->maxLinearSpeed);
    this->maxAngularSpeed = this->SdfDouble(_sdf, "max_angular_speed", this->maxAngularSpeed);
    this->animationFactor = this->SdfDouble(_sdf, "animation_factor", this->animationFactor);
    this->commandTimeout = this->SdfDouble(_sdf, "command_timeout", this->commandTimeout);

    this->node.Subscribe(this->cmdVelTopic, &HostageActorController::OnCmdVel, this);
    this->posePub = this->node.Advertise<gz::msgs::Pose>(this->poseTopic);
  }

  void PreUpdate(const sim::UpdateInfo &_info, sim::EntityComponentManager &_ecm) override
  {
    if (!this->FindActor(_ecm)) {
      return;
    }

    if (_info.paused) {
      this->PublishPose(_ecm);
      return;
    }

    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0) {
      return;
    }

    const double simTime = std::chrono::duration<double>(_info.simTime).count();
    this->lastSimTime = simTime;
    double linear = 0.0;
    double angular = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->cmdMutex);
      const bool commandFresh = (simTime - this->lastCmdTime) <= this->commandTimeout;
      if (commandFresh) {
        linear = std::clamp(this->linearCmd, -this->maxLinearSpeed, this->maxLinearSpeed);
        angular = std::clamp(this->angularCmd, -this->maxAngularSpeed, this->maxAngularSpeed);
      }
    }

    auto currentTrajectory = this->actor.TrajectoryPose(_ecm);
    if (currentTrajectory.has_value()) {
      this->trajectoryPose = *currentTrajectory;
    }

    const double yaw = this->trajectoryPose.Rot().Yaw();
    const double nextYaw = yaw + angular * dt;
    const double distance = linear * dt;
    this->trajectoryPose.Pos().X() += distance * std::cos(nextYaw);
    this->trajectoryPose.Pos().Y() += distance * std::sin(nextYaw);
    this->trajectoryPose.Rot() = gz::math::Quaterniond(0.0, 0.0, nextYaw);
    this->actor.SetTrajectoryPose(_ecm, this->trajectoryPose);

    if (std::abs(linear) > 0.01 || std::abs(angular) > 0.01) {
      this->animationTime += dt * this->animationFactor;
      auto animationDuration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(this->animationTime));
      this->actor.SetAnimationTime(_ecm, animationDuration);
    }

    this->PublishPose(_ecm);
  }

private:
  static std::string SdfString(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      const std::string &_fallback)
  {
    return _sdf && _sdf->HasElement(_name) ? _sdf->Get<std::string>(_name) : _fallback;
  }

  static double SdfDouble(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      double _fallback)
  {
    return _sdf && _sdf->HasElement(_name) ? _sdf->Get<double>(_name) : _fallback;
  }

  void OnCmdVel(const gz::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    this->linearCmd = _msg.linear().x();
    this->angularCmd = _msg.angular().z();
    this->lastCmdTime = this->lastSimTime;
  }

  bool FindActor(sim::EntityComponentManager &_ecm)
  {
    if (this->actor.Valid(_ecm)) {
      return true;
    }

    sim::Entity found = sim::kNullEntity;
    _ecm.Each<sim::components::Actor, sim::components::Name>(
        [&](const sim::Entity &_entity,
            const sim::components::Actor *,
            const sim::components::Name *_name) -> bool
        {
          if (_name->Data() == this->actorName) {
            found = _entity;
            return false;
          }
          return true;
        });

    if (found == sim::kNullEntity) {
      return false;
    }

    this->actor.ResetEntity(found);
    this->trajectoryPose = this->actor.TrajectoryPose(_ecm).value_or(gz::math::Pose3d::Zero);
    return true;
  }

  void PublishPose(sim::EntityComponentManager &_ecm)
  {
    const auto worldPose = this->WorldPose(_ecm);
    if (!worldPose.has_value()) {
      return;
    }

    gz::msgs::Pose msg;
    msg.mutable_position()->set_x(worldPose->Pos().X());
    msg.mutable_position()->set_y(worldPose->Pos().Y());
    msg.mutable_position()->set_z(worldPose->Pos().Z());
    msg.mutable_orientation()->set_x(worldPose->Rot().X());
    msg.mutable_orientation()->set_y(worldPose->Rot().Y());
    msg.mutable_orientation()->set_z(worldPose->Rot().Z());
    msg.mutable_orientation()->set_w(worldPose->Rot().W());
    this->posePub.Publish(msg);
  }

  std::optional<gz::math::Pose3d> WorldPose(sim::EntityComponentManager &_ecm)
  {
    const auto originPose = this->actor.Pose(_ecm);
    if (!originPose.has_value()) {
      return this->actor.WorldPose(_ecm);
    }
    return *originPose * this->trajectoryPose;
  }

  sim::Actor actor;
  std::string actorName{"hostage"};
  std::string cmdVelTopic{"/hostage/cmd_vel"};
  std::string poseTopic{"/hostage/pose"};
  double maxLinearSpeed{0.8};
  double maxAngularSpeed{1.6};
  double animationFactor{1.0};
  double commandTimeout{0.5};
  double animationTime{0.0};
  double lastSimTime{0.0};
  gz::math::Pose3d trajectoryPose{gz::math::Pose3d::Zero};

  std::mutex cmdMutex;
  double linearCmd{0.0};
  double angularCmd{0.0};
  double lastCmdTime{-1.0e9};

  gz::transport::Node node;
  gz::transport::Node::Publisher posePub;
};
}  // namespace hostage_sim

IGNITION_ADD_PLUGIN(
    hostage_sim::HostageActorController,
    ignition::gazebo::System,
    hostage_sim::HostageActorController::ISystemConfigure,
    hostage_sim::HostageActorController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    hostage_sim::HostageActorController,
    "hostage_sim::HostageActorController")
