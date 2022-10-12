#include <gtest/gtest.h>

#include <gz/transport/Node.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>

#include "../helpers/EnvTestFixture.hh"

const char *kModelName{"body1"};
const char *kLinkName{"link"};
const char *kWorldFilePath{"/test/worlds/added_mass.sdf"};
const double kRate{1000};

// Set the body's pose, velocity, and wrench, in the preupdate and reset; check
// reported accelerations in the postupdate.
class AccelerationCheckPlugin:
  public gz::sim::System,
  public gz::sim::ISystemReset,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  public: void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr
  ) override;

  public: void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm
  ) override;

  public: void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm
  ) override;

  public: void Reset(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm
  ) override;

  private: gz::sim::Entity link_entity{gz::sim::kNullEntity};
};

// Store the value of the link entity, add pose and velocity components, and
// enable acceleration checks.
void AccelerationCheckPlugin::Configure(
  const gz::sim::Entity &,
  const std::shared_ptr<const sdf::Element> &,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &
)
{
  if (this->link_entity == gz::sim::kNullEntity) {
    gz::sim::Entity model_entity = _ecm.EntityByComponents(
      gz::sim::components::Name(kModelName),
      gz::sim::components::Model()
    );
    ASSERT_NE(model_entity, gz::sim::kNullEntity);
    gz::sim::Model model = gz::sim::Model(model_entity);
    ASSERT_TRUE(model.Valid(_ecm));
    this->link_entity = model.LinkByName(_ecm, kLinkName);
    ASSERT_NE(this->link_entity, gz::sim::kNullEntity);
  }
  _ecm.CreateComponent(this->link_entity, gz::sim::components::WorldPose());
  _ecm.CreateComponent(
    this->link_entity,
    gz::sim::components::WorldLinearVelocity()
  );
  _ecm.CreateComponent(
    this->link_entity,
    gz::sim::components::WorldAngularVelocity()
  );
  gz::sim::Link link = gz::sim::Link(this->link_entity);
  ASSERT_TRUE(link.Valid(_ecm));
  link.EnableAccelerationChecks(_ecm);
};

// Set wrench (force and torque).
void AccelerationCheckPlugin::PreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm
)
{
  ASSERT_NE(this->link_entity, gz::sim::kNullEntity);
  gz::sim::Link link = gz::sim::Link(this->link_entity);
  ASSERT_TRUE(link.Valid(_ecm));
  // TODO: Initialize/set force and torque to the desired values.
  gz::math::Vector3d force;
  gz::math::Vector3d torque;
  link.AddWorldWrench(_ecm, force, torque);
};

// Check linear and angular acceleration.
void AccelerationCheckPlugin::PostUpdate(
  const gz::sim::UpdateInfo &,
  const gz::sim::EntityComponentManager &_ecm
)
{
  ASSERT_NE(this->link_entity, gz::sim::kNullEntity);
  gz::sim::Link link = gz::sim::Link(this->link_entity);
  ASSERT_TRUE(link.Valid(_ecm));
  std::optional<gz::math::Vector3d> maybe_lin_acc =
    link.WorldLinearAcceleration(_ecm);
  ASSERT_TRUE(maybe_lin_acc);
  if (maybe_lin_acc) {
    gz::math::Vector3d lin_acc = maybe_lin_acc.value();
    // TODO: Check acceleration against something meaningful.
    EXPECT_LT((lin_acc - lin_acc).Length(), 1e-6);
  }
  std::optional<gz::math::Vector3d> maybe_ang_acc =
    link.WorldAngularAcceleration(_ecm);
  ASSERT_TRUE(maybe_ang_acc);
  if (maybe_ang_acc) {
    gz::math::Vector3d ang_acc = maybe_ang_acc.value();
    // TODO: Check acceleration against something meaningful.
    EXPECT_LT((ang_acc - ang_acc).Length(), 1e-6);
  }
};

// Set pose and velocity.
void AccelerationCheckPlugin::Reset(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm
)
{
  // TODO: Initialize world_pose to the desired value.
  gz::math::Vector3d pos = gz::math::Vector3d();
  gz::math::Quaternion<double> quat = gz::math::Quaternion<double>();
  gz::math::Pose3d world_pose = gz::math::Pose3d(pos, quat);
  _ecm.SetComponentData<gz::sim::components::WorldPose>(
    this->link_entity,
    world_pose
  );
  // TODO: Initialize lin_vel and ang_vel to the desired value.
  gz::math::Vector3d lin_vel = gz::math::Vector3d();
  gz::math::Vector3d ang_vel = gz::math::Vector3d();
  _ecm.SetComponentData<gz::sim::components::WorldLinearVelocity>(
    this->link_entity,
    lin_vel
  );
  _ecm.SetComponentData<gz::sim::components::WorldAngularVelocity>(
    this->link_entity,
    ang_vel
  );
};

// Reset the simulation world via transport.
void worldReset()
{
  gz::msgs::WorldControl req;
  gz::msgs::Boolean rep;
  req.mutable_reset()->set_all(true);
  gz::transport::Node node;

  unsigned int timeout = 1000;
  bool result;
  bool executed =
    node.Request("/world/default/control", req, timeout, rep, result);

  // FIXME: This asserts should pass.
  ASSERT_TRUE(executed);
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());
  // std::cout << "executed:\t" << executed << std::endl;
  // std::cout << "result:\t\t" << result << std::endl;
  // std::cout << "rep.data:\t" << rep.data() << std::endl;
}

class EmptyTestFixture: public InternalFixture<::testing::Test> {};

// Reset the server and run a single iteration.
// TODO: Test different values.
TEST_F(EmptyTestFixture, AccelerationTest) {
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(
    common::joinPaths(PROJECT_SOURCE_PATH, kWorldFilePath)
  );
  serverConfig.SetUpdateRate(kRate);
  gz::sim::Server server = gz::sim::Server(serverConfig);
  std::shared_ptr<AccelerationCheckPlugin> system =
    std::make_shared<AccelerationCheckPlugin>();
  std::optional<bool> add_system_success = server.AddSystem(system);
  ASSERT_TRUE(add_system_success);
  if (add_system_success) {
    ASSERT_TRUE(add_system_success.value());
  }
  ASSERT_FALSE(server.Running());

  // TODO: Run the test multiple times (with meaningful values).
  worldReset();
  ASSERT_TRUE(server.RunOnce(false));
};
