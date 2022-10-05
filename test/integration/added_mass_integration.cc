#include <math.h>

#include <gtest/gtest.h>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>

#include "../helpers/EnvTestFixture.hh"

// World file path.
const std::string kWorldFilePath = "/test/worlds/added_mass.sdf";

// Name of the model.
const char* kModelName = "body";

// Name of the link.
const char* kLinkName = "link";

// Update rate of the server.
const double kRate = 1000;

// Force excitation amplitude and direction [N].
const double kForceVec[3] = {2000, 2000, 0};

// Force excitation angular velocity [rad / s].
const double kForceAngVel = 3 * M_PI;

// Torque excitation amplitude and direction [Nm].
const double kTorqueVec[3] = {200, 200, 0};

// Torque excitation angular velocity [rad / s].
const double kTorqueAngVel = 2 * M_PI;

// Total duration of the motion in iterations.
const uint64_t kIter= 1000;

// Expected position.
const gz::math::Vector3d kExpectedPosition = gz::math::Vector3d(
  0.16147154876755235,
  0.10708045802276464,
  -0.002493013844239572
);

// Expected rotation angle.
const double kExpectedAngle = 0.31351006941205767;

// Expected rotation axis.
const gz::math::Vector3d kExpectedAxis = gz::math::Vector3d(
  0.9487144000843207,
  0.29674923500860395,
  -0.10899944309240961
);

// Expected linear velocity.
const gz::math::Vector3d kExpectedLinVel = gz::math::Vector3d(
  0.32087533192979123,
  0.21277308875470216,
  -0.010283686950627922
);

// Expected angular velocity.
const gz::math::Vector3d kExpectedAngVel = gz::math::Vector3d(
  0.003151465100706373,
  0.0022439434320139203,
  -0.0525504921830876
);

// Plugin that applies a sinusoidal wrench.
class SinusoidalWrenchPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
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

  public: gz::sim::EntityComponentManager *ecm{nullptr};

  public: gz::sim::Entity link_entity;
};

// Store a reference to the ECM and to the link entity.
//
// The reference to the ECM is stored so it can be retrieved after the server
// has run.
void SinusoidalWrenchPlugin::Configure(
  const gz::sim::Entity &,
  const std::shared_ptr<const sdf::Element> &,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &
)
{
  this->ecm = &_ecm;

  gz::sim::Entity model_entity = _ecm.EntityByComponents(
    gz::sim::components::Name(kModelName),
    gz::sim::components::Model()
  );
  ASSERT_NE(model_entity, gz::sim::kNullEntity);

  gz::sim::Model model = gz::sim::Model(model_entity);
  ASSERT_TRUE(model.Valid(_ecm));

  this->link_entity = model.LinkByName(_ecm, kLinkName);
  ASSERT_NE(this->link_entity, gz::sim::kNullEntity);
  _ecm.CreateComponent(this->link_entity, gz::sim::components::WorldPose());

  gz::sim::Link link = gz::sim::Link(link_entity);
  ASSERT_TRUE(link.Valid(_ecm));
  link.EnableVelocityChecks(_ecm);
};

// Apply sinusoidal wrench before integration.
void SinusoidalWrenchPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm
)
{
  gz::sim::Link link = gz::sim::Link(this->link_entity);
  ASSERT_TRUE(link.Valid(_ecm));

  // Get time in seconds.
  double t = std::chrono::duration<double>(_info.simTime).count();

  gz::math::Vector3d force(
    kForceVec[0] * std::sin(kForceAngVel * t),
    kForceVec[1] * std::sin(kForceAngVel * t),
    kForceVec[2] * std::sin(kForceAngVel * t)
  );
  gz::math::Vector3d torque(
    kTorqueVec[0] * std::sin(kTorqueAngVel * t),
    kTorqueVec[1] * std::sin(kTorqueAngVel * t),
    kTorqueVec[2] * std::sin(kTorqueAngVel * t)
  );

  link.AddWorldWrench(_ecm, force, torque);
};

class EmptyTestFixture: public InternalFixture<::testing::Test> {};

// Check that the link state at the end of the motion matches the expected
// state.
TEST_F(EmptyTestFixture, MotionTest) {

  // Start server and run.
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(
    common::joinPaths(PROJECT_SOURCE_PATH, kWorldFilePath)
  );
  serverConfig.SetUpdateRate(kRate);
  gz::sim::Server server = gz::sim::Server(serverConfig);
  auto system = std::make_shared<SinusoidalWrenchPlugin>();
  server.AddSystem(system);
  ASSERT_FALSE(server.Running());
  ASSERT_TRUE(server.Run(true, kIter, false));

  gz::sim::EntityComponentManager *ecm = system->ecm;
  gz::sim::Link link = gz::sim::Link(system->link_entity);
  ASSERT_TRUE(link.Valid(*ecm));

  const std::optional<gz::math::Pose3d> maybe_pose = link.WorldPose(*ecm);
  EXPECT_TRUE(maybe_pose);
  if (maybe_pose) {
    const gz::math::Pose3d pose = maybe_pose.value();
    const std::optional<gz::math::Vector3d> maybe_pos = pose.Pos();
    EXPECT_TRUE(maybe_pos);
    if (maybe_pos) {
      gz::math::Vector3d pos = maybe_pos.value();
      // TODO: Remove printouts.
      std::cout << "Gazebo pos:\t" << pos << std::endl;
      std::cout << "Expected pos:\t" << kExpectedPosition << std::endl;
      EXPECT_LE((pos - kExpectedPosition).Length(), 1e-6);
    };

    const gz::math::Quaternion quat = pose.Rot();
    gz::math::Vector3d axis;
    double angle = 0;
    quat.AxisAngle(axis, angle);
    // TODO: Remove printouts.
    std::cout << "Gazebo axis:\t" << axis << std::endl;
    std::cout << "Expected axis:\t" << kExpectedAxis << std::endl;
    std::cout << "Gazebo angle:\t" << angle << std::endl;
    std::cout << "Expected angle:\t" << kExpectedAngle << std::endl;
    EXPECT_LE((axis - kExpectedAxis).Length(), 1e-6);
    EXPECT_LE(std::abs(angle - kExpectedAngle), 1e-6);
  };

  // Check velocities.
  std::optional<gz::math::Vector3d> maybe_lin_vel =
    link.WorldLinearVelocity(*ecm);
  EXPECT_TRUE(maybe_lin_vel);
  if (maybe_lin_vel) {
    gz::math::Vector3d lin_vel = maybe_lin_vel.value();
    // TODO: Remove printouts.
    std::cout << "Gazebo linear velocity:\t\t" << lin_vel << std::endl;
    std::cout << "Expected linear velocity:\t" << kExpectedLinVel << std::endl;
    EXPECT_LE((lin_vel - kExpectedLinVel).Length(), 1e-6);
  };

  std::optional<gz::math::Vector3d> maybe_ang_vel =
    link.WorldAngularVelocity(*ecm);
  EXPECT_TRUE(maybe_ang_vel);
  if (maybe_ang_vel) {
    gz::math::Vector3d ang_vel = maybe_ang_vel.value();
    // TODO: Remove printouts.
    std::cout << "Gazebo angular velocity:\t" << ang_vel << std::endl;
    std::cout << "Expected angular velocity:\t" << kExpectedAngVel << std::endl;
    EXPECT_LE((ang_vel - kExpectedAngVel).Length(), 1e-6);
  };
}
