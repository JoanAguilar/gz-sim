#include <gtest/gtest.h>

#include <gz/transport/Node.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>

#include "../helpers/EnvTestFixture.hh"

const char * kWorldFilePath{"/test/worlds/added_mass_full_matrix.sdf"};
const char *kWorldName{"added_mass"};
const char *kModelName{"body"};
const char *kLinkName{"link"};
const double kRate{1000};

// Struct to define the test cases.
// Position, quaternion, linear velocity, angular velocity, force, and torque
// are prescribed; linear and angular acceleration are expected values.
struct TestCase {
  gz::math::Vector3d pos;
  gz::math::Quaternion<double> quat;
  gz::math::Vector3d lin_vel;
  gz::math::Vector3d ang_vel;
  gz::math::Vector3d force;
  gz::math::Vector3d torque;
  gz::math::Vector3d lin_acc;
  gz::math::Vector3d ang_acc;
};

// Test cases.
// Test case #1
const TestCase kTestCase1{
  // pos
  gz::math::Vector3d(1, 2, 3),
  // quat
  gz::math::Quaternion<double>(0.5, 0.5, 0.5, 0.5),
  // lin_vel
  gz::math::Vector3d(4, 5, 6),
  // ang_vel
  gz::math::Vector3d(7, 8, 9),
  // force
  gz::math::Vector3d(
    -0.2425045525045464,
    0.33840815652747214,
    -0.15400580887792237
  ),
  // torque
  gz::math::Vector3d(
    -0.3790807469203843,
    -0.09043362906571706,
    -0.8710546810958875
  ),
  // lin_acc
  gz::math::Vector3d(
    0.3874663983293944,
    1.7746303236798542,
    0.6515154938490635
  ),
  // ang_acc
  gz::math::Vector3d(
    -0.48991265864373107,
    1.5425403599907457,
    -3.3613871949733767
  )
};

// Vector of test cases.
const std::vector<TestCase> kTestCases{kTestCase1};

const int kNCases{kTestCases.size()};


// Set the body's pose, velocity, and wrench, in the preupdate and reset; check
// reported accelerations in the postupdate.
class AccelerationCheckPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemReset,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  public: AccelerationCheckPlugin();

  public: void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr
  ) override;

  public: void Reset(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm
  ) override;

  public: void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm
  ) override;

  public: void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm
  ) override;

  private: int counter{0};

  private: gz::sim::Entity link_entity{gz::sim::kNullEntity};
};

// Store the number of test cases and initialize counter.
AccelerationCheckPlugin::AccelerationCheckPlugin() {
  this->counter = 0;
}

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
  _ecm.CreateComponent(
    this->link_entity,
    gz::sim::components::WorldLinearAcceleration()
  );
  _ecm.CreateComponent(
    this->link_entity,
    gz::sim::components::WorldAngularAcceleration()
  );
  gz::sim::Link link = gz::sim::Link(this->link_entity);
  ASSERT_TRUE(link.Valid(_ecm));
};

// Set pose and velocity.
void AccelerationCheckPlugin::Reset(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm
)
{
  // FIXME: This function never runs!
  // TODO: Remove assert, it's only there to see if this code runs.
  assert(false);
  gz::math::Vector3d pos = kTestCases[this->counter].pos;
  gz::math::Quaternion<double> quat = kTestCases[this->counter].quat;
  gz::math::Pose3d world_pose = gz::math::Pose3d(pos, quat);
  _ecm.SetComponentData<gz::sim::components::WorldPose>(
    this->link_entity,
    world_pose
  );
  gz::math::Vector3d lin_vel = kTestCases[this->counter].lin_vel;
  gz::math::Vector3d ang_vel = kTestCases[this->counter].ang_vel;
  _ecm.SetComponentData<gz::sim::components::WorldLinearVelocity>(
    this->link_entity,
    lin_vel
  );
  _ecm.SetComponentData<gz::sim::components::WorldAngularVelocity>(
    this->link_entity,
    ang_vel
  );
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
  gz::math::Vector3d force = kTestCases[this->counter].force;
  gz::math::Vector3d torque = kTestCases[this->counter].torque;
  link.AddWorldWrench(_ecm, force, torque);
};

// Check linear and angular acceleration and increase the counter.
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
  EXPECT_TRUE(maybe_lin_acc);
  if (maybe_lin_acc) {
    gz::math::Vector3d lin_acc = maybe_lin_acc.value();
    // TODO: Remove printouts.
    std::cout << "Counter:\t" << this->counter << std::endl;
    std::cout << "Expected:\t" << kTestCases[this->counter].lin_acc << std::endl;
    std::cout << "Actual:\t\t" << lin_acc << std::endl;
    EXPECT_LT((lin_acc - kTestCases[this->counter].lin_acc).Length(), 1e-6);
  }
  std::optional<gz::math::Vector3d> maybe_ang_acc =
    link.WorldAngularAcceleration(_ecm);
  EXPECT_TRUE(maybe_ang_acc);
  if (maybe_ang_acc) {
    gz::math::Vector3d ang_acc = maybe_ang_acc.value();
    EXPECT_LT((ang_acc - kTestCases[this->counter].ang_acc).Length(), 1e-6);
  }

  this->counter += 1;
};

// Reset the simulation world via transport.
void worldReset()
{
  std::string topic{"/world/" + std::string(kWorldName) + "/control"};
  gz::msgs::WorldControl req;
  gz::msgs::Boolean rep;
  req.mutable_reset()->set_all(true);
  gz::transport::Node node;

  gzdbg << "Requesting world reset." << std::endl;
  unsigned int timeout = 1000;
  bool result;
  bool executed =
    node.Request(topic, req, timeout, rep, result);
  ASSERT_TRUE(executed);
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());
}

class EmptyTestFixture: public InternalFixture<::testing::Test> {};

// Reset the server and run a single iteration.
TEST_F(EmptyTestFixture, AccelerationTest) {
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(
    common::joinPaths(PROJECT_SOURCE_PATH, kWorldFilePath)
  );
  // TODO: Is it really needed to set the rate?
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

  for (int i = 0; i < kNCases; i++) {
    worldReset();
    ASSERT_TRUE(server.RunOnce(false));
  }
}
