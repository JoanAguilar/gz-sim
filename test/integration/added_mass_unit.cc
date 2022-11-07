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

const char *kWorldFilePath{"/test/worlds/added_mass_full_matrix.sdf"};
const char *kWorldName{"added_mass"};
const char *kModelName{"body"};
const char *kLinkName{"link"};
const double kRate{1000};

// Struct to define test inputs.
struct TestInputs {
  gz::math::Vector3d pos;
  gz::math::Quaternion<double> quat;
  gz::math::Vector3d lin_vel;
  gz::math::Vector3d ang_vel;
  gz::math::Vector3d force;
  gz::math::Vector3d torque;
};

// Struct to define test outputs.
struct TestOutputs {
  gz::math::Vector3d lin_acc;
  gz::math::Vector3d ang_acc;
};

// Struct to define test cases (inputs and outputs).
struct TestCase {
  TestInputs inputs;
  TestOutputs outputs;
};

// Test cases.
// Test case #1
const TestCase kTestCase1{
  // inputs
  TestInputs{
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
    )
  },
  // outputs
  TestOutputs{
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
  }
};

// Vector of test cases.
const std::vector<TestCase> kTestCases{kTestCase1};

const std::size_t kNCases{kTestCases.size()};


// Set the body's pose, velocity, and wrench, in PreUpdate and Reset; check
// reported accelerations in PostUpdate.
class AccelerationCheckPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemReset,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  public: AccelerationCheckPlugin() = default;

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

  // Whether to perform a check with the expected output values in PostUpdate.
  public: bool performChecks{false};

  // Determines which test case to use when setting up input values in
  // Reset and PreUpdate, and output values in PostUpdate.
  public: int caseNumber{0};

  // Used to ensure that all the tests have run.
  public: int testCounter{0};

  private: void SetLink(gz::sim::EntityComponentManager &_ecm);

  private: gz::sim::Entity link_entity{gz::sim::kNullEntity};
};

// Used by Configure and Reset to initialize the link ECM state.
// Stores the value of the link entity, adds pose, velocity, and acceleration
// components.
void AccelerationCheckPlugin::SetLink(gz::sim::EntityComponentManager &_ecm) {
  gzdbg << "Setting link." << std::endl;
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
  ASSERT_NE(
    _ecm.CreateComponent(this->link_entity, gz::sim::components::WorldPose()),
    nullptr
  );
  ASSERT_NE(
    _ecm.CreateComponent(
      this->link_entity,
      gz::sim::components::WorldLinearVelocity()
    ),
    nullptr
  );
  ASSERT_NE(
    _ecm.CreateComponent(
      this->link_entity,
      gz::sim::components::WorldAngularVelocity()
    ),
    nullptr
  );
  ASSERT_NE(
    _ecm.CreateComponent(
      this->link_entity,
      gz::sim::components::WorldLinearAcceleration()
    ),
    nullptr
  );
  ASSERT_NE(
    _ecm.CreateComponent(
      this->link_entity,
      gz::sim::components::WorldAngularAcceleration()
    ),
    nullptr
  );
  gz::sim::Link link = gz::sim::Link(this->link_entity);
  ASSERT_TRUE(link.Valid(_ecm));
};

void AccelerationCheckPlugin::Configure(
  const gz::sim::Entity &,
  const std::shared_ptr<const sdf::Element> &,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &
)
{
  gzdbg << "Configure happening." << std::endl;
  this->SetLink(_ecm);
};

// Sets link ECM state, pose, and velocity.
void AccelerationCheckPlugin::Reset(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm
)
{
  gzdbg << "Reset happening." << std::endl;

  this->SetLink(_ecm);

  TestInputs inputs = kTestCases[this->caseNumber].inputs;
  gz::math::Vector3d pos = inputs.pos;
  gz::math::Quaternion<double> quat = inputs.quat;
  gz::math::Pose3d world_pose = gz::math::Pose3d(pos, quat);

  gzdbg << "Setting link world position to: " << pos << std::endl;
  gzdbg << "Setting link world orientation to: " << quat << std::endl;
  _ecm.SetComponentData<gz::sim::components::WorldPose>(
    this->link_entity,
    world_pose
  );
  gz::math::Vector3d lin_vel = inputs.lin_vel;
  gz::math::Vector3d ang_vel = inputs.ang_vel;
  gzdbg << "Setting link world linear velocity to: " << lin_vel << std::endl;
  _ecm.SetComponentData<gz::sim::components::WorldLinearVelocity>(
    this->link_entity,
    lin_vel
  );
  gzdbg << "Setting link world angular velocity to: " << ang_vel << std::endl;
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
  gzdbg << "PreUpdate happening." << std::endl;
  // TODO: Remove. SetLink has already been called at either Configure or
  // Reset.
  this->SetLink(_ecm);

  ASSERT_NE(this->link_entity, gz::sim::kNullEntity);
  gz::sim::Link link = gz::sim::Link(this->link_entity);
  ASSERT_TRUE(link.Valid(_ecm));
  TestInputs inputs = kTestCases[this->caseNumber].inputs;

  // TODO: Remove. Either this block here or the exact same block in Reset.
  gz::math::Vector3d pos = inputs.pos;
  gz::math::Quaternion<double> quat = inputs.quat;
  gz::math::Pose3d world_pose = gz::math::Pose3d(pos, quat);
  gzdbg << "Setting link world position to: " << pos << std::endl;
  gzdbg << "Setting link world orientation to: " << quat << std::endl;
  _ecm.SetComponentData<gz::sim::components::WorldPose>(
    this->link_entity,
    world_pose
  );
  gz::math::Vector3d lin_vel = inputs.lin_vel;
  gz::math::Vector3d ang_vel = inputs.ang_vel;
  gzdbg << "Setting link world linear velocity to: " << lin_vel << std::endl;
  _ecm.SetComponentData<gz::sim::components::WorldLinearVelocity>(
    this->link_entity,
    lin_vel
  );
  gzdbg << "Setting link world angular velocity to: " << ang_vel << std::endl;
  _ecm.SetComponentData<gz::sim::components::WorldAngularVelocity>(
    this->link_entity,
    ang_vel
  );
  // END TODO

  // TODO: Remove.
  if (this->link_entity != gz::sim::kNullEntity && (gz::sim::Link(this->link_entity)).Valid(_ecm)) {
    gz::sim::Link link = gz::sim::Link(this->link_entity);

    std::optional<gz::math::Pose3d> maybe_pose = link.WorldPose(_ecm);
    if (maybe_pose) {
      gz::math::Pose3d pose = maybe_pose.value();
      const gz::math::Vector3d pos = pose.Pos();
      std::cout << "Position:\t" << pos << std::endl;
      const gz::math::Quaternion<double> quat = pose.Rot();
      std::cout << "Rotation:\t" << quat << std::endl;
    }
    else
    {
      std::cout << "Unable to retrieve link pose." << std::endl;
    }

    std::optional<gz::math::Vector3d> maybe_lin_vel = link.WorldLinearVelocity(_ecm);
    if (maybe_lin_vel) {
      gz::math::Vector3d lin_vel = maybe_lin_vel.value();
      std::cout << "Linear velocity:\t" << lin_vel << std::endl;
    }
    else
    {
      std::cout << "Unable to retrieve linear velocity." << std::endl;
    }

    std::optional<gz::math::Vector3d> maybe_ang_vel = link.WorldAngularVelocity(_ecm);
    if (maybe_ang_vel) {
      gz::math::Vector3d ang_vel = maybe_ang_vel.value();
      std::cout << "Angular velocity:\t" << ang_vel << std::endl;
    }
    else
    {
      std::cout << "Unable to retrieve angular velocity." << std::endl;
    }
  }
  else
  {
    std::cout << "Link is not available." << std::endl;
  }
  // END TODO

  gz::math::Vector3d force = inputs.force;
  gz::math::Vector3d torque = inputs.torque;
  gzdbg << "Setting link world force to: " << force << std::endl;
  gzdbg << "Setting link world torque to: " << torque << std::endl;
  link.AddWorldWrench(_ecm, force, torque);
};

// Check linear and angular acceleration.
void AccelerationCheckPlugin::PostUpdate(
  const gz::sim::UpdateInfo &,
  const gz::sim::EntityComponentManager &_ecm
)
{
  gzdbg << "PostUpdate happening." << std::endl;

  // TODO: Remove.
  if (this->link_entity != gz::sim::kNullEntity && (gz::sim::Link(this->link_entity)).Valid(_ecm)) {
    gz::sim::Link link = gz::sim::Link(this->link_entity);

    std::optional<gz::math::Pose3d> maybe_pose = link.WorldPose(_ecm);
    if (maybe_pose) {
      gz::math::Pose3d pose = maybe_pose.value();
      const gz::math::Vector3d pos = pose.Pos();
      std::cout << "Position:\t" << pos << std::endl;
      const gz::math::Quaternion<double> quat = pose.Rot();
      std::cout << "Rotation:\t" << quat << std::endl;
    }
    else
    {
      std::cout << "Unable to retrieve link pose." << std::endl;
    }

    std::optional<gz::math::Vector3d> maybe_lin_vel = link.WorldLinearVelocity(_ecm);
    if (maybe_lin_vel) {
      gz::math::Vector3d lin_vel = maybe_lin_vel.value();
      std::cout << "Linear velocity:\t" << lin_vel << std::endl;
    }
    else
    {
      std::cout << "Unable to retrieve linear velocity." << std::endl;
    }

    std::optional<gz::math::Vector3d> maybe_ang_vel = link.WorldAngularVelocity(_ecm);
    if (maybe_ang_vel) {
      gz::math::Vector3d ang_vel = maybe_ang_vel.value();
      std::cout << "Angular velocity:\t" << ang_vel << std::endl;
    }
    else
    {
      std::cout << "Unable to retrieve angular velocity." << std::endl;
    }
  }
  else
  {
    std::cout << "Link is not available." << std::endl;
  }
  // END TODO

  if (this->performChecks) {

    gzdbg << "Performing checks." << std::endl;
    this->testCounter += 1;
    gzdbg << "Check number: " << this->testCounter << std::endl;

    ASSERT_NE(this->link_entity, gz::sim::kNullEntity);
    gz::sim::Link link = gz::sim::Link(this->link_entity);
    ASSERT_TRUE(link.Valid(_ecm));
    TestOutputs outputs = kTestCases[this->caseNumber].outputs;

    std::optional<gz::math::Vector3d> maybe_lin_acc =
      link.WorldLinearAcceleration(_ecm);
    EXPECT_TRUE(maybe_lin_acc);
    if (maybe_lin_acc) {
      gz::math::Vector3d lin_acc = maybe_lin_acc.value();
      // TODO: Remove printouts.
      std::cout << "Expected linear acceleration:\t" << outputs.lin_acc << std::endl;
      std::cout << "Actual linear acceleration:  \t" << lin_acc << std::endl;
      EXPECT_LT((lin_acc - outputs.lin_acc).Length(), 1e-6);
    }

    std::optional<gz::math::Vector3d> maybe_ang_acc =
      link.WorldAngularAcceleration(_ecm);
    EXPECT_TRUE(maybe_ang_acc);
    if (maybe_ang_acc) {
      gz::math::Vector3d ang_acc = maybe_ang_acc.value();
      EXPECT_LT((ang_acc - outputs.ang_acc).Length(), 1e-6);
    }
  }
};

// Request a world reset via transport.
void requestWorldReset()
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
  std::shared_ptr<AccelerationCheckPlugin> accelerationChecker =
    std::make_shared<AccelerationCheckPlugin>();
  std::optional<bool> maybe_system_added =
    server.AddSystem(accelerationChecker);
  ASSERT_TRUE(maybe_system_added);
  if (maybe_system_added) {
    ASSERT_TRUE(maybe_system_added.value());
  }
  ASSERT_FALSE(server.Running());

  // Run one iteration in order to "initialize" physics.
  ASSERT_TRUE(server.RunOnce(false));

  for (int i = 0; i < kNCases; i++) {
    accelerationChecker->performChecks = false;
    accelerationChecker->caseNumber = i;
    requestWorldReset();
    // It takes two iterations for a reset to happen. Only perform checks once
    // it has happened.
    ASSERT_TRUE(server.RunOnce(false));  // Reset requested.
    ASSERT_TRUE(server.RunOnce(false));  // Reset happening.
    accelerationChecker->performChecks = true;
    ASSERT_TRUE(server.RunOnce(false));  // Checks happening.
  }

  // Ensure that the right number of checks have been performed.
  EXPECT_EQ(accelerationChecker->testCounter, kNCases);
}
