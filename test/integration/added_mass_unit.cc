// Plugin that sets a prescribed state and applies a prescribed wrench during
// PreUpdate and checks accelerations against expected values on PostUpdate.
class ForceAccelerationPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  public: void ForceAccelerationPlugin(const int _max_iter);

  public: void Configure(
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

  private: int *iter = {0};

  private: int *max_iter = {0};

  private: gz::sim::Link *link{nullptr};
};

// Store how many values are to be checked.
void ForceAccelerationPlugin::ForceAccelerationPlugin(const int _max_iter) {
  ASSERT_GT(_max_iter, 0);
  // TODO: Check that _max_iter matches the size of the array/vector of values
  // to check.
  *(this->max_iter) = _max_iter;
};

// Store link and enable acceleration checks.
void ForceAccelerationPlugin::Configure(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm
)
{
  gz::sim::Entity model_entity = _ecm.EntityByComponents(
    gz::sim::components::Name(kModelName),
    gz::sim::components::Model()
  );
  ASSERT_NE(model_entity, gz::sim::kNullEntity);
  gz::sim::Model model = gz::sim::Model(model_entity);
  ASSERT_TRUE(model.Valid(_ecm));

  gz::sim::Entity link_entity = model.LinkByName(_ecm, kLinkName);
  ASSERT_NE(link_entity, gz::sim::kNullEntity);
  gz::sim::Link link = gz::sim::Link(link_entity);
  ASSERT_TRUE(link.Valid(_ecm));
  link.EnableAccelerationChecks(_ecm);
  this->link = &link;
};

// Set the link state and external inputs to known values.
void ForceAccelerationPlugin::PreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm
)
{
  // TODO: The intent is to set the link's pose, velocities, forces, and
  // torques, but that does not seem possible at the moment:
  //
  //  - The Link class does not seem to implement a method to set the pose.
  //
  //  - The Link class implements methods to set velocities and wrench, but
  //    these are incompatible with each other. In particular, documentation
  //    for Link::SetLinearVelocity and Link::SetAngularVelocity says: _If this
  //    is set, wrenches on this link will be ignored for the current time
  //    step_.
  //
  // Ideally (ignoring bound checks), we would have some code like:
  //
  // this->link.SetWorldPose(kPoseVector[this->iter]);
  // this->link.SetWorldLinearVelocity(kLinVelVector[this->iter]);
  // this->link.SetWorldAngularVelocity(kAngVelVector[this->iter]);
  // this->link.SetWorldWrench(kWrenchVector[this->iter]);
};

// Check the link acceleration and increase the iteration counter.
void ForceAccelerationPlugin::PostUpdate(
  const gz::sim::UpdateInfo &,
  const gz::sim::EntityComponentManager &_ecm
)
{
  std::optional<gz::math::Vector3d> maybe_lin_acc =
    this->link.WorldLinearAcceleration(_ecm);
  EXPECT_TRUE(maybe_lin_acc);
  if (maybe_lin_acc) {
    gz::math::Vector3d lin_acc = maybe_lin_acc.value();
    EXPECT_LE((lin_acc - kExpectedLinAccVector[this->iter]).Length(), 1e-6);
  };

  std::optional<gz::math::Vector3d> maybe_ang_acc =
    this->link.WorldAngularAcceleration(_ecm);
  EXPECT_TRUE(maybe_ang_acc);
  if (maybe_ang_acc) {
    gz::math::Vector3d ang_acc = maybe_ang_acc.value();
    EXPECT_LE((ang_acc - kExpectedAngAccVector[this->iter]).Length(), 1e-6);
  };

  EXPECT_LE(this->iter, this->max_iter);
  (this->iter)++;
};

class EmptyTestFixture: public InternalFixture<::testing::Test> {};

// Add system and run server.
TEST_F(EmptyTestFixture, AccelerationTest) {
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(
    common::joinPaths(PROJECT_SOURCE_PATH, kWorldFilePath)
  );
  serverConfig.SetUpdateRate(kRate);
  gz::sim::Server server = gz::sim::Server(serverConfig);
  auto system = std::make_shared<ForceAccelerationPlugin>(kMaxIter);
  server.AddSystem(system);
  ASSERT_FALSE(server.Running());
  ASSERT_TRUE(server.Run(true, kMaxIter, false));
};
