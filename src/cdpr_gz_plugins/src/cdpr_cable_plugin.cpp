#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>

using namespace gz;
using namespace sim;

class CDPRCablePlugin : public System,
                        public ISystemConfigure,
                        public ISystemPreUpdate
{
public:
  void Configure(
    const Entity &entity,
    const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &ecm,
    EventManager &) override
  {
    this->model = Model(entity);
  }

  void PreUpdate(
    const UpdateInfo &,
    EntityComponentManager &) override
  {
    // Placeholder for dynamic cable update
  }

private:
  Model model{kNullEntity};
};

GZ_ADD_PLUGIN(
  CDPRCablePlugin,
  System,
  ISystemConfigure,
  ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(CDPRCablePlugin, "cdpr::CDPRCablePlugin")
