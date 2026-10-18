#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <sdf/Element.hh>

namespace tiago_pro {
// Each follower has exactly one force writer. Never reset position or velocity.
class PhysicalMimic final : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate {
  struct Follower {
    gz::sim::Joint leader, joint;
    double multiplier, offset, p, d, maxEffort;
  };
  std::vector<Follower> followers;
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    for (auto element = sdf->FindElement("follower"); element;
         element = element->GetNextElement("follower")) {
      auto leader = model.JointByName(ecm, element->Get<std::string>("leader"));
      auto joint = model.JointByName(ecm, element->Get<std::string>("joint"));
      if (leader == gz::sim::kNullEntity || joint == gz::sim::kNullEntity)
        throw std::runtime_error("PhysicalMimic: missing leader or follower joint");
      Follower f{gz::sim::Joint(leader), gz::sim::Joint(joint),
                 element->Get<double>("multiplier"), element->Get<double>("offset"),
                 element->Get<double>("p"), element->Get<double>("d"),
                 element->Get<double>("max_effort")};
      if (!(f.maxEffort > 0) || !(f.p >= 0) || !(f.d >= 0))
        throw std::runtime_error("PhysicalMimic: invalid force or gains");
      f.leader.EnablePositionCheck(ecm);
      f.leader.EnableVelocityCheck(ecm);
      f.joint.EnablePositionCheck(ecm);
      f.joint.EnableVelocityCheck(ecm);
      followers.push_back(f);
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    for (auto &f : followers) {
      auto q = f.joint.Position(ecm), ql = f.leader.Position(ecm);
      auto v = f.joint.Velocity(ecm), vl = f.leader.Velocity(ecm);
      if (!q || !ql || !v || !vl || q->empty() || ql->empty() ||
          v->empty() || vl->empty()) continue;
      const double force = f.p * (f.multiplier * (*ql)[0] + f.offset - (*q)[0])
                         + f.d * (f.multiplier * (*vl)[0] - (*v)[0]);
      if (!std::isfinite(force))
        throw std::runtime_error("PhysicalMimic: non-finite joint state");
      f.joint.SetForce(ecm, {std::clamp(force, -f.maxEffort, f.maxEffort)});
    }
  }
};
}
GZ_ADD_PLUGIN(tiago_pro::PhysicalMimic, gz::sim::System,
              tiago_pro::PhysicalMimic::ISystemConfigure,
              tiago_pro::PhysicalMimic::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(tiago_pro::PhysicalMimic, "tiago_pro::PhysicalMimic")
