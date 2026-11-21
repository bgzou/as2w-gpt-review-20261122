#pragma once

#include "cpro_policy.hpp"
#include "isaaclab/algorithms/algorithms.h"
#include <string>
#include <vector>

namespace as2w_unified {
class OrtPolicy {
public:
    explicit OrtPolicy(const std::string& onnx) : runner_(onnx) {}
    void enter(const Motors& q) { controller_.enter(q); }
    void exit() { controller_.exit(); }

    Act infer(const Obs& obs) {
        auto output=runner_.forward({{"observation",std::vector<float>(obs.begin(),obs.end())}});
        const auto& values=output.at("leg_action");
        Act action{};
        if (values.size()!=action.size()) throw std::runtime_error("Incorrect AS2-W actor action size");
        std::copy(values.begin(),values.end(),action.begin());
        return action;
    }

    Setpoints step(const Sensor& s, const Vec3& c, float sensor_age_s) {
        return controller_.step(s,c,sensor_age_s,[this](const Obs& obs) { return infer(obs); });
    }

private:
    isaaclab::OrtRunner runner_;
    Controller controller_;
};
}  // namespace as2w_unified
