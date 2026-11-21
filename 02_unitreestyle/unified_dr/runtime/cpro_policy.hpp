#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace as2w_unified {
using Vec3 = std::array<float, 3>;
using Quat = std::array<float, 4>;
using Act = std::array<float, 12>;
using Motors = std::array<float, 16>;
using Obs = std::array<float, 58>;

inline constexpr Motors defaults{0,.8f,-1.6f,0,.8f,-1.6f,0,.8f,-1.6f,0,.8f,-1.6f,0,0,0,0};
inline constexpr std::array<std::size_t, 16> dds_state_slot{3,4,5,0,1,2,9,10,11,6,7,8,13,12,15,14};
inline constexpr Motors dds_state_sign{-1,1,1,-1,1,1,-1,1,1,-1,1,1,1,1,1,1};
inline constexpr Vec3 command_max{.8f,.45f,.8f};
inline const float leg_alpha = -std::expm1(-2.*3.14159265358979323846*10.*.02);
inline const float wheel_alpha = -std::expm1(-2.*3.14159265358979323846*5.*.02);

struct Sensor {
    Motors q{}, dq{};
    Vec3 body_gyro{};
    Quat body_to_world_wxyz{};
};

struct Setpoints {
    Motors q{}, dq{}, kp{}, kd{}, tau{};
};

template <std::size_t N>
bool finite(const std::array<float, N>& values) {
    return std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); });
}

inline Obs observation(const Sensor& s, const Vec3& command, const Motors& previous) {
    const auto [w,x,y,z] = s.body_to_world_wxyz;
    const float norm = w*w+x*x+y*y+z*z;
    if (std::abs(norm-1.f)>.02f) throw std::domain_error("Invalid body-to-world quaternion");
    Obs obs{command[0],command[1],command[2],s.body_gyro[0],s.body_gyro[1],s.body_gyro[2],
            2.f*(w*y-x*z),-2.f*(w*x+y*z),2.f*(x*x+y*y)-1.f};
    for (std::size_t i=0; i<16; ++i) {
        obs[9+i] = i<12 ? s.q[i]-defaults[i] : 0.f;
        obs[25+i] = s.dq[i]*.05f;
        obs[41+i] = previous[i]/(i<12 ? .25f : 12.f);
    }
    obs[57] = .37f;
    return obs;
}

inline Motors filter(const Motors& previous, const Act& raw, float vx) {
    Motors result{};
    for (std::size_t i=0; i<12; ++i)
        result[i]=previous[i]+leg_alpha*(.25f*std::clamp(raw[i],-1.f,1.f)-previous[i]);
    const float wheel=std::clamp(vx/.086f,-12.f,12.f);
    for (std::size_t i=12; i<16; ++i) {
        const float next=previous[i]+wheel_alpha*(wheel-previous[i]);
        result[i]=(wheel==0.f && std::abs(next)<.02f) ? 0.f : next;
    }
    return result;
}

class Controller {
public:
    void enter(const Motors& q) {
        exit();
        if (!finite(q)) throw std::domain_error("Invalid handoff q");
        for (std::size_t i=0; i<12; ++i) previous_[i]=q[i]-defaults[i];
        active_=true;
    }
    void exit() {
        previous_={};
        active_=false;
    }

    // infer must call the ONNX deterministic actor observation[1,58] -> leg_action[1,12].
    // Sensors and setpoints use policy order. The host FSM owns DDS and failsafe.
    template <typename Infer>
    Setpoints step(const Sensor& sensors, const Vec3& command, float sensor_age_s, Infer infer) {
        try {
            if (!active_ || !finite(sensors.q) || !finite(sensors.dq) || !finite(sensors.body_gyro) ||
                !finite(sensors.body_to_world_wxyz) || !finite(command) || !std::isfinite(sensor_age_s) ||
                sensor_age_s<0.f || sensor_age_s>.04f)
                throw std::domain_error("Invalid or stale AS2-W policy frame");
            for (std::size_t i=0; i<3; ++i) {
                if (std::abs(command[i])>command_max[i]+1e-6f)
                    throw std::domain_error("Command outside trained range");
            }
            const auto obs=observation(sensors,command,previous_);
            if (obs[8]>-.52f) throw std::domain_error("Body tilt outside learned range");
            const Act raw=infer(obs);
            if (!finite(raw)) throw std::domain_error("Invalid ONNX output");
            previous_=filter(previous_,raw,command[0]);
            Setpoints targets;
            targets.kd.fill(2.f);
            for (std::size_t i=0; i<12; ++i) {
                targets.q[i]=defaults[i]+previous_[i];
                targets.kp[i]=55.f;
            }
            for (std::size_t i=12; i<16; ++i) targets.dq[i]=previous_[i];
            return targets;
        } catch (...) {
            exit();
            throw;
        }
    }
private:
    Motors previous_{};
    bool active_=false;
};
}  // namespace as2w_unified
