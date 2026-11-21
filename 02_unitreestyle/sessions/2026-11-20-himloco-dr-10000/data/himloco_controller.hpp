#pragma once

// AS2-W "HIMLoco" stair/arc policy (HIM estimator + actor) — isolated candidate.
//
// Contract extracted 2026-11-19 from the training run
//   /root/gpufree-data/as2w-runs/him-stairs-20261119-013122/  (model_7836.pt)
//   source: him_stairs/{run_quality,quality_environment,arc_environment,
//                       stair_environment}.py  +  common/environment.py
//   upstream: upstream/HIMLoco/rsl_rl/rsl_rl/modules/{him_actor_critic,him_estimator}.py
//
// ── network ────────────────────────────────────────────────────────────────
// The exported graph must contain ESTIMATOR + ACTOR, because the actor is NOT a
// function of the raw observation:
//     vel(3), latent(16) = estimator(history 360)          # L2-normalised latent
//     actor_input        = cat(one_step_frame 60, vel 3, latent 16) = 79
//     out 16             = actor(79 -> 512 -> 256 -> 128 -> 16, ELU)
// The actor's trailing tanh is COMMENTED OUT in him_actor_critic.py, and the
// training env applies `latent.tanh()` itself (arc_environment.py:36) — so the
// ONNX output is PRE-tanh and this controller applies the tanh.
// Estimator is stateless (plain MLP over the flattened 6x60 history).
//
// ── observation ───────────────────────────────────────────────────────────
// obs = 6 x 60 history, newest frame FIRST (common/environment.py:_get_observations).
// One 60-wide frame (quality_environment.py:_frame, base 58 + 2 phase dims):
//   [0:3]   command (vx, vy, wz)
//   [3:6]   body angular velocity (IMU gyro, body frame)
//   [6:9]   projected gravity, UPRIGHT = (0,0,-1)
//   [9:25]  joint_pos - default, 16 axes in policy order, WHEELS POSITION ZEROED
//   [25:41] joint_vel * 0.05, all 16 axes
//   [41:53] applied leg target / 0.25   (12)   <- applied, not the raw action
//   [53:57] applied wheel target / 12   (4)
//   [57]    HEIGHT constant 0.37
//   [58:60] sin(phi)*strength, cos(phi)*strength.  For this stair/arc line
//           ArcEnv._step_strength() returns ZERO (arc_environment.py:14-15), so
//           both dims are identically 0. Kept explicit so the layout matches.
//
// ── action (16) ───────────────────────────────────────────────────────────
//   b = tanh(out)                                        # arc_environment.py:36
//   leg_physical[i] = leg_gain[i] * b[i]                 # StairEnv.leg_gain()
//     leg_gain = [0.6, 1.0, 1.0] per leg (hip, thigh, calf), leg-major
//     => hip command range +-0.15 rad, thigh/calf +-0.25 rad  (0.25 * gain)
//   wheel_physical[j] = clamp((vx - wz*side[j])/R + 2*b[12+j], +-12)
//     side = [0.1654, -0.1654, 0.1654, -0.1654] (FL, FR, RL, RR lateral offsets)
//   target_leg   <- lerp(target_leg,   0.25 * leg_physical, alpha_leg)
//   target_wheel <- lerp(target_wheel, wheel_physical,      alpha_wheel)
//     alpha = -expm1(-2*pi*f*dt) with f = 10 Hz (legs) / 5 Hz (wheels)
//     NO idle-zero rule for the him/himloco mode (common/environment.py:158).
//   applied == target here: the training 3-deep latency queue is only randomised
//   when cfg.randomize is on, and this run used randomize=False.
//   motors: legs q = default + target_leg, kp 55 kd 2; wheels dq = target_wheel,
//   kp 0 kd 2 (common/environment.py actuators).
//
// Public API deliberately mirrors as2w_armd_deploy (publisher-facing), so the
// shared as2w_publish.cpp only needs its own AS2W_POLICY_HIMLOCO branch.

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>

#include "isaaclab/algorithms/algorithms.h"

namespace as2w_himloco {

inline constexpr std::size_t MotorCount = 16;
inline constexpr std::size_t FrameWidth = 60;
inline constexpr std::size_t HistoryLen = 6;
inline constexpr std::size_t ObsSize = FrameWidth * HistoryLen;   // 360
inline constexpr std::size_t ActionSize = 16;
// NB: no namespace-level LegCount — as2w_publish.cpp owns a global one and pulls
// this namespace in with `using namespace`.

using Vector3 = std::array<float, 3>;
using Motors = std::array<float, MotorCount>;
using Features = std::array<float, ObsSize>;

struct SensorState {
    Vector3 gyro{};
    std::array<float, 4> body_to_world_wxyz{};
    Motors position{};
    Motors velocity{};
};

struct MotorTarget { float q = 0, dq = 0, kp = 0, kd = 0; };

struct Targets {
    std::array<MotorTarget, MotorCount> motors{};
    Motors action{};                      // tanh-bounded action (16), for logging
    bool active = false;
};

struct Config {
    std::filesystem::path onnx;
    std::string model_version;
    std::array<std::string, MotorCount> joint_names;
    Motors defaults{}, stiffness{}, damping{};
    float dt = .02f;
    float vx_limit = .25f;                // trained command is vx=0.25 constant
    float vy_limit = 0.f;
    float wz_limit = 0.f;
    // contract constants (hard-validated by load_config)
    float leg_target_cutoff_hz = 10.f;
    float wheel_target_cutoff_hz = 5.f;
    float leg_action_scale = .25f;
    float leg_gain_hip = .6f;
    float leg_gain_thigh = 1.f;
    float leg_gain_calf = 1.f;
    float wheel_policy_gain = 2.f;
    float wheel_rate_clamp = 12.f;
    float wheel_radius = .086f;
    float wheel_side = .1654f;
    float height = .37f;
    // Flat/arc line only: the two trailing observation dims carry a gait phase
    // clock (sin/cos * step_strength). The stair line leaves it false and keeps
    // both dims at zero, matching its ArcEnv._step_strength() == 0.
    bool phase_clock = false;
    // Handoff ramp (dr_environment.py:_pre_physics_step): after every arm the
    // command is forced to zero and the applied leg/wheel target is scaled by
    // w=(k+1)/handoff_steps for the first handoff_steps control cycles.
    int handoff_steps = 0;   // stairs line: no ramp; DR line sets 100 in deploy.yaml
    // Gravity-compensation offset added to the motor command (policy order,
    // 12 leg axes). Training's sim has no droop, so the real robot settles
    // below the commanded angle; this offset makes the actual joint reach the
    // policy's target again. The observation's applied block still reports the
    // POLICY target (the offset is a plant-side correction, not an action).
    std::array<float, 12> leg_offset{};
};

Config load_config(const std::filesystem::path& yaml);

class Controller {
public:
    explicit Controller(Config config);
    void arm();
    void stop();
    bool armed() const { return armed_; }
    Targets step(const SensorState& sensors, const Vector3& command, float age_seconds);

    const Features& last_observation() const { return obs_; }
    const Motors& last_raw_action() const { return last_raw_; }   // pre-tanh latent
    const Config& config() const { return config_; }
    float leg_alpha() const { return leg_alpha_; }
    float wheel_alpha() const { return wheel_alpha_; }

private:
    Config config_;
    isaaclab::OrtRunner runner_;
    bool armed_ = false;
    bool history_valid_ = false;
    Motors target_leg_{};                 // rad offset from defaults (= applied)
    std::array<float, 4> target_wheel_{};
    std::array<float, FrameWidth * HistoryLen> obs_{};
    Motors last_raw_{};
    float leg_alpha_ = 0.f;
    float wheel_alpha_ = 0.f;
    float phase_ = 0.f;                   // 0..1 gait phase, flat/arc line only
    int handoff_count_ = 0;               // cycles since arm(), for the ramp
    std::array<float, 12> applied_leg_{}; // post-ramp target, what obs[41:53] uses
    std::array<float, 4> applied_wheel_{};
};

}  // namespace as2w_himloco
