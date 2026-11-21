#include "himloco_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace as2w_himloco {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr std::size_t LegAxes = 12;

template <class Range> bool finite(const Range& xs) {
    return std::all_of(xs.begin(), xs.end(), [](float x) { return std::isfinite(x); });
}

template <typename T, std::size_t N>
std::array<T, N> read_array(const YAML::Node& n, const char* what) {
    if (!n || !n.IsSequence() || n.size() != N)
        throw std::runtime_error(std::string("himloco deploy.yaml needs ") + what);
    std::array<T, N> out{};
    for (std::size_t i = 0; i < N; ++i) out[i] = n[i].as<T>();
    return out;
}

// Model contract gate. The ONNX must be the COMBINED estimator+actor graph:
//   360 (6 x 60 history) -> 16 pre-tanh latent.
// An ONNX cut from the actor alone (79 -> 16) would silently produce garbage
// because the actor expects the estimator's vel+latent in its input, so pin the
// widths and input name here and let the output name float (act() takes the
// first output).
std::string checked_model(const std::filesystem::path& path) {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "as2w_himloco_contract");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    Ort::Session session(env, path.string().c_str(), options);
    Ort::AllocatorWithDefaultOptions allocator;
    if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)
        throw std::runtime_error("himloco expects one ONNX input and one output");
    const auto input = session.GetInputNameAllocated(0, allocator);
    const auto output = session.GetOutputNameAllocated(0, allocator);
    const auto input_type = session.GetInputTypeInfo(0);
    const auto output_type = session.GetOutputTypeInfo(0);
    const auto in = input_type.GetTensorTypeAndShapeInfo();
    const auto out = output_type.GetTensorTypeAndShapeInfo();
    const auto in_shape = in.GetShape();
    const auto out_shape = out.GetShape();
    if (std::string(input.get()) != "obs" ||
        in_shape.size() != 2 || in_shape[1] != static_cast<int64_t>(ObsSize) ||
        out_shape.size() != 2 || out_shape[1] != static_cast<int64_t>(ActionSize) ||
        in.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        out.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
        throw std::runtime_error(
            "himloco ONNX tensor contract mismatch (need obs[N,360] -> [N,16] float32, "
            "estimator+actor combined)");
    std::cout << "HIMLOCO_MODEL input=" << input.get() << "[N," << ObsSize
              << "] output=" << output.get() << "[N," << ActionSize << "] float32\n";
    return path.string();
}

float alpha_from_cutoff(float hz, float dt) { return -std::expm1(-2.f * Pi * hz * dt); }

// StairEnv.leg_gain(): per-leg [hip, thigh, calf] = [0.6, 1.0, 1.0], leg-major.
float part_gain(std::size_t i, const Config& c) {
    switch (i % 3) {
        case 0: return c.leg_gain_hip;
        case 1: return c.leg_gain_thigh;
        default: return c.leg_gain_calf;
    }
}

}  // namespace

Config load_config(const std::filesystem::path& yaml_path) {
    const auto y = YAML::LoadFile(yaml_path.string());
    Config c;
    if (!y["model_version"])
        throw std::runtime_error("himloco deploy.yaml needs model_version");
    c.model_version = y["model_version"].as<std::string>();
    if (!y["policy_file"]) throw std::runtime_error("himloco deploy.yaml needs policy_file");
    c.onnx = std::filesystem::absolute(yaml_path.parent_path() /
                                       y["policy_file"].as<std::string>());
    c.dt = y["step_dt"] ? y["step_dt"].as<float>() : .02f;
    const int obs = y["observation_size"] ? y["observation_size"].as<int>() : 0;
    const int act = y["action_size"] ? y["action_size"].as<int>() : 0;
    if (obs != static_cast<int>(ObsSize))
        throw std::runtime_error("himloco requires observation_size 360 (6 x 60 history)");
    if (act != static_cast<int>(ActionSize))
        throw std::runtime_error("himloco requires action_size 16");
    c.joint_names = read_array<std::string, MotorCount>(y["joint_names"], "joint_names(16)");
    c.defaults = read_array<float, MotorCount>(y["default_joint_pos"], "default_joint_pos(16)");
    c.stiffness = read_array<float, MotorCount>(y["stiffness"], "stiffness(16)");
    c.damping = read_array<float, MotorCount>(y["damping"], "damping(16)");
    if (y["vx_limit"]) c.vx_limit = y["vx_limit"].as<float>();
    if (y["vy_limit"]) c.vy_limit = y["vy_limit"].as<float>();
    if (y["wz_limit"]) c.wz_limit = y["wz_limit"].as<float>();
    const auto need = [&](const char* key) {
        if (!y[key])
            throw std::runtime_error(std::string("himloco deploy.yaml needs ") + key);
        return y[key].as<float>();
    };
    c.leg_target_cutoff_hz = need("leg_target_cutoff_hz");
    c.wheel_target_cutoff_hz = need("wheel_target_cutoff_hz");
    c.leg_action_scale = need("leg_action_scale_rad");
    c.leg_gain_hip = need("leg_gain_hip");
    c.leg_gain_thigh = need("leg_gain_thigh");
    c.leg_gain_calf = need("leg_gain_calf");
    c.wheel_policy_gain = need("wheel_policy_gain");
    c.wheel_rate_clamp = need("wheel_rate_clamp");
    c.wheel_radius = need("wheel_radius");
    c.wheel_side = need("wheel_side_offset");
    c.height = need("obs_height");
    if (y["phase_clock"]) c.phase_clock = y["phase_clock"].as<bool>();
    if (y["handoff_steps"]) c.handoff_steps = y["handoff_steps"].as<int>();
    if (c.handoff_steps < 0 || c.handoff_steps > 500)
        throw std::runtime_error("himloco handoff_steps out of range");
    if (y["leg_offset_rad"]) {
        c.leg_offset = read_array<float, 12>(y["leg_offset_rad"], "12 leg_offset_rad");
        for (float v : c.leg_offset) {
            if (!std::isfinite(v) || std::abs(v) > .5f)
                throw std::runtime_error("himloco leg_offset_rad out of range (+-0.5)");
        }
    }

    if (!(c.dt > 0.f) || !(c.leg_target_cutoff_hz > 0.f) ||
        !(c.wheel_target_cutoff_hz > 0.f) || !(c.leg_action_scale > 0.f) ||
        !(c.wheel_rate_clamp > 0.f) || !(c.wheel_radius > 0.f) || !(c.height > 0.f))
        throw std::runtime_error("himloco deploy.yaml has a non-positive contract constant");
    // Envelope ceiling. The default candidate keeps the conservative 0.30 rad/s
    // / m/s guard; the DR field-drive build (AS2W_HIMLOCO_STICK_DRIVE) lifts it
    // to the training command range (|vx|<=0.8, |vy|<=0.45, |wz|<=0.8) so the
    // joystick can reach the policy's full capability. vy is not mapped by the
    // publisher, so only vx/wz are reachable in practice.
#if defined(AS2W_HIMLOCO_STICK_DRIVE)
    constexpr float kMaxEnvelope = .80f;
    constexpr float kMaxVy = .45f;
#else
    constexpr float kMaxEnvelope = .30f;
    constexpr float kMaxVy = .30f;
#endif
    if (!finite(c.defaults) || !finite(c.stiffness) || !finite(c.damping) ||
        !std::isfinite(c.vx_limit) || !std::isfinite(c.vy_limit) ||
        !std::isfinite(c.wz_limit) || c.vx_limit <= 0.f || c.vx_limit > kMaxEnvelope ||
        c.vy_limit < 0.f || c.vy_limit > kMaxVy ||
        c.wz_limit < 0.f || c.wz_limit > kMaxEnvelope)
        throw std::runtime_error("himloco deploy.yaml envelope/defaults invalid");

    // ── exact training contract ────────────────────────────────────────────
    // Joint order is LEG-MAJOR (common/environment.py LEGS):
    //   FL_hip, FL_thigh, FL_calf, FR_hip, ... , RR_calf, FL_foot..RR_foot
    // leg_gain [0.6, 1, 1] is applied in this same order.
    static const std::array<std::string, MotorCount> expected{
        "FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
        "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf",
        "FL_foot", "FR_foot", "RL_foot", "RR_foot"};
    for (std::size_t i = 0; i < MotorCount; ++i) {
        const float def = i >= LegAxes ? 0.f
            : i % 3 == 0 ? 0.f : i % 3 == 1 ? .8f : -1.6f;
        if (c.joint_names[i] != expected[i] ||
            std::abs(c.defaults[i] - def) > 1e-5f ||
            std::abs(c.stiffness[i] - (i < LegAxes ? 55.f : 0.f)) > 1e-5f ||
            std::abs(c.damping[i] - 2.f) > 1e-5f)
            throw std::runtime_error("himloco joint/default/gain contract mismatch");
    }
    if (c.model_version != "himloco-360")
        throw std::runtime_error("himloco deploy.yaml model_version must be 'himloco-360'");
    // ── leg action gain: two known training variants ───────────────────────
    // The gain comes from the ENVIRONMENT CLASS the weights were trained in, and
    // it changed between the two runs of this line:
    //   stairs  (StairEnv.leg_gain)   = [0.6, 1.0, 1.0] per leg
    //           -> hip +-0.15 rad, thigh/calf +-0.25 rad   (model_7836, deployed)
    //   flat    (ArcEnv._pre_physics_step hardcodes 0.6*bounded)
    //           -> ALL leg joints +-0.15 rad                (him_arc flat retrain)
    // Anything else means the YAML and the weights disagree; fail closed instead
    // of publishing a different closed loop.
    const bool stair_gain =
        std::abs(c.leg_gain_hip - .6f) < 1e-6f &&
        std::abs(c.leg_gain_thigh - 1.f) < 1e-6f &&
        std::abs(c.leg_gain_calf - 1.f) < 1e-6f;
    const bool flat_gain =
        std::abs(c.leg_gain_hip - .6f) < 1e-6f &&
        std::abs(c.leg_gain_thigh - .6f) < 1e-6f &&
        std::abs(c.leg_gain_calf - .6f) < 1e-6f;
    if (!stair_gain && !flat_gain)
        throw std::runtime_error(
            "himloco leg_gain must be [0.6,1,1] (stair weights) or [0.6,0.6,0.6] "
            "(flat/arc weights)");
    std::cout << "HIMLOCO_GAIN variant=" << (stair_gain ? "stairs[0.6,1,1]" : "flat[0.6,0.6,0.6]")
              << " leg_range_rad hip=" << .25f * c.leg_gain_hip
              << " thigh=" << .25f * c.leg_gain_thigh
              << " calf=" << .25f * c.leg_gain_calf << '\n';
    if (std::abs(c.dt - .02f) > 1e-6f ||
        std::abs(c.leg_target_cutoff_hz - 10.f) > 1e-6f ||
        std::abs(c.wheel_target_cutoff_hz - 5.f) > 1e-6f ||
        std::abs(c.leg_action_scale - .25f) > 1e-6f ||
        std::abs(c.wheel_policy_gain - 2.f) > 1e-6f ||
        std::abs(c.wheel_rate_clamp - 12.f) > 1e-6f ||
        std::abs(c.wheel_radius - .086f) > 1e-6f ||
        std::abs(c.wheel_side - .1654f) > 1e-6f ||
        std::abs(c.height - .37f) > 1e-6f)
        throw std::runtime_error(
            "himloco contract constants mismatch (dt .02 / 10-5 Hz / scale .25 / "
            "wheel gain 2 / clamp 12 / R .086 / side .1654 / HEIGHT .37)");
    return c;
}

Controller::Controller(Config config)
    : config_(std::move(config)), runner_(checked_model(config_.onnx)),
      leg_alpha_(alpha_from_cutoff(config_.leg_target_cutoff_hz, config_.dt)),
      wheel_alpha_(alpha_from_cutoff(config_.wheel_target_cutoff_hz, config_.dt)) {}

void Controller::arm() {
    target_leg_.fill(0.f);
    target_wheel_.fill(0.f);
    obs_.fill(0.f);
    last_raw_.fill(0.f);
    phase_ = 0.f;
    handoff_count_ = 0;
    applied_leg_.fill(0.f);
    applied_wheel_.fill(0.f);
    history_valid_ = false;
    armed_ = true;
}

void Controller::stop() { arm(); armed_ = false; }

Targets Controller::step(const SensorState& s, const Vector3& command, float age_seconds) {
    if (!armed_) return {};
    // Envelope gate: the trained command space for this line is vx=0.25, vy=wz=0.
    // Anything outside the configured envelope disarms (the publisher treats a
    // disarmed controller as EmergencyDamping) instead of feeding the net an
    // out-of-distribution command.
    const bool invalid_command =
        std::abs(command[0]) > config_.vx_limit * 1.00002f ||
        std::abs(command[1]) > config_.vy_limit * 1.00002f + 1e-6f ||
        std::abs(command[2]) > config_.wz_limit * 1.00002f + 1e-6f;
    if (!std::isfinite(age_seconds) || age_seconds < 0.f || age_seconds > .05f ||
        !finite(s.gyro) || !finite(s.body_to_world_wxyz) || !finite(s.position) ||
        !finite(s.velocity) || !finite(command) || invalid_command) {
        stop();
        return {};
    }
    // ── zero-command handoff with a linear applied-target ramp ─────────────
    // dr_environment.py:_pre_physics_step: while dr_handoff_step < handoff_steps
    // the command is forced to zero and applied = w * target with
    // w = (k + 1) / handoff_steps. The observation's applied block must show the
    // ramped value, so applied_* is what gets written to the frame and motors.
    const bool handoff = handoff_count_ < config_.handoff_steps;
    const Vector3 cmd = handoff ? Vector3{0.f, 0.f, 0.f} : command;

    // ── body-frame projected gravity (upright = (0,0,-1)) ──────────────────
    const auto& q = s.body_to_world_wxyz;
    const float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (n2 < .81f || n2 > 1.21f) { stop(); return {}; }
    const float inv = 1.f / std::sqrt(n2);
    const float w = q[0]*inv, x = q[1]*inv, y = q[2]*inv, z = q[3]*inv;
    const float gx = -2.f * (x * z - w * y);
    const float gy = -2.f * (y * z + w * x);
    const float gz = 2.f * (x * x + y * y) - 1.f;

    // ── one 60-wide frame (quality_environment.py:_frame) ──────────────────
    std::array<float, FrameWidth> frame{};
    std::size_t k = 0;
    frame[k++] = cmd[0]; frame[k++] = cmd[1]; frame[k++] = cmd[2];
    frame[k++] = s.gyro[0];  frame[k++] = s.gyro[1];  frame[k++] = s.gyro[2];
    frame[k++] = gx; frame[k++] = gy; frame[k++] = gz;
    for (std::size_t i = 0; i < MotorCount; ++i) frame[k++] = s.position[i] - config_.defaults[i];
    frame[9 + 12] = frame[9 + 13] = frame[9 + 14] = frame[9 + 15] = 0.f;  // obs[21..24]
    for (std::size_t i = 0; i < MotorCount; ++i) frame[k++] = s.velocity[i] * .05f;
    // applied target block (the value commanded on the PREVIOUS step, i.e. what
    // the previous frame's _apply_action used — training builds this frame after
    // _apply_action, so the stack is shifted by exactly one frame).
    for (std::size_t i = 0; i < LegAxes; ++i)
        frame[k++] = applied_leg_[i] / config_.leg_action_scale;
    for (std::size_t i = 0; i < 4; ++i)
        frame[k++] = applied_wheel_[i] / config_.wheel_rate_clamp;
    frame[k++] = config_.height;
    if (config_.phase_clock) {
        // quality_environment.py: phase += dt*gait_frequency(cmd); dims are
        // sin/cos(phase*2pi) * step_strength(cmd).
        const float speed = std::abs(cmd[1]) + .25f * std::abs(cmd[2]);
        const float freq = std::clamp(1.25f + .95f * speed, 1.25f, 1.75f);
        phase_ = std::fmod(phase_ + config_.dt * freq, 1.f);
        const float strength = std::clamp(
            (std::abs(cmd[1]) + .28f * std::abs(cmd[2]) - .04f) / .16f, 0.f, 1.f);
        const float phi = phase_ * 2.f * Pi;
        frame[k++] = std::sin(phi) * strength;
        frame[k++] = std::cos(phi) * strength;
    } else {
        // ArcEnv._step_strength() == 0 for the stair/arc line -> both dims are 0.
        frame[k++] = 0.f;
        frame[k++] = 0.f;
    }
    if (k != FrameWidth) { stop(); return {}; }

    // ── push into the 6-deep history (newest first) ────────────────────────
    if (history_valid_) {
        std::copy_backward(obs_.begin(), obs_.begin() + (HistoryLen - 1) * FrameWidth,
                           obs_.begin() + HistoryLen * FrameWidth);
        std::copy(frame.begin(), frame.end(), obs_.begin());
    } else {
        // Training resets the stack to the current frame on every episode start
        // (common/environment.py:_get_observations with episode_length_buf==0).
        for (std::size_t h = 0; h < HistoryLen; ++h)
            std::copy(frame.begin(), frame.end(), obs_.begin() + h * FrameWidth);
        history_valid_ = true;
    }

    // ── estimator + actor (combined graph): 360 -> 16 pre-tanh latent ──────
    const auto raw = runner_.act({{"obs", std::vector<float>(obs_.begin(), obs_.end())}});
    if (raw.size() != ActionSize) { stop(); return {}; }
    for (float v : raw) if (!std::isfinite(v)) { stop(); return {}; }
    last_raw_.fill(0.f);
    std::copy(raw.begin(), raw.end(), last_raw_.begin());

    // ── action mapping (arc_environment.py:36-40, stair_environment.py:31-34) ──
    Targets out;
    out.active = true;
    for (std::size_t i = 0; i < MotorCount; ++i) {
        const float bounded = std::tanh(raw[i]);
        out.action[i] = bounded;
        if (i >= LegAxes) continue;
        const float physical = part_gain(i, config_) * bounded;   // [0.6,1,1] per leg
        target_leg_[i] += (config_.leg_action_scale * physical - target_leg_[i]) * leg_alpha_;
    }
    for (std::size_t j = 0; j < 4; ++j) {
        // side = [.1654, -.1654, .1654, -.1654] for FL, FR, RL, RR
        const float side = (j % 2 == 0) ? config_.wheel_side : -config_.wheel_side;
        const float ff = (cmd[0] - cmd[2] * side) / config_.wheel_radius;
        const float wheel = std::clamp(ff + config_.wheel_policy_gain * std::tanh(raw[12 + j]),
                                       -config_.wheel_rate_clamp, config_.wheel_rate_clamp);
        target_wheel_[j] += (wheel - target_wheel_[j]) * wheel_alpha_;
    }

    // ── handoff ramp: applied = w * target for the first handoff_steps ─────
    const float ramp_w = handoff
        ? static_cast<float>(handoff_count_ + 1) /
              static_cast<float>(config_.handoff_steps)
        : 1.f;
    if (handoff) {
        for (std::size_t i = 0; i < LegAxes; ++i) applied_leg_[i] = ramp_w * target_leg_[i];
        for (std::size_t j = 0; j < 4; ++j) applied_wheel_[j] = ramp_w * target_wheel_[j];
        ++handoff_count_;
    } else {
        for (std::size_t i = 0; i < LegAxes; ++i) applied_leg_[i] = target_leg_[i];
        for (std::size_t j = 0; j < 4; ++j) applied_wheel_[j] = target_wheel_[j];
    }

    for (std::size_t i = 0; i < LegAxes; ++i) {
        // The gravity-compensation offset ramps in with the same schedule so the
        // first policy cycle never sees a discontinuous motor command.
        out.motors[i].q = config_.defaults[i] + applied_leg_[i] +
                          ramp_w * config_.leg_offset[i];
        out.motors[i].kp = config_.stiffness[i];
        out.motors[i].kd = config_.damping[i];
    }
    for (std::size_t i = LegAxes; i < MotorCount; ++i) {
        out.motors[i].dq = applied_wheel_[i - LegAxes];
        out.motors[i].kp = 0.f;
        out.motors[i].kd = config_.damping[i];
    }
    return out;
}

}  // namespace as2w_himloco
