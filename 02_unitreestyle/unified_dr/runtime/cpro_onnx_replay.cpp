#include "cpro_ort_adapter.hpp"
#include <iomanip>
#include <iostream>

template <std::size_t N>
bool read(std::array<float,N>& values) {
    for (float& f:values) if (!(std::cin >> f)) return false;
    return true;
}

template <std::size_t N>
float error(const std::array<float,N>& result,const std::array<float,N>& reference) {
    float m=0;
    for (std::size_t i=0;i<N;++i) {
        if (!std::isfinite(result[i]) || !std::isfinite(reference[i]))
            throw std::domain_error("Nonfinite CPRO replay field");
        m=std::max(m,std::abs(result[i]-reference[i]));
    }
    return m;
}

int main(int argc,char** argv) {
    if (argc!=2) {
        std::cerr<<"Expected ONNX model path\n";
        return 2;
    }
    using namespace as2w_unified;
    OrtPolicy policy(argv[1]);
    Sensor s;
    Vec3 c;
    Motors prev,desired;
    Obs reference;
    Act raw_reference;
    float obs_error=0,raw_error=0,target_error=0;
    int n=0;
    while (read(s.q) && read(s.dq) && read(s.body_gyro) && read(s.body_to_world_wxyz) &&
           read(c) && read(prev) && read(raw_reference) && read(reference) && read(desired)) {
        const auto obs=observation(s,c,prev);
        const Act raw=policy.infer(obs);
        obs_error=std::max(obs_error,error(obs,reference));
        raw_error=std::max(raw_error,error(raw,raw_reference));
        target_error=std::max(target_error,error(filter(prev,raw,c[0]),desired));
        ++n;
    }
    const bool ok=n>0 && obs_error<3e-6f && raw_error<5e-5f && target_error<1.5e-5f;
    std::cout<<std::setprecision(9)<<"{\"frames\":"<<n<<",\"observation_max_abs\":"
             <<obs_error<<",\"raw_action_max_abs\":"<<raw_error<<",\"target_max_abs\":"
             <<target_error<<",\"pass\":"<<(ok ? "true" : "false")<<"}\n";
    return ok ? 0 : 1;
}
