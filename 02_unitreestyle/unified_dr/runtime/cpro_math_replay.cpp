#include "cpro_policy.hpp"
#include <iostream>

template <std::size_t N>
bool read(std::array<float,N>& values) {
    for (float& f:values) if (!(std::cin >> f)) return false;
    return true;
}

template <std::size_t N>
float error(const std::array<float,N>& result,const std::array<float,N>& reference) {
    float m=0;
    for (std::size_t i=0;i<N;++i) m=std::max(m,std::abs(result[i]-reference[i]));
    return m;
}

int main() {
    using namespace as2w_unified;
    Sensor s;
    Vec3 c;
    Motors prev,desired;
    Obs obs;
    Act raw;
    float obs_error=0, target_error=0;
    int n=0;
    while (read(s.q) && read(s.dq) && read(s.body_gyro) && read(s.body_to_world_wxyz) &&
           read(c) && read(prev) && read(raw) && read(obs) && read(desired)) {
        obs_error=std::max(obs_error,error(observation(s,c,prev),obs));
        target_error=std::max(target_error,error(filter(prev,raw,c[0]),desired));
        ++n;
    }
    std::cout<<"frames="<<n<<" obs_abs="<<obs_error<<" target_abs="<<target_error<<"\n";
    return n>0 && obs_error<3e-6f && target_error<6e-6f ? 0 : 1;
}
