#ifndef PCLE_COMMON_TIME_H
#define PCLE_COMMON_TIME_H

#include <chrono>

namespace pcle {
namespace common {

using namespace std::chrono;

class TimeCount{
public: 
    TimeCount(){
        last_record = std::chrono::high_resolution_clock::now();
    };

    void start(){
        last_record = std::chrono::high_resolution_clock::now();
    }
    float countSecond(){
        auto now = std::chrono::high_resolution_clock::now();
        auto last_ = last_record;
        last_record = now;
        return std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_).count() * 1e-9;
    }

    int countMilliSecond(){
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_record).count() / 1000000;
    }
private:
    time_point<system_clock> last_record;
};

}}

#endif