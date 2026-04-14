#include <chrono>
class Timer {
    using clock = std::chrono::steady_clock;    
    public:
        void start_timing(){
            start_time = clock::now();
        }
        double stop_timing(){
            end_time = clock::now();
            dt = (end_time - start_time).count();
            return dt;
        }
    private:
        clock::time_point start_time;
        clock::time_point end_time;
        double dt;
        
};