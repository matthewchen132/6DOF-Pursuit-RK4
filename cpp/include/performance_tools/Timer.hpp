#include <chrono>
#include <vector>
class Timer {
    /*
        Simple stopwatch to measure loop times.
        
        Example
        1) start_timing
        2) elapsed_time = stop_timing
        # elapsed_time = time in micro-seconds from start_timing and elapsed_time
∏
    */
    using clock = std::chrono::steady_clock;    
    public:
        // TODO: would be nice to have a built-in storage of average time, number of iterations.
        // std::vector<double> average_time;
        // average_time.reserve();
        int n_iters = 0;
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