#include <algorithm>
#include <matplot/matplot.h>

struct wind_gust{
    Eigen::Vector3d wind_vel{0.0, 0.0, 0.0};
    Eigen::Quaterniond q_wind;
    // Define a global start and end simulation time
    double t_start;
    double t_end;
    bool active(double t) const {
        return t >= t_start && t <= t_end;
    }
};

class wind{
    public:
        // -- add a gust to the gusts vector    
        void append_gust(wind_gust g){
            gusts.push_back(g);
        }

        // -- Compute Total Wind Velocity -- 
        Eigen::Vector3d total_wind_vel(double t){
            Eigen::Vector3d net_wind_velocity{0.0, 0.0, 0.0};
            for(const wind_gust g : gusts){
                if(g.active(t)){
                    net_wind_velocity += g.wind_vel;
                }
            }
            return net_wind_velocity; // Assumes net wind velocity as the summation of all active gusts.
        }
    private:
        std::vector<wind_gust> gusts;
    // FUTURE: 
    // - worst-case function for wind to always make the strongest push away from target. HJR?
    // - less simple way of treating wind
};