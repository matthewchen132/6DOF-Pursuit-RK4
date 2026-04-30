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
        explicit wind(matplot::axes_handle ax) : ax_(ax) {}

        void append_gust(wind_gust g){
            // -- add a gust to the gusts vector   
            gusts.push_back(g);
        }

        void plot_wind_dir(const wind_gust& g, double scale) {
            // Plot the wind gust direction for one wind gust.
            Eigen::Vector3d wind_dir = g.wind_vel.normalized();
            std::vector<double> wx = {0.0, scale * wind_dir(0)};
            std::vector<double> wy = {0.0, scale * wind_dir(1)};
            std::vector<double> wz = {0.0, scale * wind_dir(2)};

            wind_plot_ = ax_->plot3(wx, wy, wz);
            wind_plot_->color("black");
            wind_plot_->line_width(1.0);
            wind_plot_->display_name("Wind Direction");
        }
        
        Eigen::Vector3d total_wind_vel(double t){
            // -- Compute Total Wind Velocity -- 
            Eigen::Vector3d net_wind_velocity{0.0, 0.0, 0.0};
            for(const wind_gust& g : gusts){
                if(g.active(t)){
                    net_wind_velocity += g.wind_vel;
                }
            }
            return net_wind_velocity; // Assumes net wind velocity as the summation of all active gusts.
        }
    private:
        std::vector<wind_gust> gusts;
        matplot::axes_handle ax_; // matplot object: 
        matplot::line_handle wind_plot_; // matplot object: customizes line plot characteristics
    // FUTURE: 
    // - worst-case function for wind to always make the strongest push away from target. HJR?
    // - less simple way of treating wind
};