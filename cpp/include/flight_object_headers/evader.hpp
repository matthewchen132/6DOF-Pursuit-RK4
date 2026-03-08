#pragma once
#include <vector>
#include <cmath>
#include <State.hpp>

class Evader{
    private:
        const double m_evader = 9298.64; // kg
        const double length = 10.0; // meters
        const double r = .1; // meters
        const double MOI = 0.5*(m_evader)*r*r*1.1; // (.5*MR^2)*1.1, approximated with a cylinder,        
    public:
        State X;
        State dXdt(double T, const State& X_2d) const;
        double A = 1.0; // m^2
        double Cd = 0.08;
        double density = 1.2754; // kg / m^3
        double thrust = 120000; // N
        // roughly modeled after an F-16
        Eigen::Vector3d r_cp_cg = Eigen::Vector3d(-length, 0.0, 0.0); // always in line in pitch direction (Body frame)
        const double Jxz = 1331.4132386;
        const double Jxx = 12820.614648;
        const double Jyy = 75673.623725;
        const double Jzz = 85552.113395;
        Eigen::Matrix3d J{{Jxx, 0.0, -Jxz},
                            {0.0, Jyy, 0.0},
                            {-Jxz, 0.0, Jzz}};     
};