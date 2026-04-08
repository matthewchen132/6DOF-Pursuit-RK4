# 6-DOF Missile Interceptor (RK4)
Missile interception simulation built in modern C++, focused on guidance, navigation, and dynamics.

---
## Features
- Runge-Kutta 4 (RK4) numerical integration  
- Quaternion-based attitude representation  
- 3D proportional navigation guidance  
- Eigen-based linear algebra  

---
## Current Status
- 3D proportional navigation **working** in simulation  
- In the process of extending from 3D kinematics to full 6-DOF rigid body dynamics  
- Adding aerodynamic force and moment models
    - Rough Version implemented, Lookup tables next.

<img width="400" height="340" alt="simulation preview" src="https://github.com/user-attachments/assets/31b3ae21-0301-443a-80de-55ae49623f2a" />

---

https://github.com/user-attachments/assets/d39109de-255b-406c-8d07-63465ddce8bd



## References

- Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems (Brian L. Stevens)
