# CarND-Controls-MPC
This project contains my implementation of MPC to control a car driving in the udacity simulator.

---
## The MPC Controller
A MPC controller is a framework to compute optimal controls for a given system model. It optimizes the control inputs over a time horizon so that the system behavior can be optimized and predicted into the future. The optimization is conducted in each control cycle, however only the first control input is sent to the system. In this way, a MPC can use the current system feedback and correct the difference between the theoretical system model and reality.   

## The vehicle model
#### Kinematic Model
Here we have four system states and two actuators to define the vehicle model. In this project it is a simplified kinematic model which can
be described by the following equation,

    x(t+1)   = x(t)   + v(t)  cos [psi(t)]  dt
    y(t+1)   = y(t)   + v(t)  sin [psi(t)]  dt
    psi(t+1) = psi(t) + v(t) / L  theta(t)  dt
    v(t+1)   = v (t)  + a(t)  dt

where **x** and **y** are the coordinates of the vehicle, **v** is the speed. **Psi** represents the orientation. **a** is the acceleration/brake of the vehicle. **theta** is the steering angle **psi**. **L** is a constant which measures the vehicle center to the front axle.

#### Cross track and orienation Error
Besides the kinematic model, two additional states are used in this project. The cross tracking error **cte** and the orientation error **epsi**. The update rules for them are as follows.

    cte (t+1)  = cte(t)  + v(t) sin(epsi(t)) dt
    epsi (t+1) = epsi(t) + v(t) / L * theta  dt


## Timestep Length **N** and Elapsed Duration **dt**
The timestamp length and the duration between the timestamps are two critical parameters for a MPC controller. The timestamp length defines the time horizon for simulating and optimizing the control inputs. The elapsed duration between timestamps defines how accurate the vehicle model can be simulated. A larger **N** allows the MPC to compute a longer trajectory into the future. A smaller **dt** provides a more accurate simualtion of the  vehicle model. In the meanwhile, a larger **N** and a smaller **dt** can also increase the computation time of the trajectory. This may lead to instability of the vehicle. In my project, I choose N = 9 and dt = 0.10 to make this tradeoff. I observed that if the computational time of the MPC takes too long (choosing **N** > 15, **dt** < 0.05), though the trajectory can be very smooth and looking forward to the future, the reference trajectory can be just off the road which finally cause the vehicle driving out of the track.

## Polynomial Fitting and MPC Preprocessing
The first step of using MPC is to feed the MPC with a reference trajectory. In each cycle, the simulator send out 6 reference way points that the vehicle should follow. Two preprocessing steps are used in my implementation. As the reference way points are given in global coordinates frame, it should be first transformed into vehicle coordinate frame. This is implemented in the function

    void transformPts(const std::vector<double> &x_pts,
                      const std::vector<double> &y_pts,
                      double x_vehicle,
                      double y_vehicle,
                      double yaw_vehicle,
                      std::vector<double> &x_ptsInvehicle,
                      std::vector<double> &y_ptsInvehicle);
Second, I fit the waypoints using a third order polynomials by function **polyfit**. The orientation error **epsi** must be calculated using the first differential of the Polynomial given by

    double epsi = - atan(3 * coeffs[3] * x * x + 2 * coeffs[2] * x + coeffs[1]);


## Latency
To handle the latancy, I tuned the cost function of the MPC, specifically the weights for orientation error **epsi**, corss tacking error **cte** and the change of steering angle **dtheta**. I choose 500 for  **epsi**, 5 for **cte**, 500 for **dtheta**. The reason for choosing **epsi** far bigger than **cte** is the vehicle can follow the curve as much as possible despite having some cross tracking. This makes the vehicle moving stable and do not cause instability unter latancy. A larger value for **dtheta** penalizes turning vehicle erratically. It also improves the stability of the driving. With the chosen parameters, the vehicle is able to drive with a reference speed of 50 and can keep stable on the track.
