#ifndef MPC_PARAM_H
#define MPC_PARAM_H

// Set the timestep length and duration
int N = 9;
double dt = 0.10;


// Vehicle model
const double Lf = 2.67;

// x, y, psi, v, cte, epsi
int dim_states    = 6 ;
// steering, gas
int dim_actuators = 2 ;

//


// Reference Velocity
double ref_v = 50;


int x_start = 0;
int y_start = x_start + N;
int psi_start = y_start + N;
int v_start = psi_start + N;
int cte_start = v_start + N;
int epsi_start = cte_start + N;
int delta_start = epsi_start + N;
int a_start = delta_start + N - 1;



#endif // MPC_PARAM_H
