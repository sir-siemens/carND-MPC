#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "mpc_param.h"

using CppAD::AD;


double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars){
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // The part of the cost based on the reference state.
        CppAD::AD<double> cte, epsi, error_v,err_seq_theta;
        for (int t = 0; t < N; t++) {

          fg[0] += 10  * CppAD::pow(vars[cte_start + t], 2);
          cte   += 10  * CppAD::pow(vars[cte_start + t], 2);
        }
        std::cout<<"cte "<<fg[0]<<std::endl;

        for (int t = 0; t < N; t++) {
          fg[0] += 500 * CppAD::pow(vars[epsi_start + t], 2);
          epsi  += 500 * CppAD::pow(vars[epsi_start + t], 2);
        }
        std::cout<<"epsi "<<epsi<<std::endl;

        for (int t = 0; t < N; t++) {
          //fg[0]   += 0.1 * CppAD::pow(vars[v_start + t] - (- ref_v / 1000 * vars[cte_start + t] * vars[cte_start + t] + ref_v), 2);
          //error_v += 0.1 * CppAD::pow(vars[v_start + t] - (- ref_v / 1000 * vars[cte_start + t] * vars[cte_start + t] + ref_v), 2);
          fg[0]   += 0.1 * CppAD::pow(vars[v_start + t] - ref_v, 2);
          error_v += 0.1 * CppAD::pow(vars[v_start + t] - ref_v, 2);
        }
        std::cout<<"error_v "<<error_v<<std::endl;

        // Minimize the use of actuators.
        for (int t = 0; t < N - 1; t++) {
          fg[0] += CppAD::pow(vars[delta_start + t], 2);
          fg[0] += CppAD::pow(vars[a_start + t], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (int t = 0; t < N - 2; t++) {
          fg[0]         += 500 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
          err_seq_theta += 500 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        }

        for (int t = 0; t < N - 2; t++) {
          fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }


        std::cout<<"err_seq_theta "<<err_seq_theta<<std::endl;

        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start]    = vars[x_start];
        fg[1 + y_start]    = vars[y_start];
        fg[1 + psi_start]  = vars[psi_start];
        fg[1 + v_start]    = vars[v_start];
        fg[1 + cte_start]  = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (int t = 1; t < N; t++) {
          // The state at time t+1 .
          AD<double> x1 = vars[x_start + t];
          AD<double> y1 = vars[y_start + t];
          AD<double> psi1 = vars[psi_start + t];
          AD<double> v1 = vars[v_start + t];
          AD<double> cte1 = vars[cte_start + t];
          AD<double> epsi1 = vars[epsi_start + t];

          // The state at time t.
          AD<double> x0 = vars[x_start + t - 1];
          AD<double> y0 = vars[y_start + t - 1];
          AD<double> psi0 = vars[psi_start + t - 1];
          AD<double> v0 = vars[v_start + t - 1];
          AD<double> cte0 = vars[cte_start + t - 1];
          AD<double> epsi0 = vars[epsi_start + t - 1];

          // Only consider the actuation at time t.
          AD<double> delta0 = vars[delta_start + t - 1];
          AD<double> a0 = vars[a_start + t - 1];

          AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0  ;
          AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 + 2 *  coeffs[2] * x0 + coeffs[1]);

          // Here's `x` to get you started.
          // The idea here is to constraint this value to be 0.
          //
          // Recall the equations for the model:
          // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
          // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
          // v_[t+1] = v[t] + a[t] * dt
          // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
          // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
          fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
          fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
          fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
          fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
          fg[1 + cte_start + t] =
              cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
          fg[1 + epsi_start + t] =
              epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }
      }
    };


//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd x0,
                          Eigen::VectorXd coeffs,
                          std::vector<double> &x_vec,
                          std::vector<double> &y_vec
                          ) {
      typedef CPPAD_TESTVECTOR(double) Dvector;

      double x = x0[0];
      double y = x0[1];
      double psi = x0[2];
      double v = x0[3];
      double cte = x0[4];
      double epsi = x0[5];

      // number of independent variables
      // N timesteps == N - 1 actuations
      int n_vars = N * 6 + (N - 1) * 2;
      // Number of constraints
      int n_constraints = N * 6;

      // Initial value of the independent variables.
      // Should be 0 except for the initial values.
      Dvector vars(n_vars);
      for (int i = 0; i < n_vars; i++) {
        vars[i] = 0.0;
      }
      // Set the initial variable values
      vars[x_start] = x;
      vars[y_start] = y;
      vars[psi_start] = psi;
      vars[v_start] = v;
      vars[cte_start] = cte;
      vars[epsi_start] = epsi;

      // Lower and upper limits for x
      Dvector vars_lowerbound(n_vars);
      Dvector vars_upperbound(n_vars);

      // Set all non-actuators upper and lowerlimits
      // to the max negative and positive values.
      for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
      }

      // The upper and lower limits of delta are set to -25 and 25
      // degrees (values in radians).
      // NOTE: Feel free to change this to something else.
      for (int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
      }

      // Acceleration/decceleration upper and lower limits.
      // NOTE: Feel free to change this to something else.
      for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
      }

      // Lower and upper limits for constraints
      // All of these should be 0 except the initial
      // state indices.
      Dvector constraints_lowerbound(n_constraints);
      Dvector constraints_upperbound(n_constraints);
      for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
      }
      constraints_lowerbound[x_start] = x;
      constraints_lowerbound[y_start] = y;
      constraints_lowerbound[psi_start] = psi;
      constraints_lowerbound[v_start] = v;
      constraints_lowerbound[cte_start] = cte;
      constraints_lowerbound[epsi_start] = epsi;

      constraints_upperbound[x_start] = x;
      constraints_upperbound[y_start] = y;
      constraints_upperbound[psi_start] = psi;
      constraints_upperbound[v_start] = v;
      constraints_upperbound[cte_start] = cte;
      constraints_upperbound[epsi_start] = epsi;

      // Object that computes objective and constraints
      FG_eval fg_eval(coeffs);

      // options
      std::string options;
      options += "Integer print_level  0\n";
      options += "Sparse  true        forward\n";
      options += "Sparse  true        reverse\n";

      // place to return solution
      CppAD::ipopt::solve_result<Dvector> solution;

      // solve the problem
      CppAD::ipopt::solve<Dvector, FG_eval>(
          options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
          constraints_upperbound, fg_eval, solution);

      //
      // Check some of the solution values
      //
      bool ok = true;
      ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

      auto cost = solution.obj_value;
      std::cout << "Cost " << cost << std::endl;

      CppAD::vector< AD<double> > fg(N * 6 + (N - 1) * 2 +1);

      CppAD::vector<double> solution_vec = solution.x;



      CppAD::vector< AD<double> >  solution_vars(solution_vec.size());

      // copy solution_vec to solution_vars
      for (int i = 0 ; i < solution_vec.size();i++)
      {
          CppAD::AD<double> value(solution_vec[i] );

          solution_vars[i] = value;
      }

      std::cout << " evaluate cost after optimization" <<std::endl;
      fg_eval(fg, solution_vars);

      // get result
      x_vec.resize(N);
      y_vec.resize(N);
      for (int i = 0; i < N; i++)
      {
            x_vec[i] = solution.x[x_start + i];
            y_vec[i] = solution.x[y_start + i];
      }

      return {solution.x[x_start + 1],   solution.x[y_start + 1],
              solution.x[psi_start + 1], solution.x[v_start + 1],
              solution.x[cte_start + 1], solution.x[epsi_start + 1],
              solution.x[delta_start],   solution.x[a_start]};
}


void MPC::transformGlobalToVehicle(double xInmap,
                                   double yInmap,
                                   double x,
                                   double y,
                                   double yaw,
                                   double &xInVehicle,
                                   double &yInvehicle)
{
    Eigen::Matrix3d vehicleTmap;



    vehicleTmap << cos( -yaw) , -sin(-yaw) , -cos(yaw) * x - sin(yaw)*y,
            sin( -yaw) ,  cos(-yaw) , -cos(yaw) * y + sin(yaw)*x,
            0 ,         0   , 1;

    // Eigen::Matrix3d vehicleTmap;// = mapTvehicle.inverse();
    Eigen::Vector3d ptsInmap ;
    ptsInmap<< xInmap,
            yInmap,
            1.0;

    Eigen::Vector3d ptsInvehicle = vehicleTmap * ptsInmap;
    xInVehicle = ptsInvehicle(0);
    yInvehicle = ptsInvehicle(1);
}

void MPC::transformPts(   const std::vector<double> &x_pts,
                          const std::vector<double> &y_pts,
                          double x_vehicle,
                          double y_vehicle,
                          double yaw_vehicle,
                          std::vector<double> &x_ptsInvehicle,
                          std::vector<double> &y_ptsInvehicle)
{
    x_ptsInvehicle.resize(x_pts.size());
    y_ptsInvehicle.resize(y_pts.size());
    for (size_t i = 0 ; i < x_pts.size(); i++)
    {
        double xInvehicle, yInvehicle;
        transformGlobalToVehicle(x_pts[i],
                                 y_pts[i],
                                 x_vehicle,
                                 y_vehicle,
                                 yaw_vehicle,
                                 xInvehicle,
                                 yInvehicle);
        x_ptsInvehicle[i] = xInvehicle;
        y_ptsInvehicle[i] = yInvehicle;
    }
}

void MPC::std_vectorToEigen(const std::vector<double> &vec,
                            Eigen::VectorXd &vec_eigen)
{
    for (size_t i = 0 ; i < vec.size() ; i++)
    {
        vec_eigen(i) =  vec[i] ;
    }
}

