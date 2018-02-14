#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state,
                       Eigen::VectorXd coeffs,
                       std::vector<double> &x_vec,
                       std::vector<double> &y_vec
                       );


  // Transform global cooridnate to vehicle cooridnate
  void transformGlobalToVehicle(double xInmap,
                                double yInmap,
                                double x,
                                double y,
                                double yaw,
                                double &xInVehicle,
                                double &yInvehicle);

  // Transform a List of pts to vehicle cooridnate system
  void transformPts(const std::vector<double> &x_pts,
                    const std::vector<double> &y_pts,
                    double x_vehicle,
                    double y_vehicle,
                    double yaw_vehicle,
                    std::vector<double> &x_ptsInvehicle,
                    std::vector<double> &y_ptsInvehicle);

  void std_vectorToEigen(const std::vector<double> &vec,
                         Eigen::VectorXd &vec_eigen);

private:
  // initial state of the vehicle
  double m_x_0, m_y_0, m_psi_0, m_v_0, m_cte_0, m_epsi_0;
};

#endif /* MPC_H */
