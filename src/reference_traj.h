#ifndef REFERENCE_TRAJ_H
#define REFERENCE_TRAJ_H
#include <vector>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <cmath>
class ReferenceTraj
{
public:
    ReferenceTraj();

    void readWayPoint(std::string waypoint_file);


    void computeReferenceTraj(double x_current,
                                             double y_current,
                                             int    n,
                                             std::vector<double> &x_ref,
                                             std::vector<double> &y_ref);

private:
    std::vector<double> m_waypoint_x;
    std::vector<double> m_waypoint_y;
};

#endif // REFERENCE_TRAJ_H
