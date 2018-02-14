#include "reference_traj.h"

ReferenceTraj::ReferenceTraj()
{
}

void ReferenceTraj::readWayPoint(std::string waypoint_file)
{
    std::cout<<"read waypoint"<<std::endl;
    std::ifstream file ( waypoint_file );

    while ( file.good() )
    {
        std::string x_str;
        std::getline ( file, x_str, ',' );
        std::string y_str;
        std::getline ( file, y_str, '\n' );
        // convert x and y
        double x = atof(x_str.c_str());
        double y = atof(y_str.c_str());
        if (x!=0 && y!=0)
        {
            m_waypoint_x.push_back(x);
            m_waypoint_y.push_back(y);
        }
    }
    std::cout<<"waypoint x size"<<m_waypoint_x.size()<<std::endl;
    std::cout<<"waypoint y size"<<m_waypoint_y.size()<<std::endl;
    file.close();
}

void ReferenceTraj::computeReferenceTraj(double x_current,
                                         double y_current,
                                         int    n,
                                         std::vector<double> &x_ref,
                                         std::vector<double> &y_ref)
{
    x_ref.resize(n);
    y_ref.resize(n);
    size_t min_dist_index = -1;
    double min_dist = 100;
    // Find the closest
    for (size_t i = 0 ; i < m_waypoint_x.size() ; i++ )
    {
        double dist = pow(x_current - m_waypoint_x[i], 2) + pow(y_current - m_waypoint_y[i], 2);
        if (dist < min_dist)
        {
            min_dist_index = i;
        }
    }
    if(min_dist_index != -1)
    {
        for (size_t i = 0 ; i < n; i++)
        {
                uint index = i % m_waypoint_x.size();
                x_ref[i] = m_waypoint_x[index];
                y_ref[i] = m_waypoint_y[index];
        }
    }
}
