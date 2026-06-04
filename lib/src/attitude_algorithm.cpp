#include "attitude_algorithm.hpp"
#include <math.h>
attitude_algorithmer::attitude_algorithmer()
{
    m_IMU_gravity_vec << acc_x ,
                         acc_y ,
                         acc_z ;
    m_FRD_gravity_vec = m_IMU_to_FRD_matrix * m_IMU_gravity_vec;    
    pitch = (acc_x);
}
