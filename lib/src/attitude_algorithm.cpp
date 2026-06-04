#include "attitude_algorithm.hpp"
#include <cmath>
AttitudeAlgorithmer::AttitudeAlgorithmer(float bmi055_acc_x,float bmi055_acc_y,float bmi055_acc_z)
{
    m_acc_x = bmi055_acc_x;
    m_acc_y = bmi055_acc_y;
    m_acc_z = bmi055_acc_z;

    m_IMU_gravity_vec << m_acc_x ,
                         m_acc_y ,
                         m_acc_z ;
                         
    m_IMU_to_FRD_matrix << 0 , -1,  0,
                           1 ,  0,  0,
                           0 ,  0,  1; 
    m_FRD_gravity_vec = m_IMU_to_FRD_matrix.inverse() * m_IMU_gravity_vec;    
    m_acc_g = sqrt(m_acc_x*m_acc_x+m_acc_y*m_acc_y+m_acc_z*m_acc_z);
    m_pitch = atan2(m_FRD_gravity_vec[0],
              sqrt(m_FRD_gravity_vec[1]*m_FRD_gravity_vec[1] + m_FRD_gravity_vec[2]*m_FRD_gravity_vec[2]))*180/M_PI;
    m_roll = atan2(-m_FRD_gravity_vec[1],-m_FRD_gravity_vec[2])*180/M_PI;
}
