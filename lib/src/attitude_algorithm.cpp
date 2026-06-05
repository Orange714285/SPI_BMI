#include "attitude_algorithm.hpp"
#include <cmath>
AccAttitudeAlgorithmer::AccAttitudeAlgorithmer()
{
    m_IMU_to_FRD_matrix << 0 , -1,  0,
                           1 ,  0,  0,
                           0 ,  0,  1; 
}
void AccAttitudeAlgorithmer::algorithmer(float bmi055_acc_x,float bmi055_acc_y,float bmi055_acc_z)
{

    m_imu_acc_x = bmi055_acc_x;
    m_imu_acc_y = bmi055_acc_y;
    m_imu_acc_z = bmi055_acc_z;

    m_IMU_acc_vec << m_imu_acc_x ,
                         m_imu_acc_y ,
                         m_imu_acc_z ;

    m_FRD_acc_vec = m_IMU_to_FRD_matrix.inverse() * m_IMU_acc_vec;
    m_acc_g = sqrt(m_imu_acc_x*m_imu_acc_x+m_imu_acc_y*m_imu_acc_y+m_imu_acc_z*m_imu_acc_z);    

    m_frd_acc_x = m_FRD_acc_vec[0];
    m_frd_acc_y = m_FRD_acc_vec[1];
    m_frd_acc_z = m_FRD_acc_vec[2];

    m_pitch = atan2(m_FRD_acc_vec[0],
              sqrt(m_FRD_acc_vec[1]*m_FRD_acc_vec[1] + m_FRD_acc_vec[2]*m_FRD_acc_vec[2]))*180/M_PI;
    m_roll = atan2(-m_FRD_acc_vec[1],-m_FRD_acc_vec[2])*180/M_PI;
}

GyroAttitudeAlgorithmer::GyroAttitudeAlgorithmer()
{
    m_IMU_to_FRD_matrix << 0 , -1,  0,
                           1 ,  0,  0,
                           0 ,  0,  1; 
}
void GyroAttitudeAlgorithmer::algorithmer(  float bmi055_gyro_x,
                                            float bmi055_gyro_y,
                                            float bmi055_gyro_z,
                                            float roll,
                                            float pitch)
{
    m_imu_gyro_x = bmi055_gyro_x;
    m_imu_gyro_y = bmi055_gyro_y;
    m_imu_gyro_z = bmi055_gyro_z;
    m_roll = roll;
    m_yaw = 0;
    m_pitch = pitch;
    m_IMU_gyro_vec << m_imu_gyro_x,
                         m_imu_gyro_y,
                         m_imu_gyro_z;
    m_FRD_gyro_vec = m_IMU_to_FRD_matrix.transpose()*m_IMU_gyro_vec;
    m_frd_gyro_x = m_FRD_gyro_vec[0];
    m_frd_gyro_y = m_FRD_gyro_vec[1];
    m_frd_gyro_z = m_FRD_gyro_vec[2];
}