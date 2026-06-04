#include <Eigen/Core>
#include <Eigen/Dense>  
#include <Eigen/Geometry> 

class AttitudeAlgorithmer
{
public:
    AttitudeAlgorithmer(float bmi055_acc_x,float bmi055_acc_y,float bmi055_acc_z);
    ~AttitudeAlgorithmer() = default;
private:
    float m_acc_x,m_acc_y,m_acc_z,m_acc_g;    
    Eigen::Matrix3f m_IMU_to_FRD_matrix; // 该矩阵旧系下描述新系的坐标；左乘动向量，逆左乘动系
    Eigen::Vector3f m_IMU_gravity_vec;
public:
    Eigen::Vector3f m_FRD_gravity_vec;
public:
    float m_pitch,m_roll;

};