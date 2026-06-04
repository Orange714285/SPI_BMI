#include <Eigen/Core>
#include <Eigen/Dense>  
#include <Eigen/Geometry> 

class attitude_algorithmer
{
public:
    attitude_algorithmer();
    ~attitude_algorithmer();
private:
    float acc_x,acc_y,acc_z;    
    float pitch,roll;
    Eigen::Matrix3d m_IMU_to_FRD_matrix;
    Eigen::Vector3f m_IMU_gravity_vec;
    Eigen::Vector3f m_FRD_gravity_vec;
    

};