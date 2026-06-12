#include <Eigen/Core>
#include <Eigen/Dense>  
#include <Eigen/Geometry> 
#include <chrono>

class AccAttitudeAlgorithmer
{
public:
    AccAttitudeAlgorithmer();
    ~AccAttitudeAlgorithmer() = default;
    void transform_coordinate(float acc_x, float acc_y, float acc_z);
    void algorithmer();
private:
    float m_imu_acc_x,m_imu_acc_y,m_imu_acc_z,m_acc_g;
    Eigen::Matrix3f m_IMU_to_FRD_matrix; // 该矩阵旧系下描述新系的坐标；左乘动向量，逆左乘动系
    Eigen::Vector3f m_IMU_acc_vec;
    Eigen::Vector3f m_FRD_acc_vec;
public:
    float m_frd_acc_x,m_frd_acc_y,m_frd_acc_z;
    float m_pitch,m_roll;

};

class GyroAttitudeAlgorithmer
{
public:
    GyroAttitudeAlgorithmer() ;
    ~GyroAttitudeAlgorithmer() = default;
    void transform_coordinate(float gyro_x, float gyro_y, float gyro_z);
    void algorithm(float roll, float pitch);

private:
    Eigen::Matrix3f m_IMU_to_FRD_matrix; // 该矩阵旧系下描述新系的坐标；左乘动向量，逆左乘动系
    Eigen::Vector3f m_IMU_gyro_vec;
    Eigen::Vector3f m_FRD_gyro_vec;

    bool m_initialized;                
    std::chrono::steady_clock::time_point m_gyro_data_time; // 最近一次陀螺数据的时间戳 (transform_coordinate 时更新)
    std::chrono::steady_clock::time_point m_last_time;      // 上一次积分的时间戳
public:
    float m_imu_gyro_x,m_imu_gyro_y,m_imu_gyro_z;
    float m_frd_gyro_x,m_frd_gyro_y,m_frd_gyro_z;
    float m_pitch,m_roll,m_yaw;
    std::chrono::microseconds m_dt_us;                          // 当前积分步长 (us)

};