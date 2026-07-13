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
    float m_pitch=0.0f,m_roll=0.0f;
    void print_attitude_comparison();
};

class GyroAttitudeAlgorithmer
{
public:
    GyroAttitudeAlgorithmer() ;
    ~GyroAttitudeAlgorithmer() = default;
    void transform_coordinate_and_kalman_filter(float gyro_x, float gyro_y, float gyro_z);
    void algorithm(float roll, float pitch);
    void print_attitude_comparison();

    // 获取 Kalman 滤波器的原始测量值（FRD 机体系，未经 Kalman 滤波）
    float raw_frd_gyro_x() const { return m_FRD_gyro_vec_measure[0]; }
    float raw_frd_gyro_y() const { return m_FRD_gyro_vec_measure[1]; }
    float raw_frd_gyro_z() const { return m_FRD_gyro_vec_measure[2]; }

private:
    Eigen::Matrix3f m_IMU_to_FRD_matrix; // 该矩阵旧系下描述新系的坐标；左乘动向量，逆左乘动系
    Eigen::Vector3f m_IMU_gyro_vec;

    bool m_initialized;                
    std::chrono::steady_clock::time_point m_gyro_data_time; // 最近一次陀螺数据的时间戳 (transform_coordinate 时更新)
    std::chrono::steady_clock::time_point m_last_time;      // 上一次积分的时间戳
public:
    float m_imu_gyro_x = 0, m_imu_gyro_y = 0 , m_imu_gyro_z = 0;
    float m_frd_gyro_x = 0, m_frd_gyro_y = 0 , m_frd_gyro_z = 0;
    float m_pitch = 0, m_roll = 0, m_yaw = 0;
    float m_Euler_roll = 0, m_Euler_pitch = 0, m_Euler_yaw = 0;     // 原始测量值积分得到的欧拉角
    float m_diff_roll  = 0, m_diff_pitch  = 0, m_diff_yaw  = 0;  // 差值 = 滤波后积分 - 原始积分
    float m_quat_roll  = 0, m_quat_pitch  = 0, m_quat_yaw  = 0;  // 四元数积分提取的欧拉角 (内旋 roll-yaw-pitch)
    std::chrono::microseconds m_dt_us;                          // 当前积分步长 (us)
    float m_dt_s = 0;
private: 

    void get_process_noise_covariance_mat(float k, float k_dt);
    void get_measure_noise_covariance_mat(float k);

    // ---- 坐标系转换 ----
    Eigen::Matrix3f m_IMU_to_FRD_matrix_inv;    // 预计算的 IMU→FRD 旋转矩阵的逆

    // ---- 6 维状态: [rate_x, rate_y, rate_z, α_x, α_y, α_z]^T ----
    Eigen::Matrix<float, 6, 6> m_state_transition_mat;
    Eigen::Matrix<float, 6, 6> m_process_noise_covariance_mat;
    Eigen::Matrix<float, 6, 6> m_state_covariance_mat_pri;
    Eigen::Matrix<float, 6, 6> m_state_covariance_mat_pos;
    Eigen::Matrix<float, 6, 6> m_identify = Eigen::Matrix<float, 6, 6>::Identity();

    Eigen::Vector3f m_FRD_gyro_vec_measure;
    Eigen::Matrix<float, 6, 1> m_FRD_gyro_vec_pos;
    Eigen::Matrix<float, 6, 1> m_FRD_gyro_vec_pri;

    float m_Q_coefficient;
    float m_R_coefficient;
    Eigen::Matrix3f m_measure_noise_covariance_mat;
    Eigen::Matrix<float, 3, 6> m_control_mat;
    Eigen::Matrix<float, 6, 3> m_kalman_gain;

    // ---- 四元数姿态（体轴 → 惯性系）----
    Eigen::Quaternionf m_q;       // 内旋更新，体轴角速度右乘

};
