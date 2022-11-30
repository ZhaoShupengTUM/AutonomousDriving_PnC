#include "carla_shenlan_lqr_pid_controller/lqr_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "math.h"

using namespace std;

namespace shenlan {
namespace control {

LqrController::LqrController() {}

LqrController::~LqrController() {}

// lqr的配置
void LqrController::LoadControlConf() {
    ts_ = 0.01;    // 每隔0.01s进行一次控制

    cf_ = 155494.663;                              // 前轮侧偏刚度,左右轮之和
    cr_ = 155494.663;                              // 后轮侧偏刚度, 左右轮之和
    wheelbase_ = 2.852;                            
    steer_ratio_ = 16;                             // 方向盘的转角到轮胎转动角度之间的比值系数
    steer_single_direction_max_degree_ = 470.0;    // 最大方向pan转角

    const double mass_fl = 1845.0/4;                     // 左前悬的质量
    const double mass_fr = 1845.0/4;                     // 右前悬的质量
    const double mass_rl = 1845.0/4;                     // 左后悬的质量
    const double mass_rr = 1845.0/4;                     // 右后悬的质量
    const double mass_front = mass_fl + mass_fr;    // 前悬质量
    const double mass_rear = mass_rl + mass_rr;     // 后悬质量
    mass_ = mass_front + mass_rear;

    lf_ = wheelbase_ * (1.0 - mass_front / mass_);    // 汽车前轮到中心点的距离
    lr_ = wheelbase_ * (1.0 - mass_rear / mass_);     // 汽车后轮到中心点的距离

    // moment of inertia
    iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;    // 汽车的转动惯量

    lqr_eps_ = 0.01;              // LQR 迭代求解精度
    lqr_max_iteration_ = 1500;    // LQR的迭代次数

    return;
}

// 初始化控制器
void LqrController::Init() {
    // Matrix init operations.
    const int matrix_size = basic_state_size_;
    matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    /*
    A matrix (Gear Drive)
    [0.0, 1.0, 0.0, 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m, (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    // 初始化A矩阵的常数项
    matrix_a_(0, 1) = 1.0;
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(2, 3) = 1.0;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    // 初始化A矩阵的非常数项
    matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

    /*
    b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
    */
    // 初始化B矩阵
    matrix_b_ = Matrix::Zero(basic_state_size_, 1);
    matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_bd_ = matrix_b_ * ts_;

    // 状态向量
    matrix_state_ = Matrix::Zero(matrix_size, 1);
    // 反馈矩阵
    matrix_k_ = Matrix::Zero(1, matrix_size);
    // lqr cost function中 输入值u的权重
    matrix_r_ = Matrix::Identity(1, 1);
    matrix_r_(0, 0) = 10;
    // lqr cost function中 状态向量x的权重
    matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

    // int q_param_size = 4;
    matrix_q_(0, 0) = 2;    // TODO: lateral_error
    matrix_q_(1, 1) = 1;    // TODO: lateral_error_rate
    matrix_q_(2, 2) = 0.1;    // TODO: heading_error
    matrix_q_(3, 3) = 0.1;    // TODO: heading__error_rate

    matrix_q_updated_ = matrix_q_;

    return;
}

// 两点之间的距离
double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y) {
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
}

// 将角度(弧度制)归化到[-M_PI, M_PI]之间
double NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

double atan2_to_PI(const double atan2) { return atan2 * M_PI / 180; }

// **TODO **计算控制命令 该函数定频调用
bool LqrController::ComputeControlCommand(const VehicleState &localization, const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) {
    // 规划轨迹
    trajectory_points_ = planning_published_trajectory.trajectory_points;
    /*
    A matrix (Gear Drive)
    [0.0, 1.0, 0.0, 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m, (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    // TODO 01 配置状态矩阵A
    matrix_a_ = matrix_a_coeff_/localization.velocity + matrix_a_;
    
    /*
    b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
    */
    // TODO 02 动力矩阵B
    // matrix_bd_ = matrix_b_*ts_;

    // cout << "matrix_bd_.row(): " << matrix_bd_.rows() << endl;
    // cout << "matrix_bd_.col(): " << matrix_bd_.cols() << endl;
    // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading Error Rate]

    // TODO 03 计算横向误差并且更新状态向量x
    UpdateState(localization);

    // TODO 04 更新状态矩阵A并将状态矩阵A离散化
    UpdateMatrix(localization);

    // cout << "matrix_bd_.row(): " << matrix_bd_.rows() << endl;
    // cout << "matrix_bd_.col(): " << matrix_bd_.cols() << endl;

    // TODO 05 Solve Lqr Problem
    SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, lqr_eps_, lqr_max_iteration_, &matrix_k_);

    // TODO 06 计算feedback, 根据反馈对计算状态变量（误差状态）的时候的符号的理解：K里面的值实际运算中全部为正值，steer = -K *
    // state，按照代码中采用的横向误差的计算方式，横向误差为正值的时候（state中的第一项为正），参考点位于车辆左侧，车辆应该向左转以减小误差，而根据试验，仿真器中，给正值的时候，车辆向右转，给负值的时候，车辆向左转。
    //   feedback = - K * state
    Eigen::MatrixXd feedback = Matrix::Identity(1, 1);
    feedback = - matrix_k_ * matrix_state_;

    //   Convert vehicle steer angle from rad to degree and then to steer degrees
    double road_steer_deg = feedback(0,0)*180/M_PI;
    double steer_deg = road_steer_deg * steer_ratio_;
    
    //   then to 100% ratio
    std::cout << "matrix_k_: " << matrix_k_ << std::endl;
    double steer_angle_feedback = steer_deg/steer_single_direction_max_degree_;

    // TODO 07 计算前馈控制，计算横向转角的反馈量
    double steer_angle_feedforward = 0.0;
    steer_angle_feedforward = ComputeFeedForward(localization, ref_curv_);
    double steer_angle = steer_angle_feedback - 0.9 * steer_angle_feedforward;
    // 限制前轮最大转角，这里定义前轮最大转角位于 [-20度～20度]
    if (steer_angle >= atan2_to_PI(20.0)) {
        steer_angle = atan2_to_PI(20.0);
    } else if (steer_angle <= -atan2_to_PI(20.0)) {
        steer_angle = -atan2_to_PI(20.0);
    }
    // Set the steer commands
    cmd.steer_target = steer_angle;

    return true;
}

// 计算横向误差并且更新状态向量x
void LqrController::UpdateState(const VehicleState &vehicle_state) {
    // LateralControlError lat_con_err;  // 将其更改为智能指针
    std::shared_ptr<LateralControlError> lat_con_err = std::make_shared<LateralControlError>();
    // 计算横向误差
    ComputeLateralErrors(vehicle_state.x, vehicle_state.y, vehicle_state.heading, vehicle_state.velocity, vehicle_state.angular_velocity, vehicle_state.acceleration, lat_con_err);

    // State matrix update;
    matrix_state_(0, 0) = lat_con_err->lateral_error;
    matrix_state_(1, 0) = lat_con_err->lateral_error_rate;
    matrix_state_(2, 0) = lat_con_err->heading_error;
    matrix_state_(3, 0) = lat_con_err->heading_error_rate;

    // cout << "lateral_error: " << (lat_con_err->lateral_error) << endl;
    // cout << "heading_error: " << (lat_con_err->heading_error) << endl;
}

// TODO 04 更新状态矩阵A并将状态矩阵A离散化
void LqrController::UpdateMatrix(const VehicleState &vehicle_state) 
{
    matrix_a_ = matrix_a_coeff_/vehicle_state.velocity + matrix_a_;

    Eigen::Matrix4d eye;
    eye.setIdentity(4,4);
    matrix_ad_ = eye + matrix_a_*ts_;

}

// TODO 07 前馈控制，计算横向转角的反馈量
double LqrController::ComputeFeedForward(const VehicleState &localization, double ref_curvature) 
{
    Eigen::Matrix<double, 4, 1> b2;
    b2 << 0, 
    (lr_*cr_ - lf_*cf_)/(mass_*localization.velocity) - localization.velocity,
    0,
    (-1)*(lf_*lf_*cf_+lr_*lr_*cr_)/(iz_*localization.velocity);

    Eigen::Matrix<double, 4, 1> b2_d = b2*ts_;
    Eigen::Matrix<double, 1, 1> feedforward = matrix_k_*b2_d;

    double feedforward_angle = feedforward(0,0)*ref_curvature;

    return feedforward_angle;
}

// TODO 03 计算误差
void LqrController::ComputeLateralErrors(const double x, const double y, const double theta, const double linear_v, const double angular_v, const double linear_a, LateralControlErrorPtr &lat_con_err) 
{
    //find the nearest point
    TrajectoryPoint nearest_point = QueryNearestPointByPosition(x, y); //get ref_curv_, ref_curv_front

    //calculate the v_x, v_y of vehicle in the world frame
    //(assumption:the linear_v has the same direction of heading)
    double v_x = linear_v * cos(NormalizeAngle(nearest_point.heading - theta));
    double v_y = linear_v * sin(NormalizeAngle(nearest_point.heading - theta)); //[0,pi] positiv velocity to right; [-pi,0] negativ velocity to left

    //neareast_point.kappa is the yaw rate of the path
    double e_theta = NormalizeAngle(theta - nearest_point.heading);
    double e_theta_dt = angular_v - ref_curv_*v_x;

    //heading error
    lat_con_err->heading_error = e_theta;
    lat_con_err->heading_error_rate = e_theta_dt;

    //lateral error
    lat_con_err->lateral_error = PointDistanceSquare(nearest_point, x, y);

    //judge the side of the vehicle relative the the path
    double e_y = (-1) * (nearest_point.x-x) * std::sin(theta) + (nearest_point.y-y) * std::cos(theta);
    if(e_y >= 0) //point is in left side
    {
        if(v_y >= 0) {
            lat_con_err->lateral_error_rate = v_y;
        } else {
            lat_con_err->lateral_error_rate = v_y;
        }
    } else { //point is in right side
        if(v_y >=0 ) {
            lat_con_err->lateral_error_rate = -v_y;
        } else {
            lat_con_err->lateral_error_rate = -v_y;
        }
    }
    
    //calculate accelaration
    previous_heading_acceleration_ = (angular_v - previous_heading_rate_)/ts_;
    previous_ref_heaidng_acceleration_ = (ref_curv_*v_x - previous_ref_heading_rate_)/ts_;

    //store previous data
    previous_heading_rate_ = angular_v;
    previous_ref_heading_rate_ = ref_curv_*v_x;
    previous_lateral_acceleration_ = linear_a;
}

// 查询距离当前位置最近的轨迹点???
TrajectoryPoint LqrController::QueryNearestPointByPosition(const double x, const double y) {
    double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points_.size(); ++i) {
        double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    ref_curv_ = trajectory_points_[index_min].kappa;    // 对应的最近的轨迹点上的曲率

    double front_index = index_min + 5;
    if (front_index >= trajectory_points_.size()) {
        ref_curv_front_ = trajectory_points_[index_min].kappa;
    } else {
        ref_curv_front_ = trajectory_points_[front_index].kappa;
    }

    return trajectory_points_[index_min];
}

// TODO 05:求解LQR方程
//B is 1*4 vector
void LqrController::SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R, const double tolerance, const uint max_num_iteration, Matrix *ptr_K) {
    // 防止矩阵的维数出错导致后续的运算失败
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() || Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols()) {
        std::cout << "LQR solver: one or more matrices have incompatible dimensions." << std::endl;
        return;
    }

    //diedai
    Eigen::Matrix4d p_old;
    p_old = Q;
    int n = max_num_iteration;
    for(int i=0; i<n; i++)
    {
        Eigen::Matrix4d p_new = 
        A.transpose() * p_old * A - 
        A.transpose() * p_old * B.transpose() *
        (R + B * p_old * B.transpose()).inverse() *
        B * p_old * A + Q;
        
        if (fabs((p_new - p_old).maxCoeff()) < tolerance) {
            p_old = p_new;
            break;
        }
        p_old = p_new;
    }

    *ptr_K = (R + B.transpose() * p_old * B).inverse() * B.transpose() * p_old * A;
}

}    // namespace control
}    // namespace shenlan
