#include "carla_shenlan_stanley_pid_controller/stanely_controller.h"
#include "carla_shenlan_stanley_pid_controller/pid_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include <math.h>

using namespace std;

namespace shenlan {
    namespace control {

        shenlan::control::PIDController e_theta_pid_controller(1.5, 0.0, 0.4); // PID控制器中的微分环节相当于阻尼，加在航向误差引起的前轮转角上, adjust the kp!

        double atan2_to_PI(const double atan2) 
        {
            return atan2 * M_PI / 180;
        }

        double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y) 
        {
            const double dx = point.x - x;
            const double dy = point.y - y;
            return dx * dx + dy * dy;
        }

        void StanleyController::LoadControlConf() 
        {
            k_y_ = 200; //0.5
        }

        // /** to-do **/ 计算需要的控制命令, 实现对应的stanley模型,并将获得的控制命令传递给汽车
        // 提示，在该函数中你需要调用计算误差
        // 控制器中，前轮转角的命令是弧度单位，发送给carla的横向控制指令，范围是 -1~1
        void StanleyController::ComputeControlCmd(const VehicleState &vehicle_state, const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) 
        {  
            double ey{}, head_compensation{}, cross_compensation{};
            
            //load trajectory
            trajectory_points_.clear();
            for(const auto& point : planning_published_trajectory.trajectory_points) {
                trajectory_points_.push_back(point);
            }

            //target point v: km/h, vehicle_state v: km/h  
            if(vehicle_state.v > 0) {
                ComputeLateralErrors(vehicle_state.x, vehicle_state.y, vehicle_state.heading, ey, head_compensation);
                cross_compensation = -atan2_to_PI(std::atan2(k_y_*ey, (vehicle_state.v/3.6)));
                head_compensation = -head_compensation;
            }

            cout << ", heading: " << vehicle_state.heading << ", heading_compensation: " <<  head_compensation << "\ncross_dastance: " << ey << ", cross_compensation: " << cross_compensation << endl;

            //saturation
            double stanley_angle = cross_compensation + e_theta_pid_controller.Control(head_compensation, 0.01);

            if(stanley_angle > 1) {
                cmd.steer_target = 1;
            } 
            else if(stanley_angle < -1) {
                cmd.steer_target = -1;
            }
            else {
                cmd.steer_target = stanley_angle;
            }
            cout << "cmd.steering_angle: " << cmd.steer_target << endl;
        }

        // /** to-do **/ 计算需要的误差，包括横向误差，纵向误差，误差计算函数没有传入主车速度，因此返回的位置误差就是误差，不是根据误差计算得到的前轮转角
        void StanleyController::ComputeLateralErrors(const double x, const double y, const double theta, double &e_y, double &e_theta) 
        {
            //find the target point
            TrajectoryPoint target_point_stanly;
            target_point_stanly = QueryNearestPointByPosition(x, y);

            cout << "~~Stanley: target_heading: " << target_point_stanly.heading;

            //heading error
            e_theta = target_point_stanly.heading - theta;  //target-vehicle???

            //cross error
            e_y = (-1) * (target_point_stanly.x-x) * std::sin(theta) + (target_point_stanly.y-y) * std::cos(theta);

            //e_theta: [-pi, pi]
            if(e_theta >= 3.14){
                e_theta -= 2*M_PI;
            }  
            else if(e_theta <= -3.14) {
                e_theta += 2*M_PI;
            }      
        } 

        // 返回参考路径上和车辆当前位置距离最近的点，返回的是点结构体
        TrajectoryPoint StanleyController::QueryNearestPointByPosition(const double x, const double y) 
        {
            double d_min = PointDistanceSquare(trajectory_points_.front(), x, y); // 得到当前位置距离参考路径的第一个点的距离
            size_t index_min = 0;

            for (size_t i = 1; i < trajectory_points_.size(); ++i) 
            {
                double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
                if (d_temp < d_min) 
                {
                    d_min = d_temp;
                    index_min = i;
                }
            }
            // cout << " index_min: " << index_min << endl;
            //cout << "tarjectory.heading: " << trajectory_points_[index_min].heading << endl;
            theta_ref_ = trajectory_points_[index_min].heading; // 获得距离车辆当前位置最近的路径点的航向角

            return trajectory_points_[index_min];
        }
    }  // namespace control
}  // namespace shenlan