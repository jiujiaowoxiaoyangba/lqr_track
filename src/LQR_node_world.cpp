#include <ros/ros.h>
#include <iostream>
#include "LQR.h"
#include <vector>
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <random>
using namespace std;

double              freq, L, V_DESIRED;                                                                            //采样频率，车辆轴距，期望速度
double              v_max;                                                                                         //最大速度
bool                limit_v_and_kesi;                                                                              //是否限幅(对于阿卡曼转向车辆需要限幅，全向车随意)
bool                is_simulate;                                                                                   //是否是仿真模式
std::vector<double> Q_set;                                                                                         //Q矩阵
std::vector<double> R_set;                                                                                         //R矩阵
double              slow_LEVE1_DISTANCE, slow_LEVE2_DISTANCE, slow_LEVE1_V, slow_LEVE2_V, goal_tolerance_DISTANCE; //定义二级减速距离和速度
#define pi acos(-1)
#define T 1/freq                                                                                                   //采样时间
bool pose_initialized = false;
#define ADDNOISE 0

std::random_device rd;                                                                                             // 使用硬件熵源获取种子
std::mt19937       gen(rd());                                                                                      // Mersenne Twister 生成器

// 定义浮点数分布范围
std::uniform_real_distribution<double> distribution(0.9, 1.1);


vehicleState update_state(U control, vehicleState car)
{
  car.v    = control.v;
  car.kesi = control.kesi;
  if(ADDNOISE)
  {
    car.x   += car.v * cos(car.yaw) * T * distribution(gen);
    car.y   += car.v * sin(car.yaw) * T *  distribution(gen);
    car.yaw += car.kesi*T * distribution(gen);
    car.yaw  = angles::normalize_angle(car.yaw);
  }
  else
  {
    car.x   += car.v * cos(car.yaw) * T;
    car.y   += car.v * sin(car.yaw) * T;
    car.yaw += car.kesi*T;
    car.yaw  = angles::normalize_angle(car.yaw);
  }
//  car.yaw += car.v / L * tan(car.kesi) * T;
//  // Update yaw using quaternion
//  tf::Quaternion quat = tf::createQuaternionFromYaw(car.yaw);
//  tf::Vector3    axis(car.kesi, 0, 0);
//  tf::Quaternion delta_quat(tf::createQuaternionFromRPY(axis[0] * T, axis[1] * T, axis[2] * T));

//  quat = delta_quat * quat;
//  double roll, pitch;
//  tf::Matrix3x3(quat).getRPY(roll, pitch, car.yaw);

  return car;
}


class Path {
private:
  vector<waypoint> path;
public:
  //添加新的路径点
  void Add_new_point(waypoint& p)
  {
    path.push_back(p);
  }

  void Add_new_point(vector<waypoint>& p)
  {
    path = p;
  }

  //路径点个数
  unsigned int Size()
  {
    return path.size();
  }

  //获取路径点
  waypoint Get_waypoint(int index)
  {
    waypoint p;
    p.ID  = path[index].ID;
    p.x   = path[index].x;
    p.y   = path[index].y;
    p.yaw = path[index].yaw;
    return p;
  }

  vector<waypoint> Get_waypoints()
  {
    return path;
  }


  // 搜索路径点, 将小车到起始点的距离与小车到每一个点的距离对比，找出最近的目标点索引值
  int Find_target_index(vehicleState state)
  {
    double min   = abs(sqrt(pow(state.x - path[0].x, 2) + pow(state.y - path[0].y, 2)));
    int    index = 0;
    for (int i = 0; i < path.size(); i++)
    {
      double d = abs(sqrt(pow(state.x - path[i].x, 2) + pow(state.y - path[i].y, 2)));
      if (d < min)
      {
        min   = d;
        index = i;
      }
    }

    //索引到终点前，当（机器人与下一个目标点的距离L_1）小于（当前目标点到下一个目标点距离L_)时，索引下一个目标点
    if ((index + 1) < path.size())
    {
      double current_x = path[index].x; double current_y = path[index].y;
      double next_x = path[index + 1].x; double next_y = path[index + 1].y;
      double L_  = abs(sqrt(pow(next_x - current_x, 2) + pow(next_y - current_y, 2)));
      double L_1 = abs(sqrt(pow(state.x - next_x, 2) + pow(state.y - next_y, 2)));
      //ROS_INFO("L is %f,Lf is %f",L,Lf);
      if (L_1 < L_)
      {
        index += 1;
      }
    }
    return index;
  }
};

class LQR_node {
private:
  //car
  vehicleState car;     //小车状态
  U            control; //小车控制量[v,kesi]
  double       Q[3];
  double       R[2];
  int          lastIndex; //最后一个点索引值
  waypoint     lastPoint; //最后一个点信息
  string       action;    //小车目前动作：跟踪或跟踪完成(tracking or reach goal!)

  //ROS
  ros::Subscriber            path_sub;         //订阅路径，消息类型为nav_msgs::Path
  ros::Subscriber            odom_sub;         //订阅小车当前位姿信息，消息类型为nav_msgs::Odometry
  ros::Subscriber            initial_pose_sub; // 初始位姿
  ros::Publisher             vel_pub;          //发布速度信息，消息类型为geometry_msgs::Twist
  ros::Publisher             actual_state_pub; //发布小车实际位姿，消息类型为geometry_msgs::Pose2D
  ros::Publisher             visual_state_pub; //向rviz发布小车虚拟轨迹，消息类型为visualization_msgs::Marker
  geometry_msgs::Point       visual_state_pose;
  visualization_msgs::Marker visual_state_trajectory;
  geometry_msgs::Pose2D      actual_pose;
  geometry_msgs::Twist       vel_msg;
  geometry_msgs::Pose2D      pose2d_robot;
  int                        temp;//计数，达到终点时，用于判断控制器是否需要关闭


public:
  LQRController lqr_controller_;
  Path*         path;

  LQR_node(ros::NodeHandle& nh)      //初始化中添加轨迹、小车初始位姿
  {
    //权重矩阵
    for(int q = 0; q<Q_set.size(); q++)
      Q[q] = Q_set[q];
    for(int r = 0; r<R_set.size(); r++)
      R[r] = R_set[r];
    lqr_controller_.ChangeParams(T, double(L), 0, Q, R);
    path = new Path();

    //ROS:
    path_sub         = nh.subscribe("path", 10, &LQR_node::addpointcallback, this);
    vel_pub          = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    visual_state_pub = nh.advertise<visualization_msgs::Marker>("visualization_pose", 10);
    actual_state_pub = nh.advertise<geometry_msgs::Pose2D>("LQR_pose", 10);
//    odom_sub         = nh.subscribe<nav_msgs::Odometry>("odom_tf", 1, &LQR_node::odomcallback, this);
    odom_sub         = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/navi/base/pose/dist", 1, &LQR_node::robotPosecallback, this);
    initial_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initial_pose", 1, &LQR_node::initialPoseCallBack, this);
    //robot state initialize:
    //car.x = car.y = car.yaw = car.v = car.kesi = 0;
    action = "the car is tracking!!";
  }

  ~LQR_node()
  {
    delete(path);
  }

  void odomcallback(const nav_msgs::Odometry::ConstPtr& odom_value)
  {
    pose2d_robot.x     = odom_value->pose.pose.position.x;
    pose2d_robot.y     = odom_value->pose.pose.position.y;
    pose2d_robot.theta = tf::getYaw(odom_value->pose.pose.orientation);
    car.x              = pose2d_robot.x;
    car.y              = pose2d_robot.y;
    car.yaw            = pose2d_robot.theta;
  }

  void initialPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    pose2d_robot.x     = msg->pose.pose.position.x;
    pose2d_robot.y     = msg->pose.pose.position.y;
    pose2d_robot.theta = tf::getYaw(msg->pose.pose.orientation);
    car.x              = pose2d_robot.x;
    car.y              = pose2d_robot.y;
    car.yaw            = pose2d_robot.theta;
    ROS_WARN_STREAM("initial pose x:" << car.x  <<  "   y:" <<car.y
                                      << "    yaw: " <<car.yaw);
    pose_initialized = true;
  }


  void robotPosecallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    pose2d_robot.x     = msg->pose.pose.position.x;
    pose2d_robot.y     = msg->pose.pose.position.y;
    pose2d_robot.theta = tf::getYaw(msg->pose.pose.orientation);
    car.x              = pose2d_robot.x;
    car.y              = pose2d_robot.y;
    car.yaw            = pose2d_robot.theta;
  }

  /**
   * @brief addpointcallback
   * @param msg
   * 计算点的方向，通过atan2计算
   * 最后一个点方向用倒数第二个点的
   */
  void addpointcallback(const nav_msgs::Path::ConstPtr& msg)
  {
    vector<waypoint> waypoints;
    static double    yaw;
    if(msg->poses.size()<=1)
    {
      ROS_ERROR_STREAM("path size must > 1");
      return;
    }
    for(int i = 0; i<msg->poses.size()-1; i++)
    {
      waypoint waypoint;
      //ROS_INFO("THE PATH[%d]'s ID is %d",i,msg->poses[i].header.seq);
      waypoint.ID = msg->poses[i].header.seq;

      waypoint.x = msg->poses[i].pose.position.x;
      waypoint.y = msg->poses[i].pose.position.y;
      //获取角度
//      double         roll, pitch;
//      tf::Quaternion quat;
//      tf::quaternionMsgToTF(msg->poses[i].pose.orientation, quat);
//      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

      yaw = atan2(msg->poses[i+1].pose.position.y-msg->poses[i].pose.position.y,
                  msg->poses[i+1].pose.position.x-msg->poses[i].pose.position.x);

      waypoint.yaw = yaw;
      waypoints.push_back(waypoint);
      path->Add_new_point(waypoints);
    }
// 最后一个点用倒数第二个点的角度
    waypoint waypoint;
    waypoint.ID  = msg->poses[msg->poses.size()-1].header.seq;
    waypoint.x   = msg->poses[msg->poses.size()-1].pose.position.x;
    waypoint.y   = msg->poses[msg->poses.size()-1].pose.position.y;
    waypoint.yaw = yaw;
    waypoints.push_back(waypoint);
    path->Add_new_point(waypoints);

    //路径点vector数组传到path类中
    lastIndex = path->Size() - 1;
    lastPoint = path->Get_waypoint(lastIndex);
  }



  double slow_judge(double distance)
  {
    if (distance>=slow_LEVE2_DISTANCE&&distance <= slow_LEVE1_DISTANCE)
    {
      return slow_LEVE1_V;
    }
    else if (distance>=goal_tolerance_DISTANCE&&distance < slow_LEVE2_DISTANCE)
    {
      return slow_LEVE2_V;
    }
    else if (distance < goal_tolerance_DISTANCE)
    {
      action = "the car has reached the goal!";
      return 0.0;
    }
    else
    {
      return V_DESIRED;
    }
  }

  //控制器流程
  /*步骤：
              1.获取小车当前位姿和路径点，在while(ros::ok())函数下面ros::spinOnce()阶段订阅回调函数odomcallback和addpointcallback时完成
      2.搜索路径点
      3.获取路径点信息，构造期望控制量
      4.先进行一次减速判断，确定纵向期望速度v_r，和机器人当前动作：tracking或reached the goal
      5.构造权重矩阵，从launch文件的参数Q=[],R=[]导入
      6.使用LQR控制器，计算速度和前轮转角，其中有限幅过程
      7.发布话题，其中有角速度的计算
      8.判断是否需要关闭控制器，如果到达终点就关闭
      9.loop_rate.sleep()，结束一次循环，准备开始下一次。
   */
  void LQR_track()
  {
    U        U_r;
    waypoint Point;
    double   K;
    //ROS_INFO("path size is: %d",path->Size());
    if(path->Size()!=0 &&pose_initialized == true)
    {
      //搜索路径点
      int target_index = path->Find_target_index(car);
      //ROS_INFO("target index is : %d", target_index);

      //获取路径点信息，构造期望控制量
      Point = path->Get_waypoint(target_index);                  //获取x,y
      //ROS_INFO("waypoint information is x:%f,y:%f,yaw:%f", Point.x, Point.y, Point.yaw);
      K = cal_K(path->Get_waypoints(), target_index);            //计算曲率
      //减速判断
//      double kesi       = atan2(L * K, 1);
//      double v_distance = abs(sqrt(pow(car.x - lastPoint.x, 2) + pow(car.y - lastPoint.y, 2)));

      //      double line_yaw_rad, car_yaw_rad;
      //      line_yaw_rad = atan2(Point.y-lastPoint.y, Point.x-lastPoint.x);
      //      line_yaw_rad = angles::from_degrees(lastPoint.yaw);
      //      car_yaw_rad  = angles::from_degrees(car.yaw);
      //      car_yaw_rad  = angles::normalize_angle(car_yaw_rad);
      //      double kesi       = angles::to_degrees(angles::shortest_angular_distance(line_yaw_rad, car_yaw_rad));  // 这里这个角度要注意一下取最小角度
//现在是为了生成输入矩阵U 包含线速度和角速度 线速度先指定 0.5，角速度为机器人的角度和两点之间的连线和x轴的夹角的差


      double line_yaw_rad, car_yaw_rad;
      line_yaw_rad = cal_Angle(path->Get_waypoints(), target_index);
      line_yaw_rad = angles::normalize_angle(line_yaw_rad);
      car_yaw_rad  = car.yaw;
      car_yaw_rad  = angles::normalize_angle(car_yaw_rad);

      double kesi       = angles::shortest_angular_distance(car_yaw_rad, line_yaw_rad);
      double v_distance = abs(sqrt(pow(car.x - lastPoint.x, 2) + pow(car.y - lastPoint.y, 2)));

      //ROS_INFO("the distance is %f\n", v_distance);
      U_r.v    = slow_judge(v_distance);
      U_r.kesi = kesi;
//      ROS_INFO("%s", action.c_str());                 //机器人动作
      ROS_INFO("the desired v is: %f,the desired kesi is: %f", U_r.v, U_r.kesi);
      ROS_INFO_STREAM("Car yaw: " << car_yaw_rad << "   line_yaw_rad: "<< line_yaw_rad);


      //使用LQR控制器
      lqr_controller_.UpdatePoses(car, Point, U_r);
      control = lqr_controller_.Outputvel();

      if(U_r.v==0)
        control.v = 0;                                  //判断，期望速度为0，则机器人停下
      if(limit_v_and_kesi)
        control = v_and_kesi_limit(control);            //速度和前轮转角限幅
      ROS_INFO("the speed is: %f,the kesi is: %f", control.v, control.kesi);
      //ROS_INFO("the car position is x: %f, y: %f", car.x, car.y);
      ROS_WARN_STREAM("before Move car x:" << car.x << "    y: " << car.y  << "   yaw: " << car.yaw);
      ROS_WARN_STREAM("target Pose x:" << Point.x << "      y: " << Point.y << "   yaw: " << Point.yaw);
      //话题发布
      PUB();
      // 仿真的情况下直接给一个模拟的位姿
      if(is_simulate)
      {
        car = update_state(control, car);
      }
      ROS_WARN_STREAM("After Move car x:" << car.x << "    y: " << car.y  << "   yaw: " << car.yaw);

      //控制器关闭判断
      shutdown_controller();

      ROS_INFO("--------------------------------------");
    }
  }

  //控制启停函数
  void node_control()
  {
    ros::Rate loop_rate(freq);
    Marker_set();            //设置Marker属性
    while (ros::ok())
    {
      ros::spinOnce();
      LQR_track();
      loop_rate.sleep();
    }
  }

  void PUB()
  {
    visual_state_pose.x = car.x; visual_state_pose.y = car.y;
    actual_pose.x       = car.x; actual_pose.y = car.y; actual_pose.theta = car.yaw;
    vel_msg.linear.x    = control.v;
//    vel_msg.angular.z = control.v*tan(control.kesi)/L; //横摆角速度为w = v*tan(kesi)/L
    vel_msg.angular.z = control.kesi;                            //横摆角速度为w = v*tan(kesi)/L
    visual_state_trajectory.points.push_back(visual_state_pose); //visualization_msgs::Marker为一个容器，所以现需要向里面push_back结构体，再发布
    visual_state_pub.publish(visual_state_trajectory);           //发布虚拟轨迹
    vel_pub.publish(vel_msg);                                    //发布速度
    actual_state_pub.publish(actual_pose);                       //发布位姿
  }

  void shutdown_controller()
  {
    if(action == "the car has reached the goal!")
    {
      temp += 1;
      if(temp ==50)
      {
        ROS_WARN("shutdown the LQR controller!");
        temp = 0;
        ros::shutdown();
      }
    }
  }

  void Marker_set()
  {
    //设置消息类型参数
    visual_state_trajectory.header.frame_id = "map";
    visual_state_trajectory.header.stamp    = ros::Time::now();
    visual_state_trajectory.action          = visualization_msgs::Marker::ADD;
    visual_state_trajectory.ns              = "LQR";
    //设置点的属性
    visual_state_trajectory.id      = 0;
    visual_state_trajectory.type    = visualization_msgs::Marker::POINTS;
    visual_state_trajectory.scale.x = 0.02;
    visual_state_trajectory.scale.y = 0.02;
    visual_state_trajectory.color.r = 1.0;
    visual_state_trajectory.color.a = 1.0;
  }

  U v_and_kesi_limit(U control_value)
  {
//    if(control_value.v>=v_max)            //速度限幅
//    {
//      control_value.v = v_max;
//      ROS_WARN("The calculated value may be inaccurate ");
//    }
//    else if(control_value.v<=-v_max)
//    {
//      control_value.v = -v_max;
//      ROS_WARN("The calculated value may be inaccurate ");
//    }


//    if(control_value.kesi>=pi/2)            //前轮转角限幅
//    {
//      control_value.kesi = pi/2;
//      ROS_WARN("The calculated value may be inaccurate ");
//    }
//    else if(control_value.kesi<=-pi/2)
//    {
//      control_value.kesi = -pi/2;
//      ROS_WARN("The calculated value may be inaccurate ");
//    }

    // 当角速度太大时 ,把线速度写成0，然后对角速度做限制
    if(control_value.v>=v_max)            //速度限幅
    {
      control_value.v = v_max;
      ROS_WARN("The calculated value may be inaccurate ");
    }
    else if(control_value.v<=-v_max)
    {
      control_value.v = -v_max;
      ROS_WARN("The calculated value may be inaccurate ");
    }
    if(control_value.kesi>=0.4)            //前轮转角限幅
    {
      control_value.kesi = 0.4;
      control_value.v    = 0;
      ROS_WARN("The calculated value may be inaccurate ");
    }
    else if(control_value.kesi<=-0.4)
    {
      control_value.kesi = -0.4;
      control_value.v    = 0;
      ROS_WARN("The calculated value may be inaccurate ");
    }



    return control_value;
  }
};

int main(int argc, char* * argv)
{
  ros::init(argc, argv, "LQR_node");
  ros::NodeHandle n;
  ros::NodeHandle n_prv("~");

  n_prv.param<double>("freq", freq, 20);
  n_prv.param<double>("L", L, 0.2);
  n_prv.param<double>("V_DESIRED", V_DESIRED, 0.5);
  n_prv.param<double>("v_max", v_max, 1.0);
  n_prv.param<double>("slow_LEVE1_DISTANCE", slow_LEVE1_DISTANCE, 5.0);
  n_prv.param<double>("slow_LEVE2_DISTANCE", slow_LEVE2_DISTANCE, 2.0);
  n_prv.param<double>("goal_tolerance_DISTANCE", goal_tolerance_DISTANCE, 0.1);
  n_prv.param<double>("slow_LEVE1_V", slow_LEVE1_V, 0.35);
  n_prv.param<double>("slow_LEVE2_V", slow_LEVE2_V, 0.15);
  n_prv.param<bool>("limit_v_and_kesi", limit_v_and_kesi, false);
  n_prv.param<bool>("is_simulate", is_simulate, false);

  n_prv.param("Q_set", Q_set, Q_set);
  n_prv.param("R_set", R_set, R_set);

  LQR_node* node = new LQR_node(n);
  node->node_control();
  return (0);
}


