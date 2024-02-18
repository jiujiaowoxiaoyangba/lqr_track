#include <iostream>
#include <Eigen/Dense>
#include "Tool.h"
using namespace std;

typedef Eigen::Matrix<double, 3, 3> Matrix3x3;
typedef Eigen::Matrix<double, 3, 1> Matrix3x1;
typedef Eigen::Matrix<double, 2, 1> Matrix2x1;
typedef Eigen::Matrix<double, 2, 2> Matrix2x2;
typedef Eigen::Matrix<double, 3, 2> Matrix3x2;
typedef Eigen::Matrix<double, 2, 3> Matrix2x3;


enum ModelType
{
  centeredDiff,   // 中置双轮差速模型
  ackermann,      // 阿克曼模型
  nonCenteredDiff // 轮子不在机器人中心的双轮差速模型  ，这里加这个特殊情况下会出现轮子在路径上，但是车体中心不在路径上的情况
};

//状态方程变量: X = [x_e  y_e  yaw_e]^T    x_e 是小车和目标点x方向的偏差   y_e 是小车和目标点y方向的偏差 yaw_e 是小车和目标点航向角方向的偏差
//状态方程控制输入: U = [v_e  kesi_e]^T    v_e 是预计的线速度   kesi_e 是预计的角速度

class LQRController
{
  // X[k+1] = A·X[k]+B·U
private:
  Matrix3x3 A_;
  Matrix3x2 B_;
  Matrix3x3 Q_;
  Matrix2x2 R_;
  Matrix3x1 X_e_;


  double L_;                                                                                               //车辆轴距 （中值双轮差速模型中可以不用，留着给阿克曼模型或者非中值差速模型的底盘用）
  double T_;                                                                                               //采样间隔 控制周期
  double x_car_, y_car_, yaw_car_;                                                                         //车辆位姿
  double x_target_, y_target_, yaw_target_;                                                                //目标点位姿
  double v_d_, kesi_d_;                                                                                    //期望速度和前轮偏角
  double Q3_[3];                                                                                           //Q权重，三项 状态矩阵权重
  double R2_[2];                                                                                           //R权重，两项 输入矩阵权重
  int    model_type_ = ModelType::centeredDiff;                                                            // 机器人模型 默认中置双轮差速模型

public:
  LQRController();
  LQRController(double control_time, double wheel_bais, int model_type, double* Q, double* R);
  ~LQRController();
  void      ChangeParams(double control_time, double wheel_bais, int model_type, double* Q, double* R); //外部调用
  void      UpdatePoses(vehicleState car, waypoint waypoint, U U_r);                                    //更新机器人位姿和目标点位姿   外部调用
  void      UpdateMatrixAndInputs();                                                                    //构造状态方程参数           内部调用
  Matrix2x3 SolveRiccati();                                                                             //黎卡提方程求解             内部调用
  U         Outputvel();                                                                                //LQR控制器计算速度          外部调用
  U         LimitSpeed(U control_value);                                                                //  限制线速度和角速度        内部调用
};



