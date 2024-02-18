#include <iostream>
#include "LQR.h"
#include "angles/angles.h"
using namespace std;
#define PRINT

/**
 * @brief LQRController::LQRController 构造方法
 * @param control_time  控制周期
 * @param wheel_bais     轴距
 * @param model_type    机器人模型类型
 * @param Q            状态转移矩阵权重
 * @param R            输入矩阵权重
 */
LQRController::LQRController()
{
  ;
}

LQRController::LQRController(double control_time, double wheel_bais, int model_type, double* Q, double* R)
{
  T_          = control_time;
  L_          = wheel_bais;
  model_type_ = model_type;
  for (int i = 0; i < 3; i++)
  {
    Q3_[i] = Q[i];
  }
  for (int j = 0; j < 2; j++)
  {
    R2_[j] = R[j];
  }
  Q_ << Q3_[0], 0.0, 0.0,
    0.0, Q3_[1], 0.0,
    0.0, 0.0, Q3_[2];

  R_ << R2_[0], 0.0,
    0.0, R2_[1];

  std::cout << "LQRController()" <<std::endl;
  std::cout << "Q矩阵为：\n" << Q << std::endl;
  std::cout << "R矩阵为：\n" << R << std::endl;
  std::cout << "控制周期为:\n" << T_ << std::endl;
  std::cout << "机器人轴距为:\n" << L_ << std::endl;
  std::cout << "机器人模型为:\n" << model_type_ << std::endl;
}

/**
 * @brief LQRController::~LQRController
 */
LQRController::~LQRController()
{
  std::cout << "~LQRController()" <<std::endl;
}

void LQRController::ChangeParams(double control_time, double wheel_bais, int model_type, double* Q, double* R)
{
  T_          = control_time;
  L_          = wheel_bais;
  model_type_ = model_type;
  for (int i = 0; i < 3; i++)
  {
    Q3_[i] = Q[i];
  }
  for (int j = 0; j < 2; j++)
  {
    R2_[j] = R[j];
  }
  Q_ << Q3_[0], 0.0, 0.0,
    0.0, Q3_[1], 0.0,
    0.0, 0.0, Q3_[2];

  R_ << R2_[0], 0.0,
    0.0, R2_[1];
  std::cout << "ChangeParams()" <<std::endl;
  std::cout << "Q矩阵为：\n" << Q_ << std::endl;
  std::cout << "R矩阵为：\n" << R_ << std::endl;
  std::cout << "控制周期为:\n" << T_ << std::endl;
  std::cout << "机器人轴距为:\n" << L_ << std::endl;
  std::cout << "机器人模型为:\n" << model_type_ << std::endl;
}





/**
 * @brief LQRController::UpdatePoses
 * @param car  当前机器人位姿
 * @param waypoint 目标点位姿
 * @param U_r   预计线速度、角速度
 */
void LQRController::UpdatePoses(vehicleState car, waypoint waypoint, U U_r)
{
  // 机器人位姿、目前点位姿、预计速度、角速度每次调用更新
  x_car_    = car.x; y_car_ = car.y; yaw_car_ = car.yaw;
  x_target_ = waypoint.x; y_target_ = waypoint.y; yaw_target_ = waypoint.yaw;
  v_d_      = U_r.v; kesi_d_ = U_r.kesi;
}


/**
 * @brief LQR::UpdateMatrixAndInputs
 *  对欧拉法做方程离散化转换的矩阵 A和B赋值
 *  这里不同的模型A B矩阵的赋值不同
 *  以下的代码暂时只考虑 中值差速模型和阿克曼模型 （非中值差速模型的后面再写）
 */
void LQRController::UpdateMatrixAndInputs()
{
/**
   对于双轮差速模型
   x(k+1) = x(k)+vtcos(theta)  非线性部分暂时不考虑  考虑的话只做一阶泰勒展开  -v*t*t*cos(theta)
   y(k+1) = y(k)+vtsin(theta)  非线性部分暂时不考虑  考虑的话只做一阶泰勒展开  -V*t*t*sin(theta)
   theta(k+1) = theta(k)+wt

   X(k+1) = AX(K)+BU
   U为[v;w]
 **/
// 更新A B 矩阵
  if(model_type_ == ModelType::centeredDiff)
  {
    A_ << 1.0, 0.0, -v_d_ * T_ * sin(yaw_target_),
      0.0, 1.0, v_d_* T_* cos(yaw_target_),
      0.0, 0.0, 1.0;
    B_ << T_ * cos(yaw_target_), 0.0,
      T_* sin(yaw_target_), 0.0,
      0, T_;
  }
  else if (model_type_ == ModelType::ackermann)
  {
    A_ << 1.0, 0.0, -v_d_ * T_ * sin(yaw_target_),
      0.0, 1.0, v_d_* T_* cos(yaw_target_),
      0.0, 0.0, 1.0;

    B_ << T_ * cos(yaw_target_), 0.0,
      T_* sin(yaw_target_), 0.0,
      T_* tan(kesi_d_), v_d_* T_ / (L_ * cos(kesi_d_) * cos(kesi_d_));
  }
  else if(model_type_ == ModelType::nonCenteredDiff)
  {
    std::cout << "后续补充非中置双轮差速模型"  << std::endl;
  }
  else
  {
    std::cout << "请补充相应模型"  << std::endl;
  }
// 更新误差项
  X_e_ << x_car_ - x_target_, y_car_ - y_target_, angles::shortest_angular_distance(yaw_target_, yaw_car_);

#ifdef PRINT
//  cout << "Q矩阵为：\n" << Q_ << endl;
//  cout << "R矩阵为：\n" << R_ << endl;
//  cout << "A矩阵为:\n" << A_ << endl;
//  cout << "B矩阵为:\n" << B_ << endl;
//  cout << "机器人当前位姿为x :" << x_car_ << "    y:" << y_car_ << "      yaw:" << yaw_car_ << endl;
//  cout << "目前点位姿为x :" << x_target_ << "    y:" << y_target_ << "      yaw:" << yaw_target_ << endl;
//  cout << "X_e矩阵为:\n" << X_e_ << endl;
#endif
}

Matrix2x3 LQRController::SolveRiccati()
{
  int       N             = 150;  //迭代终止次数
  double    err           = 100;  //误差值
  double    err_tolerance = 0.01; //误差收敛阈值
  Matrix3x3 Qf            = Q_;
  Matrix3x3 P             = Qf;   //迭代初始值
  //cout << "P初始矩阵为\n" << P << endl;
  Matrix3x3 Pn;                   //计算的最新P矩阵
  for (int iter_num = 0; iter_num < N; iter_num++)
  {
    Pn = Q_ + A_.transpose() * P * A_ - A_.transpose() * P * B_ * (R_ + B_.transpose() * P * B_).inverse() * B_.transpose() * P * A_;            //迭代公式
#ifdef PRINT
//    cout << "收敛误差为" << (Pn - P).array().abs().maxCoeff() << endl;
#endif
    err = (Pn - P).lpNorm<Eigen::Infinity>();
    if(err < err_tolerance)            //
    {
      P = Pn;
#ifdef PRINT
      //cout << "迭代次数" << iter_num << endl;
#endif
      break;
    }
    P = Pn;
  }

#ifdef PRINT
  //cout << "P矩阵为\n" << P << endl;
#endif
  Matrix2x3 K = -(R_ + B_.transpose() * P * B_).inverse() * B_.transpose() * P * A_;      //反馈率K
  return K;
}

U LQRController::Outputvel()
{
  U output;
  UpdateMatrixAndInputs();
  Matrix2x3 K = SolveRiccati();
  Matrix2x1 U = K * X_e_;
  output.v    = U[0] + v_d_;
  output.kesi = U[1] + kesi_d_;
  output      = LimitSpeed(output);
#ifdef PRINT
  cout << "预计速度为: v: " << v_d_ << "    w:" << kesi_d_ << std::endl;
  cout << "反馈增益K为：\n" << K << endl;
  cout << "控制输入U为：\n" << U << endl;
  cout << "偏差Xe为: \n" << X_e_ << endl;
  cout << "机器人当前位姿为x :" << x_car_ << "    y:" << y_car_ << "      yaw:" << yaw_car_ << endl;
  cout << "目前点位姿为x :" << x_target_ << "    y:" << y_target_ << "      yaw:" << yaw_target_ << endl;
  cout << "实际输出速度为: v: " << output.v << "    w:" << output.kesi << std::endl;
#endif
  return output;
}

U LQRController::LimitSpeed(U control_value)
{
  // 当角速度太大时 ,把线速度写成0，然后对角速度做限制
  if(control_value.v>=1.0)            //速度限幅
  {
    control_value.v = 1.0;
  }
  else if(control_value.v<=-1.0)
  {
    control_value.v = -1.0;
  }
  if(control_value.kesi>=0.7)            //前轮转角限幅
  {
    control_value.kesi = 0.7;
    control_value.v    = 0;
  }
  else if(control_value.kesi<=-0.7)
  {
    control_value.kesi = -0.7;
    control_value.v    = 0;
  }
// 反馈速度跟不上所以在有前进速度时限制一下角速度
  if(fabs(control_value.v>0.3) && control_value.kesi>0.3)
  {
    control_value.kesi = 0.3;
    control_value.v    = 0;
  }
  if(fabs(control_value.v>0.3) && control_value.kesi<-0.3)
  {
    control_value.kesi = -0.3;
    control_value.v    = 0;
  }

  return control_value;
};


