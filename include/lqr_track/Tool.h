#pragma once
#include <iostream>
#include <math.h>
#include <vector>
using namespace std;
#define pi acos(-1)

//定义路径点
typedef struct waypoint {
  int ID;
  double x, y, yaw;      //x,y
}waypoint;

//定义小车状态
typedef struct vehicleState {
  double x, y, yaw, v, kesi;      //x,y,yaw,前轮偏角kesi
}vehicleState;

//定义控制量
typedef struct U {
  double v;
  double kesi;      //速度v,前轮偏角kesi
}U;

double cal_K(vector<waypoint> waypoints, int index);     //计算曲率K
double cal_Angle(vector<waypoint> waypoints, int index); // 计算两点和x轴的夹角
