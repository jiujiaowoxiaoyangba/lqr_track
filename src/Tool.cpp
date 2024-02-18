#include <iostream>
#include <Tool.h>

double cal_K(vector<waypoint> waypoints, int index)
{
  double res;
  //差分法求一阶导和二阶导
  double dx, dy, ddx, ddy;
  if (index == 0)
  {
    dx  = waypoints[1].x - waypoints[0].x;
    dy  = waypoints[1].y - waypoints[0].y;
    ddx = waypoints[2].x + waypoints[0].x - 2 * waypoints[1].x;
    ddy = waypoints[2].y + waypoints[0].y - 2 * waypoints[1].y;
  }
  else if (index == (waypoints.size() - 1))
  {
    dx  = waypoints[index].x - waypoints[index - 1].x;
    dy  = waypoints[index].y - waypoints[index - 1].y;
    ddx = waypoints[index].x + waypoints[index - 2].x - 2 * waypoints[index].x;
    ddy = waypoints[index].y + waypoints[index - 2].y - 2 * waypoints[index].y;
  }
  else
  {
    dx  = waypoints[index + 1].x - waypoints[index].x;
    dy  = waypoints[index + 1].y - waypoints[index].y;
    ddx = waypoints[index + 1].x + waypoints[index - 1].x - 2 * waypoints[index].x;
    ddy = waypoints[index + 1].y + waypoints[index - 1].y - 2 * waypoints[index].y;
  }
  //res.yaw = atan2(dy, dx);//yaw
  //计算曲率：设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
  res = (ddy * dx - ddx * dy) / (sqrt(pow((pow(dx, 2) + pow(dy, 2)), 3)));
  return res;
}

double cal_Angle(vector<waypoint> waypoints, int index)
{
  double res;
  //差分法求一阶导和二阶导
  double dx, dy;
  if (index == 0)
  {
    dx = waypoints[1].x - waypoints[0].x;
    dy = waypoints[1].y - waypoints[0].y;
  }
  else if (index == (waypoints.size() - 1))
  {
    dx = waypoints[index].x - waypoints[index - 1].x;
    dy = waypoints[index].y - waypoints[index - 1].y;
  }
  else
  {
    dx = waypoints[index + 1].x - waypoints[index].x;
    dy = waypoints[index + 1].y - waypoints[index].y;
  }
  //res.yaw = atan2(dy, dx);//yaw
  //计算曲率：设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
  res = atan2(dy, dx);
  return res;
}
