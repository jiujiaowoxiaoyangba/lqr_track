#include <iostream>
#include <vector>
#include "LQR_track/trajectory.h"
#include <math.h>
using namespace std;
double dt = 0.01;//轨迹计算频率

void trajectory::line()
{
  waypoint PP;
  for (int i = 0; i < limit_x/dt; i++)
  {
    PP.ID = i;
    PP.x  = x_start + dt * i;   //x:10米
    PP.y  = y_start;            //y
    waypoints.push_back(PP);
  }
}

void trajectory::wave1()
{
  waypoint PP;
  for (int i = 0; i < limit_x/dt; i++)
  {
    PP.ID = i;
    PP.x  = x_start + dt * i;                                        //x
    PP.y  = y_start + 1.0 * sin(dt*i / 1.5) + 0.5 * cos(dt*i / 1.0); //y
    waypoints.push_back(PP);
  }
}

void trajectory::wave2()
{
  waypoint PP;
  for (int i = 0; i < limit_x/dt; i++)
  {
    PP.ID = i;
    PP.x  = x_start + dt * i;                //x
    PP.y  = y_start - 0.2 * sin(dt*i / 0.4); //y
    waypoints.push_back(PP);
  }
}

//write the path you design
/*void trajectory::custom_path(){
        waypoint PP;
        for (int i = 0; i < limit_x/dt; i++)
        {
                PP.ID = i;
                PP.x = ...;//x
                PP.y = ...;//y
                waypoints.push_back(PP);
        }
   }*/

void trajectory::refer_path()
{
  if(trajectory_type == "wave1")
    wave1();
  else if(trajectory_type == "line")
    line();
  else if(trajectory_type == "wave2")
    wave2();
  //else if(trajectory_type == "custom_path")custom_path();//set the index


  //计算切线方向并储存
  for (int j = 0; j<waypoints.size(); j++)
  {
    double dx, dy, yaw;
    if (j == 0)
    {
      dx = waypoints[1].x - waypoints[0].x;
      dy = waypoints[1].y - waypoints[0].y;
    }
    else if (j == (waypoints.size() - 1))
    {
      dx = waypoints[j].x - waypoints[j - 1].x;
      dy = waypoints[j].y - waypoints[j - 1].y;
    }
    else
    {
      dx = waypoints[j + 1].x - waypoints[j].x;
      dy = waypoints[j + 1].y - waypoints[j].y;
    }
    yaw              = atan2(dy, dx);//yaw
    waypoints[j].yaw = yaw;
  }
}

vector<waypoint> trajectory::get_path()
{
  return waypoints;
}

void trajectory::set_path(vector<waypoint> s_path)
{
  waypoints.clear();
  for(int i = 0; i<s_path.size(); i++)
  {
    waypoints.push_back(s_path[i]);
  }
}
