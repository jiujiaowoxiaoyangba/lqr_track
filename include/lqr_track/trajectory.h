#include <iostream>
#include <vector>
#include "LQR_track/Tool.h"
#include <string>
using namespace std;

class trajectory {
private:
  vector<waypoint> waypoints;
  double           x_start, y_start, limit_x, limit_y;
  string           trajectory_type;
public:
  trajectory(double initial_x_, double initial_y_, string type_, double limit_x_, double limit_y_)
    : x_start(initial_x_)
    , y_start(initial_y_)
    , trajectory_type(type_)
    , limit_x(limit_x_)
    , limit_y(limit_y_)
  {
  };
  //set reference trajectory
  void             refer_path();
  vector<waypoint> get_path();
  void             set_path(vector<waypoint> s_path);
  //If you need to add a custom path:1.please add the function; 2.Overwrite trajectory.cpp synchronously
  //void custom_path();
  void line();
  void wave1();
  void wave2();
};
