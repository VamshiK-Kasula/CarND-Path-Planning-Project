
#ifndef LANE_PREDICTION_H_
#define LANE_PREDICTION_H_

#include <cmath>
#include <vector>
#include "helpers.h"

Lanes LanePrediction(double ego_d, double ego_s, double &ego_velocity, const std::vector<std::vector<double>> &sensor_fusion) 
{
  Lanes next_lane{false, false, false, false};
  Lanes current_lane{false, false, false};
  double target_speed = SPEED_LIMIT;
  double target_s = SAFE_DISTANCE;

  if (ego_d < 4.0) 
  {
    next_lane.left_lane = true;
  } 
  else if (ego_d > 8.0) 
  {
    next_lane.right_lane = true;
  }
  for (auto &object : sensor_fusion) 
  {
    double vx = object[3];
    double vy = object[4];
    double v_object = sqrt(vx * vx + vy * vy);
    double relative_object_s = object[5] - ego_s;
    double relative_object_d = object[6] - ego_d;

    if (relative_object_s >= 0.0) 
    {
      if ((v_object < ego_velocity) && (relative_object_s < SAFE_DISTANCE)) 
      {
        if ((abs(relative_object_d) < 2.0))
        {
          next_lane.too_close = true;
          next_lane.center_lane = true;
          target_speed = std::min(target_speed, v_object);
          target_s = std::min(SAFE_DISTANCE, relative_object_s);
        }
        else if ((relative_object_d > 2.0) && (relative_object_d <= 6.0)) 
        {
          next_lane.right_lane = true;
        }
        else if ((relative_object_d < -2.0) && (relative_object_d >= -6.0)) 
        {
          next_lane.left_lane = true;
        } 
      }
    }
    else if (((v_object >= ego_velocity) && (relative_object_s > -1.5 * SAFE_DISTANCE)) || (relative_object_s > -1.0 * SAFE_DISTANCE)) 
    {
      if ((relative_object_d <= -2.0) && (relative_object_d >= -6.0)) 
      {
        next_lane.left_lane = true;
      } 
      else if ((relative_object_d >= 2.0) && (relative_object_d <= 6.0)) {
        next_lane.right_lane = true;
      }
    }
  }
  
  return next_lane;
}

#endif