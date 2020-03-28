# CarND-Path-Planning-Project

[gif_smooth_start]: ./output/smooth_start.gif "Smooth start"
[gif_lane_change]: ./output/lane_change.gif "Lane change"
[gif_deccelerate]: ./output/decelerate.gif "Deceleration"
[gif_speed_oscillate]: ./output/speed_oscillate.gif "speed oscillation"


Goal of the project is to safely navigate through the traffic without collisions and changing the lane when possible. Also the rules such as maximum speed and acceleration and jerk are followed

```
speed limit = 50 mph
max acceleration = 10 m/s^2
max jerk = 10 m/s^3
```

The project can be divided into following sub-goals

* Trajectory generation
* Speed control
* Change lane

### Trajectory generation

As discussed in the course, a path of 50 points which include the previous and future points from the map. Spline tool is used to generate a smooth transition between these points.

``` c++
for (int i = 0; i < ptsx.size(); i++) 
{
double shift_x = ptsx[i] - ref_x;
double shift_y = ptsy[i] - ref_y;
ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
}

tk::spline s;

s.set_points(ptsx, ptsy);
```

### Speed Control

Ego speed should not exceed 50 mph at any point of time and the speed should be decreased when the other car is present in front of the ego and the speed can be increased when the lane is free.

In our project the maximum speed is defined as 49.5 mph

``` c++
#define SPEED_LIMIT 49.50
```

and the minimum spacing between the ego and the other car in front is selected as 30 meters

``` c++
#define SAFE_DISTANCE 30.0
```

The speed of the ego is updated as follows
``` c++
//update_velocity
if (predict_next_lane.too_close)
{
ref_vel  -= 0.224; 
}
else if (ref_vel < 49.5)
{
ref_vel +=0.224;
}
```

* If the lane is free and the ego speed is less than max speed, ego speed is increased by 0.224 mph.
* If there is a car present in front of the ego less than a safe distance, ego speed is decreased by 0.224 mph.

When the simulation is started, the ego velocity is set to 0 mph at the start to avoid jerk and start the car smoothly

![alt text][gif_smooth_start]

As the car in front of the ego is moving slowly and as the ego approaches the car in front, the velocity of ego car is reduced to maintain the safe distance.

![alt text][gif_deccelerate]


### Lane

In every iteration, the data from the sensor fusion is used to determine the next lane of the ego

``` c+++
auto sensor_fusion = j[1]["sensor_fusion"];
```
All the relative positions of the other cars on the road with respect to ego are calculated. If a car is present in front of the ego, a change of lane is required if not lane is maintained. If lane change is needed, position and velocity of the cars in the neighbouring lanes are considered if a possible lane change can be made.

If lane change is possible, the d parameter of the ego is updated to the respective lane.

![alt text][gif_lane_change]

### Improvements

* Handling sudden lane changes by other cars are not considered
* Handling sudden change in velocity by other cars
* If there is a car in front of the ego, Ego speed oscillates between the maximum speed and reduces when approaching the car in front. A constant speed while maintaining the distance is desired until a lane change is possible
* Far future prediction to change to appropriate lane which has better average velocity instead of changing the lane based on only present situation

![alt text][gif_speed_oscillate]
