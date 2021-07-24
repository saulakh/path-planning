## Highway Driving
Self Driving Car Nanodegree Project

### Project Overview

For this project, localization and sensor fusion data are provided, and a map list of waypoints along the left edge of the highway. To meet the project requirements, the car needs to drive around the highway track, avoid collisions with other vehicles, stay within its lane, drive below the speed limit of 50 mph, stay below the max acceleration limit of 10 m/s^2, and stay below the max jerk limit of 10 m/s^3. In addition, the car needs to be able to change lanes to drive around slower cars when possible. 

The goal of this project is to generate a list of (x,y) path points for the car to follow, and the set of points in the path planner list will update dynamically based on changes in the car's environment. The car drives with a perfect controller, visiting the next (x,y) point every 0.02 seconds. The (x,y) points are in meters, and the speed of the car depends on the spacing of the waypoints, and the angle of the car comes from the vector going from one point to the next in the path planner list.

### Project Build Instructions

This project uses the Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2), and the original project repository from Udacity can be found [here](https://github.com/udacity/CarND-Path-Planning-Project).

The main program can be built and run by doing the following from the project top directory:

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./path_planning

### Data Provided

##### Map Waypoints

The list of map waypoints can be found in _highway_map.txt_, located in the ```data``` folder.

Each waypoint in the list contains [x,y,s,dx,dy] values. The x and y values are the waypoint's position in map coordinates, the s value is the distance along the road to get to that waypoint in meters, and the dx and dy values define the unit normal vector pointing outward of the highway loop. The s values loop around the highway, with values ranging from 0 to 6945.554 m.

##### Localization Data (No Noise)

The localization data provided includes the car's x and y position in map coordinates, the car's s and d position in Frenet coordinates, the car's yaw angle in the map, and the car's speed (in mph). The corresponding variables are [car_x, car_y, car_s, car_d, car_yaw, and car_speed]. 

##### Sensor Fusion Data (No Noise)

The sensor fusion data is provided as a 2D vector of cars on the same side of the road, along with their id, x and y position in map coordinates, velocity in x and y directions (in m/s), and the car's s and d position in Frenet coordinates. The data format for each car is: [id, x, y, vx, vy, s, d].

The (s,d) Frenet coordinates are used throughout this project, due to simpler calculations. The s value is the car's distance traveled along the path of the road, and the d value is the lateral direction across the road. The d value begins with 0 on the left side of the left lane, and each lane is 4 m across. This information is used to detect which lane the other cars are in, and their position. The vx and vy values are used to calculate the speed of the other cars.

![image](https://user-images.githubusercontent.com/74683142/126800209-dd65af60-a986-49d0-b024-9319f0228f0e.png)

The _helpers.h_ header file is located in the ```src ``` folder, and contains several helper functions. These functions include getXY and getFrenet, which are used to convert position values between the Cartesian coordinate system and the Frenet coordinate system.

### Project Code

The project file is [main.cpp](https://github.com/saulakh/path-planning/blob/main/src/main.cpp), located in the ```src ``` folder. The three main sections are perception, behavior planning, and path planning.

##### Perception

This section uses sensor_fusion data to check the position and velocity of other cars on the road, with several booleans as lane checks. The checks include whether a car ahead is getting too close, and whether the left and right lanes are clear.

If a car ahead is driving in the same lane, the too_close flag is set as true when a car is within 15 m. The car_ahead flag is set as true for cars within 25 m. Additionally, the code checks lanes to the left and right of the ego vehicle, and flags the lane as clear if there is at least 20 m of space. Here is a section of the code used for perception:

```
// Check if car is in the same lane
if (check_car_lane == lane) {

   // Check if car ahead is within 15 m
   if ((check_car_s > car_s) && (check_car_s - car_s < 15)) {
      too_close = true;
   }
   
   // Check if car ahead is within 25 m
   if ((check_car_s > car_s) && (check_car_s - car_s < 25)) {
      car_ahead = true;
      car_ahead_speed = 2.24 * check_car_speed; // convert m/s to mph
   }
}

// Check if left lane is clear
if ((check_car_lane + 1 == lane) && (fabs(car_s - check_car_s) < 20)) {
   car_left = true; // false when lane is clear
   car_left_speed = 2.24 * check_car_speed; // convert m/s to mph
}

// Check if right lane is clear
if ((check_car_lane - 1 == lane) && (fabs(car_s - check_car_s) < 20)) {
   car_right = true; // false when lane is clear
   car_right_speed = 2.24 * check_car_speed; // convert m/s to mph
}
```

##### Behavior Planning

This section uses the lane checks from perception, and decides how to maneuver around other cars. If there is a slower car ahead, the ego vehicle slows down and checks if there is a safe lane change available.

If the car ahead is within 15 m, the car decelerates at -0.3 mph/s, which is still below the max acceleration limit. This helps in situations where a car suddenly changes into the same lane or slows down. If a car is within 15 - 25 m ahead, the car decelerates at a rate of -0.224 mph/s instead. If the lane is clear, the car accelerates until reaching the target velocity.

```
// Slow down if car ahead is too close
if (car_ahead == true) {
   if (too_close == true) {
      target_vel -= 0.3; // Still below 10 m/s^2
   }
   // Otherwise drive slightly slower than car ahead
   else if (target_vel > car_ahead_speed * 0.9) { // buffer speed
      target_vel -= max_acc;
   }
   // If left lane is clear, move left
   if (car_left == false && lane > 0) {
      lane--;
   }
   // Otherwise, move right if right lane is clear
   else if (car_right == false && lane < 2) {
      lane++;
   }
}
// If there is no car ahead, speed up to target velocity
else if (target_vel < max_speed) {
   target_vel += max_acc;
}
```

##### Path Planning

This section generates the waypoints for the car to follow, using the spline from the project walkthrough. The path starts with two points from the end of the previous path to ensure continuity and a smooth transition. Three new points are generated using the road's s and d values, spaced 30 m ahead from the car's s position, and d values based on the current set lane. The spline generates the points in between, and continues to create a smooth path for the vehicle as new points are added.

The outputs from the behavior planning section are used to update the lane and target velocity, and these changes are integrated into the spline path planner. The lane variable is updated during lane changes, and the d value is set in reference to the lane value. The spline incorporates this lane change when generating new waypoints, since the getXY function uses (s, d, map_s, map_x, map_y) as inputs:

```
// Create waypoints spaced further apart
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

If the target velocity changes, the spacing between waypoints changes to match the new velocity, since the car reaches the next waypoint every 0.02 seconds: 

```
// Fill up the remaining points for path planning list
for (int i = 1; i <= 50-previous_path_x.size(); i++) {
   double N = target_dist / (0.02 * target_vel / 2.24); // velocity converted to m/s
   double x_point = x_add_on + target_x / N;
   double y_point = s(x_point);
```

The spline.h header file used for creating smooth trajectories can be found here: https://kluge.in-chemnitz.de/opensource/spline/

### Project Discussion

I started off playing around with the code and focusing on understanding the project. At this stage, I ran into several problems including driving off the road, teleporting across lanes without continuity between waypoints, colliding with other cars from unsafe lane changes, exceeding max acceleration and jerk, etc. I worked through one problem at a time and learned more with each solution. These were some of my initial failed attempts:
   
![image](https://user-images.githubusercontent.com/74683142/126872848-940a4e29-4fde-44bf-bed2-83c7e0e2280e.png) ![image](https://user-images.githubusercontent.com/74683142/126872851-2841cbb5-6a23-4c2a-bbf5-61d635aed6cd.png)

After resolving these issues, the car was able to drive over 10 miles without an incident several times, which was enough for this project. 

I continued running the simulation, and found a few other areas for improvement. Occasionally, the ego vehicle starts to change lanes when the car ahead slows down, so the ego car comes back to its original lane. In this situation, sometimes the car swerves or attempts double lane changes, and triggers the 3 second outside of lane warning. To resolve this issue, I could add a flag for the car to wait a few seconds before attempting another lane change. 

![image](https://user-images.githubusercontent.com/74683142/126872767-0156db76-2eca-43a5-ba87-ce12821fa434.png)

There are also situations when a car gets stuck behind a slower car in the right or left lane. To resolve this issue, I could add a preference for the ego vehicle to stay in the center lane whenever possible.

![image](https://user-images.githubusercontent.com/74683142/126872755-d9eceb55-9604-47bd-b3e3-e29e67f3fc57.png)

### Project Results

The car was able to drive over 15 miles without any incidents and meet all the project requirements. The car stayed below the max speed limit, acceleration, and jerk values, did not have any collisions, stayed in its lane, and maneuvered around other vehicles on the road successfully. 

![image](https://user-images.githubusercontent.com/74683142/126872836-0eb8e76e-f7c2-454f-8d71-ef4a18e44111.png)