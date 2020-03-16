# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program Project

![](https://i.imgur.com/ew1YZaG.png)

If you want to learn more, please refer to my documentation:

1. Path Planning-Highway Driving project.md
2. Path Planning-search.md

### Goals

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```



## Step of path planning

## overview ##

![](https://i.imgur.com/tzEWA49.png)

Layers involved in path planning — from Udacity

path planning requires the cooperation of different layers of an autonomous vehicle. The diagram above provides an overview of how such components may be layered in a given self-driving system: Motion Control, Sensor Fusion, Localization, Prediction, Behaviour, Trajectory. 

## Trajectory planning ##

There are many approaches to trajectory generation, and in this project we have opted for computing trajectories in a Frenet coordinate system.


![](https://i.imgur.com/6vZRmQS.png)

Trajectories in Frenet (left) and normal Cartesian (right) coordinate systems

This is precisely what a Frenet coordinate system offers: in such a system we split our plane into a longitudinal and lateral axis, respectively denoted as S and D.

**Therefore, it is important for path planning to obtain the S and D values of future pah points.**

**1. Analyze the data from the sensor fusion and categorize other vehicles by lane.**

		// find lane number of each car.
		double findlane(double car_d) {
		    int lane_number = -1;
		    if (car_d > 0 && car_d < 4.0) {
		        lane_number = 0;
		    } else if (car_d > 4.0 && car_d < 8.0) {
		        lane_number = 1;
		    } else if (car_d > 8.0 && car_d < 12.0) {
		        lane_number = 2;
		    }
		    return lane_number;
		}

**2. Find the closest vehicle in each lane and record their longitudial and lateral distance and velocity.**

		for (int i = 0; i < sensor_fusion.size(); ++i) {
		                        auto fusion_data = sensor_fusion[i];
		                        double fusion_id = fusion_data[0];
		                        double fusion_vx = fusion_data[3];
		                        double fusion_vy = fusion_data[4];
		                        double fusion_s = fusion_data[5];
		                        int fusion_d = fusion_data[6];
		                        double fusion_speed = sqrt(fusion_vx * fusion_vy + fusion_vy * fusion_vy);
		
		                        double dist=fusion_s-car_s;// distance between my cars
		
		                        // find lane of other cars
		                        int others_lane = findlane(fusion_d);
		
		                        // is it on the same lane we are
		                        if (others_lane == my_lane) {// in my lane
		                            if( (fusion_s > car_s) && (fusion_s - car_s < 40)){// safe traffic-gap (0,40)
		                                car_head=true;
		                                if (fusion_speed) {
		                                    closest_front_vec = ms_to_mph(
		                                            fusion_speed); // need change unit because the speed in fusion data is m/s
		                                }
		
		                            }
		                            if( (fusion_s > car_s )&& (fusion_s - car_s < 15)){ // Emergency collision avoidance
		                                car_crash=true;
		                            }
		
		                            if (dist > 0) {
		                                closest_front_dist = min(dist,
		                                                         closest_front_dist); // find the minimal distance to the closest front car
		                            }
		                        } else if (others_lane == my_lane - 1) { // in left lane
		                            if( (car_s - 30 < fusion_s) && (car_s + 30 > fusion_s)){ // safe traffic-gap (-30,30)
		                                car_left= true;
		                            }
		
		                            if (dist > 0) {
		                                if (dist<closest_leftfront_dist){
		                                    closest_leftfront_dist=dist;
		                                    if(dist<10){
		                                        closest_left_d=car_d-fusion_d;// calculate the lateral distance between the two cars
		                                    }
		
		                                }


​		
		                            } else {
		                                closest_leftback_dist = min(abs(dist),
		                                                            closest_leftback_dist);// find the minimal distance to the closest rightback car
		
		                            }


​		
​		
		                        } else if (others_lane == my_lane + 1) {// in right lane
		                            if( (car_s - 30 < fusion_s) && (car_s + 30 > fusion_s)){// safe traffic-gap (-30,30)
		                                car_right= true;
		                            }
		                            if (dist > 0) {
		                                if (dist<closest_rightfront_dist){
		                                    closest_rightfront_dist=dist;
		                                    if(dist<10){
		                                        closest_right_d=fusion_d-car_d;// calculate the lateral distance between the two cars
		                                    }
		                                }
		
		                            } else {
		                                closest_rightback_dist = min(abs(dist),
		                                                             closest_rightback_dist);// find the minimal distance to the closest leftback car
		
		                            }
		
		                        }
		                    }
**3. Calculate cost of decisions and select the action with minimal cost.**

I thought of 4 possible decisions, my car could make at any instance:

1. Continue in my lane with max velocity
2. Change to the right
3. Change to the left lane
4. Continue in my lane but slow down to about the speed of the car in front

I decided to assign costs for all these different functions:

**Cost function — Continue in my lane**

- If there is traffic but closest car in front < buffer (ahead= true) then ->  cost = 500(cost_collision).
- If there is not closest traffic in my lane and my car stay in mid lane(lane=1), then -> cost = 50, otherwise -> cost=100 (cost=50, minimium cost is the best outcome).
- If the closest car in front is > 150 units then it is more safer to stay in this lane ->cost+=0, otherwise -> cost+=100.
- *In a straight line operation, it is safest to stay in the middle lane where there are no vehicles at a long distance in the front.*

	double keep_lane_cost(bool car_head, int my_lane, double closest_front_dist, int cost_collision) {
	    double cost=0;
	    if (car_head) {
	        cost = cost_collision;
	    } else {
	        if (my_lane == 1) {// Cars should try to stay in the middle of the road
	            cost = 50;
	        } else {
	            cost = 100;
	        }
	        if (closest_front_dist >= 150) {// If there are no cars within 150 m, the chance of a collision is very small
	            cost += 0;
	        } else {
	            cost += 100;
	        }
	    }
	
	    return cost;
	
	}

**Cost function — Change to left lane/right lane**

If lane in which we can turn doesn’t exist or closest car in side lane < buffer interval (car_right / car_left= true) then collision is likely hence cost increases to 500.

If lane exists and safe to make the turn then -> cost = 0.5*250(turn_cost), otherwise -> cost = 250.

(Safest to make a turn defined as closest car front is about distance unit of 150 away and closest car back is a distance unit of 30 away.In this situation, lane emptyish in front. I decided these numbers by playing around in the simulator)


	double turn_right_cost(bool car_right, int my_lane, double closest_rightfront_dist, double closest_rightback_dist,
	                       int cost_collision, int right_turn_cost) {
	    double cost=0;
	    if (car_right || my_lane == 2) { //Cars in lane 2 cannot change lanes to the right
	
	        cost = cost_collision;
	    } else {
	        if (closest_rightfront_dist > 150 && closest_rightback_dist > 30) {// more safer lane changing
	            cost = 0.5 * right_turn_cost;
	        } else {
	            cost = right_turn_cost;
	        }
	    }
	
	    return cost;
	}

**Cost Function — Slow down in my lane**

I set the cost of slow down = 200. This was done so that the action is less preferred than lane change but better than collision.
At every instance, the cost associated with all the decisions are calculated and the optimal decision is the one with the minimal cost.

	double slow_down(double slow_down_cost) {
	    double cost=0;
	    cost = slow_down_cost;
	    return cost;
	}

**4. Make decisions based on cost-function.**

caluculate cost of each of the four actions at each moment and find the lowest cost.


	    int decision = -1;
	    double min_cost = 1000;
	
	    for (int i = 0; i < costs.size(); ++i) {
	        double the_cost = costs[i];
	        if (the_cost < min_cost) {
	            min_cost = the_cost;
	            decision = i;
	        }
	    }
	
	    cout << "Min cost: " << min_cost << endl;
	
	    if (decision == 0) {
	        cout << "decision==0: Continue with max velocity " << endl;
	    } else if (decision == 1) {
	        cout << "decision==1: Change to right lane  " << endl;
	    } else if (decision == 2) {
	        cout << "decision=2: Change to left lane  " << endl;
	    } else if (decision == 3) {
	        cout << "decision==3: Continue with lower velocity " << endl;
	    }

**5. Once decision has been made, decide the target speed and lane for that decision.**

- One thing to be aware of is that when you're going straight, you need to be aware of other cars changing lanes and hitting our cars. So I determined the lateral distance between the two cars closest to my car in side lanes. 
- (Due to the lane width of 4m, the width of the vehicle is about 3m, and the safety threshold for the lateral distance between the two vehicles is at least 3/2 *2 +0.5= 3.5m)
- I set the maximum acceleration(0.224 mph) to reduce jerk. When going straight at maximum speed, my car will accelerate to maximum speed at maximum acceleration.
- When I continue in my lane but need slow down my car, the speed of my car decreased at maximum deceleration to about the speed of the car in front when my car is faster than that car. Otherwise the current speed will be maintained to follow the car in front.


        double speed_diff = 0; // speed difference for each step
        const double MAX_SPEED = 49.5; //mph
        const double MAX_ACC = .224; // how to calculate it?
    
        if ((decision == 0) or (decision == -1)) { // continue
            lane = my_lane;
            if (ref_vel < MAX_SPEED) {// accelerate
                speed_diff += MAX_ACC;
            }
            if (closest_left_d<3.5 ||closest_right_d<3.5){ // Avoid other cars that might hit our car when they change lanes(car width ~ 3 m)
                speed_diff -= MAX_ACC;
            }
        } else if (decision == 1) {// change right lane
            lane = my_lane + 1;
        } else if (decision == 2) {// change left lane
            lane = my_lane - 1;
        } else if (decision == 3) {// slow down
            lane = my_lane;
    
            if (car_crash){ // Emergency collision avoidance
                speed_diff -= MAX_ACC;
            }else{
                if (ref_vel - closest_front_vec>0) {// decelerate
                    speed_diff -= MAX_ACC;
                } else {
                    speed_diff -= 0.0;// if front car speed higher than my car, keep the car speed to follow the front car
                }
            }
    
        } else {
            cout << "Error" << endl;
        }


**6. Initialize the spline and output the location of the future path point.**


- Firstly I use 2 previous points and 3 furture points to initialize the spline. 
- Secondly I Output path points from previous path for continuity. 
- Thirdly Calculate target distance (y position) on 30 m ahead using spline.
- Fourthly divide target distance into N segements, calculate corresponding y position using spline. 
- Finally make conversation of coordination system from Frenet to  Cartesian and output furture point location to simulator.



## Next steps: ##

1. Different combinations of cost functions can make vehicles have different driving behaviors. At present, I have obtained a combination through a lot of tests in the simulator, but there may be collisions in some special cases.
2. In some cases the vehicle ahead was not identified successfully.
3. How can vehicles predict the behavior of vehicles in other lanes to better prevent collisions?

