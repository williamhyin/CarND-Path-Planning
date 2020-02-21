#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <math.h>

using namespace std;

// for convenience
using json = nlohmann::json;

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

// unit conversion: ms/mph=1600.0/3600.0
double mph_to_ms(double mph) {
    return mph * (1600.0 / 3600.0);
}

double ms_to_mph(double ms) {
    return ms * (3600.0 / 1600.0);
}

// define cost function for each action

/*
 * car_head: Determine if there is a vehicle within 30m ahead of the vehicle
 * my_lane: which lane
 * closest_front_dist： distance between the cloesest front car
 * cost_collision: cost for possible collision
 * car_right: Determine if there is a vehicle within 30m in the right lane of my car
 * closest_rightfront_dist: distance between my car to cloesest right-front vehicle
 * closest_rightback_dist: distance between my car to cloesest right-back vehicle
 * right_turn_cost: cost for change to right lane
 * left_turn_cost: cost for change to left lane
 * slow_down_cost: cost for change to slow down
 */
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

double turn_left_cost(bool car_left, int my_lane, double closest_leftfront_dist, double closest_leftback_dist,
                      int cost_collision, int left_turn_cost) {
    double cost=0;
    if (car_left || my_lane == 0) { //Cars in lane 0 cannot change lanes to the left
        cost = cost_collision;
    } else {
        if (closest_leftfront_dist > 150 && closest_leftback_dist > 30) {// more safer lane changing
            cost = 0.5 * left_turn_cost;
        } else {
            cost = left_turn_cost;
        }
    }

    return cost;
}


double slow_down(double slow_down_cost) {
    double cost=0;
    cost = slow_down_cost;
    return cost;
}

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;


    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    int lane = 1;//  Initialization of lane
    double ref_vel = 0.0; // mph

    h.onMessage([&ref_vel, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                        &map_waypoints_dx, &map_waypoints_dy, &max_s]
                        (uWS::WebSocket <uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // previous_path points size
                    int prev_size = previous_path_x.size();

                    // Preventing collitions.
                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }
                    // find lane of my car
                    int my_lane = findlane(car_d);

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.

                    auto sensor_fusion = j[1]["sensor_fusion"];

                    // Initialization of distance and velocity
                    double closest_front_dist = 999;
                    double closest_front_vec = 0; // velocity of front car
                    double closest_leftfront_dist = 999;
                    double closest_leftback_dist = 999;
                    double closest_rightfront_dist = 999;
                    double closest_rightback_dist = 999;
                    double closest_right_d=999;
                    double closest_left_d=999;

                    bool car_head = false;
                    bool car_left = false;
                    bool car_right = false;
                    bool car_crash = false;



                    for (int i = 0; i < sensor_fusion.size(); ++i) {
                        auto fusion_data = sensor_fusion[i];
                        double fusion_id = fusion_data[0];
                        double fusion_vx = fusion_data[3];
                        double fusion_vy = fusion_data[4];
                        double fusion_s = fusion_data[5];
                        int fusion_d = fusion_data[6];
                        double fusion_speed = sqrt(fusion_vx * fusion_vy + fusion_vy * fusion_vy);
//                        fusion_s+= ((double)prev_size*0.02*fusion_speed); // Does this work?

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
//                                closest_leftfront_dist = min(dist,
//                                                             closest_leftfront_dist);// find the minimal distance to the closest rightfront car

                            } else {
                                closest_leftback_dist = min(abs(dist),
                                                            closest_leftback_dist);// find the minimal distance to the closest rightback car

                            }



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
//                                closest_rightfront_dist = min(dist,
//                                                              closest_rightfront_dist);// find the minimal distance to the closest leftfront car

                            } else {
                                closest_rightback_dist = min(abs(dist),
                                                             closest_rightback_dist);// find the minimal distance to the closest leftback car

                            }

                        }
                    }

                    cout<<endl;
                    cout<< "Sensor element size : "<<sensor_fusion.size()<<endl;
                    cout << "mylane: " << my_lane << endl;
                    cout << "closest_car_front_dist: " << closest_front_dist << endl;
                    cout << "closest_front_vec: " << closest_front_vec << endl;
                    cout << "closest_leftfront_dist: " << closest_leftfront_dist << endl;
                    cout << "closest_leftfback_dist: " << closest_leftback_dist << endl;
                    cout << "closest_rightfront_dist: " << closest_rightfront_dist << endl;
                    cout << "closest_rightback_dist: " << closest_rightback_dist << endl;
                    cout << "closest_left_d: " << closest_left_d << endl;
                    cout << "closest_right_d: " << closest_right_d << endl;

                    // calculate cost of each options and choose the minimum cost.

                    double keep_lane_c;
                    double turn_right_c;
                    double turn_left_c;
                    double slow_down_c;
                    double cost_collision = 500;
                    double right_turn_cost = 250;
                    double left_turn_cost = 250;
                    double slow_down_cost = 300;

                    vector<double> costs;

                    keep_lane_c = keep_lane_cost(car_head, my_lane, closest_front_dist, cost_collision);
                    costs.push_back(keep_lane_c);
                    turn_right_c = turn_right_cost(car_right, my_lane, closest_rightfront_dist, closest_rightback_dist,
                                                   cost_collision, right_turn_cost);
                    costs.push_back(turn_right_c);
                    turn_left_c = turn_left_cost(car_left, my_lane, closest_leftfront_dist, closest_leftback_dist,
                                                 cost_collision, left_turn_cost);
                    costs.push_back(turn_left_c);
                    slow_down_c = slow_down(slow_down_cost);
                    costs.push_back(slow_down_c);

                    cout << "Cost keep lane: " << keep_lane_c << endl;
                    cout << "Cost slow down: " << slow_down_c << endl;
                    cout << "Cost left turn: " << turn_left_c << endl;
                    cout << "Cost Right turn: " << turn_right_c << endl;

                    // make decision with the minimum cost.

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

                    // trajectory planning
                    vector<double> ptsx;
                    vector<double> ptsy;

                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    // Do I have have previous points
                    if (prev_size < 2) {
                        // if not too many previous points
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    } else {
                        // Use the last two points.
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    // Setting up three target points in the future.
                    vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x,
                                                    map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x,
                                                    map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x,
                                                    map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    // Making coordinates to local car coordinates.
                    for (int i = 0; i < ptsx.size(); i++) {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                    }

                    // Create the spline.
                    tk::spline s;
                    s.set_points(ptsx, ptsy); // using 2 previous points and 3 furture points to initialize the spline

                    // Output path points from previous path for continuity.
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    for (int i = 0; i < prev_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // Calculate distance y position on 30 m ahead.
                    double target_x = 30.0;
                    double target_y = s(target_x);
                    double target_dist = sqrt(target_x * target_x + target_y * target_y);

                    double x_add_on = 0;

                    for (int i = 1; i < 50 - prev_size; i++) {
                        ref_vel += speed_diff;
                        if (ref_vel > MAX_SPEED) {// dont exceed the maximum speed
                            ref_vel = MAX_SPEED;
                        } else if (ref_vel < MAX_ACC) {
                            ref_vel = MAX_ACC;
                        }
                        double N = target_dist / (0.02 * ref_vel / 2.24); // divided into N segments
                        double x_point = x_add_on + target_x / N;
                        double y_point = s(x_point); // To calculate distance y position .

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);


                    }
                    json msgJson;

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket <uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket <uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}