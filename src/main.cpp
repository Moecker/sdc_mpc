#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "json.hpp"

#include "auxiliary.h"
#include "coordinate_systems.h"
#include "mpc.h"

// For convenience
using json = nlohmann::json;

// Prototypes
string hasData(string s);
json Loop(vector<double> ptsx, vector<double> ptsy, double px, double py, double psi, double v, MPC& mpc);

int main()
{
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
        {
            string s = hasData(sdata);
            if (s != "")
            {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry")
                {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];

                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];

                    // Here we call the main loop which takes the vector of x and y point, the current vehicle position,
                    // speed and heading.
                    auto msg_json = Loop(ptsx, ptsy, px, py, psi, v, mpc);

                    auto msg = "42[\"steer\"," + msg_json.dump() + "]";
                    std::cout << msg << std::endl;

                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection(
        [&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}

json Loop(vector<double> ptsx, vector<double> ptsy, double px, double py, double psi, double v, MPC& mpc)
{
    // Collection for predicted (next) x and y values
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Transformation of received points from map coordinate system into local coordinate system. Rational is to only
    // work in local (vehicle) coordinate system to ease all computations.
    CoordinateSystems coordinate_system(px, py, psi);
    coordinate_system.Transform(ptsx, ptsy, next_x_vals, next_y_vals);

    // Fit the future reference points, using a polynomial as suggested in the class.
    // Requires std::vector -> Eigen transform
    Eigen::VectorXd reference_x = Eigen::Map<Eigen::VectorXd>(next_x_vals.data(), next_x_vals.size());
    Eigen::VectorXd reference_y = Eigen::Map<Eigen::VectorXd>(next_y_vals.data(), next_y_vals.size());

    // Fit received points as a 3rd order polynomial
    Eigen::VectorXd coeffs;
    coeffs = Polyfit(reference_x, reference_y, 3);

    // All posittion and heading related variables are zero as the vehicle coordinate system has its
    // origin here.
    double vehicle_px = 0;
    double vehicle_py = 0;
    double vehicle_phi = 0;

    // Move forward the projected distance in x-direction to cover 100 ms of latency
    vehicle_px += v * -100 / (3600);

    const auto kVelFactorToMS = 0.44704;
    double vehicle_v = v * kVelFactorToMS;

    Eigen::VectorXd state(MPC::kStateSize);
    state << vehicle_px, vehicle_py, vehicle_phi, vehicle_v;

    vector<double> result;
    result = mpc.Solve(state, coeffs);

    // Calculate steeering angle and throttle using MPC.
    // Both are in between [-1, 1].

    // Steer value of 1 corresponds to 25 degrees to the right which is negative, so we take the sign
    // into account
    double steer_value = -result[0] / Deg2Rad(25.0);
    double throttle_value = result[1] / kVelFactorToMS;

    json msg_json;
    msg_json["steering_angle"] = steer_value;
    msg_json["throttle"] = throttle_value;

    vector<double> mpc_x_vals;
    vector<double> mpc_y_vals;

    //.. add (x,y) points to list here, points are in reference to the
    // vehicle's coordinate system the points in the simulator are connected by a Green line
    mpc_x_vals.push_back(vehicle_px);
    mpc_y_vals.push_back(vehicle_py);

    for (int i = 0; i < result.size(); i += 2)
    {
        SimulateTimestep(vehicle_px, vehicle_py, vehicle_phi, vehicle_v, result[i], result[i + 1], MPC::dt, MPC::Lf);

        mpc_x_vals.push_back(vehicle_px);
        mpc_y_vals.push_back(vehicle_py);
    }

    msg_json["mpc_x"] = mpc_x_vals;
    msg_json["mpc_y"] = mpc_y_vals;

    // Displays the waypoints/reference line
    msg_json["next_x"] = next_x_vals;
    msg_json["next_y"] = next_y_vals;

    return msg_json;
}

string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}
