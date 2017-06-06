#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}
double deg2rad(double x)
{
    return x * pi() / 180;
}
double rad2deg(double x)
{
    return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

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
        std::cout << sdata << endl;
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

                    //                    Eigen::Map<Eigen::VectorXd> all_x(&ptsx[0], ptsx.size());
                    //                    Eigen::Map<Eigen::VectorXd> all_y(&ptsy[0], ptsy.size());

                    Eigen::VectorXd all_x(6);
                    all_x << ptsx[0], ptsx[1], ptsx[2], ptsx[3], ptsx[4], ptsx[5];
                    Eigen::VectorXd all_y(6);
                    all_y << ptsy[0], ptsy[1], ptsy[2], ptsy[3], ptsy[4], ptsy[5];

                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];

                    std::cout << "Polyfitting ..." << endl;
                    Eigen::VectorXd coeffs = polyfit(all_x, all_y, 1);
                    std::cout << "Coeffs are: \n" << coeffs << endl;

                    //                    auto f_of_x = polyeval(coeffs, px);

                    //                    Eigen::VectorXd coeffs_prime(3);
                    //                    coeffs_prime[0] = 3 * coeffs[0];
                    //                    coeffs_prime[1] = 2 * coeffs[1];
                    //                    coeffs_prime[2] = 1 * coeffs[2];

                    //                    auto f_prime_of_x = polyeval(coeffs_prime, px);
                    //                    auto psi_dest = atan(f_prime_of_x);

                    //                    auto e_psi = psi - psi_dest;

                    // TODO: calculate the cross track error
                    //                    double cte_own = f_of_x - py + (v * sin(e_psi) * MPC::dt);
                    double cte = polyeval(coeffs, px) - py;

                    // TODO: calculate the orientation error
                    //                    auto delta = 0.0;
                    //                    double epsi_own = psi - psi_dest + (v / MPC::Lf * delta * MPC::dt);
                    double epsi = psi - atan(coeffs[1]);

                    Eigen::VectorXd state(MPC::kStateVectorSize);
                    state << px, py, psi, v, cte, epsi;

                    // TODO: Calculate steeering angle and throttle using MPC.
                    std::cout << "Solving ..." << endl;
                    vector<double> solution = mpc.Solve(state, coeffs);
                    std::cout << "Solved with solution: " << solution.size() << endl;

                    //                    Eigen::VectorXd next_state(MPC::kStateVectorSize);
                    //                    auto dt = 0.1;
                    //                    next_state[0] = px + v * cos(psi) * dt;
                    //                    next_state[1] = py + v * sin(psi) * dt;
                    //                    next_state[2] = psi + (v / Lf) * delta * dt;
                    //                    next_state[3] = v + a * dt;
                    //                    next_state[4] = 0.0;
                    //                    next_state[5] = 0.0;

                    std::cout << "s[0] = " << solution[0] << std::endl;
                    std::cout << "s[1] = " << solution[1] << std::endl;
                    std::cout << "s[2] = " << solution[2] << std::endl;
                    std::cout << "s[3] = " << solution[3] << std::endl;
                    std::cout << "s[4] = " << solution[4] << std::endl;
                    std::cout << "s[5] = " << solution[5] << std::endl;
                    std::cout << "s[6] = " << solution[6] << std::endl;
                    std::cout << "s[7] = " << solution[7] << std::endl;

                    double steer_value = solution[6];
                    double throttle_value = solution[7];

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    std::cout << "Steering Angle: " << steer_value << endl;
                    std::cout << "Throttle: " << throttle_value << endl;

                    // Display the MPC predicted trajectory
                    for_each(ptsx.begin(), ptsx.end(), [px](double& p) { p = p - px; });
                    for_each(ptsy.begin(), ptsy.end(), [py](double& p) { p = p - py; });
                    vector<double> mpc_x_vals = ptsx;
                    vector<double> mpc_y_vals = ptsy;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    // Display the waypoints/reference line

                    std::cout << "ptsx[0] = " << ptsx[0] << std::endl;
                    std::cout << "ptsx[1] = " << ptsx[1] << std::endl;

                    std::cout << "ptsy[0] = " << ptsy[0] << std::endl;
                    std::cout << "ptsy[1] = " << ptsy[1] << std::endl;

                    vector<double> next_x_vals = ptsx;
                    vector<double> next_y_vals = ptsy;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    // std::cout << msg << std::endl;

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
    // program
    // doesn't compile :-(
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
