#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <functional>
#include <ctime>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

using namespace std::placeholders;

static void MessageHandler(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode, MPC& mpc, int starttime)
{
  // "42" at the start of the message means there's a websocket message event.
  // The 4 signifies a websocket message
  // The 2 signifies a websocket event
  string sdata = string(data).substr(0, length);
  static time_t prev_telemetery_time = 0;
  static double prev_steering_angle = 0.0;
  static double prev_throttle = 0.0;
  const double default_latency = 0.1;

//  cout << sdata << endl;
  if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
    string s = hasData(sdata);
    if (s != "") {
      auto j = json::parse(s);
      string event = j[0].get<string>();
      if (event == "telemetry") {
        // j[1] is the data JSON object
        vector<double> ptsx = j[1]["ptsx"];
        vector<double> ptsy = j[1]["ptsy"];
        assert(ptsx.size() == ptsy.size());
        double px = j[1]["x"];
        double py = j[1]["y"];
        double psi = j[1]["psi"];
        double v = j[1]["speed"];

        cout << "IN: " << ((clock() - starttime) / double(CLOCKS_PER_SEC)) << " px="<<px<<" py="<<py<<" v="<<v<<" psi="<<psi << endl;

        // adjust start state for latency
        double latency = prev_telemetery_time==0 ? default_latency : ((clock() - prev_telemetery_time) / double(CLOCKS_PER_SEC)) ;
        if (true and fabs(prev_steering_angle) > 0.001) { // lame approximation
          latency = 0.1;
          px = px + v * (1609. / 3600.) * cos(psi) * latency;
          py = py + v * (1609. / 3600.) * sin(psi) * latency;
          for (int i = 0; i < ptsx.size(); i++) {
            ptsx[i] = ptsx[i] + v * (1609. / 3600.) * cos(psi) * latency;
            ptsy[i] = ptsy[i] + v * (1609. / 3600.) * sin(psi) * latency;
          }
          const double Lf = 2.67;
          psi = psi - v * deg2rad(prev_steering_angle * 25.) / Lf * latency;
          v = v + prev_throttle * 5.0 * latency;
        }
        else if (true and v>0.0001) { // bicycle model equations from EKF lectures
          latency = 0.1;
          const double Lf = 2.67;
          double psidot = - v * deg2rad(prev_steering_angle * 25.) / Lf;
          const double conv_factor = (1609. / 3600.);
          px = px + (v / psidot) * conv_factor * (sin(psi + psidot * latency) - sin(psi)) + 0.5 * latency * latency * cos(psi) * prev_throttle * 5.0;
          py = py + (v / psidot) * conv_factor * (-cos(psi + psidot * latency) + cos(psi)) + 0.5 * latency * latency * sin(psi) * prev_throttle * 5.0;
          for (int i = 0; i < ptsx.size(); i++) {
            ptsx[i] = ptsx[i] + (v / psidot) * conv_factor * (sin(psi + psidot * latency) - sin(psi)) + 0.5 * latency * latency * cos(psi) * prev_throttle * 5.0;
            ptsy[i] = ptsy[i] + (v / psidot) * conv_factor * (-cos(psi + psidot * latency) + cos(psi)) + 0.5 * latency * latency * sin(psi) * prev_throttle * 5.0;
          }
          psi = psi + psidot * latency;
          v = v + prev_throttle * 5.0 * latency;
          cout << px << " " << py << " " << psi << " " << v << endl;
        }

        // convert waypoints to car coordinates
        Eigen::VectorXd ptsx_car(ptsx.size());
        Eigen::VectorXd ptsy_car(ptsy.size());
        vector<double> v_ptsx_car(ptsx.size());
        vector<double> v_ptsy_car(ptsy.size());
        for (int i=0; i<ptsx.size(); i++)
        {
          ptsx_car[i] = (ptsx[i]-px) * cos(psi) + (ptsy[i]-py) * sin(psi);
          ptsy_car[i] = -(ptsx[i]-px) * sin(psi) + (ptsy[i]-py) * cos(psi);

          v_ptsx_car[i] = ptsx_car[i];
          v_ptsy_car[i] = ptsy_car[i];
        }

        // fit polynomial
        int order = 3;
        Eigen::VectorXd coeff = polyfit(ptsx_car, ptsy_car, order);

        /*
        * TODO: Calculate steeering angle and throttle using MPC.
        *
        * Both are in between [-1, 1].
        *
        */
        Eigen::VectorXd x0(6);
        x0[0] = 0.0; // x, in unity units which look like meters
        x0[1] = 0.0; // y, meters
        x0[2] = 0.0; // psi, radians -- car points along x axis
        x0[3] = v*1609/3600; // speed, m/s. 1mph=1609m/3600s
        x0[4] = polyeval(coeff, 0.0); // cross track error, meters. is simply y(0.0) as car is in the center of coordinates
        x0[5] = -atan(coeff[1]); // heading error, radians. this is simply angle of f(x) wrt x axis at zero. arctan(f'(0)). f=ax^3+bx^2+cx+d. f'(0)=c
        // assume throttle of 1 is 5m/s2 acceleration

        double steer_value = -0.02;
        double throttle_value = 0.1;

        //Display the MPC predicted trajectory
        vector<double> mpc_x_vals;
        vector<double> mpc_y_vals;

        bool status = false;
        vector<double> solution = mpc.Solve(x0, coeff, mpc_x_vals, mpc_y_vals, status);

        if (status) {
          steer_value = solution[0] / deg2rad(25); // already in radians
          throttle_value = solution[1] / 5.0;
        } else {
          steer_value = prev_steering_angle;
          throttle_value = prev_throttle;
        }

        json msgJson;
        msgJson["steering_angle"] = steer_value;
        msgJson["throttle"] = throttle_value;

        //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
        // the points in the simulator are connected by a Green line
        msgJson["mpc_x"] = mpc_x_vals;
        msgJson["mpc_y"] = mpc_y_vals;

        //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
        // the points in the simulator are connected by a Yellow line
        msgJson["next_x"] = v_ptsx_car;
        msgJson["next_y"] = v_ptsy_car;

        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//        std::cout << msg << std::endl;

        prev_telemetery_time = clock();
        prev_steering_angle = steer_value;
        prev_throttle = throttle_value;

        // Latency
        // The purpose is to mimic real driving conditions where
        // the car does actuate the commands instantly.
        //
        // Feel free to play around with this value but should be to drive
        // around the track with 100ms latency.
        //
        // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
        // SUBMITTING.
        cout << "pre-latency: " << ((clock() - starttime) / double(CLOCKS_PER_SEC)) << endl;
        auto start = chrono::high_resolution_clock::now();

        this_thread::sleep_for(chrono::milliseconds(100));

        cout << chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start).count() << "ms\n";
        cout << "OUT: " << ((clock() - starttime) / double(CLOCKS_PER_SEC)) << " st="<<steer_value<<" thrt="<<throttle_value  << endl;

        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } else {
      // Manual driving
      std::string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
  }
}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  // Start timer.
  int starttime = clock();

//  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
//                     uWS::OpCode opCode) {});
  h.onMessage(std::bind(MessageHandler, _1, _2, _3, _4, mpc, starttime));

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
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
