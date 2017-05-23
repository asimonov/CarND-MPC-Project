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
#include "HiResTimer.h"

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

using namespace std::placeholders; // for _1, _2 etc


static void MessageHandler(uWS::WebSocket<uWS::SERVER> ws,
                           char *data,
                           size_t length,
                           uWS::OpCode opCode,
                           MPC& mpc,
                           HiResTimer &hrt)
{
  // "42" at the start of the message means there's a websocket message event.
  // The 4 signifies a websocket message
  // The 2 signifies a websocket event
  string sdata = string(data).substr(0, length);

  static HiResTimer prev_telemetery_hrt;

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

        cout << prev_telemetery_hrt.GetElapsedSecs() << " secs latency since controls sent "<< endl;
        cout << "IN: " << hrt.GetElapsedSecs() << " px="<<px<<" py="<<py<<" v="<<v<<" psi="<<psi << endl;

        HiResTimer calc_hrt;

        // adjust start state for latency
        double latency = 0.12 ;
        bool adjust_for_latency = true;
        const double Lf = 2.67;
        const double v_conv = v * (1609. / 3600.);

        if (adjust_for_latency and fabs(prev_steering_angle) < 0.0001) { // lame approximation
          px = px + v_conv * cos(psi) * latency;
          py = py + v_conv * sin(psi) * latency;
//          for (int i = 0; i < ptsx.size(); i++) {
//            ptsx[i] = ptsx[i] + v * (1609. / 3600.) * cos(psi) * latency;
//            ptsy[i] = ptsy[i] + v * (1609. / 3600.) * sin(psi) * latency;
//          }
          psi = psi - v_conv * deg2rad(prev_steering_angle * 25.) / Lf * latency;
          //v = v + prev_throttle * 5.0 * latency;
        }
        else if (adjust_for_latency and v>0.0001) { // bicycle model equations from EKF lectures
          double psidot = - v_conv * deg2rad(prev_steering_angle * 25.) / Lf;
          double psi_new = psi + psidot * latency;
          px = px + (v_conv / psidot) * ( sin(psi_new) - sin(psi));// + 0.5 * latency * latency * cos(psi) * prev_throttle * 5.0;
          py = py + (v_conv / psidot) * (-cos(psi_new) + cos(psi));// + 0.5 * latency * latency * sin(psi) * prev_throttle * 5.0;
//          for (int i = 0; i < ptsx.size(); i++) {
//            ptsx[i] = ptsx[i] + (v / psidot) * conv_factor * (sin(psi + psidot * latency) - sin(psi)) + 0.5 * latency * latency * cos(psi) * prev_throttle * 5.0;
//            ptsy[i] = ptsy[i] + (v / psidot) * conv_factor * (-cos(psi + psidot * latency) + cos(psi)) + 0.5 * latency * latency * sin(psi) * prev_throttle * 5.0;
//          }
          psi = psi_new;
//          v = v + prev_throttle * 5.0 * latency;
//          cout << px << " " << py << " " << psi << " " << v << endl;
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
        int order = 2;
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
        x0[3] = v*1609./3600.; // speed, m/s. 1mph=1609m/3600s
        x0[4] = polyeval(coeff, 0.0); // cross track error, meters. is simply y(0.0) as car is in the center of coordinates
        x0[5] = -atan(coeff[1]); // heading error, radians. this is simply angle of f(x) wrt x axis at zero. arctan(f'(0)). f=ax^3+bx^2+cx+d. f'(0)=c
        // assume throttle of 1 is 5m/s2 acceleration

        double steer_value = -0.02;
        double throttle_value = 0.1;

        //Display the MPC predicted trajectory
        vector<double> mpc_x_vals;
        vector<double> mpc_y_vals;

        bool status = false;

        double curvature = (fabs(coeff[2]) > 0.0001) ? pow(1.0+pow(coeff[1], 2), 1.5) / fabs(2.*coeff[2]) : 10000. ;
        double v_ref = 100; // in mph. MPC will convert to m/s2
        if (v < 40.)
          v_ref = 150;
        else
          if (curvature < 70)
            v_ref = 80;
//          else if (curvature < 80)
//            v_ref = 80;
//          else if (curvature < 100)
//            v_ref = 100;
          else
            v_ref = 150;
        cout << "curvature: " << curvature << " v: " << v << " ref_v: " << v_ref << endl;

        vector<double> solution = mpc.Solve(x0, coeff, v_ref, mpc_x_vals, mpc_y_vals, status);

        cout << calc_hrt.GetElapsedSecs() << " secs calculation "<< endl;

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
        HiResTimer sleep_hrt;

        this_thread::sleep_for(chrono::milliseconds(100));

        cout << sleep_hrt.GetElapsedSecs() << " secs slept\n";
        cout << "OUT: " << hrt.GetElapsedSecs() << " st="<<steer_value<<" thrt="<<throttle_value  << endl;

        prev_telemetery_hrt.Reset(); // start measuring time from last telemetery

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

  MPC mpc; // Model Predictive Controller
  HiResTimer hrt; // High Resolution Timer to measure latencies

  h.onMessage(std::bind(MessageHandler, _1, _2, _3, _4, mpc, hrt));

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
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
