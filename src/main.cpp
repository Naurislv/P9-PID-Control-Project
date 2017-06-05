#include <vector>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Frame counter for twiddling
bool TWIDDLE = false;

int nFrames = 0;
int step0 = 0;
int step1 = 0;
int i0 = 0;
int i1 = 0;
int cte_time = 0;

double err0 = 0;
double best_err0 = std::numeric_limits<double>::infinity();
double err1 = 0;
double best_err1 = std::numeric_limits<double>::infinity();

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws)
{
    // reset
    std::string msg("42[\"reset\", {}]");
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  // Kp, Ki, Kd, Kcte, Ksteer, K speed -->
  pid.Init(0.090298, 0.0034298, 1.538, 0.239206, 0.413927, 0.514466);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double speed_value;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */


          pid.UpdateError(cte, speed, angle);
          steer_value = pid.Predict_steer();
          speed_value = pid.Predict_throttle();

          // DEBUG
          std::cout << " speed_sum " << pid.speed_sum << " cte_sum " << pid.cte_sum << " best_err0 "
                    << best_err0 << " best_err1 " << best_err1 << "  CTE " << cte << " throttle "
                    << speed_value << "  angle " << angle << "  Frame " << nFrames << " steering "
                    << steer_value << "  dt  " << pid.dt <<  std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = speed_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          if (TWIDDLE) {
              if (fabs(cte) > 4.5) {
                  cte_time += 1;
              } else {
                  cte_time = 0;
              }

              if (cte_time > 2 or nFrames == 1150 or nFrames == 0) {
                  reset_simulator(ws);
                  pid.i_error = 0;

                  err0 = pid.cte_sum;
                  err1 = - pid.speed_sum;

                  if (nFrames != 1150) {
                      err0 = pid.cte_sum + 5000;
                  }

                  std::cout << "\n\nerr0  " << err0 << '\n';
                  std::cout << "err1  " << err1 << '\n';
                  std::cout << "distance component  " << - pid.speed_sum << '\n';
                  std::cout << "cte component  " << pid.cte_sum << '\n';


                  pid.Twiddle(step0, i0, err0, best_err0, pid._p, pid._dp);
                  pid.Twiddle(step1, i1, err1, best_err1, pid._p2, pid._dp2);

                  best_err0 += fabs(best_err0) * 0.003;
                  best_err1 += fabs(best_err1) * 0.003;

                  nFrames = 0;

                  unsigned int microseconds = 1000000;
                  usleep(microseconds);
              }
          } else {
              pid.cte_sum = 0;
              pid.speed_sum = 0;
          }

          nFrames += 1;
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
