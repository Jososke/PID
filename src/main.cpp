#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::cout;
using std::endl;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Resetting the Simulator
void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws) {
  std::string msg("42[\"reset\",{}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  bool twiddle = true; //set here to turn on parameter tuning with twiddle - params will be displayed in cout
  
  PID pid;
  //pid.Init(0.15, 0.0, 2.5); //tuned twittle parameters
  pid.Init(.1, 0.0, 0.0); //tuned twittle parameters
  PID speed_pid;
  speed_pid.Init(0.5,0.1,2); //tuned twittle parameters

  Twiddle twid;
  twid.Init(0.1, 0.0, 0.0);
  int twiddle_steps = 500; //enough steps to determine error from twiddle tuning
  int num_iter = 0; //main loop counter
  int twid_tol = .01; //tolerance to tune twiddle params against

  double target_speed = 30; //set target speed here for speed control

  h.onMessage([&pid, &speed_pid, &twid, &target_speed, &twiddle, &num_iter, &twiddle_steps, &twid_tol]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle;
          num_iter++;

          //remove cte before connected
          if (num_iter > 2) {

            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            //limiting steering angles to [-1 1] per mechanics of system
            steer_value = fminf(1.0, steer_value);
            steer_value = fmaxf(-1.0, steer_value);
            pid.SquareError(); //finding total square error

            speed_pid.UpdateError(speed - target_speed);
            throttle = speed_pid.TotalError();
            //limiting throttle values to [-.7 .7]
            throttle = fminf(0.7, throttle);
            throttle = fmaxf(-0.7, throttle);
            
            //tune twiddle params after specified number of steps and reset simulator
            if ((pid.steps > twiddle_steps || fabs(cte) > 3 ) & twiddle) //parameter tuning
            {
              twid.avg_err = pid.total_err / pid.steps;
              //assign large error if the car out of road
              if (fabs(cte) > 3) twid.cur_err = twid.avg_err + 100.0;
              else twid.cur_err = twid.avg_err;
              
              if (twid.cur_err < twid_tol) twiddle = false; //end if performance is good
              
              twid.Update(); //update pid params using twiddle
              //Debug
              cout << "kp: " << twid.p[0] << "\tki: " << twid.p[1] << "\tkd: " << twid.p[2] << endl;
              cout << "dkp: " << twid.dp[0] << "\tdki: " << twid.dp[1] << "\tdkd: " << twid.dp[2] << endl;
              cout << "Best PID params: " << twid.best_params.p[0] << "\t" << twid.best_params.p[1] << "\t" << twid.best_params.p[2] << endl;
              cout << "Best dp params: " << twid.best_params.dp[0] << "\t" << twid.best_params.dp[1] << "\t" << twid.best_params.dp[2] << endl;
              cout << "Best err: " << twid.best_err << "\tcurrent err: " << twid.cur_err << "\tcurrent idx: " << twid.idx << endl;
              cout << "--------------------" << endl;
              //initialize pid with new params
              pid.Init(twid.p[0], twid.p[1], twid.p[2]);
              reset_simulator(ws); //reset simulator
              num_iter = 0; //reset loop
              sleep(0.5); // waiting for reseting sim
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = .3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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