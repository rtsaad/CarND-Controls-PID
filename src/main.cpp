#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <numeric>
#include <sstream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <limits>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/*
 * Twiddle 
 * Use twiddle algorithm to search for the best PID gains.
 * Use xdotool to restart the simulator and to sincronize each
 * iteration. It emulates the mouse and the keyboard to restart
 * the simulator. Check file 'automate'.
 */
//enable twiddle
bool auto_tunning = false;
//number of iterations for the simulator
int  tunning_iteration = 800;
//control twiddle stage
int  tunning_stage = 0;
//control which gain is being considered: kp=0, ki=1 and kd=2.
int  tunning_gain = 0;
//save best error
double tunning_best_error = std::numeric_limits<long int>::max();
double threshold = 0.001;
std::vector<double> p = {0.01,0,0.0007};
std::vector<double> best_p = {0,0,0};
std::vector<double> dp = {0.1,.001,0.0001};
//PID BEST 0.11 0.00415711 0.00087019 || 0.708059
  
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

int main()
{
  uWS::Hub h;

  //steering controller
  PID pid;  
  pid.Init(0.11,0.00415711,0.00087019);  

  //speed controller
  PID pidSpeed;  
  pidSpeed.Init(0.1, 0.002,0);
  //set point of 30 mph
  pidSpeed.SetPoint(30);
  

  h.onMessage([&pid, &pidSpeed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
	  //remove warning
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

	  if(auto_tunning){
	    //Auto-tunning selected
	    //Twiddle algorithm
	    double sum = std::accumulate(dp.begin(),dp.end(),0.0);
	    if(sum > threshold){
	      if(tunning_gain > 2){
		tunning_gain = 0;
	      }
	      if(tunning_iteration < 800){
		//iterate with simulator for 800 times
		pid.UpdateError(cte);
		tunning_iteration++;
	      } else {
		//check error and, if necessary, change gains
		double error = pid.TotalError();
		std::cout << "STAGE " << tunning_stage << " GAIN " << tunning_gain << "\n";
		std::cout << "PID " << p[0] << " " << p[1] << " " << p[2] << "\n";
		std::cout << " ERRO - BEST " <<  error << " " << tunning_best_error << " TOLERANCE " << sum << "\n";
		switch (tunning_stage){
		case 0:
		  p[tunning_gain] += dp[tunning_gain];
		  //go to next twiddle stage
		  tunning_stage = 1;
		  tunning_iteration = 0;
		  break;
		case 1:
		  if(error < tunning_best_error){
		    tunning_best_error = error;
		    dp[tunning_gain] *= 1.1;
		    tunning_stage = 0;
		    //go to next gain
		    tunning_gain++;
		    best_p[0] = p[0];
		    best_p[1] = p[1];
		    best_p[2] = p[2];
		  } else {
		    p[tunning_gain] -= 2*dp[tunning_gain];
		    //go to next twiddle stage
		    tunning_stage = 2;
		    tunning_iteration = 0;
		  }
		  break;
		case 2:
		  if(error < tunning_best_error){
		    tunning_best_error = error;
		    dp[tunning_gain] *= 1.05;
		    best_p[0] = p[0];
		    best_p[1] = p[1];
		    best_p[2] = p[2];
		  } else {
		    p[tunning_gain] += dp[tunning_gain];
		    dp[tunning_gain] *= 0.90;
		  }
		  tunning_stage = 0;
		  //go to next gain
		  tunning_gain++;		  
		  break;
		}
		pid.Init(p[0], p[1], p[2]);		
		std::string input = "";
		std::cout << "PID NEW " << p[0] << " " << p[1] << " " << p[2] << "\n";
		std::cout << "PID BEST " << best_p[0] << " " << best_p[1] << " " << best_p[2] << "\n";
		std::cout << "Reset Simulator\n";
		//sincronize with simulator
		//wait for key stroke to continue		
		getline(std::cin, input);
	      }	      
	    } else {
	      std::cout << "Finish " << sum << "\n";
	      std::cout << p[0] << " " << p[1] << " " << p[2] << "\n";
	    }

	  } else { 
	    pid.UpdateError(cte);
	  }

	  //Compute control response
	  double control = pid.ComputeControlResponse();
	  //check steering limits
	  if (control > 1){
	    steer_value = 1;
	  } else if(control < -1){
	    steer_value = -1;
	  } else {
	    steer_value = control;
	  }	  
       
          json msgJson;
          msgJson["steering_angle"] = steer_value;

	  //update speed error
	  pidSpeed.UpdateError(speed);
	  
          msgJson["throttle"] = pidSpeed.ComputeControlResponse();//0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
