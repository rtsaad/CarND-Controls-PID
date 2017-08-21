# PID Controller for the Udacity Simulator

This project consists of a c++ implementation of PID controller to control the steering angle of a car using the Udacity simulator. The main goals of this project is to develop a c++ PID controller that successfully drives the vehicle around the track (Udacity simulator). Figure 1 depicts the car being controlled by the PID controller. 


## PID 

The PID (Proportional, Integral and Derivative) controller is a closed loop controller widely used by the industry.  It computes the system input variable from the error e(t) between the desired set point and the system output (process variable). The control response (system input) is calculated by applying the proportional, derivative and integral gains over e(t).

For this project, the PID controller is used to control the steering angle from the ``cross track error`` e(t) (car distance from the track center). Therefore, the system input is the steering angle, the output is the car distance from the center and the setpoin it zero (closest to the center as possible). 

![equation](http://latex.codecogs.com/gif.latex?%5Calpha%20%3D%20-K_pe%28t%29%20-K_d%5Cfrac%7Bde%28t%29%7D%7Bdt%7D%20-%20K_i%5Csum%20e%28t%29)

### Kp (Proportional Gain)

The Kp gain results into a proportional control response. In the context of this project, it means that the steer input is in proportion to the Cross Track error. However, the proportional control alone results into a marginally stable system because the car will never converge to the set point, it will slightly overshoot. In addition, increasing the value of Kp will make the car react faster but it will also oscillate more around the center lane (set point). This [video](https://github.com/otomata/CarND-Controls-PID/blob/master/images/kp.mp4) shows the oscillation of the car trajectory using a proportional controller. 

### Kd (Derivative Gain)

The Kd gain considers the rate change of error and tries to bring this rate to zero. The derivative gain complements the proportional output by reducing the overshoot, it mains goal is to flattening the car trajectory. This [video](https://github.com/otomata/CarND-Controls-PID/blob/master/images/kd.mp4) presents the smoothed trajectory of the car using a proportional-derivative controller. 


### Ki (Integral Gain)

The Ki gain reduces the persistent error, i.e. the accumulated error. The integral gain helps the controller to deal with the  ``systematic bias`` problem witch leads to a systematic error. The Ki gain should help to remove the residual error to approximate the car near to the center of the track. This [video](https://github.com/otomata/CarND-Controls-PID/blob/master/images/ki.mp4) illustrates the car following the lane center using a complete PID controller. It is important to mention that the car oscillates a little bit more than PD controller because the ki gain has been manually tuned. Next section, we presented the twiddle algorithm we used to fine tuning our PID controller. 

## PID Tunning

The PID tuning for this project followed a mix of manual and automatic tuning approaches. First, due to the simulation time (~90 seconds) required to evaluate a set of PID gains, we decided to first follow a manual approach to find the gain orders. We observed the following orders of magnitude for the PID gains:

* Proportional: order of 0.1
* Integral: order of 0.001
* Derivative: order of 0.0001


The second step it to use the [Twiddle Algorithm](https://martin-thoma.com/twiddle/) to fine tuning the PID gains. We used the gain orders we had previously manually found to help the Twiddle algorithm to converge faster by setting the tiwdle starting vectors to: 

* Initial PID Vector: {0.01, 0, 0.0007} (values manually tunned)
* Potential changes: {.1,.001,.0001}

The twiddle algorithm evaluates every new set of parameters and checks if the error is smaller. If the error is greater, the potential change vector is changed and a new set of parameters is attempted. However, because of the different corners radius of the track, we have to evaluate the PID controller into a long run, in order to test if the set of parameter (gains) is well adapted for the track, and not only for a small subset. This approach makes the experiment really long; 40 iterations took almost one hour; we have decided to stop the algorithm after 100 iterations. 

The final PID controller successfully drives the car around the track without pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans where in the vehicle). The file PID parameters are shown below:

* Kp: 0.11
* Ki: 0.00415711
* Kd: 0.00087019

This [video](https://github.com/otomata/CarND-Controls-PID/blob/master/images/pid.mp4) shows the car driving around the corner with the PID controller set with the parameters presented above. 

