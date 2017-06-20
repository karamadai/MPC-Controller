----------

Model Predictive Controller
----------
The goal of this project is to develop a Model Predict Controller to drive a car around a course in a simulator. The simulator feeds the position of the vehicle (px, py), the way points along the track (ptsx,ptsy) , the speed (v) and heading of the vehicle (psi).

**The Vehicle Model** 
The project uses a Kinematic Vehicle Model to describe the motion of car. The Kinematic Vehicle Model is a simplified form of the Dynamic Vehicle and does not take into account the influence of vehicle mass, inertia, tire forces, air resistance, drag etc.  

In the Kinematic Vehicle model, the state  and actuators vectors are described as:
	
	State of the Vehicle: [x, y, v, psi]
	Actuators : [delta, a]
  where delta is the steering input and "a" is accelerator. The parameter "a" represents throttle when positive and braking when negative.

The following equation describes how the state of the model changes over time at **t+1** based on the previous state and actuator input at time **t**
	  
	 x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
     y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
     psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
     v_[t+1] = v[t] + a[t] * dt
 Lf is a measure of the distance between the front of the vehicle and it's CG.
 dt is the time elapsed between the two states
 
The Cross Track error (**cte**) and the orientation error  (**epsi**) is described as:
		  
	 cte[t+dt] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
	 epsi[t+dt] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

**Note:** The steering angle is multiplied by -1 before sending to the simulator.

**Time Step and Elapsed Duration**
The time horizon (T) over which the motion of the model is predicted is given by N x dt. Where N is the number of timesteps and dt is the time elapsed between each step. 
Based on trail and error,  the following values of N and dt choose:
Num of timesteps (N)=10 
Time between steps (dt)=0.1 (100 milliseconds). 
This gives the model prediction over a time period of 1 second.  
Using a larger time can make the model unreliable due the rapidly changing external conditions. Using larger number of time steps (N) or smaller time intervals (dt) increases the computing time.  The model tends to be less accurate as the computation time increases.

**Polynomial Fitting and MPC Preprocessing**
The way points feed provided by the simulator is in the global co-ordinate system.  However all the MPC computations are performed in the vehicle co-ordinate system.  Hence, the global ways points are shifted to the vehicle co-ordinate and rotated to align the vehicle's heading using  the following lines of code in main.cpp (line 97-102)

	 for(size_t i=0;i<ptsx.size();i++){
            double shift_x=ptsx[i]-px;
            double shift_y=ptsy[i]-py;
            ptsx[i]=(shift_x*cos(0-psi)-shift_y*sin(0-psi));
            ptsy[i]=(shift_x*sin(0-psi)+shift_y*cos(0-psi));
     }
A third order polynomial is now fitted to the transformed way points.  The cross track error and the heading error with reference to the fitted polynomial is computed at x=0. Since the way points are transformed to the car's coordinate system the car is now located at the origin with px=0, py=0 and psi=0.  Hence,  the state vectors  is represented as:

	State=[0, 0, 0, v, psi]

**Model Predictive Control and Latency**
The model predictive control frames the problem of following a trajectory as an optimization problem. The MPC finds the optimal solution by finding the lowest cost associated with actuator control inputs and a set of constraints provide by the Kinematic Motion Model. 

The cost function at each time step consists of the following:
1. Cost based on reference state: 
	This consists of the cross track error, orientation heading error and velocity error with reference to a reference state.
	
		for (size_t t = 0; t < N; t++) {
		      fg[0] += 3000*CppAD::pow(vars[cte_start + t]-ref_cte, 2);
		      fg[0] += 3000*CppAD::pow(vars[epsi_start + t]-ref_epsi, 2);
		      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
	   }
Each error value in multiplied by a constant factor to force the optimizer to pay more attention to that particular attribute. In the code above the optimizer is forced to pay more attention to the cte and epsi errors.
2. Cost based on the actuator use
	
		for (size_t t = 0; t < N - 1; t++) {
	      fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);
	      fg[0] += 5*CppAD::pow(vars[a_start + t], 2);
	    }
    The constant multipliers for the cost was determined by trail and error
3. Cost based on the difference between sequential actuation 

		for (size_t t = 0; t < N - 2; t++) {
	      fg[0] += 180*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
	      fg[0] += 5*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }
     The constant multipliers for the cost was determined by trail and error
     
The constraints on the model is applied by the following lines of code for each time step:

	  fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =  cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt)
       where the values of x0, y0, x1, y1, psi1, psides0 etc are defined in lines 80 to 100 (mpc.cpp)
 The upper and lower limits on the actuators are set as follows (line 166-177, mpc.cpp)
	
	 for (size_t i = delta_start; i < a_start; i++) {
	    vars_lowerbound[i] = -0.436332;
	    vars_upperbound[i] = 0.436332;
	  }
	  // Acceleration/decceleration upper and lower limits.
	  for (size_t i = a_start; i < n_vars; i++) {
	    vars_lowerbound[i] = -1.0;
	    vars_upperbound[i] = 1.0;
	  }
	  
**Predicting with Latency**
The model predictive controller accounts for latency in mechanical systems by predicting the state of the vehicle at the point of time when the actuation will have an effect (t+latency_time) and then determine the actuator input needed to minimize the cost at the projected state. 

The project uses a latency of 100 milliseconds with 10 time steps and a time interval between  steps as 0.1 (100 milliseconds). Hence the actuator values obtained after the first prediction step time is fed as the control input to the simulator.