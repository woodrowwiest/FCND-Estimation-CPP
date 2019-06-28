# Estimation Project #

Woodrow Wiest - 28 June, 2019 - Göteborg, Sweden

Welcome to my variation of the estimation project.  In this project, we are tasked to develop the estimation portion of the controller used in the CPP simulator.  The final result, our simulated quad will be flying with the estimator and custom controller from the previous project: [FCND-Controls-CPP](https://github.com/woodrowwiest/FCND-Controls-CPP)  

This README takes on the basic form from the original project and the content is as follows:

 - [Setup](#setup) - the environment and code setup required to get started and a brief overview of the project structure
 - [The Tasks](#the-tasks) - the tasks you will need to complete for the project
 - [Tips and Tricks](#tips-and-tricks) - some additional tips and tricks you may find useful along the way
 - [Submission](#submission) - overview of the requirements for your project submission
 - [Acknowledgements](#Acknowledgements)


## Setup ##

This project will continue to use the C++ development environment you set up in the Controls C++ project.

 1. Clone the repository
 ```
 git clone https://github.com/udacity/FCND-Estimation-CPP.git
 ```

 2. Import the code into our IDE like done in the [Controls C++ project](https://github.com/udacity/FCND-Controls-CPP#development-environment-setup). I am using Xcode.
 
 3. We should now be able to compile and run the estimation simulator.


### Project Structure ###

For this project, we will be primarily interacting with the following files:

 - The EKF `QuadEstimatorEKF.cpp` which is already partially implemented for us. 

 - Parameters for tuning the EKF `QuadEstimatorEKF.txt`

 - When we turn on various sensors (the scenarios configure them, e.g. `Quad.Sensors += SimIMU, SimMag, SimGPS`), additional sensor plots will become available to see what the simulated sensors measure.

 - The EKF implementation exposes both the estimated state and a number of additional variables. In particular:

   - `Quad.Est.E.X` is the error in estimated X position from true value.  More generally, the variables in `<vehicle>.Est.E.*` are relative errors, though some are combined errors (e.g. MaxEuler).

   - `Quad.Est.S.X` is the estimated standard deviation of the X state (that is, the square root of the appropriate diagonal variable in the covariance matrix). More generally, the variables in `<vehicle>.Est.S.*` are standard deviations calculated from the estimator state covariance matrix.

   - `Quad.Est.D` contains miscellaneous additional debug variables useful in diagnosing the filter. We may or might not find these useful but they were helpful to us in verifying the filter and may give us some ideas if we hit a block.


#### `config` Directory ####

In the [`config`](https://github.com/woodrowwiest/FCND-Estimation-CPP/tree/master/config) directory, in addition to finding the configuration files for our controller and estimator, we also see configuration files for each of the simulations.  For this project, we work with simulations 06 through 11.



and you may find it insightful to take a look at the configuration for the simulation.

As an example, if we look through the configuration file for scenario 07, we see the following parameters controlling the sensor:

```
# Sensors
Quad.Sensors = SimIMU
# use a perfect IMU
SimIMU.AccelStd = 0,0,0
SimIMU.GyroStd = 0,0,0
```

This configuration tells us that the simulator is only using an IMU and the sensor data will have no noise.  You will notice that for each simulator these parameters change slightly as additional sensors are being used and the noise behavior of the sensors change.


## The Tasks ##

We begin by building up our estimator incrementally.  At each step, there will be a set of success criteria that will be displayed both in the plots and in the terminal output.

Project outline:

 - [01: Sensor Noise](#01-sensor-noise)
 - [02: Attitude Estimation](#02-attitude-estimation)
 - [03: Prediction Step](#03-prediction-step)
 - [04: Magnetometer Update](#04-magnetometer-update)
 - [05: Closed Loop + GPS Update](#05-closed-loop--gps-update)
 - [06: Adding Your Controller](#06-adding-your-controller)



### 01: Sensor Noise ###

For the controls project, the simulator was working with a perfect set of sensors, meaning none of the sensors had any noise.  Unfortunately the real world is inconsistent and noisy.  For our simulation to be more consistent with the real world, we add simulated noise to the quad's sensors.  For the first step, we collect this simulated noisy sensor data and calculate the standard deviation of the quad's sensors.

01.0 Run the simulator

01.1 Choose scenario `06_NoisySensors`.
  - The quad does not move.
  - The two plots give us a visual of the for GPS X position and the accelerometer's x measurement data.  The dashed lines are a visualization of a single standard deviation from 0 for each signal. 
  - The standard deviations were initially set to arbitrary values. If they were set correctly, we should see ~68% of the measurement points fall into the +/- 1 sigma bound.  
  - When we run this scenario, the graph data will be recorded to the following csv files with headers: `config/log/Graph1.txt` (GPS X data) and `config/log/Graph2.txt` (Accelerometer X data).

01.2 Process the logged files to figure out the standard deviation of the the GPS X signal and the IMU Accelerometer X signal.  This can be accomplished by creating a python script to parse the files, using a program like Matlab, plug the data into a spreadsheet, or calculate it by hand. Either way, here is the formula for standard deviation of a sample:

![Standard Deviation from sample Formula from Wikipedia](https://wikimedia.org/api/rest_v1/media/math/render/svg/067067e579e43b39ca1e57d9be52bda5b80cd284)

I wrote a simple script using [Python](https://www.python.org), [Pandas](https://pandas.pydata.org), and visualized it with [Matplotlib](https://matplotlib.org) like this:
![How to parse a .csv file with Pandas and plot it with Matplotlib in Python](https://github.com/woodrowwiest/FCND-Estimation-CPP/blob/master/images/01.2_std_GPS_IMU.jpg)

01.3 Plug in our result into the top of [`config/6_Sensornoise.txt`](https://github.com/woodrowwiest/FCND-Estimation-CPP/blob/master/config/06_SensorNoise.txt).  
  - Specially, we set the values for `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY` to our calculated values.

01.4 Run the simulator. Our values are correct, the dashed lines in the simulation turn green, indicating we’re capturing approx 68% of the respective measurements (which is what we expect within +/- 1 sigma bound for a Gaussian noise model)

**Success!**
![Passing Sensor Noise](https://github.com/woodrowwiest/FCND-Estimation-CPP/blob/master/images/01.4_sensor_noise_pass.jpg)

NOTE: Our answer should match the settings in `SimulatedSensors.txt`, where we can also grab the simulated noise parameters for all the other sensors.


### 02: Attitude Estimation ###

Now let's look at the first step to our state estimation: including information from our IMU.  In this step, we will improve the complementary filter-type attitude filter with a better rate gyro attitude integration scheme.

1. Run scenario `07_AttitudeEstimation`.  For this simulation, the only sensor used is the IMU and noise levels are set to 0 (see `config/07_AttitudeEstimation.txt` for all the settings for this simulation).  There are two plots visible in this simulation.
   - The top graph is showing errors in each of the estimated Euler angles.
   - The bottom shows the true Euler angles and the estimates.

2. In `QuadEstimatorEKF.cpp`, we locate the function `UpdateFromIMU()` contains a complementary filter-type attitude filter.  To reduce the errors in the estimated attitude (Euler Angles), comment out the supplied code and implement a better rate gyro attitude integration scheme like this:

```
// reduce attitude estimation error by converting to a quaternion
Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));

// integrate with the Quaternion class function IntegrateBodyRate()
attitude.IntegrateBodyRate(gyro, dtIMU);

// convert back to Euler angles using Roll(), Pitch(), Yaw() convenience functions
float predictedPitch = attitude.Pitch();
float predictedRoll = attitude.Roll();
ekfState(6) = attitude.Yaw();
    
// normalize yaw to -pi .. pi
if (ekfState(6) > F_PI) ekfState(6) -= 2.f * F_PI;
if (ekfState(6) < -F_PI) ekfState(6) += 2.f * F_PI;
```

**Success!** *within 0.1 rad for each of the Euler angles for at least 3 seconds.*
![Passing Attitude Estimation](https://github.com/woodrowwiest/FCND-Estimation-CPP/blob/master/images/02_attitude_pass.jpg)

Note: see section 7.1.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on a good non-linear complimentary filter for attitude using quaternions.


### 03: Prediction Step ###

In this next step we will implement the prediction step of our filter.


03.1 Run scenario `08_PredictState`.  This scenario is configured to use a perfect IMU (only an IMU). Due to the sensitivity of double-integration to attitude errors, Udacity has made the accelerometer update very insignificant (`QuadEstimatorEKF.attitudeTau = 100`).  The plots on this simulation show element of our estimated state and that of the true state.

03.2 In `QuadEstimatorEKF.cpp`, we implement the state prediction step in the `PredictState()` functon as follows:

```
// Position
predictedState(0) += predictedState(3) * dt;  // x
predictedState(1) += predictedState(4) * dt;  // y
predictedState(2) += predictedState(5) * dt;  // z
    
// Velocity
// account for earth's gavitational constant
V3F velAccel = attitude.Rotate_BtoI(accel) - V3F(0.0, 0.0, CONST_GRAVITY);
predictedState(3) += velAccel.x * dt;
predictedState(4) += velAccel.y * dt;
predictedState(5) += velAccel.z * dt;
```

**Success!** When we run scenario `08_PredictState` we see the estimator state track the actual state, with only reasonably slow drift.

![Predict Drift Pass](https://github.com/woodrowwiest/FCND-Estimation-CPP/blob/master/images/03_prediction_pass.jpg)

3.3 Now let's introduce a realistic IMU, one with noise.  When we run scenario `09_PredictionCov`, we see a small fleet of quadcopter all using your prediction code to integrate forward. There are two plots:
   - The top graph shows 10 (prediction-only) position X estimates
   - The bottom graph shows 10 (prediction-only) velocity estimates

3.4 In `QuadEstimatorEKF.cpp`, we calculate the partial derivative of the body-to-global rotation matrix in the function `GetRbgPrime()`.  Here is the formula:

![GetRbgPrime Equasion](https://github.com/woodrowwiest/FCND-Estimation-CPP/blob/master/images/eq-getRgbPrime.jpg)

Here is the code:

```
float cosTheta = cos(pitch);
float sinTheta = sin(pitch);
    
float cosPhi = cos(roll);
float sinPhi = sin(roll);
    
float sinPsi = sin(yaw);
float cosPsi = cos(yaw);
    
RbgPrime(0,0) = - cosTheta * sinPsi;
RbgPrime(0,1) = - sinPhi  * sinTheta * sinPsi - cosTheta * cosPsi;
RbgPrime(0,2) = - cosPhi  * sinTheta * sinPsi + sinPhi   * cosPsi;
    
RbgPrime(1,0) = cosTheta * cosPsi;
RbgPrime(1,1) = sinPhi  * sinTheta * cosPsi - cosPhi * sinPsi;
RbgPrime(1,2) = cosPhi  * sinTheta * cosPsi + sinPhi * sinPsi;
```

Once you have that function implement, implement the rest of the prediction step (predict the state covariance forward) in `Predict()`.

Note: see section 7.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on the the transition model and the partial derivatives.

5. Run your covariance prediction and tune the `QPosXYStd` and the `QVelXYStd` process parameters in `QuadEstimatorEKF.txt` to try to capture the magnitude of the error you see. Note that as error grows our simplified model will not capture the real error dynamics (for example, specifically, coming from attitude errors), therefore  try to make it look reasonable only for a relatively short prediction period (the scenario is set for one second).  A good solution looks as follows:

![good covariance](images/predict-good-cov.png)

Looking at this result, you can see that in the first part of the plot, our covariance (the white line) grows very much like the data.

If we look at an example with a `QPosXYStd` that is much too high (shown below), we can see that the covariance no longer grows in the same way as the data.

![bad x covariance](images/bad-x-sigma.PNG)

Another set of bad examples is shown below for having a `QVelXYStd` too large (first) and too small (second).  As you can see, once again, our covariances in these cases no longer model the data well.

![bad vx cov large](images/bad-vx-sigma.PNG)

![bad vx cov small](images/bad-vx-sigma-low.PNG)

***Success criteria:*** *This step doesn't have any specific measurable criteria being checked.*


### 04: Magnetometer Update ###

Up until now we've only used the accelerometer and gyro for our state estimation.  In this step, you will be adding the information from the magnetometer to improve your filter's performance in estimating the vehicle's heading.

1. Run scenario `10_MagUpdate`.  This scenario uses a realistic IMU, but the magnetometer update hasn’t been implemented yet. As a result, you will notice that the estimate yaw is drifting away from the real value (and the estimated standard deviation is also increasing).  Note that in this case the plot is showing you the estimated yaw error (`quad.est.e.yaw`), which is drifting away from zero as the simulation runs.  You should also see the estimated standard deviation of that state (white boundary) is also increasing.

2. Tune the parameter `QYawStd` (`QuadEstimatorEKF.txt`) for the QuadEstimatorEKF so that it approximately captures the magnitude of the drift, as demonstrated here:

![mag drift](images/mag-drift.png)

3. Implement magnetometer update in the function `UpdateFromMag()`.  Once completed, you should see a resulting plot similar to this one:

![mag good](images/mag-good-solution.png)

***Success criteria:*** *Your goal is to both have an estimated standard deviation that accurately captures the error and maintain an error of less than 0.1rad in heading for at least 10 seconds of the simulation.*

**Hint: after implementing the magnetometer update, you may have to once again tune the parameter `QYawStd` to better balance between the long term drift and short-time noise from the magnetometer.**

**Hint: see section 7.3.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on the magnetometer update.**


### 05: Closed Loop + GPS Update ###

1. Run scenario `11_GPSUpdate`.  At the moment this scenario is using both an ideal estimator and and ideal IMU.  Even with these ideal elements, watch the position and velocity errors (bottom right). As you see they are drifting away, since GPS update is not yet implemented.

2. Let's change to using your estimator by setting `Quad.UseIdealEstimator` to 0 in `config/11_GPSUpdate.txt`.  Rerun the scenario to get an idea of how well your estimator work with an ideal IMU.

3. Now repeat with realistic IMU by commenting out these lines in `config/11_GPSUpdate.txt`:
```
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0
```

4. Tune the process noise model in `QuadEstimatorEKF.txt` to try to approximately capture the error you see with the estimated uncertainty (standard deviation) of the filter.

5. Implement the EKF GPS Update in the function `UpdateFromGPS()`.

6. Now once again re-run the simulation.  Your objective is to complete the entire simulation cycle with estimated position error of < 1m (you’ll see a green box over the bottom graph if you succeed).  You may want to try experimenting with the GPS update parameters to try and get better performance.

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated position error of < 1m.*

**Hint: see section 7.3.1 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on the GPS update.**

At this point, congratulations on having a working estimator!

### 06: Adding Your Controller ###

Up to this point, we have been working with a controller that has been relaxed to work with an estimated state instead of a real state.  So now, you will see how well your controller performs and de-tune your controller accordingly.

1. Replace `QuadController.cpp` with the controller you wrote in the last project.

2. Replace `QuadControlParams.txt` with the control parameters you came up with in the last project.

3. Run scenario `11_GPSUpdate`. If your controller crashes immediately do not panic. Flying from an estimated state (even with ideal sensors) is very different from flying with ideal pose. You may need to de-tune your controller. Decrease the position and velocity gains (we’ve seen about 30% detuning being effective) to stabilize it.  Your goal is to once again complete the entire simulation cycle with an estimated position error of < 1m.

**Hint: you may find it easiest to do your de-tuning as a 2 step process by reverting to ideal sensors and de-tuning under those conditions first.**

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated position error of < 1m.*


## Tips and Tricks ##

 - When it comes to transposing matrices, `.transposeInPlace()` is the function you want to use to transpose a matrix

 - The [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) document contains a helpful mathematical breakdown of the core elements on your estimator

## Submission ##

For this project, we submit the following files:

 - a completed estimator that meets the performance criteria for each of the steps by submitting:
   - [`QuadEstimatorEKF.cpp`](https://github.com/woodrowwiest/FCND-Estimation-CPP/blob/master/src/QuadEstimatorEKF.cpp)
   - [`config/QuadEstimatorEKF.txt`](https://github.com/woodrowwiest/FCND-Estimation-CPP/blob/master/config/QuadEstimatorEKF.txt)

 - a re-tuned controller that, in conjunction with your tuned estimator, is capable of meeting the criteria laid out in Step 6 by submitting:
   - [`QuadControll.cpp`](https://github.com/woodrowwiest/FCND-Estimation-CPP/blob/master/src/QuadControl.cpp)
   - [`config/QuadControlParams.txt`](https://github.com/woodrowwiest/FCND-Estimation-CPP/blob/master/config/QuadControlParams.txt)

 - a write up addressing all the points of the rubric
   - This [README](https://github.com/woodrowwiest/FCND-Estimation-CPP/blob/master/README.md)

## Acknowledgements ##

Thanks:
 - to Fotokite for the initial development of the project code and simulator.
 - to [Udacity](https://www.udacity.com/) for the riveting course.  I'll look past the headaches this time ;)
