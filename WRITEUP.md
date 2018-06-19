# Building an estimator in C++

## Sensor noise (GPS and accelerometer data standard deviation)
### Specifications
- The calculated standard deviation should correctly capture ~68% of the sensor measurements
- Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements
### GPS and accelerometer data standard deviation details
- Purpose is to calculate the standard deviation of the GPS and accelerometer data
- Generate log files config/log/Graph1.txt for GPS data and config/log/Graph2.txt for accelerometer data by running the simulator using scenario 6
- Use Python to inject file data into arrays
- Use Python standard deviation function to calculate separate standard deviation for GPS and accelerometer
- We end up with:
standard deviation = 0.72 for GPS
standard deviation = 0.52 for accelerometer

Commented code:
```

import numpy as np    

# Loads the GPS data file
gps_data = np.loadtxt('config/log/Graph1.txt', delimiter=',', dtype='Float64', skiprows=1)

# Calculates and prints standard deviation for the timestamps and the GPS data, we want the second
print("gps:{}".format(np.std(gps_data,axis=0)))

# Loads the accelerometer data file
acc_data = np.loadtxt('config/log/Graph2.txt', delimiter=',', dtype='Float64', skiprows=1)

# Calculates and prints standard deviation for the timestamps and the accelerometer data, we want the second
print("acc:{}".format(np.std(acc_data,axis=0)))
```


## Attitude estimation
### Specifications
- The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation
- The integration scheme should use quaternions to improve performance over the current simple integration scheme

### Attitude estimation optimization details
- Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.

Commented code:
```
void QuadEstimatorEKF::UpdateFromIMU(V3F accel, V3F gyro)
{
  // Improve a complementary filter-type attitude filter
  //
  // Currently a small-angle approximation integration method is implemented
  // The integrated (predicted) value is then updated in a complementary filter style with attitude information from accelerometers
  //
  // Implement a better integration method that uses the current attitude estimate (rollEst, pitchEst and ekfState(6))
  // to integrate the body rates into new Euler angles.
  //
  // HINTS:
  //  - there are several ways to go about this, including:
  //    1) create a rotation matrix based on your current Euler angles, integrate that, convert back to Euler angles
  //    OR
  //    2) use the Quaternion<float> class, which has a handy FromEuler123_RPY function for creating a quaternion from Euler Roll/PitchYaw
  //       (Quaternion<float> also has a IntegrateBodyRate function, though this uses quaternions, not Euler angles)

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // SMALL ANGLE GYRO INTEGRATION:
  // (replace the code below)
  // make sure you comment it out when you add your own code -- otherwise e.g. you might integrate yaw twice

    // Using the Quaternion class initialized with current attitude estimate (rollEst, pitchEst and ekfState(6))
    Quaternion<float> q1 = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));

    // Integrate pqr in gyro in the quaternion
    q1.IntegrateBodyRate(gyro,dtIMU);

    // Convert back from quaternion to Euler angles
    V3D eulerRPY = q1.ToEulerRPY();

    float predictedPitch = eulerRPY.y;
    float predictedRoll  = eulerRPY.x;
    ekfState(6) = eulerRPY.z;

    // normalize yaw to -pi .. pi
    if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
    if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  // CALCULATE UPDATE
  accelRoll = atan2f(accel.y, accel.z);
  accelPitch = atan2f(-accel.x, 9.81f);

  // FUSE INTEGRATION AND UPDATE
  rollEst = attitudeTau / (attitudeTau + dtIMU) * (predictedRoll)+dtIMU / (attitudeTau + dtIMU) * accelRoll;
  pitchEst = attitudeTau / (attitudeTau + dtIMU) * (predictedPitch)+dtIMU / (attitudeTau + dtIMU) * accelPitch;

  lastGyro = gyro;
}
```

## Prediction step
### Specifications
- The prediction step should include the state update element (PredictState() function)
a correct calculation of the Rgb prime matrix
and a proper update of the state covariance.
- The acceleration should be accounted for as a command in the calculation of gPrime
- The covariance update should follow the classic EKF update equation.
#### Prediction step details
- Implement all of the elements of the prediction step for the estimator


Commented code
```
VectorXf QuadEstimatorEKF::PredictState(VectorXf curState, float dt, V3F accel, V3F gyro)
{
  assert(curState.size() == QUAD_EKF_NUM_STATES);
  VectorXf predictedState = curState;
  // Predict the current state forward by time dt using current accelerations and body rates as input
  // INPUTS:
  //   curState: starting state
  //   dt: time step to predict forward by [s]
  //   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
  //   gyro: body rates of the vehicle, in body frame [rad/s]
  //   
  // OUTPUT:
  //   return the predicted state as a vector

  // HINTS
  // - dt is the time duration for which you should predict. It will be very short (on the order of 1ms)
  //   so simplistic integration methods are fine here
  // - we've created an Attitude Quaternion for you from the current state. Use
  //   attitude.Rotate_BtoI(<V3F>) to rotate a vector from body frame to inertial frame
  // - the yaw integral is already done in the IMU update. Be sure not to integrate it again here

  Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, curState(6));

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Build predictedState from curState
  // curState(0) : x position in global frame
  // curState(1) : y position in global frame
  // curState(2) : y position in global frame
  // curState(3) : x velocity in global frame
  // curState(4) : y velocity in global frame
  // curState(5) : z velocity in global frame
  // curState(6) : yaw angle in global frame

  // position update in global frame
  predictedState(0) = curState(0) + curState(3) * dt; // x position update in global frame
  predictedState(1) = curState(1) + curState(4) * dt; // y position update in global frame
  predictedState(2) = curState(2) + curState(5) * dt; // z position update in global frame

  // first convert accel vector from body frame to global frame
  V3F globalFrameAccel = attitude.Rotate_BtoI(accel);

  // velocity update in global frame
  predictedState(3) = curState(3) + globalFrameAccel.x * dt; // x velocity update in global frame
  predictedState(4) = curState(4) + globalFrameAccel.y * dt; // y velocity update in global frame
  predictedState(5) = curState(5) + (globalFrameAccel.z - CONST_GRAVITY) * dt; // z velocity update in global frame

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return predictedState;
}

MatrixXf QuadEstimatorEKF::GetRbgPrime(float roll, float pitch, float yaw)
{
  // first, figure out the Rbg_prime
  MatrixXf RbgPrime(3, 3);
  RbgPrime.setZero();

  // Return the partial derivative of the Rbg rotation matrix with respect to yaw. We call this RbgPrime.
  // INPUTS:
  //   roll, pitch, yaw: Euler angles at which to calculate RbgPrime
  //   
  // OUTPUT:
  //   return the 3x3 matrix representing the partial derivative at the given point

  // HINTS
  // - this is just a matter of putting the right sin() and cos() functions in the right place.
  //   make sure you write clear code and triple-check your math
  // - You can also do some numerical partial derivatives in a unit test scheme to check
  //   that your calculations are reasonable

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    float cPhi = cosf(roll);
    float sPhi = sinf(roll);

    float cThe = cosf(pitch);
    float sThe = sinf(pitch);

    float cPsi = cosf(yaw);
    float sPsi = sinf(yaw);



    RbgPrime(0,0) = -cThe * sPsi; // −cosθsinψ
    RbgPrime(0,1) = -sPhi * sThe * sPsi - cPhi * cPsi; // −sinφsinθsinψ − cosφcosψ
    RbgPrime(0,2) = -cPhi * sThe * sPsi + sPhi * cPsi; // −cosφsinθsinψ + sinφcosψ

    RbgPrime(1,0) = cThe * cPsi; // cosθcosψ
    RbgPrime(1,1) = sPhi * sThe * cPsi - cPhi * sPsi; // sinφsinθcosψ−cosφsinψ
    RbgPrime(1,2) = cPhi * sThe * cPsi + sPhi * sPsi; // cosφsinθcosψ+sinφsinψ

    RbgPrime(2,0) = 0.0f;
    RbgPrime(2,1) = 0.0f;
    RbgPrime(2,2) = 0.0f;


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return RbgPrime;
}

void QuadEstimatorEKF::Predict(float dt, V3F accel, V3F gyro)
{
  // predict the state forward
  VectorXf newState = PredictState(ekfState, dt, accel, gyro);

  // Predict the current covariance forward by dt using the current accelerations and body rates as input.
  // INPUTS:
  //   dt: time step to predict forward by [s]
  //   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
  //   gyro: body rates of the vehicle, in body frame [rad/s]
  //   state (member variable): current state (state at the beginning of this prediction)
  //   
  // OUTPUT:
  //   update the member variable cov to the predicted covariance

  // HINTS
  // - update the covariance matrix cov according to the EKF equation.
  //
  // - you may find the current estimated attitude in variables rollEst, pitchEst, state(6).
  //
  // - use the class MatrixXf for matrices. To create a 3x5 matrix A, use MatrixXf A(3,5).
  //
  // - the transition model covariance, Q, is loaded up from a parameter file in member variable Q
  //
  // - This is unfortunately a messy step. Try to split this up into clear, manageable steps:
  //   1) Calculate the necessary helper matrices, building up the transition jacobian
  //   2) Once all the matrices are there, write the equation to update cov.
  //
  // - if you want to transpose a matrix in-place, use A.transposeInPlace(), not A = A.transpose()
  //

  // we'll want the partial derivative of the Rbg matrix
  MatrixXf RbgPrime = GetRbgPrime(rollEst, pitchEst, ekfState(6));

  // we've created an empty Jacobian for you, currently simply set to identity
  MatrixXf gPrime(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
  gPrime.setIdentity();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    // update gPrime values from the Jacobian Matrix
    gPrime(0,3) = dt;
    gPrime(1,4) = dt;
    gPrime(2,5) = dt;

    gPrime(3,6) = (RbgPrime(0,0) * accel.x + RbgPrime(0,1) * accel.y + RbgPrime(0,1) * accel.z) * dt;
    gPrime(4,6) = (RbgPrime(1,0) * accel.x + RbgPrime(1,1) * accel.y + RbgPrime(1,1) * accel.z) * dt;
    gPrime(5,6) = (RbgPrime(2,0) * accel.x + RbgPrime(2,1) * accel.y + RbgPrime(2,1) * accel.z) * dt;

    // from the covariance update formula
    ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  ekfState = newState;
}
```
## magnetometer update
### Specifications
- The update should properly include the magnetometer data into the state
- Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way)
### magnetometer details
- Implement the magnetometer update

Commented code
```
void QuadEstimatorEKF::UpdateFromMag(float magYaw)
{
  VectorXf z(1), zFromX(1);
  z(0) = magYaw;

  MatrixXf hPrime(1, QUAD_EKF_NUM_STATES);
  hPrime.setZero();

  // MAGNETOMETER UPDATE
  // Hints:
  //  - Your current estimated yaw can be found in the state vector: ekfState(6)
  //  - Make sure to normalize the difference between your measured and estimated yaw
  //    (you don't want to update your yaw the long way around the circle)
  //  - The magnetomer measurement covariance is available in member variable R_Mag
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // From measurement fuction derivative
  hPrime(0,6) = 1;
  zFromX(0) = ekfState(6);

  // normalize yaw to -pi .. pi
  float delta = z(0) - zFromX(0);
  float normalizer = 2.f*F_PI;
  if (delta > F_PI) {
      zFromX(0) += normalizer;
  }
  else if (delta < -F_PI) {
      zFromX(0) -= normalizer;
  }


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  Update(z, hPrime, R_Mag, zFromX);
}
```
### GPS update
#### Specifications
- The estimator should correctly incorporate the GPS information to update the current state estimate
#### yaw details
- Implement the GPS update

Commented code
```
void QuadEstimatorEKF::UpdateFromGPS(V3F pos, V3F vel)
{
  VectorXf z(6), zFromX(6);
  z(0) = pos.x;
  z(1) = pos.y;
  z(2) = pos.z;
  z(3) = vel.x;
  z(4) = vel.y;
  z(5) = vel.z;

  MatrixXf hPrime(6, QUAD_EKF_NUM_STATES);
  hPrime.setZero();

  // GPS UPDATE
  // Hints:
  //  - The GPS measurement covariance is available in member variable R_GPS
  //  - this is a very simple update
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Set the Identity part by hand
  hPrime(0,0) = 1;
  hPrime(1,1) = 1;
  hPrime(2,2) = 1;
  hPrime(3,3) = 1;
  hPrime(4,4) = 1;
  hPrime(5,5) = 1;

  // Get measurements
  zFromX(0) = ekfState(0);
  zFromX(1) = ekfState(1);
  zFromX(2) = ekfState(2);
  zFromX(3) = ekfState(3);
  zFromX(4) = ekfState(4);
  zFromX(5) = ekfState(5);


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  Update(z, hPrime, R_GPS, zFromX);
}
```
## flight evaluation
### Specifications 1
- For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided
- The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements
### details 1
- Meet the performance criteria of each step
### Results 1
- Constraints are respected, and the drone flies it's path

### Specifications 2
- The controller developed in the previous project should be de-tuned to successfully meet the performance criteria of the final scenario (<1m error for entire box flight).
### details 2
- De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.
### Results 2
- Constraints are respected, and the drone flies it's path with <1m error for entire box flight
