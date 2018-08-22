# FCND - Building a Controller


# Content
1、Implemented body rate control in C++.
2、Implement roll pitch control in C++.
3、Implement altitude controller in C++.
4、Implement lateral position control in C++.
5、Implement yaw control in C++.
6、Implement calculating the motor commands given commanded thrust and moments in C++.


### Starter Code

The C++ implementation of controls is in source file QuadController.cpp, and the tuned parameters are in text file QuadControlParams.txt.


#### 1. Implemented body rate control in C++. The body rate control method returns a proportional controller on body rates to commanded moments. The method takes as input the actual pqr and the commanded pqr rates while also using the moments of inertia of the drone when calculating the commanded moments. 
I calculated the error of the body rotation rates from the difference between the actual pqr from the commanded pqr rates. Then multiply the moment of inertia by the error of the kp constant for each pqr (obtained body rotation rate by kpPQR rate) to get the momentCmd.


#### 2. Implement roll pitch control in C++. The roll pitch controller takes as input the desired acceleration in global XY coordinates, current or estimated attitude of the vehicle and desired collective thrust of the quad. It then calculates and returns the a V3F containing the desired pitch and roll rates. 
To calculate the desired pitch and roll rates, I started by dividing collThrustCmd by mass to get collective acceleration. This is used to compute the b_xTarget and b_yTarget values by constraining (the division of desired acceleration by collective acceleration on each axis x & y) in in -maxTiltAngle and maxTiltAngle. The p-terms for both b_x and b_y was computed by multiplying each by the kpBank variable. finally, the commanded p and q were computed with the formular: by substituting the p terms into the formula:
pqrCmd.x = (R(1,0) * b_x_Pterm - R(0,0) * b_y_Pterm) / R(2,2);
pqrCmd.y = (R(1,1) * b_x_Pterm - R(0,1) * b_y_Pterm) / R(2,2)


#### 3. Implement altitude controller in C++. The Altitude controller method accepts as input posZCmd, velZCmd: desired vertical position and velocity in NED [m]; posZ, velZ: current vertical position and velocity in NED [m]; accelZCmd: feed-forward vertical acceleration in NED [m/s2]; dt: the time step of the measurements [seconds]. The Altitude method then computes and returns the collective thrust command. For this method, I implemented a PID control. 
First I computed the error in the vertical position of the drone zErr as the difference between the actual position posZ and the commanded position posZCmd. Then calculated the The p term by multiplying zErrby by kpPosZ.
Next I calculated the vertical velocity error z_dotErr as the difference between the actual velocity velZ and the commanded velocity velZCmd. Then also calculated the The d term by multiplying z_dotErr by kpVelZ.
The integratedAltitudeError was computed from zErr by dt, the z_Iterm computed from KiPosZ by integratedAltitudeError.
The u_Lbar was computed from z_Pterm + z_Dterm + z_Iterm + accelZCmd.
Finally Thrust was calculated as below:
thrust= (u_Lbar - CONST_GRAVITY) / R(2,2);
thrust = -mass * CONSTRAIN(thrust, -maxAscentRate/dt, maxAscentRate/dt);


#### 4. Implement lateral position control in C++. The lateral position control method receives as inputs the desired position, desired velocity, current position, current velocity, feed-forward acceleration and it calculates and returns the desired horizontal accelerations. I calculated the posErr from the difference between the desired position and the current position, likewise the velErr from the difference between desired velocity and current velocity. The computation for horizontal acceleration was computed as below:
```
accelCmd = kpPosXY * posErr + kpVelXY * velErr + accelCmd;
    
    if(accelCmd.mag() > maxAccelXY) {
        accelCmd = accelCmd.norm() * maxAccelXY;
    }
```


#### 5. Implement yaw control in C++.The yaw controller method take as input yawCmd and yaw then computes and returns the commanded yaw rate.
I calculated the psi error by computing the difference between the actual yaw and the commanded yaw. Next I used the fmodf function to unwrap the measure to range. Finally, I calculated the commanded yaw rate by multiplying kpYaw by psi error.
```
float psiErr = yawCmd - yaw;
   psiErr = fmodf(psiErr, 2.0 * M_PI);
   yawRateCmd = kpYaw * psiErr;
```


#### 6. Implement calculating the motor commands given commanded thrust and moments in C++. This is the GenerateMotorCommands method which takes as input the desired collective thrust and desired rotation moment then sets the desired thrusts as below:

```
float l = L / sqrtf(2.f);
    
    float cBar = collThrustCmd;
    float pBar = momentCmd.x / l;
    float qBar = momentCmd.y / l;
    float rBar = -momentCmd.z / kappa;
    
    cmd.desiredThrustsN[0] = (cBar + pBar + qBar + rBar) / 4.f;
    cmd.desiredThrustsN[1] = (cBar - pBar + qBar - rBar) / 4.f;
    cmd.desiredThrustsN[2] = (cBar + pBar - qBar - rBar) / 4.f;
    cmd.desiredThrustsN[3] = (cBar - pBar - qBar + rBar) / 4.f;
```


### Execute the controler
#### 1. Does it work?
It works!
