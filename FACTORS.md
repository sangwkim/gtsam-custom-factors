At the time of writing, there are 9 custom factors that are used in the project. 
```
TactileTransformFactor_3D
PoseOdometryFactor
StiffnessFactor
PenetrationFactor_2
StiffnessRatioFactor
ContactMotionFactor
FTLineFactor
NOCFactor
IDeformFactor
```
The source code of these factors can be found at 'cpp/' folder.

## TactileTransformFactor_3D

This factor impose a relative motion from one timestep to another. 

There are 4 Input Variables and 1 Measurement Pose:
1. Reference Pose at Reference Timestep (P1)
2. Reference Pose at Current Timestep (P2)
3. Another Pose at Reference Timestep (P3)
4. Another Pose at Current Timestep (P4)

In other words, it imposes: ((P1)^(-1)*(P3))^(-1)*((P2)^(-1)*(P4)) == (Measurement)

## PoseOdometryFactor

This factor impose that the last Pose variable (P3) be equal to the Pose difference between the first (P1) and second (P2) Pose variable.

In other words, it imposes: (P1)^(-1)*(P2) == (P3)

## StiffnessFactor

This factor is used to factorize the predicted deformation energy at future timesteps. The cost value (squared norm) of this factor is equivalent to the estimated deformation energy if the sensor is approximated as a linear spring with decoupled spring constant in each direction.

There are 5 Input Variables:
1. Undeformed Object Pose at Timestep 0 (P1)
2. Deformed Object Pose at Future Timestep (P2)
3. Gripper Pose at Timestep 0 (P3)
4. Gripper Pose at Future Timestep (P4)
5. Sensor Compliance (Stiffness) in Each Direction (Rx, Ry, Rz, x, y, z) (V)

It is very similar to the TactileTransformFactor_3D that it computes: ((P1)^(-1)*(P3))^(-1)*((P2)^(-1)*(P4)).
The only difference is that this vector is normalized by the Sensor Compliance Vector (V) (done by gtsam::ediv_ function in the code). The sensor compliance represents how compliant is the sensor in each direction. If the comliance in the x-direction is 1 and in the y-direction is 3, then deforming the sensor in x-direction by 1 will require the same energy as deforming in y-direction by 3. The sensor compliance is also equivalent to (spring constant)^(-1/2).

## PenetrationFactor_2

This factor impose the minimum penetration distance of contact through the object.

There are 3 Input Variables:
1. Undeformed Object Pose (Pn)
2. Estimated Contact (Pc)
3. Deformed Object Pose (Po)

It first computes the penetration distance by taking the z-component of the following Pose: ((Pn)^(-1)*(Pc))^(-1)*((Po)^(-1)*(Pc))

Then, it impose an inequality relation (penetration distance) >= (minimum distance) by using hinge loss function (implemented by if statement in the code).

## StiffnessRatioFactor

This factor infers the Compliance (Stiffness) Ratio between different components. It does this by imposing the fact that there should be no torque action at the point of the contact. The first, second, and third term of the output represents the torque in Rx, Ry, Rz direction. The fourth term is not being used currently so you can just ignore it.

There are 6 Input Variables:
1. Undeformed Object Pose at Timestep 0 (P1)
2. Deformed Object Pose at Current Timestep (P2)
3. Gripper Pose at Timestep 0 (P3)
4. Gripper Pose at Current Timestep (P4)
5. Contact Estimation at Current Timestep (P5)
6. Sensor Compliance (Stiffness) in Each Direction (Rx, Ry, Rz, x, y, z) (V)

## ContactMotionFactor

This factor impose that the control input is equal to the motion at the estimated contact point between consecutive timesteps.

There are 4 Input Variables:
1. Gripper Pose at previous timestep
2. Gripper Pose at next timestep
3. Estimated Contact
4. Control Input

In other words, the control input is the same as the local motion of the gripper at the estimated contact point.

## FTLineFactor

This factor plays a similar role as StiffnessRatioFactor but for the line contact rather than the point contact. It impose the fact that there should be no torque component along the line contact.

## NOCFactor

This factor is imposed when we want the penetration to be evenly distribution along the contact line or contact patch.

## IDeformFactor

This factor is used when transitioning from point to line contact. It just impose that the relative sensor deformation should be identical before and after the transition.


