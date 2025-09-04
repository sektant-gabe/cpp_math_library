# (WIP) Attitute C++ Math Library for Aerospacial/3D Games/Robotics/Graphics/Animations etc

## Requirements

- C++ 11
- Simple
- Lightweight
- Modular
- Fast
- Usable on embedded platforms (i.e arduino, microprocessors, etc...)
	- For example to use on a embedded Flight Control System (e.g. UAV Quadcopter Drones)
	- No virtual classed/inheritance
	- No dynamic memory allocation (Typical requirements for safety critical control code)


## Usage Structure

- Classes to represent the basic linear math elements:
	- Vector3
	- Matrix33

- Classes to represent the attitude representations:
	- DCM
	- EulerAngles
	- Quaternions

- Easy Class Construction
	- `Vector3 v1(1.0,2.0,3.0);`
	- `Matrix33 dcm = DCM::rotationX(1.2);`

- Easy Element Access
	- `double x = v1.x`
	- `double y = v1.data[1]`
	- `double m11 = dcm.m11`
	- `double m12 = dcm.data[0][1]`

- Classes to have overloaded mathematical operators (i.e `+`, `-`, `/`, `*`):
	- `Vector3 v3 = v1 * v2;`
	- `Matrix33 dcm2 = DCM::rotationX(0.1) * DCM::rotationY(2.5);`

- Free functions for special operations
	- `double d = dot(v1, v2);`
	- `Vector3 v3 = cross(v1, v2);`

- Vector transformations are handled automatically
	- `Vector3 v2_dcm = dcm * v1;`
	- `Vector3 v2_quat = quat * v1;`
	- `Vector3 v2_euler = euler * v1;`

- Conversion function between classes
	- `EulerAngles angles = dcm2EulerAngles(dcm, EulerAngles::EulerSequence::XYZ);`
	- `Matrix33 dcm = eulerAngles2DCM(angles);`
	- `Quaternion q = eulerAngles2Quat(angles);`

## Why do we need an attitude representation:

- Allows the attitude (or orientation) of an object to be mathematically described **relative to a reference frame**.
- Mathematical description which links the orientation **between two frames** of reference together, **allowing quantities in one frame to be express in the other frame.**
- Very important across a wide range of disciplines(physics, science, engineering, robotics, computer games, graphics, animation)

## Coordinate System

- Used to uniquely specify a point or quantity using a sequence of numbers (i.e. coordinates).
	- 3D Cartesian Coordinate System
	- Each axis is orthogonal to the other two(i.e a change in one will not affect the other)


$$\Large
\vec{v} = \begin{bmatrix}
x \\
y  \\
53\\
\end{bmatrix}
$$

## Reference Frames and Coordinate System

- World (Reference) Coordinate System
	- $\Large\mathcal{F}^w$
	- Fixed in space (stays at same location and attitude)

- Body/Local (Rotated) Coordinate System
	- $\Large\mathcal{F}^b$
	- Rigidly attached to the object whose attitude we would like to describe relative to the world frame

$$
\Large\Phi : \mathcal{F}^w   \leftrightarrow \mathcal{F}^b
$$


## Two Conventions:

1. Attitude = Rotation of *Rotated Frame* relative to *Reference Frame*

$$\Large\Phi : \mathcal{F}^w   \rightarrow \mathcal{F}^b$$

> Im using this one

2. Attitude = Rotation of *Rotated Frame* relative to *Reference Frame*

$$\Large\Phi : \mathcal{F}^b   \rightarrow \mathcal{F}^w$$

## How to represent the Attitude:

- Linear Transforms (Direct Cosine Matrix)
- Euler Angles
- Quaternions
- Rotations Vectors

### Each method has pros and cons:

- Ease of User Interpretation and Interaction (can you mentally picture it and describe it)
- Storage (size in memory)
- Numerical Issues (stability, computation performance, uniqueness)
- Integration and Kinematics (how to describe a rotating object or changing attitude)
- Interpolation (computer graphics and animation, smoothly changing between two orientations)
- Common problem in mathematics and engineering is to calculate how the attitude of an object **changes with time** when the **object is rotating**
	- Mathematical Analysis
	- Numerical Simulation
	- Computer Games
	- Computer Graphics


$$
\large\dot{\Phi} = \frac{\partial\Phi}{\partial t}
$$

$$
\large\Phi(t) = \int_0^t \dot{\Phi}dt
$$

## Instantaneous Body/World Rotation Rates


$$
\Large \vec{w} = \begin{bmatrix}
p \\
q \\
r \\
\end{bmatrix}
$$

## Attitude Rates and Angular Velocity
- We usually know (or measure/describe) the angular velocity in the body frame or world frame.
	- Need to find a relationship between the Angular Rates to Attitude Rates (nased on the method of representing the attitude)

$$
\Large \dot{\Phi} \neq \vec{w}
$$

$$
\Large \dot{\Phi} = f(\Phi,\vec{w})
$$
