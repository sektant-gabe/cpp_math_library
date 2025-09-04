# Attitute C++ Math Library for Aerospacial/Game Development/Robotis/Graphics/Animation etc

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
> **Info**
> We are using this one

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

- A matrix is a M x N rectangular array of numbers or expressions
- Notation: **BOLD CAPITAL LETTER**
$$
\Large
\textbf{A}
$$


## Addition and Subtraction

- Matrices **MUST** be of the same size
- Operation is applied on a **per element** basis

$$\begin{align}
\large \symbf{C = A + B}\\
\large \symbf{C = A - B}\\
\end{align}
$$
$$
\Large
\begin{bmatrix}
a_{11} + b_{11} & a_{12} + b_{12} \\
a_{21} + b_{21} & a_{22} + b_{22}
\end{bmatrix}=
\begin{bmatrix}
a_{11} & a_{12} \\
a_{21} & a_{22}
\end{bmatrix}=
\begin{bmatrix}
b_{11} & b_{12} \\
b_{21} & b_{22}
\end{bmatrix}
$$


## Scalar Multiplication and Division

- Operation is applied on a **per element** basis
$$
\begin{gather}
\large \mathbf{B} = s_{}\mathbf{A}\\
\\
\large \mathbf{B} = \frac{1}{s_{}}\mathbf{A}\\
\end{gather}
$$
$$
\Large
\begin{bmatrix}
sa_{11} & sa_{12} \\[0.3em]
sa_{21} & sa_{22}
\end{bmatrix}
=s
\begin{bmatrix}
a_{11} & a_{12} \\[0.3em]
a_{21} & a_{22}
\end{bmatrix}
$$


## Matrix Multiplication

- Matrices can be multiplied if they are **conformable!**
$$
\large\mathbf{C}(m \times n) = \mathbf{A}(m \times t) \mathbf{B}(t \times n)
$$
$$\large
c_{ij} = \sum_{k=1}^{t} \ a_{ik}\ b_{kj}
$$
$$
\begin{gather}
j = 1,\dots ,t\\
i = 1,\dots ,m
\end{gather}
$$

![[Pasted image 20250903181501.png]]
$$
\stackrel{\mbox{$3\times2$}}{%
\large\begin{bmatrix}
c_{11} & c_{12} \\[0.3em]
c_{21} & c_{22} \\[0.3em]
c_{31} & c_{32} \\[0.3em]
\end{bmatrix}
}=
\stackrel{\mbox{$3\times3$}}{%
\large\begin{bmatrix}
a_{11} & a_{12} & a_{13}\\[0.3em]
a_{21} & a_{22} & a_{23}\\[0.3em]
a_{31} & a_{32} & a_{33}\\[0.3em]
\end{bmatrix}
}
\stackrel{\mbox{$3\times2$}}{%
\large\begin{bmatrix}
b_{11} & b_{12} \\[0.3em]
b_{21} & b_{22} \\[0.3em]
b_{31} & b_{32} \\[0.3em]
\end{bmatrix}
}
$$

- Corresponding elements multiplied together and then summed


$$
\large
\begin{bmatrix}
(a_{11}b_{11} + a_{12}b_{21} + a_{13}b_{31})\ \ (a_{11}b_{12} + a_{12}b_{22} + a_{13}b_{32}) \\[0.3em]
(a_{21}b_{11} + a_{22}b_{21} + a_{23}b_{31})\ \ (a_{21}b_{12} + a_{12}b_{22} + a_{13}b_{32}) \\[0.3em]
(a_{31}b_{11} + a_{32}b_{21} + a_{33}b_{31})\ \ (a_{31}b_{12} + a_{22}b_{22} + a_{33}b_{32}) \\[0.3em]
\end{bmatrix}
$$

- Matrix Multiplication is **not commutative**
$$
\mathbf{AB}\neq \mathbf{BA}
$$
- Left or Right multiplication order matters with matrices


## Matrix Transpose

$$
\Huge\mathbf{A}^T
$$

- Flips the elements of the matrix along the diagonal
![[Pasted image 20250903183630.png]]

$$\large
\mathbf{A}^{T} =
\begin{bmatrix}
a_{11} & a_{12} & \cdots & a_{m1} \\[0.3em]
a_{21} & a_{22} & \cdots & a_{m2} \\[0.3em]
\vdots & \vdots & \ddots & \vdots \\[0.3em]
a_{1n} & a_{2n} & \cdots & a_{mn}
\end{bmatrix}
\ \ \ \ \ \ \ \ \ \ \
\mathbf{A} =
\begin{bmatrix}
a_{11} & a_{12} & \cdots & a_{1n} \\[0.3em]
a_{21} & a_{22} & \cdots & a_{2n} \\[0.3em]
\vdots & \vdots & \ddots & \vdots \\[0.3em]
a_{m1} & a_{m 2} & \cdots & a_{mn}
\end{bmatrix}
$$


## Identity Matrix

- Square matrix of size (**N** x **N**) with all ones on the diagonal
- Whenever we multiple a matrix by the *Identity matrix*, we just get the original matrix
$$\mathbf{A}\times \mathbf{I} = \mathbf{A}$$
$$\large
\mathbf{I} =
\begin{bmatrix}
1 & 0 & \cdots & 0 \\[0.3em]
1 & 0 & \cdots & 0 \\[0.3em]
\vdots & \vdots & \ddots & \vdots \\[0.3em]
1 & 0 & \cdots & 0 \\[0.3em]
\end{bmatrix}
$$


## Matrix Determinant

$$\Huge
\det(\mathbf{A})
$$

- Applies to **square matrices** only (**N** x **N**)
- Encodes properties of the matrix

$$
\det \begin{pmatrix}
\begin{bmatrix}
a & b \\[0.3em]
c & d \\[0.3em]
\end{bmatrix}
\end{pmatrix}
= ad - bc
$$
$$
\det \begin{pmatrix}
\begin{bmatrix}
a & b & c \\[0.3em]
d & e & f \\[0.3em]
g & h & i \\[0.3em]
\end{bmatrix}
\end{pmatrix}
= a(ei - fh) - b(di-fg) + c(dh-eg)
$$


## Matrix Inverse

$$\Huge
\mathbf{A}^{-1}
$$

- A **square matrix** which satisfies the condition:
$$\large\mathbf{A}^{-1} \mathbf{A} = \mathbf{AA}^{-1} = \mathbf{I}$$
- **Not all square matrices are invertible**
- A square matrix is invertible if the determinant does not equal zero
$$\large
\det(\mathbf{A}) \neq 0
$$

Consider the **scalar** equation:
$$
\large y = ax
$$
To find the value for $\large x$:
$$\large
x = \frac{1}{a}y=a^{-1}y
$$
Can we do the same with matrices?
$$\Huge
\begin{gather}
\mathbf{X} = \mathbf{AY}\\
\mathbf{A}^{-1}\mathbf{X} = \mathbf{A}^{-1}\mathbf{AY}\\
\end{gather} \ \ \ \ \
\mathbf{\rightarrow}\ \ \ \ \
\mathbf{A}^{-1}\mathbf{X} = \mathbf{Y}
$$


## Orthogonal Matrix

- Square Matrix (**N** x **N**)
- Transpose is equal to its inverse
- Determinant is equal to **1**
$$\Huge
\begin{gather}
\mathbf{A}^{T}  = \mathbf{A^{-1}}\\
\mathbf{A}^{T}\mathbf{A} = \mathbf{A}\mathbf{A}^{T} = \mathbf{I}
\end{gather}
$$
