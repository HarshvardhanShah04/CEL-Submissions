# 📘 Extended Kalman Filter for Vehicle State Estimation

## Project Title

**Extended Kalman Filter Implementation for Nonlinear Vehicle System**

---

## Objective

To design and implement an Extended Kalman Filter (EKF) for a nonlinear dynamic system representing the longitudinal motion of a vehicle, in order to estimate position and velocity from noisy position measurements.

The system models a vehicle moving along an inclined road under the influence of driving force, aerodynamic drag, and gravity. Since only position measurements are available (similar to GPS), the EKF is used to estimate velocity and improve overall state estimation accuracy.

---

## System Description

The system represents a **vehicle moving in one dimension** with:

* Applied driving force
* Quadratic drag force
* Gravitational component due to road inclination

### State Vector

$$
x = \begin{bmatrix}
x_1 \
x_2
\end{bmatrix}
$$

* $x_1$: Position
* $x_2$: Velocity

---

## Mathematical Model

### System Dynamics

$$
\dot{x} =
\begin{bmatrix}
x_2 \
\frac{F - c x_2^2 - m g \sin\theta}{m}
\end{bmatrix}
$$

This describes how the vehicle accelerates under force, drag, and gravity.

---

### Measurement Model

$$
z_k = H x_k + v_k
$$

$$
H = [1 ;; 0]
$$

Only position is measured (velocity is unobserved).

---

## Noise Modelling

### Process Noise

$$
w_k \sim \mathcal{N}(0, Q)
$$

Represents uncertainty in the system dynamics.

### Measurement Noise

$$
v_k \sim \mathcal{N}(0, R)
$$

Represents sensor noise.

---

## Implementation Details

### Simulation

* Ground truth generated using **Runge–Kutta 4 (RK4)**
* Process noise added to simulate real system behavior
* Noisy measurements generated from position

---

## EKF Algorithm

### Prediction Step

$$
\hat{x}*{k|k-1} = \hat{x}*{k-1} + dt , f(\hat{x}_{k-1})
$$

Predicts the next state using system dynamics.

$$
P_{k|k-1} = F_k P_{k-1} F_k^T + Q
$$

Updates uncertainty after prediction.

---

### Update Step

$$
y_k = z_k - H \hat{x}_{k|k-1}
$$

Measurement residual (error between actual and predicted measurement).

$$
S_k = H P_{k|k-1} H^T + R
$$

Uncertainty in measurement prediction.

$$
K_k = P_{k|k-1} H^T S_k^{-1}
$$

Kalman Gain — determines trust between model and measurement.

$$
\hat{x}*k = \hat{x}*{k|k-1} + K_k y_k
$$

Corrected state estimate.

$$
P_k = (I - K_k H) P_{k|k-1}
$$

Updated estimation uncertainty.

---

## Key Features

* Handles **nonlinear vehicle dynamics**
* Estimates **unmeasured velocity**
* Reduces effect of **sensor noise**
* Uses **Jacobian-based linearization**

---

## Results

* EKF successfully smooths noisy position measurements
* Velocity is accurately estimated despite not being measured
* Estimated states closely follow true system states

---

## Applications

* Autonomous vehicles
* GPS-based tracking systems
* Robotics state estimation
* Navigation and control systems

---

## Conclusion

The Extended Kalman Filter effectively estimates the full state of a nonlinear vehicle system using only partial noisy measurements, demonstrating its usefulness in real-world estimation problems.

---
