@ingroup group_kalman_filter

In the realm of estimation and signal processing, the Kalman Filter (KF) and its extension, the Extended Kalman Filter (EKF), stand as fundamental tools for state estimation in systems prone to noise and uncertainty. These filters are renowned for their ability to assimilate noisy sensor data with dynamic system models, producing optimal estimates of the true state of a system.

There are lots of equivalent implementations of the KF and the EKF, most of them with different nomencalture. In this code, the implementation followed is the one present in the book:

- Probabilistic Robotics (S. Thrun et al)

If the user wishes, they can check chapters 3.2 and 3.3 for a more in-depth explanation.

### Kalman Filter (KF):

The Kalman Filter, pioneered by Rudolf E. Kalman in the early 1960s, is a recursive algorithm designed to estimate the state of a linear dynamic system disturbed by Gaussian noise. It operates in two main steps: prediction and update.

- Prediction: Utilizing the system's dynamics model and the previous state estimate, the KF predicts the current state of the system.

- Update: This step incorporates the measurements obtained from sensors, adjusting the state estimate based on the disparity between predicted and observed values, taking into account the uncertainties associated with both.

Despite its elegance and effectiveness, the classic Kalman Filter is limited to linear system models and Gaussian noise assumptions, making it unsuitable for many real-world applications characterized by non-linearities.

### Extended Kalman Filter (EKF):

To address the limitations of the KF in handling nonlinear systems, the Extended Kalman Filter (EKF) was developed. Introduced by R.E. Kalman and R.S. Bucy in the late 1960s, the EKF extends the KF framework to nonlinear systems by linearizing the system dynamics and measurement functions around the current estimate.

- Linearization: At each iteration, the EKF linearizes the system and measurement equations using first-order Taylor series expansion, allowing it to apply the standard Kalman Filter equations to estimate the system state.
- Propagation of Uncertainty: Despite linearization, the EKF propagates the uncertainty through the nonlinear transformations, ensuring a more accurate estimation of the system state while considering the nonlinearity of the system.

The Extended Kalman Filter has found widespread application in various fields, including aerospace, robotics, and navigation, where nonlinearities are prevalent, and accurate state estimation is crucial.

In summary, while the Kalman Filter provides an optimal solution for linear systems with Gaussian noise, the Extended Kalman Filter extends this capability to nonlinear systems, making it a versatile tool for state estimation in a wide range of real-world applications.

### Block Diagram

We attach a block diagram to aid in the understanding of the filter and it's behavior

<div style="text-align: center;">
    <img src="https://cernbox.cern.ch/remote.php/dav/files/jplayang/eos/project/c/cern-robotic-framework/CppRoboticFramework/Modules/Math/KalmanFilter/docs/KalmanFilter.png?access_token=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJyZXZhIiwiZXhwIjoxNzIyODA4Nzk5LCJpYXQiOjE3MjIzNDgyMTMsImlzcyI6Imh0dHBzOi8vYXV0aC5jZXJuLmNoL2F1dGgvcmVhbG1zL2Nlcm4iLCJ1c2VyIjp7ImlkIjp7ImlkcCI6Imh0dHBzOi8vYXV0aC5jZXJuLmNoL2F1dGgvcmVhbG1zL2Nlcm4iLCJvcGFxdWVfaWQiOiJqcGxheWFuZyIsInR5cGUiOjF9LCJ1c2VybmFtZSI6ImpwbGF5YW5nIiwibWFpbCI6ImpvcmdlLnBsYXlhbi5nYXJhaUBjZXJuLmNoIiwiZGlzcGxheV9uYW1lIjoiSm9yZ2UgUGxheWFuIEdhcmFpIiwidWlkX251bWJlciI6MTM1MjczLCJnaWRfbnVtYmVyIjoyNzY2fSwic2NvcGUiOnsidXNlciI6eyJyZXNvdXJjZSI6eyJkZWNvZGVyIjoianNvbiIsInZhbHVlIjoiZXlKd1lYUm9Jam9pTHlKOSJ9LCJyb2xlIjoxfX19.sFAv76blqG0c7vmKD1D18-W3XtEsT2g3JgLO73RT3Ro" alt="Image" style="width:60%;height:auto;" />
</div>
