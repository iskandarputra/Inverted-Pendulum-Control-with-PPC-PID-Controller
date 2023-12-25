# Inverted Pendulum Control Simulation (PPC-PID vs PID Controller!) 🎡
An interactive Python simulation for controlling an inverted pendulum using Prescribed Performance Control (PPC) based PID controller 📈 and comparing against a standard PID controller.

## Background 🔬
The inverted pendulum is one of the most fundamental unstable systems 📉 used in control systems research and education. It is easy to visualize but exhibits complex dynamics that must be actively controlled to balance the pendulum upright.

PID (Proportional-Integral-Derivate) controllers are the most widely used control algorithms in industry 🏭. However, tuning PID gains to get good setpoint tracking, transient performance and disturbance rejection can be challenging, especially for nonlinear systems like the pendulum.

**Prescribed Performance Control (PPC)** 🎯 provides a methodical way to get guaranteed transient and steady state performance for uncertain nonlinear systems like the inverted pendulum. This is achieved by transforming the error to bound it within a decaying performance function p(t) 📈.

This simulation allows visualizing PPC-PID 📈 and PID control of a pendulum side-by-side. The effect of PPC on the pendulum response can be clearly understood. 😃

## Features ✨
- Animated visualization of inverted pendulum system 🎪
- Simulation of pendulum dynamics and actuator ⚙️
- Implementation of PPC-PID control algorithm 📈
- Conventional PID controller for comparison
- Adjustable setpoint and configurable PID gains 🎚️
- Interactive sliders to tune parameters 🕹️
- Pause ⏸, resume ▶ and reset 🔄 functionality
- Plots for angle, error bounds 📊, control input over time
- Performance comparison via indices 📈

## Getting Started 🚀
**Dependencies**: numpy, matplotlib, tkinter

Clone this repo and run:

```bash
python ppc_pid_pendulum.py
```

## Instructions: 📋

- Use sliders to tune setpoint and PID gains 🕹️
- Click buttons to pause/resume/reset ⏸ ▶ 🔄
- Observe pendulum response visually 👀
- Analyze tracking performance from plots 📈
- Compare indices between controllers ⚖️
