###########################################################################################################################
#
# @ppc_pid_pendulum.py
# @brief Simulates a pendulum system with PID & PPC-PID control.
#
# @details This script provides classes and functions to simulate a pendulum system with two PID controllers:
# PPC-PID (Prescribed Performance Control PID) and Conventional PID.
#
# @author Mohd Iskandar Putra <iskandarputra1995@gmail.com>
# @date December 10, 2023
# @version 1.0
# @bugs Feel free to fix and optimize it...
#
# NOTICE: Kind Request for Citation
# If you intend to use or publish this PPC formulation, we kindly request proper citation of the original work.
# Access the original publication at: https://www.sciencedirect.com/science/article/pii/S2667241323000149
# Your adherence to citation guidelines is greatly appreciated when utilizing or referring to this code.
#
# Installation:
# pip install numpy
# pip install matplotlib
# sudo apt-get install python3-tk
#
###########################################################################################################################

import tkinter as tk
from tkinter import Scale, Button
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from math import cos, sin, pi

# Define the prescribed performance function p(t)
def p(t):
    P0 = 0.6  # Change based on your scenario : PPC starting point
    Pi = 0.2  # Change based on your scenario : End of the PPC bounds
    t0 = 0.5  # Change based on your scenario : Time to reach to the Pi
    L = 1     # Change based on your scenario : PPC convergence rate
    
    # IMPORTANT NOTICE: If you intend to publish or use this formulation, citation is required.
    # Please refer to the original published work by Mohd Iskandar Putra:
    # https://www.sciencedirect.com/science/article/pii/S2667241323000149
    return np.real(((P0 - Pi) * np.exp((-5 / (t0**2)) * (t * (np.tanh(L * (t - t0)) + 1))**2)) + Pi)

# Define the inverse error transformation function
def error_trans(e, pt):
    if abs(e) >= pt:
        return 0
    
    eps = 1e-6
    if abs(e / pt) >= 1 - eps:
        return np.sign(e) * 0.5 * np.log(eps)
    
    return np.real(0.5 * np.log(((e / pt) + 1) / (1 - (e / pt))))

class PPC_PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.time = 0.0
        self.theta = 0.0
        self.theta_dot = 0.0
        
    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def set_Kp(self, Kp):
        self.Kp = Kp

    def set_Ki(self, Ki):
        self.Ki = Ki

    def set_Kd(self, Kd):
        self.Kd = Kd

    def update(self, feedback_value, dt):
        error = self.setpoint - feedback_value
        pt = p(self.time)

        if abs(error) > 0.2:
            transformed_error = error
        else:
            transformed_error = error_trans(error, pt) # Use Error transformation for PPC instead of the usual feebback error
            
        self.integral += transformed_error * dt
        derivative = (transformed_error - self.last_error) / dt
        output = self.Kp * transformed_error + self.Ki * self.integral + self.Kd * derivative
        output = max(min(output, 10), -10)
        self.last_error = transformed_error
        self.time += dt
        return output

class conv_pid:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.time = 0.0
        self.theta = 0.0 
        self.theta_dot = 0.0

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def set_Kp(self, Kp):
        self.Kp = Kp

    def set_Ki(self, Ki):
        self.Ki = Ki

    def set_Kd(self, Kd):
        self.Kd = Kd

    def update(self, feedback_value, dt):
        error = self.setpoint - feedback_value
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = max(min(output, 10), -10)
        self.last_error = error
        return output

class PendulumApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Pendulum Control System : Introduce Prescribed Performance Control Approach")

        self.canvas = tk.Canvas(root, width=400, height=400)
        self.canvas.pack(side=tk.LEFT)
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        plt.subplots_adjust(hspace=0.5) 

        self.ax1.set_xlabel('Time')
        self.ax1.set_ylabel('Theta')
        self.ppc_line, = self.ax1.plot([], [], label='PPC-PID')
        self.pid_line, = self.ax1.plot([], [], label='PID')
        self.ax1.legend()

        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Decay Function & Tracking Error')
        self.ppc_error_line, = self.ax2.plot([], [], label='PPC-PID Error')
        self.pid_error_line, = self.ax2.plot([], [], label='PID Error')
        self.pt_line1, = self.ax2.plot([], [], label='p(t) Bound')
        self.pt_line2, = self.ax2.plot([], [], label='-p(t) Bound')
        self.ax2.legend()

        self.ax3.set_xlabel('Time')
        self.ax3.set_ylabel('PID Output')
        self.ppc_pid_output, = self.ax3.plot([], [], label='PPC PID Output')
        self.conv_pid_output, = self.ax3.plot([], [], label='PID Output')  
        self.ax3.legend()

        self.ax4.set_xlabel('Performance Index')
        self.ax4.set_ylabel('Values')

        self.indices = ['IAE', 'ITAE', 'ISE']
        self.index_values_ppc_pid = [0, 0, 0]
        self.index_values_pid = [0, 0, 0]
        bar_width = 0.35

        x = np.arange(len(self.indices))
        self.bars_ppc_pid = self.ax4.bar(x - bar_width / 2, self.index_values_ppc_pid, bar_width, label='PPC-PID')
        self.bars_pid = self.ax4.bar(x + bar_width / 2, self.index_values_pid, bar_width, label='PID')
        self.ax4.set_xticks(x)
        self.ax4.set_xticklabels(self.indices)
        self.ax4.legend()


        self.graph = FigureCanvasTkAgg(self.fig, master=root)
        self.graph.get_tk_widget().pack(side=tk.LEFT)

        self.theta = 0.0
        self.theta_dot = 0.0

        self.setup_simulation()
        self.setup_pid_controller()
        self.create_sliders()
        self.create_restart_button()
        self.pause_button = Button(self.root, text="Pause", command=self.pause_simulation, width=15)
        self.pause_button.pack()
        self.resume_button = Button(self.root, text="Resume", command=self.resume_simulation, width=15)
        self.resume_button.pack()
        self.stop_button = Button(self.root, text="Stop", command=self.stop_simulation, width=15)
        self.stop_button.pack()
        
        self.is_paused = False
        self.is_stopped = False
        self.start_simulation()

    def setup_simulation(self):
        self.time = 0.0
        self.dt = 0.05
        self.g = 9.81
        self.l = 1.0
        self.m = 1.0
        self.k = 0.1
        self.tau = 0.0

    def setup_pid_controller(self):
        self.ppc_pid = PPC_PID(Kp=25.0, Ki=10.0, Kd=1.0)   # Change based on your scenario
        self.conv_pid = conv_pid(Kp=25.0, Ki=10.0, Kd=1.0) # Change based on your scenario
        self.ppc_pid.set_setpoint(0.5)                     # Change based on your scenario
        self.conv_pid.set_setpoint(0.5)                    # Change based on your scenario

    def create_sliders(self):
        self.setpoint_slider = Scale(self.root, from_=0, to=1, resolution=0.01, orient=tk.HORIZONTAL,
                                     label="Setpoint", command=self.update_setpoint, length=200) #to=2*np.pi

        self.setpoint_slider.set(self.ppc_pid.setpoint)
        self.setpoint_slider.pack()

        self.Kp_slider = Scale(self.root, from_=0, to=100, resolution=0.1, orient=tk.HORIZONTAL,
                               label="Kp", command=self.update_Kp, length=200)
        self.Kp_slider.set(self.ppc_pid.Kp)
        self.Kp_slider.pack()

        self.Ki_slider = Scale(self.root, from_=0, to=50, resolution=0.1, orient=tk.HORIZONTAL,
                               label="Ki", command=self.update_Ki, length=200)
        self.Ki_slider.set(self.ppc_pid.Ki)
        self.Ki_slider.pack()

        self.Kd_slider = Scale(self.root, from_=0, to=10, resolution=0.01, orient=tk.HORIZONTAL,
                               label="Kd", command=self.update_Kd, length=200)
        self.Kd_slider.set(self.ppc_pid.Kd)
        self.Kd_slider.pack()

    def create_restart_button(self):
        self.restart_button = Button(self.root, text="Restart Simulation", command=self.restart_simulation, width=15)
        self.restart_button.pack()

    def pause_simulation(self):
        self.is_paused = True

    def resume_simulation(self):
        self.is_paused = False
        self.update_pendulum()  # Restart the simulation loop

    def stop_simulation(self):
        self.is_stopped = True
        self.time = 0.0
        self.ppc_pid.theta = 0.0
        self.ppc_pid.theta_dot = 0.0
        self.conv_pid.theta = 0.0
        self.conv_pid.theta_dot = 0.0
        self.ppc_pid.set_setpoint(float(self.setpoint_slider.get()))
        self.conv_pid.set_setpoint(float(self.setpoint_slider.get()))
        self.canvas.delete("all")
        self.ax1.clear()
        self.ax1.set_xlabel('Time')
        self.ax1.set_ylabel('Theta')
        self.ppc_line, = self.ax1.plot([], [], label='PPC-PID')
        self.pid_line, = self.ax1.plot([], [], label='PID')
        self.ax2.clear()
        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Decay Function & Tracking Error')
        self.ppc_error_line, = self.ax2.plot([], [], label='PPC-PID Error')
        self.pid_error_line, = self.ax2.plot([], [], label='PID Error')
        self.pt_line1, = self.ax2.plot([], [], label='p(t) Bound')
        self.pt_line2, = self.ax2.plot([], [], label='-p(t) Bound')
        self.ax2.legend()
        self.graph.draw()

    def update_setpoint(self, value):
        self.ppc_pid.set_setpoint(float(value))
        self.conv_pid.set_setpoint(float(value))

    def update_Kp(self, value):
        self.ppc_pid.set_Kp(float(value))
        self.conv_pid.set_Kp(float(value))

    def update_Ki(self, value):
        self.ppc_pid.set_Ki(float(value))
        self.conv_pid.set_Ki(float(value))

    def update_Kd(self, value):
        self.ppc_pid.set_Kd(float(value))
        self.conv_pid.set_Kd(float(value))

    def update_pendulum(self):
        if not self.is_paused and not self.is_stopped:
            feedback_value_ppc = self.ppc_pid.theta
            feedback_value_conv = self.conv_pid.theta

            # Update PPC-PID
            ppc_control_signal = self.ppc_pid.update(feedback_value_ppc, self.dt)
            ppc_tau = ppc_control_signal  
            alpha_ppc = (ppc_tau - self.k * self.ppc_pid.theta_dot - self.m * self.g * self.l * np.sin(self.ppc_pid.theta)) / (self.m * self.l**2)
            self.ppc_pid.theta_dot += alpha_ppc * self.dt
            self.ppc_pid.theta += self.ppc_pid.theta_dot * self.dt

            # Update Conventional PID
            conv_control_signal = self.conv_pid.update(feedback_value_conv, self.dt)
            conv_tau = conv_control_signal
            alpha_conv = (conv_tau - self.k * self.conv_pid.theta_dot - self.m * self.g * self.l * np.sin(self.conv_pid.theta)) / (self.m * self.l**2)
            self.conv_pid.theta_dot += alpha_conv * self.dt
            self.conv_pid.theta += self.conv_pid.theta_dot * self.dt

            self.draw_pendulum()
            self.update_graph(ppc_tau, conv_tau)
            
            self.time += self.dt
            if not self.is_stopped:
                self.root.after(1, self.update_pendulum)
            else:
                self.root.update_idletasks() 
            
    def draw_pendulum(self):
        self.canvas.delete("all")
        x_center, y_center = 200, 200
        pendulum_length = 100

        # Draw PPC-PID Pendulum
        x_pendulum_ppc = x_center + pendulum_length * np.sin(self.ppc_pid.theta)
        y_pendulum_ppc = y_center + pendulum_length * np.cos(self.ppc_pid.theta)
        self.canvas.create_line(x_center, y_center, x_pendulum_ppc, y_pendulum_ppc, fill="blue", width=4)
        self.canvas.create_oval(x_pendulum_ppc - 15, y_pendulum_ppc - 15, x_pendulum_ppc + 15, y_pendulum_ppc + 15, fill="blue")

        # Draw CONVENTIONAL-PID Pendulum
        x_pendulum_conv = x_center + pendulum_length * np.sin(self.conv_pid.theta)
        y_pendulum_conv = y_center + pendulum_length * np.cos(self.conv_pid.theta)
        self.canvas.create_line(x_center, y_center, x_pendulum_conv, y_pendulum_conv, fill="red", width=4)
        self.canvas.create_oval(x_pendulum_conv - 15, y_pendulum_conv - 15, x_pendulum_conv + 15, y_pendulum_conv + 15, fill="red")

        x_setpoint = x_center + pendulum_length * np.sin(self.ppc_pid.setpoint)
        y_setpoint = y_center + pendulum_length * np.cos(self.ppc_pid.setpoint)
        self.canvas.create_line(x_center, y_center, x_setpoint, y_setpoint, fill="green", width=2)
        self.canvas.create_oval(x_setpoint - 5, y_setpoint - 5, x_setpoint + 5, y_setpoint + 5, fill="black")

        title_text = "Pendulum Visualization"
        title_x = self.canvas.winfo_reqwidth() / 2
        self.canvas.create_text(title_x, 20, text=title_text, font=("Arial", 14, "bold"), anchor="center")

        self.canvas.create_text(50, 50, anchor='nw', text='PPC-PID Pendulum', fill='blue')
        self.canvas.create_text(50, 70, anchor='nw', text='PID Pendulum', fill='red')
        self.canvas.create_text(50, 90, anchor='nw', text='Setpoint', fill='black')

    def calculate_indices(self):

        # Calculate indices for PPC-PID controller
        error_ppc_pid = np.abs(self.ppc_pid.setpoint - self.ppc_pid.theta)      # Error for PPC-PID
        squared_error_ppc_pid = error_ppc_pid ** 2                              # Squared error for PPC-PID
        iae_ppc_pid = np.sum(error_ppc_pid) * self.dt                           # Integral Absolute Error for PPC-PID
        itae_ppc_pid = np.sum(error_ppc_pid * self.time) * self.dt              # Integral Time Absolute Error for PPC-PID
        ise_ppc_pid = np.sum(squared_error_ppc_pid) * self.dt                   # Integral Squared Error for PPC-PID

        # Calculate indices for PID controller
        error_pid = np.abs(self.conv_pid.setpoint - self.conv_pid.theta)        # Error for PID
        squared_error_pid = error_pid ** 2                                      # Squared error for PID
        iae_pid = np.sum(error_pid) * self.dt                                   # Integral Absolute Error for PID
        itae_pid = np.sum(error_pid * self.time) * self.dt                      # Integral Time Absolute Error for PID
        ise_pid = np.sum(squared_error_pid) * self.dt                           # Integral Squared Error for PID

        # Update the bar heights with new values for both controllers
        self.index_values_ppc_pid[0] = iae_ppc_pid
        self.index_values_ppc_pid[1] = itae_ppc_pid
        self.index_values_ppc_pid[2] = ise_ppc_pid
        self.index_values_pid[0] = iae_pid
        self.index_values_pid[1] = itae_pid
        self.index_values_pid[2] = ise_pid

        max_index_value = max(max(self.index_values_ppc_pid), max(self.index_values_pid))
        for bar, value in zip(self.bars_ppc_pid, self.index_values_ppc_pid):
            bar.set_height(value)
        for bar, value in zip(self.bars_pid, self.index_values_pid):
            bar.set_height(value)
        self.ax4.set_ylim(0, max_index_value * 1.1)
        self.graph.draw()
        self.root.after(100, self.calculate_indices)

    def update_graph(self, ppc_tau, conv_tau):
        self.ppc_line.set_xdata(np.append(self.ppc_line.get_xdata(), self.time))
        self.ppc_line.set_ydata(np.append(self.ppc_line.get_ydata(), self.ppc_pid.theta))
        self.pid_line.set_xdata(np.append(self.pid_line.get_xdata(), self.time))
        self.pid_line.set_ydata(np.append(self.pid_line.get_ydata(), self.conv_pid.theta))

        ppc_error_value = (self.ppc_pid.setpoint - self.ppc_pid.theta)
        pid_error_value = (self.ppc_pid.setpoint - self.conv_pid.theta)
        ppc_values = p(self.time)
        negative_ppc_values = - p(self.time)
        self.ppc_error_line.set_xdata(np.append(self.ppc_error_line.get_xdata(), self.time))
        self.ppc_error_line.set_ydata(np.append(self.ppc_error_line.get_ydata(), ppc_error_value))
        self.pid_error_line.set_xdata(np.append(self.pid_error_line.get_xdata(), self.time))
        self.pid_error_line.set_ydata(np.append(self.pid_error_line.get_ydata(), pid_error_value))        
        self.pt_line1.set_xdata(np.append(self.pt_line1.get_xdata(), self.time))
        self.pt_line1.set_ydata(np.append(self.pt_line1.get_ydata(), ppc_values))
        self.pt_line2.set_xdata(np.append(self.pt_line2.get_xdata(), self.time))
        self.pt_line2.set_ydata(np.append(self.pt_line2.get_ydata(), negative_ppc_values))

        self.ppc_pid_output.set_xdata(np.append(self.ppc_pid_output.get_xdata(), self.time))
        self.ppc_pid_output.set_ydata(np.append(self.ppc_pid_output.get_ydata(), ppc_tau)) 
        self.conv_pid_output.set_xdata(np.append(self.conv_pid_output.get_xdata(), self.time))
        self.conv_pid_output.set_ydata(np.append(self.conv_pid_output.get_ydata(), conv_tau))

        for line in self.ax1.lines:
            if line.get_label() == 'Setpoint':
                line.remove()
                break

        setpoint_values = np.full_like(self.pt_line1.get_xdata(), self.ppc_pid.setpoint)
        self.ax1.plot(self.pt_line1.get_xdata(), setpoint_values, 'g--', label='Setpoint')
    
        self.ax1.relim()
        self.ax1.autoscale_view()

        self.ax2.relim()
        self.ax2.autoscale_view()

        self.ax3.relim()
        self.ax3.autoscale_view()

        self.graph.draw()

    def restart_simulation(self):
        self.time = 0.0
        self.setup_simulation()
        self.setup_pid_controller()
        self.ppc_pid.theta = 0.0
        self.ppc_pid.theta_dot = 0.0
        self.conv_pid.theta = 0.0
        self.conv_pid.theta_dot = 0.0
        self.ppc_pid.set_setpoint(float(self.setpoint_slider.get()))
        self.conv_pid.set_setpoint(float(self.setpoint_slider.get()))

        self.canvas.delete("all")
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()

        self.ax1.set_xlabel('Time')
        self.ax1.set_ylabel('Theta')
        self.ppc_line, = self.ax1.plot([], [], label='PPC-PID')
        self.pid_line, = self.ax1.plot([], [], label='PID')

        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Decay Function & Tracking Error')
        self.ppc_error_line, = self.ax2.plot([], [], label='PPC-PID Error')
        self.pid_error_line, = self.ax2.plot([], [], label='PID Error')
        self.pt_line1, = self.ax2.plot([], [], label='p(t) Bound')
        self.pt_line2, = self.ax2.plot([], [], label='-p(t) Bound')
        self.ax2.legend()
        
        self.ax3.set_xlabel('Time')
        self.ax3.set_ylabel('PID Output')
        self.ppc_pid_output, = self.ax3.plot([], [], label='PPC PID Output')
        self.conv_pid_output, = self.ax3.plot([], [], label='PID Output')  
        self.ax3.legend()

        self.graph.draw()
        self.root.after(2000, self.update_pendulum)

    def start_simulation(self):
        self.update_pendulum()
        self.calculate_indices()

def main():
    root = tk.Tk()
    app = PendulumApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()