# -*- coding: utf-8 -*-
"""
Created on Mon Feb 12 12:47:16 2024

@author: malac
"""

import tkinter as tk
from tkinter import ttk
import os
import numpy as np
import matplotlib.pyplot as plt
import pygame
import sys
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import Fetch as FE

class SimulationInterface:
    def __init__(self, root):        
        self.root = root
        self.root.title("Simulation Interface")

        # Variables
        self.controller_enabled = tk.BooleanVar()
        self.run_sim = tk.BooleanVar()
        self.kp = tk.DoubleVar()
        self.ki = tk.DoubleVar()

        self.antisway_enabled = tk.BooleanVar()
        self.antisway_gain = tk.DoubleVar()

        self.m0 = tk.DoubleVar()
        self.m1 = tk.DoubleVar()
        self.b0 = tk.DoubleVar()
        self.b1 = tk.DoubleVar()
        self.g = tk.DoubleVar()
        self.l = tk.DoubleVar()
        self.xi = tk.DoubleVar()
        self.ti = tk.DoubleVar()
        self.dt = tk.DoubleVar()
        self.sim_speed = tk.DoubleVar()
        self.sim_length = tk.DoubleVar()        

        self.vel_type = tk.StringVar()
        self.vel_type.set("Step")
        self.force_type = tk.StringVar()
        self.force_type.set("Step")
        self.ramp = tk.DoubleVar()
        self.force_ramp = tk.DoubleVar()



        # Create GUI elements
        self.create_simulation_parameters_box()
        self.create_velocity_settings_box()
        self.create_force_settings_box()
        self.create_controller_box()
        self.create_antisway_box()


        # Run button
        run_button = tk.Button(root, text="RUN", command=self.run_simulation)
        run_button.pack(pady=10)
        
        self.load_saved_parameters()

    def create_controller_box(self):
        controller_frame = ttk.LabelFrame(self.root, text="Controller")
        controller_frame.pack(padx=10, pady=10, side=tk.LEFT)
        self.controller_frame = controller_frame

        controller_enabled_check = ttk.Checkbutton(controller_frame, text="Enabled", variable=self.controller_enabled)
        controller_enabled_check.grid(row=0, column=0, columnspan=2, pady=5)

        kp_label = ttk.Label(controller_frame, text="Kp:")
        kp_label.grid(row=1, column=0, pady=5)
        kp_entry = ttk.Entry(controller_frame, textvariable=self.kp)
        kp_entry.grid(row=1, column=1, pady=5)

        ki_label = ttk.Label(controller_frame, text="Ki:")
        ki_label.grid(row=2, column=0, pady=5)
        ki_entry = ttk.Entry(controller_frame, textvariable=self.ki)
        ki_entry.grid(row=2, column=1, pady=5)

    def create_antisway_box(self):
        antisway_frame = ttk.LabelFrame(self.controller_frame, text="Anti Sway")
        antisway_frame.grid(row=3, column=0, columnspan=2, pady=10)

        antisway_enabled_check = ttk.Checkbutton(antisway_frame, text="Enabled", variable=self.antisway_enabled)
        antisway_enabled_check.grid(row=0, column=0, columnspan=2, pady=5)

        antisway_gain_label = ttk.Label(antisway_frame, text="Gain:")
        antisway_gain_label.grid(row=1, column=0, pady=5)
        antisway_gain_entry = ttk.Entry(antisway_frame, textvariable=self.antisway_gain)
        antisway_gain_entry.grid(row=1, column=1, pady=5)

    def create_simulation_parameters_box(self):
        sim_params_frame = ttk.LabelFrame(self.root, text="Simulation Parameters")
        sim_params_frame.pack(padx=10, pady=10, side=tk.LEFT)

        params = [
            ("M0", self.m0), ("M1", self.m1),
            ("B0", self.b0), ("B1", self.b1),
            ("g", self.g), ("l", self.l),
            ("Xi", self.xi), ("Ti", self.ti),
            ("dt", self.dt), ("sim_speed", self.sim_speed),
            ("sim_length", self.sim_length)
        ]

        for row, (param_name, param_var) in enumerate(params):
            label = ttk.Label(sim_params_frame, text=param_name + ":")
            label.grid(row=row, column=0, pady=5)
            entry = ttk.Entry(sim_params_frame, textvariable=param_var)
            entry.grid(row=row, column=1, pady=5)
            
        run_pygame = ttk.Checkbutton(sim_params_frame, text="Visual Simulation?", variable=self.run_sim)
        run_pygame.grid(row=row+1, column=0, columnspan=2, pady=5)

    def create_velocity_settings_box(self):
        inputs_frame = ttk.LabelFrame(self.root, text="Input Settings")
        inputs_frame.pack(padx=10, pady=10, side=tk.LEFT)
        self.inputs_frame = inputs_frame
        
        vel_settings_frame = ttk.LabelFrame(self.inputs_frame, text="Velocity")
        vel_settings_frame.pack(padx=10, pady=10, side=tk.TOP)

        vel_types = ["Step", "Ramp", "Sine", "Cosine", "Piecewise", "Constant", "None"]
        vel_type_menu = ttk.Combobox(vel_settings_frame, values=vel_types, textvariable=self.vel_type)
        vel_type_menu.grid(row=0, column=0, columnspan=2, pady=5)
        vel_type_menu.current(0)  # Set the default value
        
        ramp_label = ttk.Label(vel_settings_frame, text="Scale")
        ramp_label.grid(row=1, column=0, pady=5)
        ramp_entry = ttk.Entry(vel_settings_frame, textvariable=self.ramp)
        ramp_entry.grid(row=1, column=1, pady=5)
        
    def create_force_settings_box(self):
        force_settings_frame = ttk.LabelFrame(self.inputs_frame, text="Force")
        force_settings_frame.pack(padx=10, pady=10, side=tk.TOP)

        f_types = ["Step", "Ramp", "Sine", "Cosine", "Piecewise", "Constant", "None"]
        f_types_menu = ttk.Combobox(force_settings_frame, values=f_types, textvariable=self.force_type)
        f_types_menu.grid(row=0, column=0, columnspan=2, pady=5)
        f_types_menu.current(0)  # Set the default value
        
        force_ramp_label = ttk.Label(force_settings_frame, text="Scale")
        force_ramp_label.grid(row=1, column=0, pady=5)
        ramp_entry = ttk.Entry(force_settings_frame, textvariable=self.force_ramp)
        ramp_entry.grid(row=1, column=1, pady=5)
        
    def plot_matplotlib(self, time, X, dX, Vset, Theta, F, M1):
        # Create a new Tkinter window for the Matplotlib plot
        plot_window = tk.Toplevel(self.root)
        plot_window.title("Matplotlib Plot")
        
        # Create Matplotlib Figure and Axes
        figure, ax = Figure(figsize=(5, 4), dpi=100), None
        canvas = FigureCanvasTkAgg(figure, master=plot_window)
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        ax = figure.add_subplot(111)
        ax.plot(time, X, label='X')
        ax.plot(time, dX, label='Vel')
        ax.plot(time, Vset, label='VelSet')
        ax.plot(time, 3*np.array(Theta), label = '3*Theta')
        ax.plot(time, np.array(F)/M1, label='F/M Motor')
        ax.set_title('Outputs')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Variables')
        ax.legend()
        canvas.draw()
        
    def pygame_pendulum_animation(self, time, theta, string_length, x, f, M1, simulation_speed, dt):
        #pixel to length ratio:
        ptlr = 25
        pygame.init()
        string_length *= -1*ptlr

        # Parameters
        width, height = 800, 600
        apex_color = (0, 0, 0)  # Black color for the apex
        pendulum_color = (255, 0, 0)  # Red color for the pendulum

        # Pygame setup
        screen = pygame.display.set_mode((width, height))
        clock = pygame.time.Clock()
        xyz = (width // 2, height // 4)  # Apex position in the middle of the screen

        # Main loop
        for t, angle, x0, F0 in zip(time, theta, x, f):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            apex_position = ((width // 2)+int(x0*ptlr), height // 4)  # Apex position in the middle of the screen
            vector_end = ((width // 2)+int(x0*ptlr)+F0*ptlr/(M1), height // 4)  # Apex position in the middle of the screen

            # Convert polar coordinates to Cartesian coordinates for the pendulum
            pendulum_x = int(apex_position[0] + string_length * np.sin(-1*angle))
            pendulum_y = int(apex_position[1] - string_length * np.cos(-1*angle))


            screen.fill((255, 255, 255))  # White background
            pygame.draw.circle(screen, (255,0,255), xyz, 5)  # Draw the apex as a small black dot
            pygame.draw.circle(screen, apex_color, apex_position, 5)  # Draw the apex as a small black dot
            pygame.draw.circle(screen, pendulum_color, (pendulum_x, pendulum_y), 15)  # Draw the pendulum as a red circle
            pygame.draw.line(screen, 'black', (pendulum_x, pendulum_y),(apex_position),1)
            pygame.draw.line(screen, 'green', (apex_position),(vector_end),2)


            pygame.display.flip()
            clock.tick(2*(simulation_speed/5)/dt)  # Adjust the frame rate
        pygame.quit()
        
    def get_vset(self, v_type, time, dt, ramp):
        Vset = np.ones_like(time)
        v_type = str(v_type)
        if v_type == 'Piecewise':
            for i in range(len(Vset)):
                length = len(Vset)
                divide = int(length/5)
                if i < divide:
                    Vset[i] = i*dt*ramp
                elif i < divide*2:
                    Vset[i] = divide*dt*ramp
                elif i < divide*3:
                    Vset[i] = -1*divide*dt*ramp
                elif i < divide*4:
                    Vset[i] = -1*divide*dt*ramp + (i-3*divide)*dt*ramp
                else:
                    Vset[i] = ramp*np.sin(i*dt)
        elif v_type == 'Step':
            for i in range(len(Vset)):
                length = len(Vset)
                divide = int(length/4)
                if i < divide:
                    Vset[i] = 0
                else:
                    Vset[i] = ramp
        elif v_type == 'Ramp':
            for i in range(len(Vset)):
                length = len(Vset)
                divide = int(length/6)
                if i < 2*divide:
                    Vset[i] = 0
                elif i < divide*4:
                    Vset[i] = ramp*dt*(i-2*divide)
                else:
                    Vset[i] = ramp*dt*(divide*2)

                    
        elif v_type == 'Sine':
            Vset = ramp*np.sin(time/3)
        elif v_type == 'Cosine':
            Vset = ramp*np.cos(time)
        elif v_type == 'Constant':
            Vset = ramp*np.ones_like(time/3)
        else:
            Vset = np.zeros_like(time)
        return Vset
        

    def run_simulation(self):
        # Write your simulation logic here
        dt = self.dt.get()
        controller = self.controller_enabled.get()
        Kp = self.kp.get()
        Ki = self.ki.get()
        anti_sway = self.antisway_enabled.get()
        as_gain = self.antisway_gain.get()
        M0 = self.m0.get()
        M1 = self.m1.get()
        B0 = self.b0.get()
        B1 = self.b1.get()
        g = self.g.get()
        l = self.l.get()
        Xi = self.xi.get()
        Ti = self.ti.get()
        simulation_speed = self.sim_speed.get()
        simulation_length = self.sim_length.get()
        v_type = self.vel_type.get()
        ramp = self.ramp.get()
        percentage = 0.4
        
        self.save_parameters_to_file()
        
        #initialize
        self.time = np.arange(0,simulation_length,dt)
        time = self.time
        Vset = self.get_vset(v_type, time, dt, ramp)

        #CURRENTLY UNUSED OLD PARAMS
        #force parameters:
        DISTANCE = False #specify the distance and get there with 0 sway. else velocity mode
        F_app = 0; F = M1*self.get_vset(self.force_type.get(), time, dt, self.force_ramp.get())
        theta_max = 40 #degrees, maximum sway angle
        track_destination = 10 #meters, for distance mode
        velocity_set = 3 #for velocity mode

        ######

        stored = [F, time, dt, M0, M1, B0, B1, g, l, Xi, Ti, F_app, percentage, controller, velocity_set, Kp, Ki, Vset, anti_sway, as_gain]
        # Run simulation
        Theta, X, dX, F = FE.fetch_pendulum_mode(stored)
        
        self.plot_matplotlib(time, X, dX, Vset, Theta, F, M1)
        
        if self.run_sim.get():
            self.pygame_pendulum_animation(time, Theta, l, X, F, M1, simulation_speed, dt)
        
    def save_parameters_to_file(self):
        parameters = {
            "controller_enabled": self.controller_enabled.get(),
            "kp": self.kp.get(),
            "ki": self.ki.get(),
            "antisway_enabled": self.antisway_enabled.get(),
            "antisway_gain": self.antisway_gain.get(),
            "m0": self.m0.get(),
            "m1": self.m1.get(),
            "b0": self.b0.get(),
            "b1": self.b1.get(),
            "g": self.g.get(),
            "l": self.l.get(),
            "xi": self.xi.get(),
            "ti": self.ti.get(),
            "dt": self.dt.get(),
            "sim_speed": self.sim_speed.get(),
            "sim_length": self.sim_length.get(),
            "vel_type": self.vel_type.get(),
            "ramp": self.ramp.get(),
            "force_ramp": self.force_ramp.get(),
            "force_type": self.force_type.get(),
            "run_sim": self.run_sim.get()
        }

        with open("simulation_parameters.txt", "w") as file:
            for key, value in parameters.items():
                file.write(f"{key}: {value}\n")

    def load_saved_parameters(self):
        if os.path.exists("simulation_parameters.txt"):
            with open("simulation_parameters.txt", "r") as file:
                lines = file.readlines()

            for line in lines:
                key, value = line.strip().split(": ")
                if hasattr(self, key):
                    getattr(self, key).set(value)

if __name__ == "__main__":
    root = tk.Tk()
    app = SimulationInterface(root)
    root.mainloop()
