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
from tkinter import filedialog
import scipy.io

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
        self.dThi = tk.DoubleVar()
        self.dt = tk.DoubleVar()
        self.sim_speed = tk.DoubleVar()
        self.sim_length = tk.DoubleVar()  
        self.motor_constant = tk.DoubleVar()        
        self.amp_constant = tk.DoubleVar()   
        self.gear_radius = tk.DoubleVar()   


        self.vel_type = tk.StringVar()
        self.vel_type.set("Step")
        self.force_type = tk.StringVar()
        self.force_type.set("Step")
        self.ramp = tk.DoubleVar()
        self.force_ramp = tk.DoubleVar()
        
        self.u_type = tk.StringVar()
        self.u_type.set("Step")
        self.u_ramp = tk.DoubleVar()
        
        self.model_type = tk.StringVar()
        self.model_type.set("Anti-Sway")
        
        self.plot_F = tk.BooleanVar()
        self.plot_u = tk.BooleanVar()
        self.plot_u_Vel = tk.BooleanVar()
        self.plot_X = tk.BooleanVar()
        self.plot_Theta = tk.BooleanVar()
        self.plot_Vset = tk.BooleanVar()
        self.plot_Vact = tk.BooleanVar()
        self.plot_Vsend = tk.BooleanVar()
        self.plot_amps = tk.BooleanVar()
        self.plot_volts = tk.BooleanVar()
        self.plot_torque = tk.BooleanVar()
        self.plot_rpm = tk.BooleanVar()

        self.plot_separate = tk.BooleanVar()
        self.grid_on = tk.BooleanVar()
        self.zoom = tk.DoubleVar()

        # Create GUI elements
        self.create_simulation_parameters_box()
        self.create_velocity_settings_box()
        self.create_force_settings_box()
        self.create_u_settings_box()
        self.create_mode_box()
        self.create_controller_box()
        self.create_antisway_box()
        self.create_motor_box()
        self.create_plot_outputs_box()

        # Run button
        run_button = tk.Button(root, text="RUN", command=self.run_simulation)
        run_button.pack(pady=10)
        
        compare_button = tk.Button(root, text="COMPARE", command=self.compare)
        compare_button.pack(side = "top", pady=10)
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
    
    def create_motor_box(self):
        controller_frame = ttk.LabelFrame(self.root, text="Motor")
        controller_frame.pack(padx=10, pady=10, side=tk.LEFT)

        kp_label = ttk.Label(controller_frame, text="Motor Constant: (N-m/A)")
        kp_label.grid(row=1, column=0, pady=5)
        kp_entry = ttk.Entry(controller_frame, textvariable=self.motor_constant)
        kp_entry.grid(row=1, column=1, pady=5)

        asd = ttk.Label(controller_frame, text="Amp Constant: (A/V)")
        asd.grid(row=2, column=0, pady=5)
        asdd = ttk.Entry(controller_frame, textvariable=self.amp_constant)
        asdd.grid(row=2, column=1, pady=5)
        
        ki_label = ttk.Label(controller_frame, text="Gear Radius: (m)")
        ki_label.grid(row=3, column=0, pady=5)
        ki_entry = ttk.Entry(controller_frame, textvariable=self.gear_radius)
        ki_entry.grid(row=3, column=1, pady=5)

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
            ("sim_length", self.sim_length), ("dTheta_i", self.dThi),
            ("Zoom", self.zoom)
        ]

        for row, (param_name, param_var) in enumerate(params):
            label = ttk.Label(sim_params_frame, text=param_name + ":")
            label.grid(row=row, column=0, pady=5)
            entry = ttk.Entry(sim_params_frame, textvariable=param_var)
            entry.grid(row=row, column=1, pady=5)
            
        run_pygame = ttk.Checkbutton(sim_params_frame, text="Visual Simulation?", variable=self.run_sim)
        run_pygame.grid(row=row+1, column=0, columnspan=2, pady=5)
        
    def create_plot_outputs_box(self):
        sim_params_frame = ttk.LabelFrame(self.root, text="Plot These Outputs")
        sim_params_frame.pack(padx=10, pady=10, side=tk.LEFT)
        params = [
            ("Force on Trolley", self.plot_F), ("Force on Mass", self.plot_u),
            ("Velocity of Mass", self.plot_u_Vel), ("Trolley Position", self.plot_X),
            ("Theta", self.plot_Theta), ("Trolley V Set", self.plot_Vset), ("Trolley V_desired", self.plot_Vsend),
            ("Trolley Velocity", self.plot_Vact), ("Motor Amps", self.plot_amps),
            ("Motor Volts", self.plot_volts), ("Motor Torque", self.plot_torque),
            ("Motor RPM", self.plot_rpm),
            ("PLOT SEPARATE?", self.plot_separate),
            ("Grid On?", self.grid_on)
        ]

        for row, (param_name, param_var) in enumerate(params):
            run_pygame = ttk.Checkbutton(sim_params_frame, text=param_name, variable=param_var)
            run_pygame.pack(side="top", anchor="w", padx=5, pady=5)


    def create_velocity_settings_box(self):
        inputs_frame = ttk.LabelFrame(self.root, text="Input Settings")
        inputs_frame.pack(padx=10, pady=10, side=tk.LEFT)
        self.inputs_frame = inputs_frame
        
        vel_settings_frame = ttk.LabelFrame(self.inputs_frame, text="Velocity (Overrides Forces)")
        vel_settings_frame.pack(padx=10, pady=10, side=tk.TOP)

        vel_types = ["Step", "Ramp", "Sine", "Cosine", "Piecewise", "Constant","Square", "Double Square", "None"]
        vel_type_menu = ttk.Combobox(vel_settings_frame, values=vel_types, textvariable=self.vel_type)
        vel_type_menu.grid(row=0, column=0, columnspan=2, pady=5)
        vel_type_menu.current(0)  # Set the default value
        
        ramp_label = ttk.Label(vel_settings_frame, text="Scale")
        ramp_label.grid(row=1, column=0, pady=5)
        ramp_entry = ttk.Entry(vel_settings_frame, textvariable=self.ramp)
        ramp_entry.grid(row=1, column=1, pady=5)
        
    def create_force_settings_box(self):
        force_settings_frame = ttk.LabelFrame(self.inputs_frame, text="Force on Trolley")
        force_settings_frame.pack(padx=10, pady=10, side=tk.TOP)

        f_types = ["Step", "Ramp", "Sine", "Cosine", "Piecewise", "Constant","Square", "Double Square", "None"]
        f_types_menu = ttk.Combobox(force_settings_frame, values=f_types, textvariable=self.force_type)
        f_types_menu.grid(row=0, column=0, columnspan=2, pady=5)
        f_types_menu.current(0)  # Set the default value
        
        force_ramp_label = ttk.Label(force_settings_frame, text="Scale")
        force_ramp_label.grid(row=1, column=0, pady=5)
        ramp_entry = ttk.Entry(force_settings_frame, textvariable=self.force_ramp)
        ramp_entry.grid(row=1, column=1, pady=5)
        
    def create_u_settings_box(self):
        u_settings_frame = ttk.LabelFrame(self.inputs_frame, text="Force on Mass")
        u_settings_frame.pack(padx=10, pady=10, side=tk.TOP)

        f_types = ["Step", "Ramp", "Sine", "Cosine", "Piecewise", "Constant","Square", "Double Square", "None"]
        f_types_menu = ttk.Combobox(u_settings_frame, values=f_types, textvariable=self.u_type)
        f_types_menu.grid(row=0, column=0, columnspan=2, pady=5)
        f_types_menu.current(0)  # Set the default value
        
        u_ramp_label = ttk.Label(u_settings_frame, text="Scale")
        u_ramp_label.grid(row=1, column=0, pady=5)
        ramp_entry = ttk.Entry(u_settings_frame, textvariable=self.u_ramp)
        ramp_entry.grid(row=1, column=1, pady=5)
        
    def create_mode_box(self):
        u_settings_frame = ttk.LabelFrame(self.inputs_frame, text="Plant Model Type")
        u_settings_frame.pack(padx=10, pady=10, side=tk.TOP)

        f_types = ["Anti-Sway", "Tracking"]
        f_types_menu = ttk.Combobox(u_settings_frame, values=f_types, textvariable=self.model_type)
        f_types_menu.grid(row=0, column=0, columnspan=2, pady=5)
        f_types_menu.current(0)  # Set the default value
        
    def compare(self):
        file_path = str(filedialog.askopenfilename(filetypes=[('Mat files', '*.mat')],title="Select mat file", multiple=False))   
        data_dict = scipy.io.loadmat(file_path)
        self.open_new_toplevel(data_dict)
        
    def open_new_toplevel(self, data_dict):
        new_window = tk.Toplevel()
        keys = list(data_dict.keys())
        frame1 = ttk.LabelFrame(new_window, text="Includes")
        frame1.pack(padx=10, pady=10, side=tk.LEFT)
        
        # Checkbox variables
        checkbox_vars = {}
        checks = ['angle_x', 'trolley_vel_x', 'angle_y', 'trolley_vel_y']
        for key in data_dict:
            if key in checks:
                value = True
            else:
                value = False
            checkbox_vars[key] = tk.BooleanVar(value=value)
            tk.Checkbutton(frame1, text=key, variable=checkbox_vars[key]).pack(anchor='w')
            
        frame2 = ttk.LabelFrame(new_window, text="Conditions")
        frame2.pack(padx=10, pady=10, side=tk.LEFT)
    
        # Dropdown selection boxes and labels
        time_label = tk.Label(frame2, text="Choose your time key:")
        time_label.pack(anchor='w')
        time_dropdown = ttk.Combobox(frame2, values=keys)
        time_dropdown.current(keys.index('t'))
        time_dropdown.pack(anchor='w')
    
        angle_label = tk.Label(frame2, text="Choose your simulation angle variable:")
        angle_label.pack(anchor='w')
        angle_dropdown = ttk.Combobox(frame2, values=keys)
        angle_dropdown.current(keys.index('angle_x'))
        angle_dropdown.pack(anchor='w')
    
        velocity_label = tk.Label(frame2, text="Choose your simulation velocity variable:")
        velocity_label.pack(anchor='w')
        velocity_dropdown = ttk.Combobox(frame2, values=keys)
        velocity_dropdown.current(keys.index('trolley_vel_x'))
        velocity_dropdown.pack(anchor='w')
    
        # Textbox entries and labels
        title_label = tk.Label(frame2, text="Title:")
        title_label.pack(anchor='w')
        title_entry = tk.Entry(frame2)
        title_entry.insert(0, "Outputs")
        title_entry.pack(anchor='w')
    
        range_min_label = tk.Label(frame2, text="Range min:")
        range_min_label.pack(anchor='w')
        range_min_entry = tk.Entry(frame2)
        range_min_entry.insert(tk.END, '0')  # Initial value
        range_min_entry.pack(anchor='w')
    
        end = len(list(data_dict[list(data_dict.keys())[3]])[0]) - 1
        range_max_label = tk.Label(frame2, text=f"Range max of {end}:")
        range_max_label.pack(anchor='w')
        range_max_entry = tk.Entry(frame2)
        range_max_entry.insert(tk.END, str(end))  # Initial value
        range_max_entry.pack(anchor='w')
    
        # Dropdown select boxes for error match
        error_label = tk.Label(frame2, text="Choose error match:")
        error_label.pack(anchor='w')
        error_dropdown1 = ttk.Combobox(frame2, values=keys)
        error_dropdown1.pack(anchor='w')
        error_dropdown1.current(keys.index('voltage_x'))
        error_dropdown2 = ttk.Combobox(frame2, values=["sim_force", "sim_volts", "sim_amps", "sim_torque", "v_desired"])  # Assuming variables is a list
        error_dropdown2.pack(anchor='w')
        error_dropdown2.current(1)

    
        # Compare button
        def send_outputs():
            selected_keys = [key for key, var in checkbox_vars.items() if var.get()]
            wanted_range = [int(range_min_entry.get()), int(range_max_entry.get())]
            self.compare_plot(data_dict, selected_keys, wanted_range, title_entry.get(), time_dropdown.get(), error=False)
            if compare_question.get():
                run_error_sim()
                self.compare_plot(data_dict, [error_dropdown1.get(), error_dropdown2.get()], wanted_range, "Error plot", time_dropdown.get(), error=True)
            
        def run_error_sim():
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
            percentage = 0.4
            dThi = self.dThi.get()
            self.save_parameters_to_file()
            
            #initialize
            time = list(data_dict[time_dropdown.get()])[0][int(range_min_entry.get()):int(range_max_entry.get())]
            Vset = np.zeros_like(time)

            #CURRENTLY UNUSED OLD PARAMS
            #force parameters:
            F_app = 0
            F = np.zeros_like(time)
            u = None
            
            velocity_set = 3 #for velocity mode
            act_vel = data_dict[str(velocity_dropdown.get())][0][int(range_min_entry.get()):int(range_max_entry.get())]
            act_theta = data_dict[str(angle_dropdown.get())][0][int(range_min_entry.get()):int(range_max_entry.get())]

            stored = [F, time, dt, M0, M1, B0, B1, g, l, Xi, Ti, F_app, percentage, controller, velocity_set, Kp, Ki, Vset, anti_sway, as_gain, u, dThi, act_theta, act_vel]
            F, v_desired = FE.fetch_F_actual(stored)
            
            torque = []; amps = []; volts = []
            for fnow in F:
                torque.append(fnow*self.gear_radius.get())
                amps.append(torque[-1]/self.motor_constant.get())
                volts.append(amps[-1]/self.amp_constant.get())
                
            data_dict["sim_force"] = F
            data_dict["sim_torque"] = torque
            data_dict["sim_amps"] = amps
            data_dict["sim_volts"] = volts
            data_dict["v_desired"] = v_desired
            
            error = list(np.array(data_dict[error_dropdown1.get()])[0][int(range_min_entry.get()):int(range_max_entry.get())] - np.array(data_dict[error_dropdown2.get()]))
            data_dict["error"] = error
            
            print("done1")
            

        compare_button = tk.Button(new_window, text="Compare", command=send_outputs)
        compare_button.pack(anchor='s')
        
        compare_question = tk.BooleanVar(value=False)
        compare_answer = tk.Checkbutton(new_window, text="Calculate Error?", variable=compare_question).pack(anchor='s')
        
    def compare_plot(self, data_dict, wanted_keys, wanted_range, title, time_var, error=False):
        plot_window = tk.Toplevel(self.root)
        plot_window.title("Matplotlib Plot")
        
        def find_max(inp, decimals=3):
            return 0
        
        f = wanted_range[0]; t = wanted_range[1] 
        colors = ["green", "blue", "red", "orange", "pink", "purple", "cyan", "forestgreen", "darkblue", "magenta", "grey"]
        if error == False:
            figure, axs = plt.subplots(len(wanted_keys), 1, sharex=True, figsize=(12, 9))
            canvas = FigureCanvasTkAgg(figure, master=plot_window)
            canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
            for i, key in enumerate(wanted_keys):
                axs[i].plot(data_dict[time_var][0][f:t], data_dict[key][0][f:t], label=f'{key}, max=0', color=colors[i])
                axs[i].set_ylabel(key)
                axs[i].legend()
                axs[i].grid()
                print(f"done{i}")
                    
            figure.suptitle(f'{title}', fontsize=16)

        else:
            figure, axs = plt.subplots(2, 1, sharex=True, figsize=(12, 9))
            canvas = FigureCanvasTkAgg(figure, master=plot_window)
            canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
            axs[0].plot(data_dict[time_var][0][f:t], data_dict[wanted_keys[0]][0][f:t], color=colors[0], label="actual", linewidth=1)
            axs[0].plot(data_dict[time_var][0][f:t], data_dict[wanted_keys[1]], color=colors[1], label="theory", linewidth=1)
            axs[0].set_ylabel("superimposed")
            axs[0].legend()
            axs[0].grid()
            print("done2")
            axs[1].plot(data_dict[time_var][0][f:t], data_dict["error"], color=colors[2])
            axs[1].set_ylabel("error")
            axs[1].grid()
            figure.suptitle("Error", fontsize=16)
            print("done4")
           
        axs[len(wanted_keys)-1].set_xlabel('Time (s)', fontsize=16)
        plt.tight_layout()
        plt.savefig('plot.pdf')
        canvas.draw()

        
    def plot_matplotlib(self, time, X, dX, Vset, Theta, F, M1, vsend, u=None, dTheta=None):
        # Create a new Tkinter window for the Matplotlib plot
        plot_window = tk.Toplevel(self.root)
        plot_window.title("Matplotlib Plot")
        
        u_Vel = []
        l = self.l.get()
        for v, d0 in zip(dX, dTheta):
            u_Vel.append(v+d0*l)
            
        torque = []; amps = []; volts = []; rpm = []
        for fnow, v in zip(F, dX):
            torque.append(fnow*self.gear_radius.get())
            amps.append(torque[-1]/self.motor_constant.get())
            volts.append(amps[-1]/self.amp_constant.get())
            rev_per_s = v/(2*np.pi*self.gear_radius.get())
            curr_rpm = rev_per_s*60
            rpm.append(curr_rpm)
            
        outputs = f'Outputs: ThetaMax={int(100*max(np.abs(Theta))*180/np.pi)/100}deg, Vp Max={int(max(u_Vel)*100)/100} m/s, Vt Max={int(max(dX)*100)/100} m/s'
        inputs = f'Inputs: Ki={self.ki.get()} (Ns/m), Kp={self.kp.get()} (Ns/m), Gain={self.antisway_gain.get()}, Model={self.model_type.get()}'
        if self.plot_separate.get():
            tries = [self.plot_X, self.plot_Vset, self.plot_Vact, self.plot_u_Vel, self.plot_F, self.plot_u, self.plot_Theta, self.plot_amps, self.plot_volts, self.plot_torque, self.plot_rpm, self.plot_Vsend]
            to_plot = []
            for i in range(len(tries)):
                if tries[i].get():
                    to_plot.append(i)
            
            figure, axs = plt.subplots(len(to_plot), 1, sharex=True, figsize=(12, 9))
            canvas = FigureCanvasTkAgg(figure, master=plot_window)
            canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
            count = 0
            def find_max(inp, decimals=3):
                return(int(max(np.abs(inp))*(10**decimals))/(10**decimals))
            
            for i in to_plot:
                if i == 0:
                    axs[count].plot(time, X, label=f'Trolley X (m), max={find_max(X)}', color="green")
                    axs[count].set_ylabel("Trolley X (m)")
                elif i == 1:
                    axs[count].plot(time, Vset, label=f'V Desired Trolley (m/s), max={find_max(Vset)}', color="blue")
                    axs[count].set_ylabel("V Desired Trolley (m/s)")
                elif i == 2:
                    axs[count].plot(time, dX, label=f'V Actual Trolley (m/s), max={find_max(dX)}', color="red")
                    axs[count].set_ylabel("V Actual Trolley (m/s)")
                elif i == 3:
                    axs[count].plot(time, np.array(u_Vel), label= f'V Actual Mass (m/s), max={find_max(u_Vel)}', color="orange")
                    axs[count].set_ylabel("V Actual Mass (m/s)")
                elif i == 4:
                    axs[count].plot(time, np.array(F), label=f'F Motor (N), max={find_max(F)}', color="pink")
                    axs[count].set_ylabel("F Motor (N)")
                elif i == 5 and u is not None:
                    axs[count].plot(time, np.array(u), label= f'F on Mass (N), max={find_max(u)}', color="purple")
                    axs[count].set_ylabel("F on Mass (N)")
                elif i == 6:
                    axs[count].plot(time, 180*np.array(Theta)/np.pi, label = f"Theta (deg), max={find_max(Theta)}", color="cyan")      
                    axs[count].set_ylabel("Theta (deg)")
                elif i == 7:
                    axs[count].plot(time, np.array(amps), label = f"Motor Amps (A), max={find_max(amps)}", color="forestgreen")      
                    axs[count].set_ylabel("Motor Amps (A)")
                elif i == 8:
                    axs[count].plot(time, np.array(volts), label = f"Motor Volts (V), max={find_max(volts)}", color="darkblue")      
                    axs[count].set_ylabel("Motor Volts (V)")
                elif i == 9:
                    axs[count].plot(time, np.array(torque), label = f"Motor Torque (N-m/s), max={find_max(torque)}", color="magenta")      
                    axs[count].set_ylabel("Motor Torque (N-m/s)")
                elif i == 10:
                    axs[count].plot(time, np.array(rpm), label = f"Motor Rpm, max={find_max(rpm)}", color="grey")      
                    axs[count].set_ylabel("Motor Rpm")
                elif i == 11:
                    axs[count].plot(time, np.array(vsend), label = f"V_desired, max={find_max(vsend)}", color="orange")      
                    axs[count].set_ylabel("V_desired")
                axs[count].legend()
                count += 1
               
            if self.grid_on.get():
                for x in axs:
                    x.grid()
            figure.suptitle(f'{inputs}\n{outputs}', fontsize=16)
            axs[len(to_plot)-1].set_xlabel('Time (s)', fontsize=16)
            plt.tight_layout()
            plt.savefig('plot.pdf')
            canvas.draw()
        else:
            # Create Matplotlib Figure and Axes
            figure, ax = Figure(figsize=(12, 9), dpi=100), None
            canvas = FigureCanvasTkAgg(figure, master=plot_window)
            canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
            ax = figure.add_subplot(111)

            if self.plot_X.get():
                ax.plot(time, X, label='Trolley X (m)', color="green")
            if self.plot_Vset.get():
                ax.plot(time, Vset, label='V Desired Trolley (m/s)', color="blue")
            if self.plot_Vact.get():
                ax.plot(time, dX, label='V Actual Trolley (m/s)', color="red")
            if self.plot_u_Vel.get():
                ax.plot(time, np.array(u_Vel), label= 'V Actual Mass (m/s)', color="orange")
            if self.plot_F.get():
                ax.plot(time, np.array(F), label='F Motor (N)', color="pink")
            if u is not None and self.plot_u.get():
                ax.plot(time, np.array(u), label= 'F on Mass (N)', color="purple")
            if self.plot_Theta.get():
                ax.plot(time, 3*np.array(Theta), label = '3*Theta (rad)', color="cyan")
            if self.plot_amps.get():
                ax.plot(time, np.array(amps), label = "Motor Amps (A)", color="forestgreen")      
            if self.plot_volts.get():
                ax.plot(time, np.array(volts), label = "Motor Volts (V)", color="darkblue")      
            if self.plot_torque.get():
                ax.plot(time, np.array(torque), label = "Motor Torque (N-m/s)", color="magenta")      
            if self.plot_rpm.get():
                ax.plot(time, np.array(rpm), label = "Motor Rpm", color="grey")
            if self.plot_Vsend.get():
                ax.plot(time, np.array(vsend), label = "Motor V_desired", color="grey")  
                
            if self.grid_on.get(): ax.grid()
            ax.set_title(f'{inputs}\n{outputs}', fontsize=16)
            ax.set_xlabel('Time (s)', fontsize=16)
            ax.set_ylabel('Variables', fontsize = 16)
            ax.legend()
            figure.savefig('plot.pdf')
            canvas.draw()

        
    def pygame_pendulum_animation(self, time, theta, string_length, x, f, M1, simulation_speed, dt, u=None):
        #pixel to length ratio:
        ptlr = 500*self.zoom.get()
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
        for j in range(1000):
            clock.tick(2*(simulation_speed/5)/dt)  # Adjust the frame rate

        count = 0
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
            if u is not None:
                pygame.draw.line(screen, 'blue', (pendulum_x, pendulum_y),(pendulum_x + u[count]*ptlr/(M1), pendulum_y),2)
            count += 1

            pygame.display.flip()
            clock.tick(2*(simulation_speed/5)/dt)  # Adjust the frame rate
        pygame.quit()
        
    def get_vset(self, v_type, time, dt, ramp):
        Vset = np.ones_like(time)
        v_type = str(v_type)
        if v_type == 'Piecewise':
            length = len(Vset)
            divide = int(length/5)
            for i in range(len(Vset)):
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
            length = len(Vset)
            divide = int(length/4)
            for i in range(len(Vset)):
                if i < divide:
                    Vset[i] = 0
                else:
                    Vset[i] = ramp
        elif v_type == 'Ramp':
            length = len(Vset)
            divide = int(length/6)
            for i in range(len(Vset)):
                if i < 3.7*divide:
                    Vset[i] = 0
                elif i < divide*4:
                    Vset[i] = 15*ramp*dt*(i-3.7*divide)
                else:
                    Vset[i] = 15*ramp*dt*(0.3*divide)

                    
        elif v_type == 'Sine':
            Vset = ramp*np.sin(time/3)
        elif v_type == 'Cosine':
            Vset = ramp*np.cos(time)
        elif v_type == 'Constant':
            Vset = ramp*np.ones_like(time/3)
        elif v_type == 'Square':
            length = len(Vset)
            divide = int(length/20)
            for i in range(len(Vset)):
                if i < 2*divide:
                    Vset[i] = 0
                elif i < 3*divide:
                    Vset[i] = ramp
                else:
                    Vset[i] = 0
        elif v_type == 'Double Square':
            length = len(Vset)
            divide = int(length/20)
            for i in range(len(Vset)):
                if i < divide:
                    Vset[i] = 0
                elif i < 2*divide:
                    Vset[i] = ramp
                elif i < 3*divide:
                    Vset[i] = -ramp
                else:
                    Vset[i] = 0
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
        dThi = self.dThi.get()
        
        self.save_parameters_to_file()
        
        #initialize
        self.time = np.arange(0,simulation_length,dt)
        time = self.time
        Vset = self.get_vset(v_type, time, dt, ramp)

        #CURRENTLY UNUSED OLD PARAMS
        #force parameters:
        DISTANCE = False #specify the distance and get there with 0 sway. else velocity mode
        F_app = 0
        F = self.get_vset(self.force_type.get(), time, dt, self.force_ramp.get())
        if self.model_type.get() == "Tracking":
            u = self.get_vset(self.u_type.get(), time, dt, self.u_ramp.get())
        else:
            u = None

        theta_max = 40 #degrees, maximum sway angle
        track_destination = 10 #meters, for distance mode
        velocity_set = 3 #for velocity mode

        ######

        stored = [F, time, dt, M0, M1, B0, B1, g, l, Xi, Ti, F_app, percentage, controller, velocity_set, Kp, Ki, Vset, anti_sway, as_gain, u, dThi]
        # Run simulation
        if self.model_type.get() == "Anti-Sway":
            Theta, X, dX, F, dTheta, vsend = FE.fetch_pendulum_mode(stored)
        else:
            Theta, X, dX, F, dTheta, vsend = FE.fetch_tracking_mode(stored)
        
        self.plot_matplotlib(time, X, dX, Vset, Theta, F, M1, vsend, u, dTheta)
        
        if self.run_sim.get():
            self.pygame_pendulum_animation(time, Theta, l, X, F, M1, simulation_speed, dt, u)
        
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
            "run_sim": self.run_sim.get(),
            "u_type": self.u_type.get(),
            "u_ramp": self.u_ramp.get(),
            "plot_F": self.plot_F.get(),
            "plot_u": self.plot_u.get(),
            "plot_u_Vel": self.plot_u_Vel.get(),
            "plot_X": self.plot_X.get(),
            "plot_Theta": self.plot_Theta.get(),
            "plot_Vset": self.plot_Vset.get(),
            "plot_Vact": self.plot_Vact.get(),
            "model_type": self.model_type.get(),
            "dThi": self.dThi.get(),
            "plot_separate": self.plot_separate.get(),
            "grid_on": self.grid_on.get(),
            "zoom": self.zoom.get(),
            "motor_constant": self.motor_constant.get(),
            "amp_constant": self.amp_constant.get(),
            "gear_radius": self.gear_radius.get(),
            "plot_amps": self.plot_amps.get(),
            "plot_volts": self.plot_volts.get(),
            "plot_torque": self.plot_torque.get(),
            "plot_rpm": self.plot_rpm.get(),
            "plot_Vsend": self.plot_Vsend.get()
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
