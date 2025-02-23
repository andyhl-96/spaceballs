import glob
import tkinter
from turtle import left, mainloop, update
import comm
import numpy as np
import open3d as o3d
import time
from body import Body
from collision import check_collision
from dynamics import dynamics, solve_potential
import tkinter as tk
from tkinter import *
from functools import partial


vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window(window_name = "SIMULATION", width = 960, height = 960)
view_control = vis.get_view_control()
view_control.set_zoom(100)
render_options = vis.get_render_option()
render_options.background_color = [0, 0, 0]
external_forces = np.zeros((0, 3))
# lock variable for command exec
cmd_valid = True

# some helper functions
def create_sphere(position: list, radius: float, color: list, mass: float, exclude: bool):
    global cmd_valid
    global external_forces
    if cmd_valid:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius, 20, False)
        sphere.translate(position)
        sphere.paint_uniform_color(color)
        body = Body(xyz = np.array(position), mass = mass, geom = radius, model = sphere, exclude = exclude)
        if len(external_forces) == 0:
            external_forces = np.array([[0, 0, 0]])
        else:
            external_forces = np.vstack((external_forces, [0, 0, 0]))
        vis.add_geometry(Body.objects[-1].model)

def create_box(position: list, w: float, d: float, h: float, color: list, mass: float, exclude: bool):
    global external_forces
    box = o3d.geometry.TriangleMesh.create_box(width = w, depth = d, height = h)
    box.translate(position)
    box.paint_uniform_color(color)
    body = Body(xyz = np.array(position), mass = mass, model = box, geom = 0, exclude = exclude)
    if external_forces == None:
        external_forces = np.array([0, 0, 0])
    else:
        external_forces = np.vstack((external_forces, [0, 0, 0]))
    vis.add_geometry(Body.objects[-1].model)

def draw_axes(bounds):
    zarr = o3d.geometry.TriangleMesh.create_arrow(0.1, 0.2, 4, 0.4)
    zarr.paint_uniform_color([0, 0, 1])
    vis.add_geometry(zarr)

    yarr = o3d.geometry.TriangleMesh.create_arrow(0.1, 0.2, 4, 0.4)
    yarr.rotate(np.array([[1, 0, 0],
                          [0, 0, 1],
                          [0, -1, 0]]), center=[0, 0, 0])
    yarr.paint_uniform_color([0, 1, 0])
    vis.add_geometry(yarr)

    xarr = o3d.geometry.TriangleMesh.create_arrow(0.1, 0.2, 4, 0.4)
    xarr.paint_uniform_color([1, 0, 0])
    xarr.rotate(np.array([[0, 0, 1],
                          [0, 1, 0],
                          [-1, 0, 0]]), center=[0, 0, 0])
    vis.add_geometry(xarr)

    width=bounds[1] - bounds[0]
    depth=bounds[5] - bounds[4]
    height=bounds[3] - bounds[2]
    box = o3d.geometry.TriangleMesh.create_box(width=width,
                                                depth=depth,
                                                height=height)
    box.translate([-width / 2, -height / 2, -depth / 2])
    bbox = box.get_axis_aligned_bounding_box()
    vis.add_geometry(bbox)

def update_viewport():
    for obj in Body.objects:
        vis.update_geometry(obj.model)

def update_dynamics(dynam_new):
    Body.dynamics_matrix = dynam_new
    for i in range(len(Body.dynamics_matrix)):
        bodypos = Body.objects[i].position()
        for j in range(1, 4):
            Body.dynamics_matrix[i, j] = bodypos[j - 1]

# create the main gui for control
def create_main_gui():
    root = tk.Tk()
    root.title("SIMCONTROL")
    root.geometry("640x1080")

    # top frame for ball display
    top_frame = tk.Frame(root, width=640, height=256)
    top_frame.pack(side=TOP)

    # listbox for ball display
    scroll = tk.Listbox(top_frame)
    for i in range(len(Body.objects)):
        text = StringVar()
        text.set(str(i) + str(Body.objects[i]))
        label = tk.Label(scroll, textvariable=text, bg="white")
        label.pack(side=TOP)

    #scrollbar = tk.Scrollbar(scroll)
    #scrollbar.pack(side=RIGHT, fill=Y)
    #scroll.config(yscrollcommand=scrollbar.set)
    #scrollbar.config(command = scroll.yview) 
    scroll.pack(side=LEFT)

    mid_frame = tk.Frame(root)
    # options
    # pass the body indices as a string, force comps as numbers
    body_list_str = StringVar()
    force_entry_str = StringVar()
    body_list_label = tk.Label(mid_frame, textvariable=StringVar(value="Bodies"))
    body_list = tk.Entry(mid_frame, textvariable=body_list_str)
    force_entry_label = tk.Label(mid_frame, textvariable=StringVar(value="Force"))
    force_entry = tk.Entry(mid_frame, textvariable=force_entry_str)
    force_button = tk.Button(mid_frame, text="Apply Force", command=partial(apply_force, body_list_str, force_entry_str))
    body_list_label.pack(side=TOP)
    body_list.pack(side=TOP)
    force_entry_label.pack(side=TOP)
    force_entry.pack(side=TOP)
    force_button.pack(side=TOP)
    mid_frame.pack(side=TOP)

    # bottom panel
    bottom_frame = tk.Frame(root)
    add_body_label = tk.Label(bottom_frame, textvariable=StringVar(value="Add body"))
    body_pos_str = StringVar()
    add_body_entry = tk.Entry(bottom_frame, textvariable=body_pos_str)
    body_rad_entry = tk.Spinbox(bottom_frame, from_=0.1, to=100, textvariable=tk.DoubleVar(value=0.5))
    body_color_str = StringVar()
    body_color_entry = tk.Entry(bottom_frame, textvariable=body_color_str)
    body_mass_entry = tk.Spinbox(bottom_frame, from_=0.1, to=100, textvariable=tk.DoubleVar(value=0.5))
    body_ex_var = tk.BooleanVar()
    body_ex_entry = tk.Checkbutton(bottom_frame, textvariable=body_ex_var)
    add_body_button = tk.Button(bottom_frame, text="Add", command=partial(add_body, body_pos_str, body_rad_entry, body_color_str, body_mass_entry, body_ex_var, scroll))
    add_body_label.pack(side=TOP)
    add_body_entry.pack(side=TOP)
    body_rad_entry.pack(side=TOP)
    body_color_entry.pack(side=TOP)
    body_mass_entry.pack(side=TOP)
    body_ex_entry.pack(side=TOP)
    add_body_button.pack(side=TOP)
    bottom_frame.pack(side=TOP)

    return root

# gui callbacks
def apply_force(body_label, force_label):
    bodies_str = body_label.get()
    force_str = force_label.get()
    body_label.set("")
    force_label.set("")

    bodies = bodies_str.split(" ")
    force = force_str.split(" ")


    for i in bodies:
        external_forces[int(i)] = np.array([float(force[0]), float(force[1]), float(force[2])])

def add_body(body_pos_str, body_rad_entry, body_color_var, body_mass_entry, body_ex_var, scroll):
    pos_str = body_pos_str.get()
    pos = pos_str.split(' ')
    color_str = body_color_var.get()
    col = color_str.split(' ')
    for i in range(len(pos)):
        pos[i] = float(pos[i])
        col[i] = float(col[i])
    rad = float(body_rad_entry.get())

    mass = float(body_mass_entry.get())
    ex = body_ex_var.get()

    create_sphere(pos, rad, col, mass, ex)

    for item in scroll.winfo_children():
        item.destroy()
    for i in range(len(Body.objects)):
        label = tk.Label(scroll, text=str(Body.objects[i]))
        label.pack(side=TOP)

def run_viz(pfunc, N, size, color_random, avg_mass, sd_mass, bounds):
    dt = 0.001
    global vis
    global cmd_valid
    global external_forces
    placement_range = 10
    # create_sphere([1, 0, 0], 0.5, [1, 0, 0], 2, exclude = False)
    # create_sphere([0, 1, 0], 0.5, [0, 0, 1], 0.5, exclude=False)
    # create_sphere([0, 0, 1], 0.5, [0, 1, 0], 1, exclude=False)
    # create_sphere([-1, 0, 0], 0.5, [0, 1, 0], 0.25, exclude=False)
    for index in range(N):
        color = [np.random.random(), np.random.random(), np.random.random()] if color_random else [1, 0, 0]
        create_sphere([placement_range * (-1 + 2 * np.random.random()), placement_range * (-1 + 2 * np.random.random()), placement_range * (-1 + 2 * np.random.random())], size, color, np.abs(np.random.normal(avg_mass, sd_mass)), exclude=False)
    
    draw_axes(bounds)

    # pfunc = "0.5*75*x**2+0.5*50*y**2+0.5*100*z**2"
    # pfunc = "m*9.8*y"

    # total ms elapsed
    t = 0
    size = 10

    gradV = solve_potential(pfunc, Body.dynamics_matrix)


    # tkinter test
    root = create_main_gui()
    ## main loop ##
    while True:
        # update gui
        root.update()
        update_viewport()
        collisions, _, _ = check_collision(bounds)
        # get updated mass, positions, momentums

        cmd_valid = False
        dynam = dynamics(Body.dynamics_matrix, dt, gradV, external_forces, 0.999, bounds)
        cmd_valid = True

        vis.poll_events()
        vis.update_renderer()

        for i in range(len(dynam)):
            # only simulate if not excluded
            if not Body.objects[i].exclude:
                # velocity vector, already accouting for dt
                dpos = dynam[i, 1:4]
                Body.objects[i].model.translate((dpos).tolist())

        # update the dynamics matrix to contain positions
        t = t + dt
        update_dynamics(dynam)
        Body.objects[0].position()
        

        if not vis.poll_events():
            break

        time.sleep(dt)
    vis.destroy_window()

class SetUpGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Simulation Setup")

        # Store variables for the simulation
        self.variables = {
            "N": 5,  # Number of objects
            "ball_size": 0.5,  # Default ball size
            "pfunc": "m*98*y",  # Default potential function
            "color_random": True,  # Color is random by default
            "ball_avg_mass": 1,  # Default ball mass
            "ball_sd_mass": 1,
            "bounds": "-10 10 -10 10 -10 10" # bounds
        }

        # Setup GUI elements
        self.setup_gui()

    def setup_gui(self):
        # Title Label
        self.title_label = tk.Label(self.master, text="Simulation Configuration", font=("Arial", 16))
        self.title_label.grid(row=0, columnspan=2, pady=20)

        # Number of Objects (N)
        self.n_label = tk.Label(self.master, text="Number of Balls (N):")
        self.n_label.grid(row=1, column=0, sticky="e", padx=10)
        self.n_entry = tk.Spinbox(self.master, from_=1, to=100, width=10, textvariable=tk.IntVar(value=self.variables["N"]))
        self.n_entry.grid(row=1, column=1)

        # Ball Size
        self.size_label = tk.Label(self.master, text="Ball Size:")
        self.size_label.grid(row=2, column=0, sticky="e", padx=10)
        self.size_entry = tk.Spinbox(self.master, from_=0.01, to=1.0, increment=0.01, width=10, textvariable=tk.DoubleVar(value=self.variables["ball_size"]))
        self.size_entry.grid(row=2, column=1)

        # Potential Function
        self.pfunc_label = tk.Label(self.master, text="Potential Function:")
        self.pfunc_label.grid(row=3, column=0, sticky="e", padx=10)
        self.pfunc_entry = tk.Entry(self.master, width=40, textvariable=tk.StringVar(value=self.variables["pfunc"]))
        self.pfunc_entry.grid(row=3, column=1)
        
        # Random Color Option
        self.color_label = tk.Label(self.master, text="Random Ball Color?")
        self.color_label.grid(row=4, column=0, sticky="e", padx=10)
        self.color_var = tk.BooleanVar(value=self.variables["color_random"])  # Create a BooleanVar
        self.color_check = tk.Checkbutton(self.master, variable=self.color_var)
        self.color_check.grid(row=4, column=1)

        # Ball Mass
        self.mass_label = tk.Label(self.master, text="Ball Avg., sd. Mass:")
        self.mass_label.grid(row=5, column=0, sticky="e", padx=10)
        self.mass_avg_entry = tk.Spinbox(self.master, from_=0.1, to=100, increment=0.1, width=10, textvariable=tk.DoubleVar(value=self.variables["ball_avg_mass"]))
        self.mass_avg_entry.grid(row=5, column=1)
        self.mass_sd_entry = tk.Spinbox(self.master, from_=0.1, to=100, increment=0.1, width=10, textvariable=tk.DoubleVar(value=self.variables["ball_sd_mass"]))
        self.mass_sd_entry.grid(row=5, column=2)

        # bounds
        self.bounds_label = tk.Label(self.master, text="Bounds (xlb,xub,ylb,yub,zlb,zub)")
        self.bounds_label.grid(row=6, column=0, sticky="e", padx=10)
        self.bounds_entry = tk.Entry(self.master, width=40, textvariable=tk.StringVar(value=self.variables["bounds"]))
        self.bounds_entry.grid(row=6, column=1)
        
        # "Start Simulation" Button
        self.start_button = tk.Button(self.master, text="Start Simulation", command=self.start_simulation)
        self.start_button.grid(row=7, columnspan=2, pady=20)

    def start_simulation(self):
        # Save inputs to variables
        self.variables["N"] = int(self.n_entry.get())
        self.variables["ball_size"] = float(self.size_entry.get())
        self.variables["pfunc"] = self.pfunc_entry.get()
        self.variables["color_random"] = self.color_var.get()
        self.variables["ball_avgmass"] = float(self.mass_avg_entry.get())
        self.variables["ball_sd_mass"] = float(self.mass_sd_entry.get())
        bounds_str = self.bounds_entry.get()
        # change bounds to a 6 tuple
        bounds_list = bounds_str.split(' ')
        for i in range(len(bounds_list)):
            bounds_list[i] = float(bounds_list[i])
        self.variables["bounds"] = tuple(bounds_list)

        # Close the setup window
        self.master.quit()
        self.master.destroy()

def main():
    root = tk.Tk()
    app = SetUpGUI(root)
    root.mainloop()  # Run the GUI event loop
    if app.variables["N"] and app.variables["pfunc"]:
        run_viz(app.variables["pfunc"], app.variables["N"], app.variables["ball_size"], app.variables["color_random"], app.variables["ball_avg_mass"], app.variables["ball_sd_mass"], app.variables["bounds"])

if __name__ == "__main__":
    main()