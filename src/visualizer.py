from turtle import update
import numpy as np
import open3d as o3d
import time

from psutil import Popen
from body import Body
from collision import check_collision
from dynamics import dynamics
import subprocess
import sys


origin = [0, 0, 0]
# holds all created objects
#objects = []
vis = o3d.visualization.Visualizer()
vis.create_window(window_name = "SIMULATION", width = 1280, height = 720)
view_control = vis.get_view_control()
view_control.set_zoom(100)
render_options = vis.get_render_option()
render_options.background_color = [0, 0, 0]


# some helper functions
def create_sphere(position: list, radius: float, color: list, mass: float, exclude: bool):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius, 20, False)
    sphere.translate(position)
    sphere.paint_uniform_color(color)
    body = Body(xyz = np.array(position), mass = mass, geom = radius, model = sphere, exclude = exclude)
    vis.add_geometry(Body.objects[-1].model)

def create_box(position: list, w: float, d: float, h: float, color: list, mass: float, exclude: bool):
    box = o3d.geometry.TriangleMesh.create_box(width = w, depth = d, height = h)
    box.translate(position)
    box.paint_uniform_color(color)
    body = Body(xyz = np.array(position), mass = mass, model = box, geom = 0, exclude = exclude)
    vis.add_geometry(Body.objects[-1].model)

def update_viewport():
    for obj in Body.objects:
        vis.update_geometry(obj.model)

def update_dynamics(dynam_new):
    Body.dynamics_matrix = dynam_new
    for i in range(len(Body.dynamics_matrix)):
        bodypos = Body.objects[i].position()
        for j in range(1, 4):
            Body.dynamics_matrix[i, j] = bodypos[j - 1]

def run_viz():
    dt = 0.001
    global vis

    create_sphere([4, 2, 0], 0.5, [1, 0, 0], 1, exclude = False)
    create_sphere([4, 4, 0], 0.5, [0, 0, 1], 1, exclude=False)
    create_sphere([4, 6, 0], 0.5, [0, 1, 0], 1, exclude=False)
    create_sphere([4, 8, 0], 0.5, [0, 1, 0], 1, exclude=False)
    create_box([5, 0, -5], 10, 10, 0.1, [1, 1, 1], 1000000, exclude=True)

    # pfunc = "0.5*75*x**2+0.5*50*y**2+0.5*100*z**2"
    pfunc = "m*9.8*y"

    t = 0
    ## main loop ##
    while True:
        update_viewport()
        collisions, _ = check_collision()
        # get updated mass, positions, momentums
        dynam = dynamics(Body.dynamics_matrix, dt, pfunc, np.array([[0, 200*np.exp(-0*(t-0.1)**2), 0], [0, -50*np.exp(-0*(t-0.1)**2), 0], [0, -100*np.exp(-0*(t-0.1)**2), 0], [0, -50*np.exp(-0*(t-0.1)**2), 0], [0, 0, 0]]), 0.5)

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

def main():
    # start the controller gui as a subprocess
    proc = subprocess.Popen([sys.executable, "controller.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    # run the physics sim
    out = proc.stdout.readline()
    print(out)
    run_viz()

if __name__ == "__main__":
    main()