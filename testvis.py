import open3d as o3d
import numpy as np
import time

# Create a point cloud
pcd = o3d.geometry.TriangleMesh.create_sphere(1, 20, False)

# Visualizer setup
vis = o3d.visualization.Visualizer()
vis.create_window(window_name = "SIM", width = 1280, height = 720)

# Add point cloud to the visualizer
vis.add_geometry(pcd)

# Animation loop
for i in range(1000):  # Number of animation steps
    
    # Update the visualizer with the new point cloud data
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    
    time.sleep(0.01)  # Slow down the loop to make it visible