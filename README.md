# SOIR - Single-Object-Isolation-and-Reconstruction
Scanning a single 3D object on a table, removing the background and combining to a single 3D mesh
The libraries Eigen, Flann, Ceres and FreeImage are needed for the main pipeline. 

Currently the background is removed by a depth threshold and a color threshold that removes white surfaces. In main is a method to remove the background that also removes possible outliers but it still needs finetuning.

We use hierachical ICP to match the current frame to a pointcloud that consists of the pointcloud from the first frame and a pointclouds that updates every 20 frames to the newest frame. 

The data is captured in 30fps. And objects that are not black work the best.

Open todos are:
- adding volumetric fusion to match the current frame to a model and to be able to use raycasting to render the model (instead of overlaying the single aptured and aligned frames)
- adding loop closure to minimize propagation errors
- adding feature matching to stabilize ICP
- improving the background extraction
- Optimizing the process and parameters in respect to runtime (make online possible)


Here is an example of our code processing the first 211 frames. We observed significantly higher computation times but also better results when using lower thresholds for color and depth. SO we assume a better background removal function will increase the time efficiency and the quality of icp:

![frames 1-211 with more points](https://github.com/tja72/SOIR/blob/dev/results/final_merged_frame1-211_original.png)

When using less points and higher thresholds ICP started to mismatch at frame 271:
![frames 1-271 with less points](https://github.com/tja72/SOIR/blob/dev/results/final_merged_frame1-270.jpg)
