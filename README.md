# LSD-SLAM: Large-Scale Direct Monocular SLAM

LSD-SLAM is a novel approach to Real-Time Monocular SLAM. It is fully direct (i.e. does not use keypoints / features) and creates large-scale, 
semi-dense maps in real-time on a laptop. For more information see
[http://vision.in.tum.de/lsdslam](http://vision.in.tum.de/lsdslam)
where you can also find the corresponding publications and Youtube videos, as well as some 
example-input datasets, and the generated output as rosbag or .ply pointcloud.


### Related Papers

* **LSD-SLAM: Large-Scale Direct Monocular SLAM**, *J. Engel, T. Schöps, D. Cremers*, ECCV '14

* **Semi-Dense Visual Odometry for a Monocular Camera**, *J. Engel, J. Sturm, D. Cremers*, ICCV '13



# 1. Quickstart / Minimal Setup

First, install LSD-SLAM following 2.1 or 2.2, depending on your Ubuntu / ROS version. You don't need OpenFabMap for now.

Download the sequence from XXXXX


Launch the viewer:

		rosrun lsd_slam_viewer viewer

Launch the ros node:

		rosrun lsd_slam_core live_slam image:=/image_raw camera_info:=/camera_info

Play the sequence:

		rosbag play ~/name.bag



You should see one windows showing the current Keyframe with color-coded depth (from live_slam), 
and one window showing the 3D map (from viewer). If for some reason the initialization fails 
(i.e., after ~5s the depthmap still looks wrong), focus the depthmap and hit 'r' to re-initializee.



# 2. Installation
We tested LSD-SLAM on two differen system configurations, Using Ubuntu 12.04 (Precise) and ROS fuerte, or Ubuntu 14.04 (trusty) and ROS indigo. Note that building without ROS is not supported, however ROS is only used for input and output, facilitating easy portability to other platforms.


## 2.1 ROS fuerte + Ubuntu 12.04
Install system dependencies:

    sudo apt-get install ros-fuerte-libg2o liblapack-dev libblas-dev freeglut3-dev libqglviewer-qt4-dev

In your ROS package path, clone the repository:

    roscd
    git clone https://svncvpr.informatik.tu-muenchen.de/git/lsdslam-public lsd_slam

Compile the two package by typing:

    rosmake lsd_slam




## 2.2 ROS indigo + Ubuntu 14.04
**We do not use catkin, however fortunately old-fashioned CMake-builds are still possible with ROS indigo.**
For this you need to create a rosbuild workspace (if you don't have one yet), using:

    sudo apt-get install python-rosinstall
    mkdir ~/rosbuild_ws
    cd ~/rosbuild_ws
    rosws init . /opt/ros/indigo
    mkdir package_dir
    rosws set ~/rosbuild_ws/package_dir -t .
    echo "source ~/rosbuild_ws/setup.bash" >> ~/.bashrc
    bash

Install system dependencies:

    sudo apt-get install ros-indigo-libg2o liblapack-dev libblas-dev freeglut3-dev libqglviewer-dev

In your ROS package path, clone the repository:

    roscd
    git clone https://svncvpr.informatik.tu-muenchen.de/git/lsdslam-public lsd_slam

Compile the two package by typing:

    rosmake lsd_slam






## 2.3 OpenFabMap for large loop-closure detection [optional]
If you want to use OpenFABMAP for large loop closure detection, uncomment the following lines in `lsd_slam_core/CMakeLists.txt` :

    #add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
    #include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
    #add_definitions("-DHAVE_FABMAP")
    #set(FABMAP_LIB openFABMAP )

**Note for Ubuntu 14.04:** The packaged OpenCV for Ubuntu 14.04 does not include the nonfree module, which is required for OpenFabMap (which requires SURF features).
You need to get a full version of OpenCV with nonfree module, which is easiest by compiling your own verrsion. 
We suggest to use the [2.4.8](https://github.com/Itseez/opencv/releases/tag/2.4.8) version, to assure compatibility with the current indigo open-cv package.





# 3 Usage
LSD-SLAM is split into two ROS packages, `lsd_slam_core` and `lsd_slam_viewer`. `lsd_slam_core` contains the full SLAM system, whereas `lsd_slam_viewer` is optionally used for 3D visualization.
Please also read **Genenral Notes for good results**

## 3.1 `lsd_slam_core`
We provide two different usage modes, one meant for live-operation (`live_slam`) using ROS input/output, and one `dataset_slam` to use on datasets in the form of image files.

### 3.1.1 Using `live_slam`
If you want to directly use a camera.

    rosrun lsd_slam_core live_slam /image:=<yourstreamtopic> /camera_info:=<yourcamera_infotopic>

When using ROS camera_info, only the image dimensions and the `K` matrix from the camera info messages will be used - hence the video has to be rectified.

Alternatively, you can specify a calibration file using 

    rosrun lsd_slam_core live_slam /image:=<yourstreamtopic> _calib:=<calibration_file>

In this case, the camera_info topic is ignored, and images may also be radially distorted. See the Camera Calibration section for details on the calibration file format.


### 3.1.2  Using `dataset_slam`

    rosrun lsd_slam_core dataset_slam _files:=<files> _hz:=<hz> _calib:=<calibration_file>

Here, `<files>` can either be a folder containing image files (which will be sorted alphabetically), or a text file containing one image file per line. `<hz>` is the framerate at which the images are processed, and `<calibration_file>` the camera calibration file. 

Specify `_hz:=0` to sequentialize tracking and mapping, i.e. make sure that every frame is mapped properly. Note that while this typically will give best results, it can be much slower than real-time operation.


### 3.1.3 Camera Calibration
LSD-SLAM operates on a pinhole camera model, however we give the option to undistort images before the are being used. You can find some sample calib files in `lsd_slam_core/calib`.

#### Calibration File for FOV camera model:

    d1 d2 d3 d4 d5
    in_width in_height
    "crop" / "full" / "none" / "e1 e2 e3 e4 0"
    out\_width out_height


Here, d1 to d5 are fx/width fy/height cx/width cy/height d, as given by the PTAM cameracalibrator, in\_width and in\_height is the input image size, and out\_width out\_height is the desired undistorted image size. The third line specifies how the image is distorted, either by specifying a desired camera matrix in the same format as d1-d4, or by specifying "crop", which crops the image to maximal size.


#### Calibration File for Pre-Rectified Images
This one is with no radial Distrotion, as a special case of ATAN camera model but without the computational cost:

    fx/width fy/height cx/width cy/height 0
    width height
    none
    width height


#### Calibration File for OpenCV camera model [untested!]:

    fx fy cx cy d1 d2 d3 d4 d5 d6
    inputWidth inputHeight
    "crop" / "full" / "none" / "e1 e2 e3 e4 0"
    outputWidth outputHeight


### 3.1.4 Useful Hotkeys

- `r` Do a full reset

- `d / e` Cycle through debug displays (in particular color-coded variance and color-coded inverse depth).

- `o` Toggle on screen info display

- `m` Save current state of the map (depth & variance) as images to `lsd_slam_core/save/`

- `p` Brute-Force-Try to find new constraints. May improove the map by finding more constraints, but will block mapping for a while. 

- `l` Manually indicate that tracking is lost: will stop tracking and mapping, and start the re-localizer.



### 3.1.5 Parameters (Dynamic Reconfigure)
A number of things can be changed dynamically, using (for ROS fuerte)

    rosrun dynamic_reconfigure reconfigure_gui 

or (for ROS indigo)

    rosrun rqt_reconfigure rqt_reconfigure

Parameters are split into two parts, ones that enable / disable various sorts of debug output in `/LSD_SLAM/Debug`, and ones that affect the actual Algorithm, in `/LSD_SLAM`.


* `minUseGrad`: [double] Minimal Absolut Image Gradient for a Pixel to be used at all. Increase, if your camera has large image noise, decrease if you have low image-noise and want to also exploit small gradients.
* `cameraPixelNoise`: [double] Image intensity noise used for e.g. tracking weight calculation. Sould be set larger than the actual sensor-noise, to also account for noise originating from discretization / linear interpolation.
* `KFUsageWeight`: [double] Determines how often Keyframes are taken, depending on the Overlap to the current Keyframe. Larger -> more Keyframes.
* `KFDistWeight`: [double] Determines how often Keyframes are taken, depending on the Distance to the current Keyframe. Larger -> more Keyframes.
* `doSLAM`: [bool] Toggle Global Mapping Component on/off. Only takes effect after a reset.
* `doKFReActivation`: [bool] Toggle Keyframe Re-Activation on/off: If close to an existing keyframe, re-activate it instead of creating a new one. If false, Map will continually grow even if the camera moves in a relatively constrained area; If false, the number of keyframes will not grow arbitrarily.
* `doMapping`: [bool] Toggle entire Keyframe Creating / Update module on/off: If false, only tracking stays active, which will prevent rapid motion or moving objects from corrupting the map.
* `useFabMap`: [bool] Use OpenFABMAP to find large loop-closures. Only takes effect after a reset, and requires LSD-SLAM to be compiled with FabMap.
* `allowNegativeIdepths`: [bool] Allow idepth to be (slightle) negative, to avoid introducing a bias for far-away points.
* `useSubpixelStereo"`: [bool] Compute subpixel-accurate stereo disparity.
* `useAffineLightningEstimation`: [bool] EXPERIMENTAL: Correct for global affine intensity changes during tracking. Might help if you have Problems with Auto-Exposure.
* `multiThreading`: [bool] Toggle Multi-Threading of DepthMap Estimation. Disable for less CPU usage, but possibly slightly less quality.
* `maxLoopClosureCandidates`: [int] Maximal of Loop-Closures that are tracked initially for each new keyframe.
* `loopclosureStrictness`: [double] Threshold on reciprocal loop-closure consistency check, to be added to the map. Larger -> more (possibly wrong) Loopclosures.
* `relocalizationTH`: [double] How good a relocalization-attempt has to be, to be accepted. Larger -> More Strict.
* `depthSmoothingFactor`: [double] How much to smooth the depth map. Larger -> Less Smoothing


Useful for debug output are:

* `plotStereoImages`: [bool] Plot Searched Stereo Lines, and color-coded stereo-results. Nice Visualization of what's going on, however drastically decreases mapping speed.
* `plotTracking`: [bool] Plot final tracking residual. Nice Visualization of what's going on, however drastically decreases tracking speed.
* `continuousPCOutput`: [bool] Publish Current Keyframe's Pointcloud after each update, to be seen in the viewer. Nice Visualization, however bad for performance and bandwith.







### 3.1.6 General Notes for Good Results

* Use a **global-shutter** camera. Using a rolling shutter will lead to inferior results.
* Use a lens with a **wide field-of-view** (we use a 130° fisheye lens).
* Use a **high framerate**, at least 30fps (depending on the movements speed of course). For our experiments, we used between 30 and 60 fps. 
* We recommend an image resolution of **640x480**, significantly higher or lower resolutions may require some hard-coded parameters to be adapted.
* LSD-SLAM is a monocular SLAM system, and as such cannot estimate the absolute scale of the map. Further it requires **sufficient camera translation**: Rotating the camera without translating it at the same time will not work. Generally sideways-motion is best - depending on the field of view of your camera, forwards / backwards motion is equally good. Rotation around the optical axis does not cause any problems.
* During Initialization, it is best to move the camera in a circle parallel to the image, without rotating it. The scene should contain sufficient structure (intensity gradient at different depths).
* **Adjust** `minUseGrad` **and** `cameraPixelNoise` to fit the sensor-noise and intensity contrast of your camera.
* If tracking / mapping quality is poor, try decreasing the keyframe thresholds `KFUsageWeight` and `KFDistWeight` slightly, to generate more keyframes.
* Note that LSD-SLAM is very much non-deterministic, i.e. results will be different each time you run it on the same dataset. This is due to parallelism, and the fact that small changes regarding when keyframes are taken will have a huge impact on everything that follows afterwards.


## 3.2 LSD-SLAM Viewer
The viewer is only for visualization. It can also be used to output a generated pointcloud as .ply.
Start it using

    rosrun lsd_slam_viewer viewer

You can use rosbag to record and re-play the output generated by certain trajectories. Record & playback using

    rosbag record /lsd_slam/graph /lsd_slam/keyframes /lsd_slam/liveframes -o file.bag
    rosbag play file.bag

You should never have to restart the viewer node, it resets the graph automatically.


### 3.2.1 Useful Hotkeys

- `r` Reset, will clear all displayed data.

- `w` Print the number of points / currently displayed points / keyframes / constraints to the console.

- `p` Write currently displayed points as PointCloud to file lsd_slam_viewer/pc.ply, which can be opened e.g. in meshlab. Use in combination with sparsityFactor to reduce the number of points.


### 3.2.2 Parameters (Dynamic Reconfigure)

- `showKFCameras `: Toggle Drawing of blue Keyframe Camera-Frustrums min: False, default: True, max: True
- `showKFPointclouds `: Toggle Drawing of Pointclouds for all Keyframes min: False, default: True, max: True
- `showConstraints `: Toggle Drawing of red/green Pose-Graph Constraints min: False, default: True, max: True
- `showCurrentCamera `: Toggle Drawing of red Frustrum for the current Camera Pose min: False, default: True, max: True
- `showCurrentPointcloud `: Toggle Drawing of the latest pointcloud added to the map min: False, default: True, max: True
- `pointTesselation `: Size of Points min: 0.0, default: 1.0, max: 5.0
- `lineTesselation `: Width of Lines min: 0.0, default: 1.0, max: 5.0
- `scaledDepthVarTH `: log10 of threshold on point's variance, in the respective keyframe's scale.  min: -10.0, default: -3.0, max: 1.0
- `absDepthVarTH `: log10 of threshold on point's variance, in absolute scale. min: -10.0, default: -1.0, max: 1.0
- `minNearSupport `: only plot points that have #minNearSupport similar neighbours (higher values remove outliers) min: 0, default: 7, max: 9
- `cutFirstNKf `: do not display the first #cutFirstNKf keyframe's pointclouds, to remove artifacts left-over from the random initialization. min: 0, default: 5, max: 100
- `sparsifyFactor `: only plot one out of #sparsifyFactor points, selected at random. Use this to significantly speed up rendering for large maps. min: 1, default: 1, max: 100
- `saveAllVideo `: save all rendered images... only use if you know what you are doing. min: False, default: False, max: True
- `keepInMemory `: If set to false, Pointcloud is only stored in OpenGL buffers, and not kept in RAM. This greatly reduces the required RAM for large maps, however also prohibits saving / dynamically changing sparsifyFactor and variance-thresholds. min: False, default: True, max: True





# 4 Datasets

For convenience, we provide a number of datasets, including the video, lsd-slam's output and the generated pointcloud as .ply. See
[http://vision.in.tum.de/lsdslam](http://vision.in.tum.de/lsdslam)


# 5 Licence
LSD-SLAM is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.

For commercial purposes, we also offer a professional version under different licencing terms.




