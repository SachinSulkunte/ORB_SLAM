# ORB_SLAM
Implementation of ORB SLAM algorithm on sample images.

For this project, I chose to attack this from a total design standpoint. I defined theoretical system requirements, conducted research into algorithms based on those requirements, and implemented my possible solution.

*Scenario: Add SLAM capabilities in a low-cost drone which has a monocular camera and a few sensors (IMU, altitude)*

## 1) Understanding System Requirements
- Program shall be minimally computationally-intensive
- Program should be able to handle temporal jumps due to possible signal loss or pauses in the video stream
- Program should perform near real-time SLAM

## 2) Defining System Inputs and Outputs
- **Inputs:**
  - h264 formatted video stream - 30 Hz
  - Telemetry data - 10 Hz
  - Flight plan including waypoints

- **Outputs:**
  - Pose estimation of drone
  - Global map of drone's environment

## 3) Researching Existing Algorithms
Common SLAM algorithms use data from LIDAR sensors due to the accuracy of such sensors when obtaining depth data. As this situation involves a drone system equipped with a camera system, I researched Visual SLAM or **VSLAM**. Many VSLAM algorithms utilize stereo  or RGB-D cameras as these cameras are able to easily define depth data in the images. For the purposes of this program, we must look at *monocular VSLAM* which utilizes a monocular camera such as the one mounted on the drone. 

While systems such as Dense Tracking and Mapping (DTAM) have shown promising results without relying on feature extraction, such methods are compute-heavy and dependent upon GPU computation which marks them as infeasible for a mobile device[^1]. Semi-Direct Monocular Visual Odometry (SVO) is commonly used for aerial vehicles in outdoor environments however it does not account for a global map and is highly sensitive to lighting changes[^2]. Two of the current state-of-the-art monocular VSLAM systems are Large-Scale Direct Monocular SLAM (**LSD-SLAM**), and Oriented FAST and Rotated BRIEF SLAM (**ORB-SLAM**). While LSD-SLAM can be more powerful and deliver better 3D reconstructions, it is more computationally-intensive considering that it is a direct method and therefore processes a significantly larger quantity of data than ORB-SLAM. ORB-SLAM was also found to rank highest with regards to geometric accuracy[^3]. In addition, ORB feature detection can occur in real-time and is robust againt intense movement as well as allows for re-localization. This is important for this system considering the temporal jumps due to signal loss or image capture.

Therefore, I will be detailing an algorithm that implements *ORB-feature mapping* for the purposes of this program as it is efficient and accuracte as well as capable of running on a mobile device.

## 4) Algorithm
Below, I describe the algorithm designed to carry out the task specified by the prompt.
### Data Pipeline
This pipeline is a modified version of the one given in ORB-SLAM[^4] and Parallel Tracking and Mapping (PTAM)[^5]. Camera initialization allows for the definition of a baseline position and a set of "starting" points at which to begin tracking. For initial 2D-3D correspondence information, the altitude given by the telemetry packet can be used. The scale factor for the measurements obtained from ORB features is determined by the given altitude. Positional tracking as well as map generation is carried out sequentially. A keyframe is defined as the first frame post-initialization and the tracker utilizes feature detection and matching to estimate the new camera pose relative to the points in the previous keyframe. In order to determine when to select a new keyframe, the system evaluates the quality of tracking and either continues tracking or defined a new keyframe based on a threshold value. 

![Data Pipeline](https://user-images.githubusercontent.com/41236722/155263504-c8087b8b-bf7c-4efe-aac7-c9374077243a.png)

### Feature Extraction and Matching
The ORB (Oriented FAST and Rotated BRIEF) features are FAST corners. In order to account for features at varying scale factors, keyframes are scaled multiple times at a designated factor, typically 1.2 in common scenarios. These features are used for triangulating new points in addition to determining camera pose. Feature matching, between frames, is performed using k-Nearest-Neighbors descriptor matching. Every keyframe contains 2D image points with matching 3D object points, ORB feature descriptors, and the estimated world camera pose.

**ORB Feature Extraction:**

![ORB Feature Mapping](https://user-images.githubusercontent.com/41236722/155269337-3fef726f-4d80-4951-be28-18ffba7028d0.png)


### Tracking
Given a set of known 2D-3D points, the Perspective-n-Point (PnP) algorithm can be used to estimate the new camera pose. In this process, for every input frame, matches are calculated to the reference keyframe. The matched points are paired with corresponding 3D points from the reference keyframe to compute the pose. While not fully detailing the PnP algorithm here, it operates using an iterative non-linear optimization to compute the new pose.

### Keyframes
Over time, less features become visible in subsequent frames due to the motion of the camera. In order to maintain a significant set of triangulated points in the camera FOV, the newly calculated world camera poses need to be used to triangulate new 2D-3D correspondences. A new keyframe shall be inserted if:
- If 15 frames have passed since previous keyframe insertion
- Current keyframe contains less than 40 keypoints tracked
- Less than 75% of the keyframe keypoints tracked are in the current frame

This set of conditions ensures that a viable keyframe is always used to compute accurate camera pose over time.

### Mapping
Each keyframe is inserted into a spanning tree which links a keyframe to the keyframe with the most points in common in addition to creating a *bag of words* representation of the keyframe. The map is built by triangulating ORB features from connected keyframes in the tree. To define a match, the ORB features are triangulated and checked for scale consistency. In order to ensure the accuracy of the map, we define a requirement that the new map points must be found in more than 25% of the frames in which they are predicted to be visible (determine by tree). Additionally, to reduce computational cost, keyframes in which 90%+ of the map points can be seen by 3 other keyframes are discarded.

### Loop Closure
The *bag of words* vectors are checked to determine whether keyframes similar to one anohter can be considered candidates for loop closure. If a similarity is supported by enough optimized similarity transformations, then the loop is considered to be true. The current keyframe pose is adjusted to the initial pose, allowing for map points to be fused. A pose graph optimization performed over the graph removes errors due to loop closure and corrects for scale drift[^6].

## 5) Telemetry Data
Thus far, a purely visual SLAM based method has been proposed. In order to improve the robustness and accuracy of the algorithm, sensor fusion with the IMU data can be performed.

As mentioned before, the altitude data is used for accurate scale factor estimation. This is important for obtaining better estimates of velocity vectors. In this section, I identify a method for fusing optical pose estimations obtained from ORB and sensor data from the telemetry packet. The Extended Kalman Filter consists of both an observation model and prediction model, however, the prediction model would be primarily of use if we desired control over the system based on the real-time parameters. Since the drone is following a pre-planned path, this is not necessary.

### Observation Model
The ORB-based SLAM provides a camera pose that is not yet scaled. Applying the scale estimation factor obtained by the altitude data solves this issue. The data from the IMU consists of the yaw, pitch, and roll as well as the velocity in each plane. Using the EKF, we can obtain a pose estimate vector *Z* via the observation function. This function is defined by the state variables of interest: velocities in x and y axis, roll and pitch angles, altitude, and yaw angular velocity[^7].

## 6) System
![System Definition](https://user-images.githubusercontent.com/41236722/155268505-fcb87015-c026-4d9f-921f-953bf412344f.png)


[^1]: https://www.doc.ic.ac.uk/~ajd/Publications/newcombe_etal_iccv2011.pdf
[^2]: https://web.stanford.edu/class/cs231m/projects/final-report-shridhar-neo.pdf
[^3]: https://www.int-arch-photogramm-remote-sens-spatial-inf-sci.net/XLII-2-W17/413/2019/isprs-archives-XLII-2-W17-413-2019.pdf
[^4]: https://ieeexplore.ieee.org/document/7219438?part=1
[^5]: https://www.robots.ox.ac.uk/~gk/publications/KleinMurray2007ISMAR.pdf
[^6]: https://medium.com/@j.zijlmans/lsd-slam-vs-orb-slam2-a-literature-based-comparison-20732df431d
[^7]: https://ieeexplore.ieee.org/document/8588535
