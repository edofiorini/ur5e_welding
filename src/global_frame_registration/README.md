# Global reference frame registration
The package depends on **aruco_ros** for the fiducial marker detection and from **easy_handeye** for the estimation of the transformation between the optical frame of the endoscope and the kinematic end-effector of the ECM. The registration system will record for each configured arm a set of *n* points plus an extra one to define the normal verso. Then it will record the position of the fiducial marker and then it computes the base transformation between the **global frame** and the frame in which the tool and the marker positions are referred.

## Configuration
The following parameters can be configured:
- **markerSize**: the size in meters of the markers (default: 0.025)
- **side**: which side of the endoscope we want to use for the calibration (default: left)
- **ref_frame**: the frame_id for the marker pose (default: the parent frame)
- **global_frame**: the global frame name (default: "/world")
- **tool_points**: the number of points used during a single arm calibration (default: 3)
- **load_reference**: use an existing calibration (default: False)

The specification of which topic must be used are defined in *robots.yaml* with two different list: **tool_subscribers** and **camera_subscribers**.

## Launcher
- **saras.launch**: launches the calibration for the two saras arms and the ECM
- **davinci.launch**: launches the calibration for the two PSM arms and the ECM
- **endoscope_hand2eye.launch**: launches the optical frame to kinematic estimation for the ECM

## Remarks
- The center is computed as the centroid of the recorded points (be aware to choose a symmetric pattern)
- To define the forward direction on the calibration plane two directions are used: for the tool calibration the direction between the center and the first point; for the camera calibration the direction between the last marker and the first (sorted by id).
- The results of the calibration must be dumped manually from the parameters server and then placed inside the *calibration.yaml* file to be loadable.
- At the end of hand2eye calibration pressing the save button on the user interface automatically store the calibration and will be loaded by the saras or davinci launchers.