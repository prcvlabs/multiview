
# Products

I think the most sensible thing to do is to ship this all to Fischers. Get Mark working on it. I can be in close 1-1 to ensure that I get the data that I need.

 * Some system for generating accurate 3D color maps. For example this:
   [1] [https://structure.io/](https://structure.io/) costs about $500.
 * Some security cameras with distortions. 
   [1] Two high resolution fisheye cameras.
   [2] One more high resolution camera, wide-angle, with significant distortion.
   [3] One of our (almost non-distorted) cameras.

We should figure out the precise set of devices that we're getting. Can you contact our potential security camera collaborators?

# Intrinsic Calibration

 * Polynomial models: 
   [1] Use _multiple_ different dimensions to model
   [2] Convert OpenCV (or arbitrary) distortion function to Polynomial Model
   [3] Assess error metrics

 * Radial Centre.
   Harley & Kang (2007) Parameter-free Radial distortion correction
   Josephson Byrod [15] Unknown radial distortion, faster runtime [17], [20], more models [22], non-parametric [4]
   Tsai [40] radial only + unknown focal length. (What about distortion centre, and principal point?)
   1D radial camera model [38]
 
 * Absolute pose solvers [17], [22]
 
 * Straightening lines.
   Claus & Fitzgibbon (2005) Plumbline

# Extrinsic Calibration

 * Person correspondence... 3D lines must intersect... 
   Probably under-determined, but... a good prior (?)
   
 * SLAM-based feature mapping [5], [*12*], present
 
 * Perspective-three-points problem [9]

# Procedure

 0. Collate spare-feature-map COLMAP [33]
 1. Estimate 1D radial model
 2. Estimate extrinsics
 3. Bundle adjust, including principal point
 4. Forward translation & intrinsic estimation
 5. Bundle adjust

# References

 [5] Carrera, G., Angeli, A., Davison, A.J.: Slam-based automatic extrinsic calibration of a multi-camera rig. In: International Conference on Robotics and Automation (ICRA) (2011)

 [9] Haralick, B.M., Lee, C.N., Ottenberg, K., Nölle, M.: Review and analysis of solutions of the three point perspective pose estimation problem. International journal of computer vision 13(3), 331–356 (1994)

 [12] Heng, L., Furgale, P., Pollefeys, M.: Leveraging image-based localization for
infrastructure-based calibration of a multi-camera rig. Journal of Field Robotics 32(5), 775–802 (2015)
 
 [15] Josephson, K., Byrod, M.: Pose estimation with radial distortion and unknown
focal length. In: Computer Vision and Pattern Recognition (CVPR) (2009)

 [17] Kukelova, Z., Bujnak, M., Pajdla, T.: Real-time solution to the absolute pose problem with unknown radial distortion and focal length. In: International Conference on Computer Vision (ICCV) (2013)
 
 [20] Larsson, V., Kukelova, Z., Zheng, Y.: Making minimal solvers for absolute pose estimation compact and robust. In: International Conference on Computer Vision (ICCV) (2017)

 [22] Larsson, V., Sattler, T., Kukelova, Z., Pollefeys, M.: Revisiting radial distortion absolute pose. In: International Conference on Computer Vision (ICCV) (2019)

 [33] Schonberger, J.L., Frahm, J.M.: Structure-from-motion revisited. In: Computer Vision and Pattern Recognition (CVPR) (2016)

 [38] Thirthala, S., Pollefeys, M.: Radial multi-focal tensors. International Journal of Computer Vision (IJCV) 96(2), 195–211 (2012)

 [40] Tsai, R.: A versatile camera calibration technique for high-accuracy 3d machine vision metrology using off-the-shelf tv cameras and lenses. IEEE Journal on Robotics and Automation 3(4), 323–344 (1987)
