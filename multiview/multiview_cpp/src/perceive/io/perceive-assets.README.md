 
# Types of Documents #

// ---- Stored in calibration ---- //

ArucoResults : <<ArucoResultInfo>>               
               Initial estimates of camera positions for a scene.
               Requires a scene.
               
CAD-models   : <<Sprite>>
               An assimp-compatible (text STL file) that assists in 3D
               reconstruction.

Sensors      : <<DistortionModel>>
               An individual sensor. 

Binocular    : <<BinocularCameraInfo>>
               Bcams are (for now) a pair of sensors. It may end up
               being 1-3 sensors. (Single/Dual/Dual+Face Camera)

Scenes       : <<SceneDescriptionInfo>>
               Lists bcams, and their transformations.
               Requires a CAD model.
               Requires a set of Binocular bcams.



// ---- Other types of input/output ---- //

UndistortInv : <<CachingUndistortInverse>>
               A binary file that contains a given sensors inverse to
               its undistort function. That is: it gives the distort
               function for a sensor.

Manifest     : <<SceneManifest>>
               A manifest file is used to perform a computation in the
               pipeline. It is associated with a scene, and includes
                * one or more videos/images for each scene bcam
                  videos are optional.
                * one timestamp for each included video (required)
                * an _optional_ aruco-result-key
               Requires a scene.
               Note: turn a manifest into a <<SceneDescription>>

By-Hand-Aruco: <<EstimateCamExtrinsicInfo>>
               Computation manifest for producting an ArucoResult (above).
               Requires a scene.
               Requires sensor-info (world/image coords) for each sensor in scene.
               

