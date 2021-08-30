

Euclidean Transforms and Camera Matrices                {#et_and_cam}
========================================

## The Camera Matrix

The camera matrix works as follows: 

\f[\hat{x} = R(X - C) = RX - RC\f]

Where _R_ is the rotation matrix that transforms _World_ coordinates to the _Cameras_ points of view. _C_ is the camera center. _X_ is the point being "projected". The output, \f$\hat{X}\f$, is a ray through the origin in \f$R^3\f$, or equivalently, it's a point in \f$P^2\f$.

## The Euclidean Transform

This type actually does a _similarity transform_; however, is the scaling value is left at _1.0_, then it's a Euclidean Transform (Rotation + Translation):

\f[\hat{x} = RX + T\f]

Substituting into the camera matrix above, we have \f$T = -RC\f$, which in turn means that the camera center is \f$C = -R^{-1}T\f$.

## Euclidean Transforms in the Scene Description

The camera matrix is "world=>CAM"; however, the _scene description_ (for better or worse) stores "CAM=>world" euclidean transforms. Thus, to project a point \f$X \in R^3\f$ we have:

```
const auto et = scene_desc.sensor_transforms[sensor_ind];
const auto& cu = scene_desc.cu(sensor_ind);
const auto ray = et.inverse().apply(X);
const auto ideal = homgen_P2_to_R2(ray);
const auto pixel = cu.distort(ideal);
```

This code shows the breakdown:

```
// Assume we have a "scene_desc", and a "sensor_ind", and a point "X"
const auto et = scene_desc.sensor_transforms[sensor_ind];
const auto& cu = scene_desc.cu(sensor_ind);
const auto et_inv = et.inverse();

auto project = [&](const Vector3& Y) -> Vector2 {
   return cu.distort(homgen_P2_to_R2(et_inv.apply(Y)));
};

const auto T = et_inv.translation;
const auto q = et_inv.rotation;
const auto C = -q.inverse_rotate(T); // The camera center
const auto x = project(X);           // Pixel coordinate of X
      
// These are equivalent ways of getting the 'ray' from the cameras point of view
const auto ray_cam0 = homgen_R2_to_P2(cu.undistort(x)).normalised();
const auto ray_cam1 = (q.rotate(X) + T).normalised();
const auto ray_cam2 = (q.rotate(X - C)).normalised();

// Rotate the ray to get it in the worlds point of view:
const auto ray_world0 = q.inverse_rotate(ray_cam0);
const auto ray_world1 = (X - C).normalised(); // the same
```
