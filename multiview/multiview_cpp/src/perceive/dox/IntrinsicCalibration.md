
Intrinsic Calibration                {#intrinsic_calibration}
=====================

# Overview

We have to estimate distortion parameters, and intrinsic parameters, where the intrinsics are: \f$f\f$, \f$f\f$, \f$ppt_x\f$, \f$ppt_y\f$. We will assume that the camera sensor pixels are square.

## Calibration Procedure

The procedure is straightforward, but as yet untested. It works as follows:

 1. Detect the curved lines of the checkerboard. We have code that does some of this, but it may need refinement for higher angular distortion around the edges of the image. (1 day)
 2. Apply "Claus and Fitzgibbon (2005)" _Plumbline_ distortion correction. This will create a rectilinear image using the _Rational_ camera model. This is an approximate model. 
 3. Convert the _Rational_ model to a higher order polynomial model. 
 4. Bundle-adjust so that lines are straight, and cross-ratio constraint is satisfied. At this stage we have an undistorted _projective_ image , with unknown intrinsic parameters. (3 days for [2.-4.])
 5. We cannot recover intrinsic parameters using Zhang, because it relies on knowing the orientation of the checkerboard. (That is, we must distinguish the _direction_ of the X and Y axes.) So instead, we'll use the "back-project kite" method that I've independently developed, which fixes the slant (two possibilities) of each checkerboard from _single squares only_.
 6. Using some as yet to be determined method, I'll make a linear-least squares estimate of the intrinsic parameters. (3 days for [5.-6.])
 7. Bundle adjust so that the squares are square. (1 day)
 8. Multiply through all the terms to produce a final polynomial matrix that contains all distortion and intrinsic parameters. (1 day)
 9. As an optional step, we'll look at the error of doing steps [5..8] using all five intrinsic parameters. (1 days)
 
## Order of Battle

Steps to complete this work:
 
 1. Build and complete the calibration board. We're actually super flexible with this. The system only needs _two_ non-co-planar patterns, and then we take video footage slowly moving into the board, _filling the frame_. That is, if we get it right, we'll be able to print onto some plastic, put it on the end of a broomstick, and just hold it up to "in-store" cameras. (0.5 days)
 2. Get video footage of the board with the nest camera. (0.5 days)
 3. Work through the steps above. (10 days)


 




