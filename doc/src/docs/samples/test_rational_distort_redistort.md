# Code sample "samples/test_rational_distort_redistort.py"

```python

import numpy as np
import cv2
import os
import sys
import pdb

sys.path.append(sys.path.append('./..' if os.path.dirname(__file__) == ''
                                else os.path.dirname(__file__) + '/..'))
from multiview import *

def test_rational_distort_redistort(distort_filename, image_filename):
    
    """
    Shows how to load a *distortion parameters file*, and use it to undistort
    and then redistort an image. The `scale` variable (below) can be used
    to enlarge or shrink the undistorted image. The redistorted image is then
    rescaled from the shrunken one, to return to the original image size.
    """
    
    print_info('Distorted and redistort code sample.')

    print('Loading calibration data file "{0}"'.format(distort_filename))
    data = None
    try:
        data = calibrate.read_distortion_file(distort_filename)
    except RuntimeError as e:
        print_err("Error reading distortion file '{0}'".format(distort_filename))
        print(traceback.format_exc())
        return
    
    # Let's give it a whirl, first load the image
    print('Loading image file "{0}"'.format(image_filename))
    raw_image = cv2.imread(perceive_data_filename(image_filename))
    image_pair = np.hsplit(raw_image, 2) # This should be a dual image
    undistorted_pair = [None, None]
    redistorted_pair = [None, None]

    eye = np.identity(3, np.float32)
        
    # Note that the distortion model maps a (distorted) pixel coordinate to
    # an undistorted ray in 3D space. This ray is a normalized coordinate.
    # Thus, to get pixels "out", you need to specify the intrinsic parameters
    # for the time of camera that you are using.
    K = np.array([[200, 0, data.width/2.0],
                  [0, 200, data.height/2.0],
                  [0, 0, 1]])

    # We shrink the undistorted image... and the enlarge the "re"-distorted image
    scale = 0.667
    K_scale = np.copy(K)
    K_scale *= scale
    K_scale[2, 2] = 1.0
    
    for index in range(0, 2):
        print("Processing {0} image".format('left' if index == 0 else 'right'))        
        
        # x_d = H2^{-1} @ distort(H1 @ [x, y, 1]^t)
        d_mapx, d_mapy = make_distort_rectify_map(int(scale * data.width),
                                                  int(scale * data.height),
                                                  'rational', data.params[index],
                                                  eye, K_scale)

        # x_u = H2^{-1} @ undistort(H1 @ [x, y, 1]^t)
        u_mapx, u_mapy = make_undistort_rectify_map(data.width, data.height,
                                                    'rational', data.params[index],
                                                    K_scale, eye)

        # Use cv2.INTER_AREA for shrinking images,
        # And cv2.INTER_CUBIC (slow) and cv2.INTER_LINEAR for zooming images
        undistorted_pair[index] = cv2.remap(image_pair[index], d_mapx, d_mapy,
                                            cv2.INTER_AREA)
        redistorted_pair[index] = cv2.remap(undistorted_pair[index], u_mapx, u_mapy,
                                            cv2.INTER_LINEAR)
        
        
    # Let's output the results
    out_undistorted = np.hstack((undistorted_pair[0], undistorted_pair[1]))
    out_redistorted = np.hstack((redistorted_pair[0], redistorted_pair[1]))

    out_filename_un = '/tmp/zz-1-undistorted.png'
    out_filename_re = '/tmp/zz-2-redistorted.png'
    
    cv2.imwrite(out_filename_un, out_undistorted)
    cv2.imwrite(out_filename_re, out_redistorted)

    print_info('Output saved to "{0}" and "{1}"'.format(out_filename_un, out_filename_re))    

# -- ~ -- #
dist_file = 'calibration/regression-testing-data/Camera15_stereo_wide-angle_DISTORT_500.json'
image_file = 'calibration/regression-testing-data/corners_000020.png'
test_rational_distort_redistort(dist_file, image_file)

```
