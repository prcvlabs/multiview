
# Introduction #

To estimate wait times, we'll need to do the following:

 * Track when people enter a waiting area.
 * Track people (potentially sitting) while they stay in the waiting area.
 * Track people as they leave the area.
 
There are some complications:

 * People may leave the waiting area for a short amount of time, and then 
   return.
 * People may enter the waiting area even though they are not waiting.
 
One easy way to facilitate useful information on waiting, is to report the average amount of time that a person _spends_ in the waiting area.

# Problem Operationalization #

 * Create a zone (2d polygon on the floor) called the "waiting area".
 * Track individuals in the waiting area. 
 * Report the total number of minutes that each individual spends in the waiting area.
 * We can create a histogram of the wait times, and/or report medians, averages and variances.

# Problems with the Existing Tracker #

There are two problems to address with the existing tracker:

 * The existing tracker is optimized for counting entrances/exits, with a view to having a low number false negatives. Unfortunately, this problem will be affected by false positives. Thus we need a low false positive and a low false negative rate.
 * The existing tracker is optimized for tracks crossing a _threshold_: the entrance-way. Thus we're not concerned with tracks splitting, and transferring to other people. But the new problem space requires that tracks _stay_ on an individual person for a protracted period of time.
 
# Proposed Changes to the Tracker #

### Detection Localization ###

Our current person detections are derived from a point-cloud that is projected onto the ground. However, we *do* have the 2D view of the person, from at least 2 sensors, and probably 4 sensors. We also have a pretty good idea of the 3D space a person occupies. Currently we apply a _person model_ as follows: a person is a 3D cylinder with a configurable radius. This model is applied at the tracking stage. However, we should localize the person from 2D information using the person model, and populate the point-cloud appropriately. _This will hugely affect the fidelity of detections_.

To do this, I propose that we combine OpenPose with SLIC and change detection information. When we detect a blob of pixels floating above the ground (according to 2-sensor disparity information), we find the superpixels involved, and then find a 3D cylinder that covers the region. 

If OpenPose information is available (for that frame), then we use the OpenPose skeleton to calculate the position and size of the cylinder. 

If we have a blob of pixels, but no OpenPose information, then we find a cylinder that bounds about 90% of the mass of 3D points, keeping in mind that the _depth_ of points can be adjusted within certain bounds.

The cost function should look at the 3D cylinder with respect to its projections into all 2D sensor images. We want entire superpixels participating (so not a pointwise method), which will stabilize the person detection score.

### Track Fidelity ###

The Fowlkes (Markov) method works extremely well to smooth sequences of spotty detections into coherent tracks. It becomes unstable when applied to long sequences of frames. For example, if there's a 1% chance of a track "jumping" between two people if they are shoulder to shoulder, then there's about a 50% of this happening after about 4.5seconds of footage.

The remedy for this is to use Fowlkes on short sequences -- say 2 seconds -- where the probability of unrecoverable errors are tiny. We then use spatial and reidentification information to stitch tracks together with the Hungarian algorithm.

### Reidentification ###

In the PHADE paper, they take motion features (presumably from OpenPose data), and use PCA to create a "summary" feature matrix. This is a simple and reasonable approach for track-stitching purposes.

We should also be able to extract reasonably accurate size and height information from our 3D data.

We also have the super-pixels for the top of the person. We can try using an average perceptual distance (superpixel LAB score); however, the effectiveness of this approach will vary depending on the lighting conditions. Most indoor lighting is pretty even, so it should be okay.

### Improved Tracker ###

Putting the 3 steps above together, we get an improved tracker that can measure the length of time that people spend in waiting areas. Run-time should (potentially) be close to real time.



