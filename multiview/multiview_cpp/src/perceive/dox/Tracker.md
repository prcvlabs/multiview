
Tracker                {#tracker_stuff}
=======

# Brief #

Multiview's pipeline has the following conceptual steps:

 * Per-frame results are calculated: an annotated floor histogram that measure the likelihood that particular histogram cells are the centre of tracks.
 * Per-frame results are turned into tracklets: Multiview finds continuous runs of tracks within 20 frame segments of the epoch. (Configurable.)
 * Multiview examines adjacent sets of tracklets to combine, split, smooth, and remove tracklets in order to create continuous tracks that can be possibly continuous through the entire epoch.
 * The results are saved to an envelope.

# Current Design #

## Tracklets ##

The tracklets algorithm operates over a window of 20 frames. (Configurable.)

 * We use the hungarian algorithm to find an optimal mapping between all pose detections in a frame with its successor frame.
 * In a greedy algorithm, we start with the first frame, and iterate through the frames in order.
 * We find all pose detections for a given frame, and attempt to build a tracklet by finding successive frames (identified by the hungarian algorithm) that are below a threshold.
 
Tracklets are fickle. They disappear. They skip. Etc.

## Tracker ##

 * Merge endpoints.
 * Merge beginning/end overlaps.
 * Remove overlapping tracks.
 * Remove short tracks.
 * Smooth tracks.
 * Split tracks.
 
# Defects #

## Known Problems to Address (Testcases) ##

 * TESTCASE: How do we use 2D patches (Normalized cross-correlation in LAB space) to improve _merge-matching_? We need real-world data where the merging patches get confused, either by occlusion, or because people have weird clothing that doesn't match up when seen from different angles. That is, 2D patches should be a hint, and a soft constraint. See B7-pre-merge-frame7.
 * TESTCASE: people standing in a circle our merge-matching may be overly aggressive in matching one person onto another along a degenerate line. (i.e., when they're occluding each other.) This can be fixed by _enforcing height_ smoothing across the tracks, but does this imply moving the merge stage into tracklets themselves?
 * TESTCASE: _implausible detections_. We want to be robust against false positives, including people detected on the floor, etc. We need some sort of "is this a rational detection" that is applied in the tracklets. See B7-frame44. This will allow us to increase the false-positive rate of the pose detector.
 * TESTCASE: _plausible false-positives_. A mannequin/statue, poster, TVs. These should be dealt with using "dead-zones", but the dead-zones must be available to multiview.(Part of loading the scene.)
 * TESTCASE: we have data where the pose detections are screwy, and switch front-to-back occasionally, thus flipping gaze detection. We now have to _smooth gaze detection_.
 * TESTCASE: we have detections far from two sensors, at various angles, and the sensor calibration isn't perfect. The _detections should merge naturally_.
 * TESTCASE: we have to _interpolate across time_ better, taking advantage of 2D patches, and height. So we need testcases where interpolation should and shouldn't happen, so that we get better characterize the algorithm.
 
I'd like to see some testing suites. We could synthetically make data, or record our own videos, or find and select choice (short) clips of existing data.

 * 2D displacement of openpose pose
 * Create a bitmap around stick figure
 * Consistent track height (???)

## Other Problems ##

 * How do we find staff? We have 2D patches, and staff zones, but we've got an open ended question here. Can staff we tagged in any other way?
 * We need to apply a Kalman Filter to the final tracks.
 
## Performance ##

 1. The currently pipeline will be (conceptually) moved to the frame-pipeline. (Okay, maybe rename it.)
 2. A new controller will sit over the pipeline, and tracklets, and tracks. 
 3. The new controller tracks memory loading, _and_ CPU parallelism, and schedules tasks accordingly, with a view towards maximizing throughput for CLI mode, and latency for gui mode.

# Proposed Design #

## Updates to Localization ##

 * Move `pose_3d_greedy_select` out of localization-data, and into tracklets.

## Updates to Tracklets ##

 * The 3D pose objects are augmented with small pixel samples from the pose, and angular location in the image object.
 * For each frame, attempt to coalesce multiple pose3d objects together, using 2D pixel features, and proximity with angular location.
 * Still use the hungarian algorithm to match between pose objects in different frames, but considering proximity in coordination with angular location, as well as 2D pixel features. In addition, and importantly, we interpolate between multiple empty frames, with the cost function considering the histogram point cloud information.
 * An adaptive algorithm that tracks the average memory usage for a frame. Supply a configurable memory target. 

In theory, every tracklet window could be run in parallel, but this presents memory issues for large scenes and long epochs. A middle ground is to compute a _fixed_ set of tracklet windows at once. So, for example, we could execute 8 tracklet windows in parallel. This puts an upper-bound on the amount of memory consumed by the tracklets phase of execution. When choosing the number of parallel jobs, it should be configurable. The default behaviour is to query the amount of physical RAM, and the number of CPUs, and set the parallelism accordingly.

## Updates to Tracks ##

 * Advancing frame-by-frame, we consider if a tracklet can be joined to _any_ previously seen (or current) tracklet... interpolating over empty frames. This involves considering the angular position of the tracklet, as well as 2D pixel features, in order to merge _aliased_ tracks.
 * We smooth the paths from centre of one tracklet window to the centre of the next tracklet window. (Caveat, if the first tracklet window is visible, then we smooth from the start of that window.)
 * We then remove overlapping tracks and short tracks.
 * We then split the track as per normal.
 
This can be made more parallel as follows. Given a sequence of tracklet windows: ABCDEFGHI, we want to process: AB, BC, CD, DE, EF, FG, GH, and HI as tracks. We can assign each tracklet a number modulo 4, and process everything in the equivalence class in parallel, making a total of for large parallelized computations. So [AB, EF] in parallel, then [BC, FG] in parallel, then [CD, GH], then [DE, HI].

### Notes -- unedited etc. ###

A position error term is associated with each mapping between a camera pixel and a 3D floor position. This is because camera calibration tends to be worse at the edges of the image, and small errors at the edges can translate into larger errors on the 3D floor position.

<Here we have to describe the mechanics of the algorithm that turns camera images into tracks... I can create a flow chart, and describe each of the component processes: how we generate tracklets, how we convert tracklets to tracks, how we decide on the total number of tracks, how we deal with conflicts. Below describes going from tracklets to tracks.>

A global optimization process stitches individual tracklets into non-overlapping tracks that are consistent across all camera views. That is, tracks in specific locations are expected to appear in specific camera views, and to not appear in other camera views, and the specific set of generated tracks is the result of a global optimization process that considers image data in all sensors in all cameras at once.

Tracklets from time adjacent windows are stitched together using the Munkres Algorithm. The cost function term considers spatial proximity, the position-error-term in spatial proximity, velocity and acceleration information deduced from the tracklet shape, and a variety of 2D image features related to the set of camera image pixels deduced from the reprojection of the 3D location of the track. These 2D image features are: <we have to be specific I believe>, color histogram, height of the person -- a parameter that’s estimated -- pose [standing|sitting|lying], image feature points, the position and angle of the persons hips, shoulders and head, and the positions of their limbs in the 2D projective space of the relevant camera images. A Bayesian approach is used to place a prior on height and pose, and the likelihood of feature-point, color histogram, feature points, and body position consilience across the entire track.

Once an optimal set of tracks are generated for the time adjacent window, the track is spatially smoothed for the middle of window. That is, from the middle of the first half of the time adjacent window, to the middle of the second half. This allows us to stitch tracklets in parallel by considering the even windows and their successor (in parallel), and then the odd windows and their successor (in parallel). The resulting smoothed tracks then account for the position error terms in all relevant camera views.

# Bayesian Design #

We need a `Tracklet/Tracker` design for efficiency reasons, but note that te tracker is working on _pairs_ of Tracklet windows.

The tracker has some *failure modes* to be aware of:

 * Failures at steep angles.
 * When people walk in front of a cloths rack, the cloths become part of the skeleton.
 * Mannequins and other static objects.
 * Skeletons may become bizarre.

## Improvements to Pose Skeleton Detection ##

 * Implement new pose detections: CPU version, non-openpose, may require interprocess communication.
 * We capture raw picks along thick lines of the skeleton.
 * Need a skeleton-skeleton metric.
 * Need a soft "skeleton may be there" metric for when a skeleton disappears between frames.
 * Sometimes the skeleton "jumps" in shape... we should be robust to this.

## Bayesian Tracklets ##

 * (1) Each 2D track gets a 3D position from averaging across the point cloud.
 * (2) Do _soft_ 2D tracking, frame [t..t+1], in each projective image, represented by a 2D matrix. Use `NCC + k_1 * 3D-distance`. *1 parameter:* `k_1`. 
 * (3) Each track gets a "motion" score, based on its 2D movement.
 * (4) We look across different images and create a track-track match score, using _reprojection error_, _point-cloud distance_, and _implausible detections_. So, for example, are we placing the track outside the store... are they becoming too high.
 * (5) We do smart interpolation, by interpolating skeletons across 2D images. When interpolating, consider the fact that in frame t->t+1, openpose may fail in wonderful ways. So we should be able to "look ahead" and interpolate, as an alternative to choosing a path through detections.
 * (6) The above produces a "conceptual" _probabilistic occupancy map_ where each histogram cell of the "tracklet movie" relates to 0 or more 2D detections, each with a probability, and to a range of possible "next frame" cells, each with a probability.

AdaBoost (?)

Now create tracklets, preserving all the above information.

 * Global information, like colour profiles for each part of the skeleton, front and back.
 * For simplicity with the next step, we want to be able to see and consume/extend existing tracks.

## Bayesian Tracks ##

This is just a version of the above, that works given knowledge of existent tracks. We start at a certain offset into a pair (or 3 or more) tracklets, and run our multi-person tracker, consuming and extending tracks as we go.

# Tracker 3.0 #

 * Tracklets gives a set of Skeleton2Ds for each sensor-frame, and various affinity
   scores.
   - A tracklet is N (8) frames.
   - We operate on single tracklets and pairs of tracklets.
   - An affinity for each Skeleton2D is calculated with respect to every other Skeleton2D. 
     There is room for heuristic optimization here.
   - For a given sensor, across time, the affinity gives 2D projective and 
     colour match. Allows for possible interpolation across frames for 
     given sensor. This interpolation is cognizant of camera position,
     and so the score includes height plausibility, and 2D distance across 
     the floor.
   - We have a function that gives the score for putting a given Skeleton2D in 
     a given histogram cell (centred there). Maybe memoized? Maybe not.
     Consider: reprojection error, height feasibility.
 * Tracks
   - The tracks is a stretch of time. At each _end_ of that time period, we
     store sentinel tracklet objects. (So at most two.)
   - We have a procedure that clips the tracks region to [start..end] frames,
     for reporting to the GUI, or anything else.
   - Tracks are calculated on a (possibly empty) Track object and Tracklet object.
   - Tracks must be calculable forwards and backwards in time. That is,
     the Tracklet object can be for the time period directly after the Track,
     or directly before it.
   - We look form time-based correspondence from Skeleton2Ds in the sentinel tracklet
     object with the added tracklet
   - A track is a sequence of Pose3Ds.
   - We use the localization histogram as an additional line of evidence on 
     a Pose3D score.
   - A given Skeleton2D can be associated with zero or one Pose3Ds.
   - We greedily associate Skeleton2Ds with Pose3Ds. 
   - We have perturbation procedures that:
     ~ Remove a track.
     ~ Distribute Skeleton2Ds to existing tracks.
     ~ Remove Skeleton2Ds from existing tracks.
     
 


