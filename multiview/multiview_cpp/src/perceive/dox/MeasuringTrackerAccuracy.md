
Measuring Tracker Accuracy           {#measuring_tracker_accuracy}
============================

# Datasets #
 
 * EPFL dataset: [cvlab.epfl.ch/data/pom/](cvlab.epfl.ch/data/pom/)
 * PETS 2009 dataset: [www.cvg.reading.ac.uk/PETS2009/a.html](www.cvg.reading.ac.uk/PETS2009/a.html)
 * WILDTRACK: [https://www.epfl.ch/labs/cvlab/data/data-wildtrack/](https://www.epfl.ch/labs/cvlab/data/data-wildtrack/)

# How to measure performance #

Tracking Performance Metrics
 * MOTP: Multiple object tracking accuracy. [1]. The ability to measure precise object positions: total error in track-point positions divided by total number of matchers. In metres. 
  
 \f[MOTP = \frac{\sum_{i,t}d_t^i}{\sum_t c_t}\f]
 
 * MOTA: Multiple object tracking precision. [1]. The first term is the ratio of misses (false negatives) computed over every track. 
   The second term is ratio of false positives. The third term is the ratio of mismatches. (i.e., ID switches, see below.) These three terms together
   gives the _total error rate_.

 \f[MOTA = 1 - E_{total} = 1 - \frac{\sum_{t} m_t}{\sum_t g_t} - \frac{\sum_{t} f\,p_t}{\sum_t g_t} - \frac{\sum_{t} m m e_t}{\sum_t g_t} \f]

 * MODP: Multiple object detection precision. [3]. The average [IoU](https://medium.com/towards-artificial-intelligence/understanding-iou-metric-in-object-detection-1e5532f06a76) (i.e., intersection area divided by union area) for every tracked point. This will be very useful to us, because
   there's some bias (of about 20cm) which consistently crops up, presumably because I've fumbled some integer math somewhere. Solving this particular
   bug will show up in better MODP results. _BUT_ also note that MODP is _much_ more challenging when calculating such in a 3D environment. This means
   that comparing our MODP results to most papers (few that there are) will be comparing apples to oranges.

 * MODA: Multiple object detection accuracy. [3]. Same as MOTP, but mismatches (ID switches) aren't counted.
 
  \f[MODA = 1 - \frac{\sum_t(m_t + f\,p_t)}{\sum_t g_t}\f]

 * FP: False positives. [1]. Number of track-points produced by the tracker, that do not correspond to ground-truth.

 * FN: False negatives. [1]. Number of track-points that the tracker fails to produce, even though there is a ground-truth track there.

 * MT%: Mostly tracked. [2]. Percentage of ground-truth trajectories that are at least 80% covered by the tracker.

 * ML%: Mostly lost. [2]. Percentage of ground-truth trajectories that are less than 20% covered by the tracker.

 * PT%: Partially tracked. [2]. 1 - MT - ML.

 * ID: Identity switches, aka, mismatches. [1, 2]. The total number of times that the trajectory switches ground-truth identity. For example, when two people cross in the video, and that causes the track to switch.

 * FM: Fragmentation. [2] Total number of times that the ground-truth trajectory is interrupted in the tracking result.
 
In addition to referenced material, I think the following is actually pretty good:

 * TP: True positives. Number of track-points that the tracker produces that correspond to ground-truth
 
 * F1: The harmonic mean of precision and recall:
 
 \f[2\,\frac{precision\,*\,recall}{precision + recall}  = \frac{tp}{tp + \frac{1}{2}(fp + fn)}\f]

Note that _nobody_ tries to do gaze direction.

# References #

[1] K. Bernardin, and R. Stiefelhagen. Evaluating multiple object tracking performance: The CLEAR MOT metrics. EURASIP Journal on Image and Video Processing. 2008.

[2] Y. Li, C. Huang, and R. Nevatia. Learning to associate: Hybridboosted multi-target tracker for crowded scene. In Proc. CVPR, 2009.

[3] R. Kasturi, D. Goldgof, P. Soundararajan, V. Manohar, J. Garofolo, R. Bowers, M. Boonstra, V. Korzhova, and J. Zhang. Framework for performance evaluation of face,
text, and vehicle detection and tracking in video: Data, metrics, and protocol. IEEE Trans. PAMI, 31(2):319–336, 2009.

 
