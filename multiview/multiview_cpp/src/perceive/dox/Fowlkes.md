
Fowlkes Tracker                {#fowlkes_tracker}
===============

## About ##

The tracker takes a stack of histograms as inputs -- on for each frame of a video -- and outputs a set of tracks. Each histogram is a 2d grid, representing the floor space of the scene (i.e., the store). The height of each cell of the histogram is related to the number 3D point-cloud points over that cell. Tracks show up as moving blobs on the histogram.

The _fowlkes tracker_ uses a [greedy algorithm](https://en.wikipedia.org/wiki/Greedy_algorithm) to find global tracks.
It is _slow_ when there's lots of video data. (i.e., the stack of histograms is high.)

## Explanation of Parameters ##

This algorithm cleans up the histogram as follows:

 * Statistics (mean, median, variance) are calculated for each cell of the histogram across the entire stack.
 * The data is each cell is then zeroed.
 * We supply Gaussian parameters (mean [actually median] and stddev) for the distribution of background cells.
 * We supply Gaussian parameters (mean [actually median] and stddev) for the distribution of foreground cells: i.e., tracks.
 * A score is generated that a given cell is a track. (The ratio of log probabilities above.)

Parameters                                 | Description
------------------------------------------ | ---------------
fw_background_median, fw_background_stddev | Gaussian parameters for background cells.
fw_track_median, fw_track_stddev           | Gassuain parameters for foreground cells (i.e., tracks).

Then note that we need parameters for a birth/death model of tracks.
    
Parameters                                 | Description
------------------------------------------ | ---------------
fw_bd_at_boundary                          | Probability that a track can appear at a boundary cell in any frame.
fw_bd_edge_dist                            | Distance (in meters) that gives the boundary region used above.
fw_bd_last_frame                           | Probability that a track can disappear in the last frame.
fw_bd_otherwise                            | Probability that a track can otherwise spontaneously appear.

The tracks are people wondering around, so we need to know their expected speed and dimensions.
We also apply a cost for two people overlapping on the same cell. (That cost should be very high.)

Parameters                                 | Description
------------------------------------------ | ---------------
fw_person_diameter                         | Diameter of a person.
fw_speed_median                            | Median speed of a fast moving person in meters per second.
fw_speed_stddev                            | Standard deviation for the above.
fw_track_track_overlap_cost                | Cost of parts of two people occupying the same cell.

Instead of creating a fully connected graph for each histogram, we only connect cells within a certain cell-to-cell (manhatten) distance. That's what `fw_track_dist_delta_frame` does. The value must be large enough to
accommodate the expected speed for the given histogram size. 

The last "used" tunable parameter is `fw_max_track_cost`. The greedy algorithm is going to generate potentially lots of tracks. So you need some stopping criteria. The algorithm stops generating tracks then when the next global track cost would exceed `fw_max_track_cost`.

Finally, there's some unused parameters.

Parameters                                 | Description
------------------------------------------ | ---------------
fw_cull_connections                        | always set to 1.
fw_plus_track_score                        | usused. (Would have been part of two-pass algorithm.)
fw_minus_track_score                       | unused. (ibid.)
fw_two_pass                                | unused.




