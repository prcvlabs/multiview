
Performance Improvements                {#performance_improvements}
========================

The following projects target performance of the multiview pipeline. The target is $30 per camera per month. The current price is about $200 per camera per month. Our goal is to reach $30 per-camera-per-month by October, representing a 7x increase in speed. Implementing this document should get us there.

# Reporting #

To better track prices, we need software that:

 * Logs every time an epoch is run, storing: scene, runtime, instance configuration, wall-clock time, dollar cost.
 * We can calculate an approximate per-camera-month cost for an instance type.
 * We need live access to costs.

# Speed/Accuracy Trade-off #

We will encounter some areas where a speed-accuracy trade off can be made. Furthermore, the tuning of this trade-off will change over time. We already have an inchoate system for running test cases, and reporting F1 scores, and precision/recall metrics. This system needs:

 * More annotation data: we'll be employing a person for such.
 * Some more infrastructure work, so that we can run a single script that aggregates metrics over a (configurable) wide variety of test cases.

# Read a book on CUDA #

I think I will need to build a much deeper understanding of CUDA code and CUDA internals in order to obtain the required speed increases.

# Performance Improvement Ideas #

We actually don't have much idea about multiview's CPU/GPU utilization. This is definitely the first step. Measure the current cost, and measure utilization.

## Measuring ##

It's important to measure before optimizing. We need to familiarize ourselves with the relevant nvidia tools. We should also crack open clang's profiling tools and identify heavily used functions, and see if they can be optimized as well.

## Cut Unused Code and Results ##

The pipeline is made out of many tasks, which are invoked on demand. This allows us to create and test new functionality without gratuitously executing "the kitchen sink". Some of these tasks are not fine-grained enough, which means that some results are generated, but not used. We should identify all such results, and either split them off into separate tasks, or simply delete them.

## Incremental Improvements ##

An incremental performance improvement of 1% is almost always worth it. Implementing a series of 60 such improvements will double the performance of the pipeline.

To date, no attempt has been made to optimize multiview, so it's reasonable to expect that we could make such a series of improvements. In particular, multiview gratuitously copies memory around, and includes many redundant processes.

## CUDA pipeline ##

Multiview runs in a roughly two-phase process. There are "per-frame" results, and the "tracker". I'd like to see almost all "per-frame" results calculated directly with CUDA kernels, leaving the CPU free to run the tracker. _The CUDA code should be tuned to max out the CUDA cards_.

The tracker is currently extremely fast, but this will not always be the case. In particular, a more complicated tracker will be able to better handle errors in "per-frame", allowing for much performance/accuracy tuning.

Caveat: It's possible that the tracker will eventually utilize CUDA as well.

Still, I believe the goal should be for a mostly CUDA "per-frame" pipeline, with CPU versions available for each stage. We thus have maximum flexibility in designing our pipeline.

## CUDA Video Decoding and Memory Management ##

Nvidia ships a native CUDA h264 decoders. We should use it. And also keep decompressed video on the CUDA card throughout computation.

## 3D Point Cloud ##

We currently use StereoBM on the CPU, which is okay. We have 1 reasonable GPU implementation already implemented (but unused). There's bound to be several more easily implementable GPU point cloud routines that will have vastly better properties to StereoBM.

## Dropping the Frame Rate ##

We have some data already that indicates that we can reduce the frame-rate to 10fps, or possibly 7fps. I'd like to wait for Tracker 3.0 (2 more weeks) before implementing this. Dropping the frame rate will 

## TensorRT ##

CUDA ships with a native neural network library, called TensorRT, and it is marginally faster than anything else out there. When we settle on layered neural network, we should implement the specified technology in TensorRT. Tuning the network, including the input/output layers, will yield about a 10% performance boost for even very well designed networks.

## Pose Detection ##

Openpose is currently a bottleneck. There's now about a dozen different 2D-pose (i.e, openpose) detection algorithms out there, including one specifically designed to work on CPUs. We need to implement the CPU version, so that we can have a complete CPU-only pipeline. We should also implement openpose alternatives. 

It's probably going to be insane trying to get some of these technologies into multiview natively. As such, for road testing, we should develop a protocol to run pose detection in a separate (child) process, communicating with fifos and shared memory. C++'s boost library provides everything you need.

We should also be familiar with CUDA profiling tools, so that we can access the potential to improve (reimplement) research code.

The tracker is dependent on pose detection with few false positives. We can probably tune it to deal with a high false negative rate. This opens up a window of opportunity for performance/accuracy trade-offs.

## Change Detection ##

This is going to be the most invasive change to the "per-frame" pipeline, but should result in big gains. We should use frame-by-frame change detection, and only run our per-frame pipeline on parts of images that have changed. We should easily double the performance (on average) using this strategy, without much accuracy trade-off.

Change detection will increase the complexity of the per-frame code.

