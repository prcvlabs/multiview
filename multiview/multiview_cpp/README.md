
# Python/C++ Multiview Geometry Library #

# Setup #

Refer to '../../README.md'

# Building #

Building works through the 'run.sh' script, in three stages
 + arguments passed to 'run.sh' select the compiler, target, and build type
   Look at the top 30-40 lines of this script to see what arguments are
   supported.
 + run.sh sets some environment variables and then calls 'mobius' with
   the file './project-config/build.mobius'. A mobius file is a normal ninja
   build file with a few extra features:
   - Allows substitution of environment variables, using ${HOME} like syntax.
   - Has +src commands that generate ninja build rules.
 + The output of mobius is then piped into a custom build of ninja, which
   does the actual build. (Vanilla ninja doesn't accept stdin as an input,
   so, when building with such, just save the output for mobius to
   build.ninja, and you're away.)

# Sanitizers #

The project can be configured and built using different sanitizers:
The 'asan', 'usan', and 'tsan' sanitizers are compiled into the executable
(or shared library), and find errors at runtime. They are useful for
finding, respectively, memory errors, undefined behaviour, and thread errors.

# Benchmarks #

The 'bench' option will compile the code in a way suitable for performing
micro-benchmarks. See 'profile-and-test/graph-cut/benchmark.cxx' for an
example.
   


