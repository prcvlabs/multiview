
# The new multiview install layout #

The multiview installation can be separated in five layers:

 1. The nvidia drivers and cuda installation.
 2. A set of (debian) packages, which does NOT include any c++ libraries.
 3. A custom compiler and library installation somewhere under `/opt/multiview`
 4. The multiview source code itself.
 5. A deploy system.
 
The install script deals with [1-3], and the deploy system with [4-5].

## 1. The Nvidia Drivers and Cuda Installation ##

Nvidia and ubuntu seem to have a volatile relationship that makes it difficult to manage driver and cuda installation. As of writing, we're using CUDA 11.2. Nvidia requires a match between the driver, the cuda installation, and if running within docker, then the same with the docker container. In theory this shouldn't be to hard. In practice it can be difficult to create a reliable script.

As such, the human user is responsible for installing the driver. On the google cloud, I was able to install the nvidia driver using the instructions here: `https://cloud.google.com/compute/docs/gpus/install-drivers-gpu`

The script `multiview/build/install-cuda.sh` is a guide (or may just work) for getting CUDA 11.2 installed. There is no problem having multiple cuda installations.

## 2. Packages ##

A base installation of Ubuntu (or Fedora, or whatever) doesn't have things like `python-pip3` and `boto` installed. The install script can install just the base packages... stuff that goes into `/usr/bin`, etc.

Once the base packages are installed, they will hardly ever change. At this point it is okay to create an image of an instance, and this image will be able to run multiview. The image only needs to be updated for normal security updates.

## 3. Custom Compiler and Library Installation ##

Multiview is (in theory) able to use a variety of different compilers, c++ standards, and standard libraries. All of this presents ABI issues, and so the advice is to compile all the C++ code that you link to. All compilers and dependent libraries are installed somewhere under `/opt/multiview`, with:

```
/opt/multiview              Root of the installation
/opt/multiview/etc          Where all configuration files code. For example, the 
                            cuda version, or if we're using a custom C++ library,
                            etc.
/opt/multiview/<toolchain>  Different toolchains (e.g., gcc-10, llvm-11.0.0) live
                            along side of each other. ALL libraries are compiled 
                            and installed to the toolchain root.
```

The actual compiler is installed to `/opt/multiview/<toolchain>/opt/cc`, along with a few other tools: `cmake`, `valgrind`, `ninja`, and `mobius`.

This gives us the ability to change toolchain, or any number of c++ compilation options. For example, we can enable thin link-time-optimization, and static linking, or use an up-to-date C++ library. (Necessary to use C++ modules.)

## 4. Multiview Project ##

The multiview project depends on [1-3], but primarily on 3. above. Multiview can be compiled and run in a variety of ways (asan, tsan, gdb, valgrind, benchmarks, etc.) The project also contains a variety of different testing procedures.

## 5. Deploy System ##

The `multiview/build/build.sh` script is the primary tool for deploying multiview. Currently we build to tagged docker containers: `multiview:test`, `multiview:dev`, `multiview:staging`, and these can be retagged.

Deployment involves:

 * Running all testing procedures.
 * Tagging the repo.
 * Running `multiview/build/build.sh` with the desired tag, and destination.

# Rationale #

There currently are a few pain points with deployment:

 * Building the docker container takes _forever_, because of idiosyncrasies with docker, cuda, and cmake. Every build must rebuild _all_ cuda code (openpose, opencv, etc.) because of the way that cmake reports cuda capabilities under `docker build`. Once we have a working container, we can then attach cuda cards before entering the container, and manually build everything fine. We then snapshot the container. This is how `build.sh` currently manages the weirdness.
 * The drivers much match inside and outside the container, which means that the docker container has zero portability. 

Thus, if there's _any_ issues with running code within the docker container, it literally takes hours to figure out, and then hours more to debug.

An alternative deploy system would do the following:

 * We apply [1-2] above and create a snapshot of an instance set up to run multiview.
 * We then apply 3. above, and save it as a mountable read-only volume `/opt/multiview`. The volume can have a hashcode inside of it, which indicates the version when it was built.
 * The deploy script will build multiview against the correct volume (noting the hashcode), and save the multiview binary to an s3 bucket, along with a bash script that knows how to attach the correct `/opt/multiview` volume, and download the correct multiview-cli binary.
 
The bash script then becomes the deploy object, and gets handed to Matt/Sam. 

Running a job involves:

 * Creating an instance with [1-2] done above.
 * Getting the correct bash script and running it. The script will pass through any arguments directly to `multiview-cli`.
 
An alternative is to -- within the Dockerfile -- patch the `cmake` installation, such that it reports the cuda information that we want it to report, and then continue to use Docker.

Either way, the new `install.sh` script works much more smoothly with docker's idiosyncrasies, because of the clean separation of steps [1-4] above.

