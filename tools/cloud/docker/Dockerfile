# The mvcloud GPU worker: multiview-cli (HyperPose + CUDA) and the mvcloud
# agent. Two stages: `builder` creates the micromamba env of
# build/environment.yml with TensorRT and builds multiview-cli; the runtime
# image keeps the env's shared libraries, the binary, data/ and the agent.
#
# Build with tools/cloud/docker/build-image.sh (the context is `git archive`
# of a commit, so no build directories or caches end up in the image).
# CMake fetches HyperPose and stdtensor at configure time, so the builder
# needs network access. Run (needs nvidia-container-toolkit for the GPU):
#
#   docker run --gpus all --memory 24g -v mvcloud-cache:/cache \
#       -v /data/root:/data/root mvcloud:<tag> --control fs:///data/root --drain

ARG UBUNTU=ubuntu:24.04

# ------------------------------------------------------------------- builder
FROM ${UBUNTU} AS builder

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        ca-certificates curl bzip2 git patch unzip \
 && rm -rf /var/lib/apt/lists/*

ENV MAMBA_ROOT_PREFIX=/opt/micromamba
WORKDIR /opt/multiview/src

# The env first, so that a source change does not rebuild it. TRT_SMS are the
# compute capabilities whose TensorRT builder resources are kept (the build
# has no GPU to ask).
ARG TRT_SMS="86 89"
COPY build/environment.yml build/setup-env.sh build/
RUN MULTIVIEW_TRT_SMS="${TRT_SMS}" build/setup-env.sh --with-tensorrt \
 && /root/.local/bin/micromamba clean -a -y

# The source. CUDA_ARCHS lists the fleet's GPUs ("native" is wrong without a
# GPU); JOBS caps the compile jobs, since docker does not run them niced.
COPY . .
ARG CUDA_ARCHS="86;89"
ARG JOBS=6
ARG MULTIVIEW_VERSION=unknown
RUN printf '#!/bin/sh\necho "%s"\n' "${MULTIVIEW_VERSION}" > bin/multiview-version.sh \
 && CMAKE_BUILD_PARALLEL_LEVEL="${JOBS}" \
    MULTIVIEW_BUILD_DIR=/opt/multiview/build \
    MULTIVIEW_CMAKE_ARGS="-DMULTIVIEW_WITH_HYPERPOSE=ON -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCHS}" \
    build/build.sh Release multiview-cli \
 && /opt/multiview/build/multiview-cli -h > /dev/null

# What the runtime needs of the env: shared libraries and executables
RUN cp -a "${MAMBA_ROOT_PREFIX}/envs/multiview" /opt/runtime-env \
 && cd /opt/runtime-env \
 && rm -rf include opt/tensorrt/include conda-meta pkgs share/doc share/man \
           nvvm targets/*/include lib/cmake lib/pkgconfig \
 && find . -name '*.a' -delete

# ------------------------------------------------------------------- runtime
FROM ${UBUNTU}

RUN apt-get update \
 && apt-get install -y --no-install-recommends ca-certificates curl python3 \
 && rm -rf /var/lib/apt/lists/*

# Same paths as in the builder: the binary's RPATH and the compiled-in data
# directory point there
COPY --from=builder /opt/runtime-env /opt/micromamba/envs/multiview
COPY --from=builder /opt/multiview/build/multiview-cli /opt/multiview/bin/multiview-cli
COPY --from=builder /opt/multiview/src/data /opt/multiview/src/data
COPY --from=builder /opt/multiview/src/tools/cloud/agent /opt/multiview/agent
COPY --from=builder /opt/multiview/src/build/download-hyperpose-models.sh /opt/multiview/bin/
COPY tools/cloud/docker/entrypoint.sh /opt/multiview/entrypoint.sh

ENV PATH=/opt/multiview/bin:/opt/micromamba/envs/multiview/bin:${PATH} \
    PYTHONPATH=/opt/multiview/agent \
    MULTIVIEW_ASSET_DIR=/opt/multiview/src/data \
    MULTIVIEW_HYPERPOSE_DIR=/cache/hyperpose \
    MULTIVIEW_CACHE_DIR=/cache/multiview-cache \
    MULTIVIEW_LAZY_S3_DIR=/cache/lazy-s3

VOLUME ["/cache", "/work"]
ENTRYPOINT ["/opt/multiview/entrypoint.sh"]
