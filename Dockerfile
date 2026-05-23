# syntax=docker/dockerfile:1.6
# =============================================================================
#  Controller Toolbox — multi-stage Docker build
#
#  Stage 1 (builder):   Debian + CMake + g++ + Eigen, compiles every target.
#  Stage 2 (runtime):   Slim image with only the built binaries + libstdc++.
#
#  Build:   docker build -t controller-toolbox .
#  Run:     docker run --rm controller-toolbox                    # runs test suite
#           docker run --rm controller-toolbox ex02_ss_lqr        # runs an example
#           docker run --rm controller-toolbox boiler_turbine_case_study
# =============================================================================

# ---------- Stage 1: builder -------------------------------------------------
FROM debian:bookworm-slim AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Build toolchain + Eigen 3.4 (Debian Bookworm ships 3.4.0).
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ninja-build \
        libeigen3-dev \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src

# Copy only the manifests first so configure-stage caching survives source edits.
COPY CMakeLists.txt ./
COPY lib/CMakeLists.txt          lib/CMakeLists.txt
COPY tests/CMakeLists.txt        tests/CMakeLists.txt
COPY examples/CMakeLists.txt     examples/CMakeLists.txt
COPY scripts/CMakeLists.txt      scripts/CMakeLists.txt
COPY case-study/CMakeLists.txt   case-study/CMakeLists.txt

# Now copy the actual sources.
COPY lib/        lib/
COPY tests/      tests/
COPY examples/   examples/
COPY scripts/    scripts/
COPY case-study/ case-study/

# Configure and build everything in Release.
RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
 && cmake --build build --parallel

# Smoke-test: run CTest as part of the image build so a broken commit fails fast.
RUN ctest --test-dir build --output-on-failure


# ---------- Stage 2: runtime -------------------------------------------------
FROM debian:bookworm-slim AS runtime

# Minimal runtime: libstdc++ + libgcc (Eigen is header-only, statically linked
# into the toolbox library, so no Eigen runtime is needed).
RUN apt-get update && apt-get install -y --no-install-recommends \
        libstdc++6 \
        libgomp1 \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Non-root user for principle of least privilege.
RUN useradd --create-home --shell /bin/bash ctrl
USER ctrl
WORKDIR /home/ctrl

# Pull the compiled binaries from the builder stage. The structure mirrors
# the CMake build tree so each subdirectory's executables are co-located.
COPY --from=builder --chown=ctrl:ctrl /src/build/tests/      ./bin/tests/
COPY --from=builder --chown=ctrl:ctrl /src/build/examples/   ./bin/examples/
COPY --from=builder --chown=ctrl:ctrl /src/build/scripts/    ./bin/scripts/
COPY --from=builder --chown=ctrl:ctrl /src/build/case-study/ ./bin/case-study/

# Flatten all binaries onto PATH so users can invoke them by name.
ENV PATH="/home/ctrl/bin/tests:/home/ctrl/bin/examples:/home/ctrl/bin/scripts:/home/ctrl/bin/case-study:${PATH}"

# Default action: run the full test suite. Override with any executable name,
# e.g. `docker run --rm controller-toolbox ex02_ss_lqr`.
CMD ["test_controllers"]
