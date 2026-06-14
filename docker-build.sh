#!/bin/bash
#
# docker-build.sh - Build d2x-cios using Docker with devkitARM r32
#
# Usage:
#   ./docker-build.sh [major_version] [minor_version] [dist]
#   ./docker-build.sh clean
#
# Examples:
#   ./docker-build.sh                  # Build with defaults (999 unknown)
#   ./docker-build.sh 11 beta1         # Build v11-beta1
#   ./docker-build.sh clean            # Clean build artifacts
#
# Environment Variables:
#   DEVKITARM_R32_DISTRO  - Docker base distro (default: ubuntufocal)
#                           Options: debianbullseye, ubuntufocal, ubuntujammy, etc.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Check Docker ---
if ! command -v docker &> /dev/null; then
    echo "ERROR: Docker is not installed or not in PATH"
    echo "Install Docker Desktop: https://www.docker.com/products/docker-desktop"
    exit 1
fi

# Check Docker daemon is running
if ! docker info &> /dev/null 2>&1; then
    echo "ERROR: Docker daemon is not running"
    echo "Please start Docker Desktop and try again"
    exit 1
fi

# --- Ensure submodule is initialized ---
if [ ! -d "$SCRIPT_DIR/docker/devkitarm-r32/Dockerfiles" ]; then
    echo "Initializing devkitARM r32 submodule..."
    git -C "$SCRIPT_DIR" submodule update --init --recursive
fi

# --- Build Docker image if needed ---
if ! docker image inspect devkitarm-r32 &> /dev/null; then
    DISTRO="${DEVKITARM_R32_DISTRO:-ubuntufocal}"
    DOCKERFILE_DIR="$SCRIPT_DIR/docker/devkitarm-r32/Dockerfiles/$DISTRO"

    if [ ! -d "$DOCKERFILE_DIR" ]; then
        echo "ERROR: Distribution '$DISTRO' not found"
        echo "Available distributions:"
        ls "$SCRIPT_DIR/docker/devkitarm-r32/Dockerfiles/"
        exit 1
    fi

    echo "============================================="
    echo "Building devkitARM r32 Docker image"
    echo "Distribution: $DISTRO"
    echo "This is a one-time process (~15-30 minutes)"
    echo "============================================="
    echo ""

    docker build -t devkitarm-r32 "$DOCKERFILE_DIR"

    echo ""
    echo "âœ“ Docker image built and cached successfully"
    echo ""
fi

# --- Run the build ---
echo "============================================="
echo "Building d2x-cios via Docker"
echo "Arguments: ${@:-<defaults>}"
echo "============================================="
echo ""

docker run --rm \
    --entrypoint bash \
    -v "$SCRIPT_DIR:/build" \
    devkitarm-r32 \
    -c "
        cd /build && \
        g++ -O2 -o stripios stripios_src/main.cpp && \
        export PATH=/opt/devkitARM/bin:\$PATH && \
        export DEVKITARM=/opt/devkitARM && \
        ./maked2x.sh $*
    "
