#!/usr/bin/env bash
# docker.sh — build the ubersdr-leobodnar Docker image
#
# All binaries are built from source inside the Docker image.
# No host binaries are required.
#
# Usage:
#   ./docker.sh [build|arm64|push|run]
#
#   build    — build the image for linux/amd64 and load into local Docker (default)
#   arm64    — build the image for linux/arm64 and load into local Docker
#   push     — build linux/amd64 + linux/arm64 with buildx, push multi-arch manifest
#              to registry, then git commit + push
#   run      — run the image locally (serves on port 5123)
#
# Environment variables (build):
#   IMAGE      Docker image name/tag   (default: madpsy/ubersdr-leobodnar:latest)
#   PLATFORM   Docker --platform flag  (default: linux/amd64)
#
# Requirements for 'push':
#   docker buildx with a builder that supports linux/amd64 and linux/arm64
#   (e.g. the default 'docker-container' driver).  Run once to create one:
#     docker buildx create --name multiarch --driver docker-container --use

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE="${IMAGE:-madpsy/ubersdr-leobodnar:latest}"
PLATFORM="${PLATFORM:-linux/amd64}"
BUILDER="${BUILDER:-multiarch}"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

die() { echo "error: $*" >&2; exit 1; }

check_deps() {
    command -v docker >/dev/null || die "docker not found in PATH"
}

# Ensure a buildx builder that supports multi-arch exists and is active.
ensure_builder() {
    if ! docker buildx inspect "$BUILDER" >/dev/null 2>&1; then
        echo "Creating buildx builder '$BUILDER' (docker-container driver)..."
        docker buildx create --name "$BUILDER" --driver docker-container --bootstrap
    fi
    docker buildx use "$BUILDER"
}

# Stage source into a temp dir (excludes .git) and return the path in $TMPCTX.
stage_context() {
    TMPCTX="$(mktemp -d)"
    trap 'rm -rf "$TMPCTX"' EXIT
    echo "Staging build context in $TMPCTX..."
    rsync -a --exclude='.git' "$SCRIPT_DIR/" "$TMPCTX/"
}

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

# build [platform] — single-arch build loaded into the local Docker daemon.
build() {
    check_deps
    stage_context

    echo "Building image $IMAGE (platform=$PLATFORM) via buildx --load..."
    docker buildx build \
        --platform "$PLATFORM" \
        --tag "$IMAGE" \
        --load \
        "$TMPCTX"

    echo "Built and loaded: $IMAGE"
}

# push — multi-arch build (amd64 + arm64) pushed directly to the registry.
push() {
    check_deps
    ensure_builder
    stage_context

    local platforms="linux/amd64,linux/arm64"
    echo "Building multi-arch image $IMAGE (platforms=$platforms) and pushing..."
    docker buildx build \
        --platform "$platforms" \
        --tag "$IMAGE" \
        --push \
        "$TMPCTX"

    echo "Pushed multi-arch manifest: $IMAGE"

    echo "Committing and pushing git repository..."
    git add -A
    git diff --cached --quiet || git commit -m "Release $IMAGE"
    git push
}

run_image() {
    docker run --rm -it \
        --privileged \
        --volume /dev:/dev \
        --publish 5123:5123 \
        --platform "$PLATFORM" \
        "$IMAGE" \
        "$@"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

case "${1:-build}" in
    build) build ;;
    arm64) PLATFORM=linux/arm64 build ;;
    push)  push  ;;
    run)   shift; run_image "$@" ;;
    *)
        echo "Usage: $0 [build|arm64|push|run [args...]]" >&2
        exit 1
        ;;
esac
