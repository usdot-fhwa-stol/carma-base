#!/bin/bash

#  Copyright (C) 2018-2024 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

USERNAME=usdotfhwastol
BRANCH=$(git rev-parse --abbrev-ref HEAD)
BUILD_FOCAL=false
BUILD_JAMMY=false

cd "$(dirname "$0")"
IMAGE=$(basename `git rev-parse --show-toplevel`)

echo ""
echo "##### $IMAGE Docker Image Build Script #####"
echo ""

while [[ $# -gt 0 ]]; do
    arg="$1"
    case $arg in
        -v|--version)
            COMPONENT_VERSION_STRING="$2"
            shift
            shift
            ;;
        --system-release)
            USERNAME=usdotfhwastol
            COMPONENT_VERSION_STRING=$("./get-system-version.sh")
            shift
            ;;
        -p|--push)
            PUSH=true
            shift
            ;;
        -d|--develop)
            USERNAME=usdotfhwastoldev
            COMPONENT_VERSION_STRING=develop
            shift
            ;;
        -c|--candidate)
            if ! echo "$BRANCH" | grep -q "release/.*"; then
                        echo "Please switch to a release branch before using the -c option. Exiting script now."
                        exit 1
            else
                USERNAME=usdotfhwastolcandidate
                COMPONENT_VERSION_STRING=$(echo $BRANCH | cut -d "/" -f 2)
            fi
            shift
            ;;
        --focal)
            BUILD_FOCAL=true
            shift
            ;;
        --jammy)
            BUILD_JAMMY=true
            shift
            ;;
    esac
done

if [[ -z "$COMPONENT_VERSION_STRING" ]]; then
    COMPONENT_VERSION_STRING=$("./get-component-version.sh")
fi

build_image() {
    local dockerfile_path=$1
    local tag_suffix=$2

    echo "Building docker image for $IMAGE version: $COMPONENT_VERSION_STRING using Dockerfile: $dockerfile_path"
    echo "Final image name: $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING$tag_suffix"

    DOCKER_BUILDKIT=1 docker build --network=host -t $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING$tag_suffix \
        --build-arg VERSION="$COMPONENT_VERSION_STRING" \
        --build-arg VCS_REF=`git rev-parse --short HEAD` \
        --build-arg BUILD_DATE=`date -u +"%Y-%m-%dT%H:%M:%SZ"` \
        -f $dockerfile_path .

    TAGS+=("$USERNAME/$IMAGE:$COMPONENT_VERSION_STRING$tag_suffix")

    docker tag $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING$tag_suffix $USERNAME/$IMAGE:latest$tag_suffix
    TAGS+=("$USERNAME/$IMAGE:latest$tag_suffix")

    echo "Tagged $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING$tag_suffix as $USERNAME/$IMAGE:latest$tag_suffix"
}

TAGS=()

cd ..

# If neither --focal nor --jammy is specified, build only focal for now.
# Once all humble related changes are merged into develop, jammy should be enabled.
# https://usdot-carma.atlassian.net/browse/ARC-227
if [ "$BUILD_FOCAL" = false ] && [ "$BUILD_JAMMY" = false ]; then
    BUILD_FOCAL=true
    BUILD_JAMMY=false
fi

# TODO, distinguish with suffix when Humble is fully integrated
# until then focal will have no suffix and be the main image
# https://usdot-carma.atlassian.net/browse/ARC-227
if [ "$BUILD_FOCAL" = true ]; then
    echo "Building carma-base focal image"
    build_image "focal/Dockerfile" "" #replace with "-focal"
fi

if [ "$BUILD_JAMMY" = true ]; then
    echo "Building carma-base jammy image"
    build_image "jammy/Dockerfile" "-jammy"
fi

if [ "$PUSH" = true ]; then
    for tag in "${TAGS[@]}"; do
        docker push "${tag}"
    done
fi

echo ""
echo "##### $IMAGE Docker Image Build Done! #####"
