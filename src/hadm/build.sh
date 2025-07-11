#!/bin/bash

# Default values
DEFAULT_PLATFORM="kw47_mcxw72x_nbu"
DEFAULT_BUILD_TYPE="MinSizeRel"

# Initialize variables with defaults
PLATFORM="$DEFAULT_PLATFORM"
BUILD_TYPE="$DEFAULT_BUILD_TYPE"

# Function to display usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Build script for BLE HADM HAL library"
    echo ""
    echo "Options:"
    echo "  -p, --platform PLATFORM    Target platform (default: $DEFAULT_PLATFORM)"
    echo "  -b, --build-type TYPE       Build type (default: $DEFAULT_BUILD_TYPE)"
    echo "                              Valid types: Debug, Release, MinSizeRel, RelWithDebInfo"
    echo "  -h, --help                  Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Use all defaults"
    echo "  $0 -p kw47_mcxw72x_nbu               # Specify platform only"
    echo "  $0 -b Debug                          # Specify build type only"
    echo "  $0 -p kw47_mcxw72x_nbu -b Debug     # Specify both parameters"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--platform)
            PLATFORM="$2"
            shift 2
            ;;
        -b|--build-type)
            BUILD_TYPE="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Error: Unknown option '$1'"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Validate build type
case $BUILD_TYPE in
    Debug|Release|MinSizeRel|RelWithDebInfo)
        ;;
    *)
        echo "Error: Invalid build type '$BUILD_TYPE'"
        echo "Valid build types: Debug, Release, MinSizeRel, RelWithDebInfo"
        exit 1
        ;;
esac

# Validate platform choice
case $PLATFORM in
    kw47_mcxw72x_nbu|kw43_mcxw70_nbu)
        ;;
    *)
        echo "Error: Invalid platform '$PLATFORM'"
        echo "Valid platforms: kw47_mcxw72x_nbu, kw43_mcxw70_nbu"
        exit 1
        ;;
esac

# Display build configuration
echo "Build Configuration:"
echo "  Platform:   $PLATFORM"
echo "  Build Type: $BUILD_TYPE"
echo ""

# Clean and build
echo "Cleaning previous build..."
rm -rf build

echo "Configuring build..."
if ! cmake -S./ -B./build -G Ninja -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DPLATFORM_TARGET="$PLATFORM"; then
    echo "Error: CMake configuration failed"
    exit 1
fi

echo "Building..."
if ! cmake --build ./build; then
    echo "Error: Build failed"
    exit 1
fi

echo "Build completed successfully!"
