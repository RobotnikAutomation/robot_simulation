This readme contains some notes:

# O3DE

How to generate urdf:

xacro src/robotnik_description/robots/rbwatcher/rbwatcher.urdf.xacro > rbwatcher.urdf
check_urdf rbwatcher.urdf

Needs to be cleaned! TBD


## Configure env

export O3DE_HOME=${HOME}/o3de
export O3DE_EXTRAS_HOME=${HOME}/o3de-extras

export PROJECT_NAME=robotnik_roscon25
export PROJECT_PATH=${HOME}/projects/${PROJECT_NAME}


## Build environment
cd $PROJECT_PATH
cmake -B build/linux -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_STRIP_DEBUG_SYMBOLS=ON
cmake --build build/linux --config profile --target ${PROJECT_NAME} Editor ${PROJECT_NAME}.Assets

## Run Simulation Editor
cd $PROJECT_PATH

1. Run
./build/linux/bin/profile/Editor

## Run Simulation Release
cd $PROJECT_PATH

1. Build
cmake --build build/linux --config profile --target ${PROJECT_NAME} ${PROJECT_NAME}.Assets ${PROJECT_NAME}.GameLauncher

2. Run
./build/linux/bin/profile/robotnik_roscon25.GameLauncher"

