#@echo off

set JAVA_HOME=c:\Progra~1\Java\jdk1.7.0_40

mkdir generator

rm generator/test
clang++ -O0 -D DEBUG -D GENERATE -Wno-c++11-extensions -Wno-deprecated-writable-strings jni/test.cpp --output ./generator/test
generator/test

rm generator/Mk4Teleop
clang++ -O0 -D DEBUG -D GENERATE -Wno-c++11-extensions -Wno-deprecated-writable-strings jni/Mk4Teleop.cpp --output ./generator/Mk4Teleop
generator/Mk4Teleop

rm generator/Mk4Auto
clang++ -O0 -D DEBUG -D GENERATE -Wno-c++11-extensions -Wno-deprecated-writable-strings jni/Mk4Auto.cpp --output ./generator/Mk4Auto
generator/Mk4Auto

rm generator/Mk4AutoDeploy
clang++ -O0 -D DEBUG -D GENERATE -Wno-c++11-extensions -Wno-deprecated-writable-strings jni/Mk4AutoDeploy.cpp --output ./generator/Mk4AutoDeploy
generator/Mk4AutoDeploy

# ndk-build clean NDK_LIBS_OUT=./jniLibs
ndk-build NDK_LIBS_OUT=./jniLibs -B V=0
echo ndk build done

pushd ../../

./gradlew assembleDebug

# adb -d install -r ./build/outputs/apk/FtcRobotController-debug.apk
adb -s 10.0.0.19:5555 install -r ./build/outputs/apk/FtcRobotController-debug.apk

popd
