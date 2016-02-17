#@echo off

set JAVA_HOME=c:\Progra~1\Java\jdk1.7.0_40

rm generator/test.exe
clang++ -O0 -D DEBUG -D GENERATE -Wno-deprecated-writable-strings -Wc++11-extensions jni/test.cpp --output generator/test
generator/test

rm generator/Mk3Teleop.exe
clang++ -O0 -D DEBUG -D GENERATE -Wno-deprecated-writable-strings -Wc++11-extensions jni/Mk3Teleop.cpp --output generator/Mk3Teleop
generator/Mk3Teleop

rm generator/Mk3Auto.exe
clang++ -O0 -D DEBUG -D GENERATE -Wno-deprecated-writable-strings -Wc++11-extensions jni/Mk3Auto.cpp --output generator/Mk3Auto
generator/Mk3Auto

# ndk-build clean NDK_LIBS_OUT=./jniLibs
ndk-build NDK_LIBS_OUT=./jniLibs -B V=0

pushd ../../

./gradlew assembleDebug

adb -d install -r ./build/outputs/apk/FtcRobotController-debug.apk
# adb -s 10.0.0.17:5555 install -r ./build/outputs/apk/FtcRobotController-debug.apk

popd
