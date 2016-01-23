#@echo off

set JAVA_HOME=c:\Progra~1\Java\jdk1.7.0_40

# pushd opmodes
# for %%f in (*.javac) do (
#     echo %%~nf
#     clang -E -P -x c %%~nf.javac -o processed/%%~nf.java
# )
# popd

clang++ -O0 -D DEBUG -Wc++11-extensions generator/robot_state_element_generator.cpp --output generator/robot_state_element_generator

./generator/robot_state_element_generator jni/test.cpp
./generator/robot_state_element_generator jni/Mk3Teleop.cpp
./generator/robot_state_element_generator jni/Mk3Auto.cpp


ndk-build clean NDK_LIBS_OUT=./jniLibs
ndk-build NDK_LIBS_OUT=./jniLibs

pushd ../../

./gradlew assembleDebug

adb -d install -r ./build/outputs/apk/FtcRobotController-debug.apk
# adb -s 10.0.0.17:5555 install -r ./build/outputs/apk/FtcRobotController-debug.apk

popd
