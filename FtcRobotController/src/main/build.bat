@echo off

set JAVA_HOME=c:\Progra~1\Java\jdk1.8.0_66

REM pushd ndk
REM for %%f in (*.javac) do (
REM     echo %%~nf
REM     clang -E -P -x c %%~nf.javac -o processed/%%~nf.java
REM )
REM popd

clang++ -O0 -D DEBUG -Wc++11-extensions generator/robot_state_element_generator.cpp --output generator/robot_state_element_generator

.\generator\robot_state_element_generator jni/test.cpp
.\generator\robot_state_element_generator jni/camera_test.cpp
.\generator\robot_state_element_generator jni/Mk3Teleop.cpp

REM call ndk-build clean NDK_LIBS_OUT=./jniLibs
call ndk-build NDK_LIBS_OUT=./jniLibs

pushd ..\..\

REM call gradlew.bat assemble
call gradlew.bat assembleDebug

call adb -d install -r ./build/outputs/apk/FtcRobotController-debug.apk
REM adb -s 10.0.0.4:5555 install -r ./build/outputs/apk/FtcRobotController-debug.apk

popd
