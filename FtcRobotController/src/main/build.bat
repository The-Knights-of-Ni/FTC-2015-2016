@echo off

set JAVA_HOME=c:\Progra~1\Java\jdk1.7.0_40

REM pushd ndk
REM for %%f in (*.javac) do (
REM     echo %%~nf
REM     clang -E -P -x c %%~nf.javac -o processed/%%~nf.java
REM )
REM popd

clang++ -O0 -D DEBUG -Wc++11-extensions jni/generator/robot_state_element_generator.cpp --output jni/generator/robot_state_element_generator

.\jni\generator\robot_state_element_generator jni/test.cpp
call ndk-build

pushd ..\..\

REM call gradlew.bat assemble
call gradlew.bat assembleDebug

REM adb -d install -r ./build/outputs/apk/FtcRobotController-debug.apk
adb -s 10.0.0.16:5555 install -r ./build/outputs/apk/FtcRobotController-debug.apk

popd
