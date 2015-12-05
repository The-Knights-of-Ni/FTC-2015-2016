@echo off

set JAVA_HOME=c:\Progra~1\Java\jdk1.7.0_40

REM pushd ndk
REM for %%f in (*.javac) do (
REM     echo %%~nf
REM     clang -E -P -x c %%~nf.javac -o processed/%%~nf.java
REM )
REM popd

REM the invalid flag makes it compile faster, at least on my machine. no idea how, but it definitely works
clang++ -O2 --invalidflag -D DEBUG -Wc++11-extensions jni/generator/robot_state_element_generator.cpp --output jni/generator/robot_state_element_generator

.\jni\generator\robot_state_element_generator jni/test.cpp
REM call ndk-build

pushd ..\..\

REM call gradlew.bat assembleDebug

REM adb -s 5555 install -r ./build/outputs/apk/FtcRobotController-debug.apk

popd
