#@echo off

set JAVA_HOME=c:\Progra~1\Java\jdk1.7.0_40

# pushd opmodes
# for %%f in (*.javac) do (
#     echo %%~nf
#     clang -E -P -x c %%~nf.javac -o processed/%%~nf.java
# )
# popd

clang++ -O0 -D DEBUG -Wc++11-extensions jni/generator/robot_state_element_generator.cpp --output jni/generator/robot_state_element_generator

./jni/generator/robot_state_element_generator jni/test.cpp
ndk-build

pushd ../../

./gradlew assembleDebug

adb -d install -r ./build/outputs/apk/FtcRobotController-debug.apk
# adb -s 10.0.0.20:5555 install -r ./build/outputs/apk/FtcRobotController-debug.apk

popd
