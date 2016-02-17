@echo off

set JAVA_HOME=c:\Progra~1\Java\jdk1.8.0_66

REM TODO: use native_robot or loop over all cpp files
del generator\test
clang++ -O0 -D DEBUG -D GENERATE -Wno-c++11-compat-deprecated-writable-strings -Wc++11-extensions jni/test.cpp --output generator\test
generator\test

del generator\Mk3Teleop
clang++ -O0 -D DEBUG -D GENERATE -Wno-c++11-compat-deprecated-writable-strings -Wc++11-extensions jni/Mk3Teleop.cpp --output generator\Mk3Teleop
generator\Mk3Teleop

del generator\Mk3Auto
clang++ -O0 -D DEBUG -D GENERATE -Wno-c++11-compat-deprecated-writable-strings -Wc++11-extensions jni/Mk3Auto.cpp --output generator\Mk3Auto
generator\Mk3Auto

REM call ndk-build clean NDK_LIBS_OUT=.\jniLibs V=0
call ndk-build NDK_LIBS_OUT=.\jniLibs -B V=0

echo ndk build complete

pushd ..\..\

REM call gradlew.bat assemble
call gradlew.bat assembleDebug
popd

call adb -d install -r ..\..\build\outputs\apk\FtcRobotController-debug.apk
REM adb -s 10.0.0.4:5555 install -r ..\..\build\outputs\apk\FtcRobotController-debug.apk
