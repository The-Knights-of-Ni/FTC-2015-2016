@echo off

set JAVA_HOME=c:\Progra~1\Java\jdk1.7.0_40

pushd opmodes
for %%f in (*.javac) do (
    echo %%~nf
    clang -E -P -x c %%~nf.javac -o processed/%%~nf.java
)
popd

pushd ..\..\..\..\..\..\

call gradlew.bat assembleDebug

REM adb -s 5555 install -r ./build/outputs/apk/FtcRobotController-debug.apk

popd
