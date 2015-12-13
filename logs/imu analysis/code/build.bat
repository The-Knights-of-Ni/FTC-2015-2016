@echo off

mkdir "../build"
pushd "../build"

del "analyze.exe"
clang -O3 -g -o "analyze" -D DEBUG ../code/"main.cpp"
REM cl -MT -Zi -Od -Oi -nologo -TC /Fe"analyze" /DDEBUG ../code/"main.cpp"

echo done compiling

"analyze"
popd