mkdir "../build"
pushd "../build"

rm "analyze"
clang -g -o "analyze" -D DEBUG ../code/"main.cpp"

./"analyze"
popd