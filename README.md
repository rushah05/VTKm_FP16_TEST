# VTKm_FP16_TEST
This is  for testing Contour filter

```
Build repository
rm -r CMakeFiles/
rm cmake_install.cmake CMakeCache.txt Makefile
cmake -DVTKm_ENABLE_TESTING:BOOL=OFF -DVTKm_ENABLE_CUDA=ON  -DVTKm_ENABLE_LOGGING=OFF  ../vtk-m
make -j16
```
