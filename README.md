# VTKm_FP16_TEST
This is  for testing Contour filter


Build vtk-m first
```
rm -r CMakeFiles/
rm cmake_install.cmake CMakeCache.txt Makefile
cmake -DVTKm_ENABLE_TESTING:BOOL=OFF -DVTKm_ENABLE_CUDA=ON  -DVTKm_ENABLE_LOGGING=OFF  ../vtk-m
make -j16
```


and then change CMakeFiles.txt to point to the build

```
********Test vtkm_TEST********
rm -r CMakeFiles/
rm cmake_install.cmake CMakeCache.txt Makefile 
cmake -DCMAKE_PREFIX_PATH=/home/ruchi/fp16_vtkm_new/build  .
make -j16
```
