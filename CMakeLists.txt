cmake_minimum_required(VERSION 3.3...3.12 FATAL_ERROR)
project(VTKmDemo CXX)

#SET(VTKm_ROOT "/home/ruchi/vtkm_final/vtk-m")
SET(VTKm_DIR /usr/local/lib/cmake/vtkm-1.5)
find_package(VTKm REQUIRED)

message("${VTKm_FOUND}")
message("${VTKm_VERSION}")
message("VTKm CUDA ENABLE: ${VTKm_ENABLE_CUDA}")

if(VTKm_ENABLE_CUDA)
  cmake_minimum_required(VERSION 3.13...3.15 FATAL_ERROR)
  enable_language(CUDA)  
endif()

set(CMAKE_CUDA_FLAGS "-gencode arch=compute_70,code=sm_70")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/local/cuda-10.1/include")
add_executable(ContourFP16 ContourFP16.cu)
set_target_properties(ContourFP16 PROPERTIES INTERFACE_COMPILE_OPTIONS $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
target_link_libraries(ContourFP16 vtkm_cont vtkm_filter vtkm_rendering)
set_target_properties(ContourFP16 PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

