cmake_minimum_required(VERSION 3.3...3.12 FATAL_ERROR)
project(VTKmDemo CXX)

SET(VTKm_DIR "/home/ruchi/fp16_vtkm_new/vtk-m")
find_package(VTKm REQUIRED)

message("${VTKm_FOUND}")
message("${VTKm_VERSION}")
message("VTKm CUDA ENABLE: ${VTKm_ENABLE_CUDA}")

if(VTKm_ENABLE_CUDA)
  cmake_minimum_required(VERSION 3.13...3.15 FATAL_ERROR)
  enable_language(CUDA)  
endif()

set(CMAKE_CUDA_FLAGS "-gencode arch=compute_70,code=sm_70")
#set(CMAKE_CXX_FLAGS "$-I/usr/local/cuda-10.1/include")
SET(CMAKE_CXX_FLAGS_DEBUG "-D_DEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/local/cuda-10.1/include")
set(CMAKE_PREFIX_PATH "/home/ruchi/fp16_vtkm_new/build")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -ftemplate-backtrace-limit=50")

add_executable(ContourFP16 ContourFP16.cu)
set_target_properties(ContourFP16 PROPERTIES INTERFACE_COMPILE_OPTIONS $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
target_link_libraries(ContourFP16 vtkm_cont vtkm_filter vtkm_worklet vtkm_rendering)
set_target_properties(ContourFP16 PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

