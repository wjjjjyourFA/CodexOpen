find_package(CUDA REQUIRED)

# cuda
if(CUDA_INCLUDE_DIRS)
	include_directories(${CUDA_INCLUDE_DIRS})
	list(APPEND ALL_TARGET_LIBRARIES ${CUDA_LIBRARIES} ${CUDA_cublas_LIBRARY})
else()
	set(CUDA_DIR /usr/local/cuda)
	set(CUDA_INCLUDE_DIRS ${CUDA_DIR}/include)
	include_directories(${CUDA_INCLUDE_DIRS})
	set(CUDA_LIBRARIES ${CUDA_DIR}/lib64)
	link_directories(${CUDA_LIBRARIES})
endif()

message(STATUS ${CUDA_LIBRARIES})

# cuda nvcc compiler
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

# nvcc flags
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -O3 -Xcompiler -fPIC")

enable_language(CUDA)  # 启用CUDA语言支持
