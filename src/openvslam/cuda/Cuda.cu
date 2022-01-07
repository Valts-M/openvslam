#include <helper_cuda.h>
#include <cuda/Cuda.hpp>

namespace openvslam { namespace cuda {
  void deviceSynchronize() {
    checkCudaErrors( cudaDeviceSynchronize() );
  }
} }
