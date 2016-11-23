#ifndef ZERO_EXPORTS_H_
#define ZERO_EXPORTS_H_

// This header is created to include to NVCC compiled sources.
// Header 'pcl_macros' is not suitable since it inludes <Eigen/Core>,
// which can't be eaten by nvcc (it's too weak)

#if defined WIN32 || defined _WIN32 || defined WINCE || defined __MINGW32__
#ifdef PCLAPI_EXPORTS
#define ZERO_EXPORTS __declspec(dllexport)
#else
#define ZERO_EXPORTS
#endif
#else
#define ZERO_EXPORTS
#endif

#endif  //#ifndef ZERO_EXPORTS_H_