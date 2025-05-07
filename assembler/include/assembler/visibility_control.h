#ifndef PROCESS_MODEL_CPP__VISIBILITY_CONTROL_H_
#define PROCESS_MODEL_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PROCESS_MODEL_CPP_EXPORT __attribute__ ((dllexport))
    #define PROCESS_MODEL_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define PROCESS_MODEL_CPP_EXPORT __declspec(dllexport)
    #define PROCESS_MODEL_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef PROCESS_MODEL_CPP_BUILDING_DLL
    #define PROCESS_MODEL_CPP_PUBLIC PROCESS_MODEL_CPP_EXPORT
  #else
    #define PROCESS_MODEL_CPP_PUBLIC PROCESS_MODEL_CPP_IMPORT
  #endif
  #define PROCESS_MODEL_CPP_PUBLIC_TYPE PROCESS_MODEL_CPP_PUBLIC
  #define PROCESS_MODEL_CPP_LOCAL
#else
  #define PROCESS_MODEL_CPP_EXPORT __attribute__ ((visibility("default")))
  #define PROCESS_MODEL_CPP_IMPORT
  #if __GNUC__ >= 4
    #define PROCESS_MODEL_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define PROCESS_MODEL_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PROCESS_MODEL_CPP_PUBLIC
    #define PROCESS_MODEL_CPP_LOCAL
  #endif
  #define PROCESS_MODEL_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // PROCESS_MODEL_CPP__VISIBILITY_CONTROL_H_
