#ifndef ASSEMBLER_NODE_CPP__VISIBILITY_CONTROL_H_
#define ASSEMBLER_NODE_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ASSEMBLER_NODE_CPP_EXPORT __attribute__ ((dllexport))
    #define ASSEMBLER_NODE_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define ASSEMBLER_NODE_CPP_EXPORT __declspec(dllexport)
    #define ASSEMBLER_NODE_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ASSEMBLER_NODE_CPP_BUILDING_DLL
    #define ASSEMBLER_NODE_CPP_PUBLIC ASSEMBLER_NODE_CPP_EXPORT
  #else
    #define ASSEMBLER_NODE_CPP_PUBLIC ASSEMBLER_NODE_CPP_IMPORT
  #endif
  #define ASSEMBLER_NODE_CPP_PUBLIC_TYPE ASSEMBLER_NODE_CPP_PUBLIC
  #define ASSEMBLER_NODE_CPP_LOCAL
#else
  #define ASSEMBLER_NODE_CPP_EXPORT __attribute__ ((visibility("default")))
  #define ASSEMBLER_NODE_CPP_IMPORT
  #if __GNUC__ >= 4
    #define ASSEMBLER_NODE_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define ASSEMBLER_NODE_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ASSEMBLER_NODE_CPP_PUBLIC
    #define ASSEMBLER_NODE_CPP_LOCAL
  #endif
  #define ASSEMBLER_NODE_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ASSEMBLER_NODE_CPP__VISIBILITY_CONTROL_H_
