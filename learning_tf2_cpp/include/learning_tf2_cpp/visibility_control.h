#ifndef LEARNING_TF2_CPP__VISIBILITY_CONTROL_H_
#define LEARNING_TF2_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LEARNING_TF2_CPP_EXPORT __attribute__ ((dllexport))
    #define LEARNING_TF2_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define LEARNING_TF2_CPP_EXPORT __declspec(dllexport)
    #define LEARNING_TF2_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef LEARNING_TF2_CPP_BUILDING_DLL
    #define LEARNING_TF2_CPP_PUBLIC LEARNING_TF2_CPP_EXPORT
  #else
    #define LEARNING_TF2_CPP_PUBLIC LEARNING_TF2_CPP_IMPORT
  #endif
  #define LEARNING_TF2_CPP_PUBLIC_TYPE LEARNING_TF2_CPP_PUBLIC
  #define LEARNING_TF2_CPP_LOCAL
#else
  #define LEARNING_TF2_CPP_EXPORT __attribute__ ((visibility("default")))
  #define LEARNING_TF2_CPP_IMPORT
  #if __GNUC__ >= 4
    #define LEARNING_TF2_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define LEARNING_TF2_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LEARNING_TF2_CPP_PUBLIC
    #define LEARNING_TF2_CPP_LOCAL
  #endif
  #define LEARNING_TF2_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // LEARNING_TF2_CPP__VISIBILITY_CONTROL_H_
