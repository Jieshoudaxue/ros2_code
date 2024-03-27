#ifndef COMPONENT_DEMO__VISIBILITY_CONTROL_H_
#define COMPONENT_DEMO__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COMPONENT_DEMO_EXPORT __attribute__ ((dllexport))
    #define COMPONENT_DEMO_IMPORT __attribute__ ((dllimport))
  #else
    #define COMPONENT_DEMO_EXPORT __declspec(dllexport)
    #define COMPONENT_DEMO_IMPORT __declspec(dllimport)
  #endif
  #ifdef COMPONENT_DEMO_BUILDING_DLL
    #define COMPONENT_DEMO_PUBLIC COMPONENT_DEMO_EXPORT
  #else
    #define COMPONENT_DEMO_PUBLIC COMPONENT_DEMO_IMPORT
  #endif
  #define COMPONENT_DEMO_PUBLIC_TYPE COMPONENT_DEMO_PUBLIC
  #define COMPONENT_DEMO_LOCAL
#else
  #define COMPONENT_DEMO_EXPORT __attribute__ ((visibility("default")))
  #define COMPONENT_DEMO_IMPORT
  #if __GNUC__ >= 4
    #define COMPONENT_DEMO_PUBLIC __attribute__ ((visibility("default")))
    #define COMPONENT_DEMO_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COMPONENT_DEMO_PUBLIC
    #define COMPONENT_DEMO_LOCAL
  #endif
  #define COMPONENT_DEMO_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // COMPONENT_DEMO__VISIBILITY_CONTROL_H_
