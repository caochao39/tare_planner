
#ifndef SCIP_EXPORT_H
#define SCIP_EXPORT_H

#ifdef SCIP_STATIC_DEFINE
#  define SCIP_EXPORT
#  define SCIP_NO_EXPORT
#else
#  ifndef SCIP_EXPORT
#    ifdef libscip_EXPORTS
        /* We are building this library */
#      define SCIP_EXPORT 
#    else
        /* We are using this library */
#      define SCIP_EXPORT 
#    endif
#  endif

#  ifndef SCIP_NO_EXPORT
#    define SCIP_NO_EXPORT 
#  endif
#endif

#ifndef SCIP_DEPRECATED
#  define SCIP_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef SCIP_DEPRECATED_EXPORT
#  define SCIP_DEPRECATED_EXPORT SCIP_EXPORT SCIP_DEPRECATED
#endif

#ifndef SCIP_DEPRECATED_NO_EXPORT
#  define SCIP_DEPRECATED_NO_EXPORT SCIP_NO_EXPORT SCIP_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef SCIP_NO_DEPRECATED
#    define SCIP_NO_DEPRECATED
#  endif
#endif

#endif /* SCIP_EXPORT_H */
