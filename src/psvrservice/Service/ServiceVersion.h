#ifndef SERVICE_VERSION_H
#define SERVICE_VERSION_H

/// Conventional string-ification macro.
// From: http://stackoverflow.com/questions/5256313/c-c-macro-string-concatenation
#if !defined(PSVR_STRINGIZE)
    #define PSVR_STRINGIZEIMPL(x) #x
    #define PSVR_STRINGIZE(x)     PSVR_STRINGIZEIMPL(x)
#endif

// Current version of this SERVICE
#define PSVR_SERVICE_VERSION_PRODUCT 0
#define PSVR_SERVICE_VERSION_MAJOR   1
#define PSVR_SERVICE_VERSION_MINOR   0
#define PSVR_SERVICE_VERSION_HOTFIX  0

/// "Product.Major.Minor.Hotfix"
#if !defined(PSVR_SERVICE_VERSION_STRING)
    #define PSVR_SERVICE_VERSION_STRING PSVR_STRINGIZE(PSVR_SERVICE_VERSION_PRODUCT.PSVR_SERVICE_VERSION_MAJOR.PSVR_SERVICE_VERSION_MINOR.PSVR_SERVICE_VERSION_HOTFIX)
#endif

#endif // SERVICE_VERSION_H
