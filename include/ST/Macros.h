/*
    Macros.h

    Copyright © 2017 Occipital, Inc. All rights reserved.
    This file is part of the Bridge Engine SDK.
    Unauthorized copying of this file, via any medium is strictly prohibited.
    Proprietary and confidential.

    http://structure.io
*/

/** @brief Macros.h holds different preprocessor macros for use in the system.
    @brief Please to not modify any of the defines below.
*/

#pragma once

#    define ST_API         __attribute__((visibility("default")))
#    define ST_PRIVATE_API __attribute__((visibility("hidden")))
#    define ST_CDECL

#if __cplusplus
#   define ST_PLAIN_C_BEGIN extern "C" {
#   define ST_PLAIN_C_END }
#else
#   define ST_PLAIN_C_BEGIN
#   define ST_PLAIN_C_END
#endif

#    define ST_ALIGNED(Alignment) __attribute__((aligned(Alignment)))

#define ST_NON_COPYABLE(Class)                  \
    Class (const Class&) = delete;              \
    Class& operator = (const Class&) & = delete

#define ST_DEFAULT_CONSTRUCTIBLE(Class) \
    Class () = default

#define ST_DEFAULT_COPYABLE(Class)               \
    Class (const Class&) = default;              \
    Class& operator = (const Class&) & = default

#define ST_DECLARE_THAT(Class)                          \
public:                                                 \
    struct That;                                        \
    ST_PRIVATE_API Class (That* that_) : that(that_) {} \
    That* that

#define ST_DECLARE_OPAQUE_INTERNALS(Class) \
    ST_NON_COPYABLE(Class);                \
    ST_DECLARE_THAT(Class)

#define ST_DECLARE_CLONABLE_OPAQUE_INTERNALS(Class) \
    Class (const Class& copy);                      \
    Class& operator= (const Class& rhs) &;          \
    ST_DECLARE_THAT(Class)

#if _MSC_VER && !defined(__clang__)
#   define ST_WEAK
#else
#   define ST_WEAK __attribute__((weak))
#endif
