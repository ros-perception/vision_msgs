// Copyright 2023 Georg Novotny
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VISION_MSGS_RVIZ_PLUGINS__VISIBILITY_CONTROL_HPP_
#define VISION_MSGS_RVIZ_PLUGINS__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define DETECTION_3D_COMMON_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define DETECTION_3D_COMMON_DISPLAY_HPP_IMPORT __attribute__((dllimport))

        #define DETECTION_3D_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define DETECTION_3D_DISPLAY_HPP_IMPORT __attribute__((dllimport))

        #define DETECTION_3D_ARRAY_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define DETECTION_3D_ARRAY_DISPLAY_HPP_IMPORT __attribute__((dllimport))

        #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_IMPORT __attribute__((dllimport))

        #define BOUNDING_BOX_3D_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define BOUNDING_BOX_3D_DISPLAY_HPP_IMPORT __attribute__((dllimport))

        #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_IMPORT __attribute__((dllimport))
    #else
        #define DETECTION_3D_COMMON_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define DETECTION_3D_COMMON_DISPLAY_HPP_IMPORT __declspec(dllimport)

        #define DETECTION_3D_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define DETECTION_3D_DISPLAY_HPP_IMPORT __declspec(dllimport)

        #define DETECTION_3D_ARRAY_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define DETECTION_3D_ARRAY_DISPLAY_HPP_IMPORT __declspec(dllimport)

        #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_IMPORT __declspec(dllimport)

        #define BOUNDING_BOX_3D_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define BOUNDING_BOX_3D_DISPLAY_HPP_IMPORT __declspec(dllimport)

        #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_IMPORT __declspec(dllimport)
    #endif

    #ifdef DETECTION_3D_COMMON_DISPLAY_HPP_BUILDING_LIBRARY
        #define DETECTION_3D_COMMON_DISPLAY_HPP_PUBLIC DETECTION_3D_COMMON_DISPLAY_HPP_EXPORT
    #else
        #define DETECTION_3D_COMMON_DISPLAY_HPP_PUBLIC DETECTION_3D_COMMON_DISPLAY_HPP_IMPORT
    #endif
    #define DETECTION_3D_COMMON_DISPLAY_HPP_PUBLIC_TYPE DETECTION_3D_COMMON_DISPLAY_HPP_PUBLIC
    #define DETECTION_3D_COMMON_DISPLAY_HPP_LOCAL

    #ifdef DETECTION_3D_DISPLAY_HPP_BUILDING_LIBRARY
        #define DETECTION_3D_DISPLAY_HPP_PUBLIC DETECTION_3D_DISPLAY_HPP_EXPORT
    #else
        #define DETECTION_3D_DISPLAY_HPP_PUBLIC DETECTION_3D_DISPLAY_HPP_IMPORT
    #endif
    #define DETECTION_3D_DISPLAY_HPP_PUBLIC_TYPE DETECTION_3D_DISPLAY_HPP_PUBLIC
    #define DETECTION_3D_DISPLAY_HPP_LOCAL

    #ifdef DETECTION_3D_ARRAY_DISPLAY_HPP_BUILDING_LIBRARY
    #define DETECTION_3D_ARRAY_DISPLAY_HPP_PUBLIC DETECTION_3D_ARRAY_DISPLAY_HPP_EXPORT
    #else
        #define DETECTION_3D_ARRAY_DISPLAY_HPP_PUBLIC DETECTION_3D_ARRAY_DISPLAY_HPP_IMPORT
    #endif
    #define DETECTION_3D_ARRAY_DISPLAY_HPP_PUBLIC_TYPE DETECTION_3D_ARRAY_DISPLAY_HPP_PUBLIC
    #define DETECTION_3D_ARRAY_DISPLAY_HPP_LOCAL

    #ifdef BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_BUILDING_LIBRARY
        #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_PUBLIC BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_EXPORT
    #else
        #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_PUBLIC BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_IMPORT
    #endif
    #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_PUBLIC_TYPE BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_PUBLIC
    #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_LOCAL

    #ifdef BOUNDING_BOX_3D_DISPLAY_HPP_BUILDING_LIBRARY
        #define BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC BOUNDING_BOX_3D_DISPLAY_HPP_EXPORT
    #else
        #define BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC BOUNDING_BOX_3D_DISPLAY_HPP_IMPORT
    #endif
    #define BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC_TYPE BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC
    #define BOUNDING_BOX_3D_DISPLAY_HPP_LOCAL

    #ifdef BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_BUILDING_LIBRARY
    #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_PUBLIC BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_EXPORT
    #else
        #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_PUBLIC BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_IMPORT
    #endif
    #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_PUBLIC_TYPE BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_PUBLIC
    #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_LOCAL

#else
    #define DETECTION_3D_COMMON_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define DETECTION_3D_COMMON_DISPLAY_HPP_IMPORT

    #define DETECTION_3D_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define DETECTION_3D_DISPLAY_HPP_IMPORT

    #define DETECTION_3D_ARRAY_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define DETECTION_3D_ARRAY_DISPLAY_HPP_IMPORT

    #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_IMPORT

    #define BOUNDING_BOX_3D_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define BOUNDING_BOX_3D_DISPLAY_HPP_IMPORT

    #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_IMPORT
    #if __GNUC__ >= 4
        #define DETECTION_3D_COMMON_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define DETECTION_3D_COMMON_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

        #define DETECTION_3D_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define DETECTION_3D_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

        #define DETECTION_3D_ARRAY_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define DETECTION_3D_ARRAY_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

        #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

        #define BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define BOUNDING_BOX_3D_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

        #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

    #else
        #define DETECTION_3D_COMMON_DISPLAY_HPP_PUBLIC
        #define DETECTION_3D_COMMON_DISPLAY_HPP_LOCAL

        #define DETECTION_3D_DISPLAY_HPP_PUBLIC
        #define DETECTION_3D_DISPLAY_HPP_LOCAL

        #define DETECTION_3D_ARRAY_DISPLAY_HPP_PUBLIC
        #define DETECTION_3D_ARRAY_DISPLAY_HPP_LOCAL

        #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_PUBLIC
        #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_LOCAL

        #define BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC
        #define BOUNDING_BOX_3D_DISPLAY_HPP_LOCAL

        #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_PUBLIC
        #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_LOCAL
    #endif
    #define DETECTION_3D_COMMON_DISPLAY_HPP_PUBLIC_TYPE
    #define DETECTION_3D_DISPLAY_HPP_PUBLIC_TYPE
    #define DETECTION_3D_ARRAY_DISPLAY_HPP_PUBLIC_TYPE
    #define BOUNDING_BOX_3D_COMMON_DISPLAY_HPP_PUBLIC_TYPE
    #define BOUNDING_BOX_3D_DISPLAY_HPP_PUBLIC_TYPE
    #define BOUNDING_BOX_3D_ARRAY_DISPLAY_HPP_PUBLIC_TYPE
#endif

#endif  // VISION_MSGS_RVIZ_PLUGINS__VISIBILITY_CONTROL_HPP_
