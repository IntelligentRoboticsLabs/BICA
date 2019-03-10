/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Intelligent Robotics Core S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Intelligent Robotics Core nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************/

/* Author: Francisco Martín Rico - fmrico@gmail.com */

/* This code is based on https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/macros.hpp
 * Copyright 2014 Open Source Robotics Foundation, Inc.
 * Licensed under the Apache License, Version 2.0
*/

#include <memory>
#include <utility>

#ifndef BICA_GRAPH_MACROS_H
#define BICA_GRAPH_MACROS_H

/**
 * Disables the copy constructor and operator= for the given class.
 *
 * Use in the private section of the class.
 */
#define BICA_GRAPH_DISABLE_COPY(...) \
  __VA_ARGS__(const __VA_ARGS__ &) = delete; \
  __VA_ARGS__ & operator=(const __VA_ARGS__ &) = delete;

/**
 * Defines aliases and static functions for using the Class with smart pointers.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define BICA_GRAPH_SMART_PTR_DEFINITIONS(...) \
  BICA_GRAPH_SHARED_PTR_DEFINITIONS(__VA_ARGS__) \
  BICA_GRAPH_WEAK_PTR_DEFINITIONS(__VA_ARGS__) \
  BICA_GRAPH_UNIQUE_PTR_DEFINITIONS(__VA_ARGS__)

/**
 * Defines aliases and static functions for using the Class with smart pointers.
 *
 * Same as _SMART_PTR_DEFINITIONS except it excludes the static
 * Class::make_unique() method definition which does not work on classes which
 * are not CopyConstructable.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define BICA_GRAPH_SMART_PTR_DEFINITIONS_NOT_COPYABLE(...) \
  BICA_GRAPH_SHARED_PTR_DEFINITIONS(__VA_ARGS__) \
  BICA_GRAPH_WEAK_PTR_DEFINITIONS(__VA_ARGS__) \
  BICA_GRAPH___UNIQUE_PTR_ALIAS(__VA_ARGS__)

/**
 * Defines aliases only for using the Class with smart pointers.
 *
 * Same as _SMART_PTR_DEFINITIONS except it excludes the static
 * method definitions which do not work on pure virtual classes and classes
 * which are not CopyConstructable.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define BICA_GRAPH_SMART_PTR_ALIASES_ONLY(...) \
  BICA_GRAPH___SHARED_PTR_ALIAS(__VA_ARGS__) \
  BICA_GRAPH___WEAK_PTR_ALIAS(__VA_ARGS__) \
  BICA_GRAPH___MAKE_SHARED_DEFINITION(__VA_ARGS__)

#define BICA_GRAPH___SHARED_PTR_ALIAS(...) \
  using SharedPtr = std::shared_ptr<__VA_ARGS__>; \
  using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>;

#define BICA_GRAPH___MAKE_SHARED_DEFINITION(...) \
  template<typename ... Args> \
  static std::shared_ptr<__VA_ARGS__> \
  make_shared(Args && ... args) \
  { \
    return std::make_shared<__VA_ARGS__>(std::forward<Args>(args) ...); \
  }

/// Defines aliases and static functions for using the Class with shared_ptrs.
#define BICA_GRAPH_SHARED_PTR_DEFINITIONS(...) \
  BICA_GRAPH___SHARED_PTR_ALIAS(__VA_ARGS__) \
  BICA_GRAPH___MAKE_SHARED_DEFINITION(__VA_ARGS__)

#define BICA_GRAPH___WEAK_PTR_ALIAS(...) \
  using WeakPtr = std::weak_ptr<__VA_ARGS__>; \
  using ConstWeakPtr = std::weak_ptr<const __VA_ARGS__>;

/// Defines aliases and static functions for using the Class with weak_ptrs.
#define BICA_GRAPH_WEAK_PTR_DEFINITIONS(...) BICA_GRAPH___WEAK_PTR_ALIAS(__VA_ARGS__)

#define BICA_GRAPH___UNIQUE_PTR_ALIAS(...) using UniquePtr = std::unique_ptr<__VA_ARGS__>;

#define BICA_GRAPH___MAKE_UNIQUE_DEFINITION(...) \
  template<typename ... Args> \
  static std::unique_ptr<__VA_ARGS__> \
  make_unique(Args && ... args) \
  { \
    return std::unique_ptr<__VA_ARGS__>(new __VA_ARGS__(std::forward<Args>(args) ...)); \
  }

/// Defines aliases and static functions for using the Class with unique_ptrs.
#define BICA_GRAPH_UNIQUE_PTR_DEFINITIONS(...) \
  BICA_GRAPH___UNIQUE_PTR_ALIAS(__VA_ARGS__) \
  BICA_GRAPH___MAKE_UNIQUE_DEFINITION(__VA_ARGS__)

#endif  // BICA_GRAPH_MACROS_H
