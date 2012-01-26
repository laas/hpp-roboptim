# Copyright (c) 2011 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-wholebody-step-planner.
# hpp-wholebody-step-planner is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-wholebody-step-planner is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-wholebody-step-planner.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)

SET(CXX_DISABLE_WERROR ON)
SET(PROJECT_NAME hpp-roboptim)
SET(PROJECT_VERSION 0.1)
SET(PROJECT_DESCRIPTION
  "Bind hpp robot modelling and roboptim optimization framework.")
SET(PROJECT_URL "https://github.com/laas/hpp-roboptim")

SET(CUSTOM_HEADER_DIR hpp/roboptim)
SET(${PROJECT_NAME}_HEADERS
  include/hpp/roboptim/fwd.hh
  include/hpp/roboptim/spline-directpath.hh
)

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

SETUP_PROJECT()

# Search for Boost.
# Boost.Test is used by the test suite.
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREAD ON)
FIND_PACKAGE(Boost 1.40 COMPONENTS unit_test_framework REQUIRED)

ADD_REQUIRED_DEPENDENCY(roboptim-trajectory >= 0.6)
ADD_REQUIRED_DEPENDENCY(hpp-util >= 0.4)
ADD_REQUIRED_DEPENDENCY(hpp-model >= 2.1)
ADD_REQUIRED_DEPENDENCY(roboptim-core >= 0.6)
PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)
SETUP_PROJECT_FINALIZE()
