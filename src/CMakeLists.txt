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

ADD_LIBRARY(${PROJECT_NAME} SHARED
  spline-directpath.cc)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib)

PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} roboptim-trajectory)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-model)

