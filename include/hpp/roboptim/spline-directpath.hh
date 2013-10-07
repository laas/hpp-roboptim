// Copyright (C) 2012 CNRS-LAAS
// Author: Florent Lamiraux
//
// This file is part of the hpp-wholebody-step-planner.
//
// hpp-wholebody-step-planner is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General
// Public License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step-planner is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-wholebody-step-planner.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_ROBOPTIM_SPLINE_DIRECTPATH_HH
# define HPP_ROBOPTIM_SPLINE_DIRECTPATH_HH

# include <vector>

# include <KineoWorks2/kwsDirectPath.h>
# include <roboptim/trajectory/fwd.hh>

# include <hpp/util/kitelab.hh>

HPP_KIT_PREDEF_CLASS (CkwsPath);
class CjrlGikStateConstraint;

namespace hpp {
  namespace roboptim {
    HPP_KIT_PREDEF_CLASS (SplineDirectPath);
    using ::roboptim::CubicBSpline;

    class SplineDirectPath : public CkwsDirectPath
    {
    public:
      typedef unsigned int size_type;
      virtual ~SplineDirectPath();

      /// Create a spline direct path

      /// \param path Initial path to approximate,
      /// \param nbControlPoints number of control points,
      /// \param manifold Set of non-linear constraints defining the manifold
      ///        on which the system is constrained,
      /// \param goalconstraints Set of constraints defining the goal
      ///        configuration.
      static SplineDirectPathShPtr create
      (const CkwsPathShPtr& path, size_type nbControlPoints,
       std::vector<CjrlGikStateConstraint*> manifold,
       std::vector<CjrlGikStateConstraint*> goalconstraints);

      const CubicBSpline* spline () const
      {
	return spline_;
      }

      //FIXME: can we get rid of that?
      // Is that behavior wanted?
      CubicBSpline* spline ()
      {
	return spline_;
      }

      /// Define which degrees of freedom are active.
      ktStatus setConfigMask(const std::vector<bool> & mask)
      {
	mask_ = mask;
	recomputeIndices ();
      }

      virtual CkwsAbstractPathShPtr clone () const;
      virtual void maxAbsoluteDerivative
      (double, double, std::vector<double>&) const;
      virtual void interpolate (double, CkwsConfig&) const;
      virtual double computePrivateLength () const;
      
    protected:
      /// \param path Initial path to approximate,
      /// \param nbControlPoints number of control points,
      /// \param manifold Set of non-linear constraints defining the manifold
      ///        on which the system is constrained,
      /// \param goalConstraints Set of constraints defining the goal
      ///        configuration.
      SplineDirectPath (const CkwsPathShPtr& path, size_type nbControlPoints,
			std::vector<CjrlGikStateConstraint*> manifold,
			std::vector<CjrlGikStateConstraint*> goalConstraints,
			const CkwsConfig& startConfig,
			const CkwsConfig& endConfig);

      ktStatus init (const SplineDirectPathWkPtr& weakPtr);
    private:
      void checkDirectPath ();
      void recomputeIndices ();
      void bestFit (CkwsPathConstShPtr path, size_type nbControlPoints);

      SplineDirectPathWkPtr weakPtr_;
      std::vector<CjrlGikStateConstraint*> manifold_;
      std::vector<CjrlGikStateConstraint*> goalConstraints_;
      std::vector<bool> mask_;
      /// Mapping from roboptim vector indices to hpp-model configuration
      /// indices.
      std::vector<size_type> roboptimToModel_;
      CubicBSpline* spline_;
    }; // class SplineDirectPath
  } // namespace roboptim
} // namespace hpp
#endif // HPP_ROBOPTIM_SPLINE_DIRECTPATH_HH
