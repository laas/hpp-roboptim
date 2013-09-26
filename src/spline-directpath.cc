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

#include <KineoWorks2/kwsPath.h>
#include <KineoWorks2/kwsDevice.h>
#include <KineoModel/kppSMLinearComponent.h>

#include <hpp/kwsio/configuration.hh>

#include <roboptim/trajectory/cubic-b-spline.hh>
#include <roboptim/core/function.hh>

#include <hpp/util/debug.hh>

#include <jrl/mal/matrixabstractlayer.hh>
#include <gikTask/jrlGikStateConstraint.h>

#include "hpp/roboptim/spline-directpath.hh"

namespace hpp {
  namespace roboptim {

    SplineDirectPath::~SplineDirectPath() {}

    SplineDirectPathShPtr SplineDirectPath::create
    (const CkwsPathShPtr& path, unsigned int nbControlPoints,
     std::vector<CjrlGikStateConstraint*> manifold,
     std::vector<CjrlGikStateConstraint*> goalconstraints)
    {
      hppDout (info, "");
      CkwsConfig startConfig (path->device ());
      CkwsConfig endConfig (path->device ());
      path->getConfigAtStart (startConfig);
      path->getConfigAtEnd (endConfig);
      hppDout (info, "");
      SplineDirectPath* ptr = new SplineDirectPath (path, nbControlPoints,
						    manifold, goalconstraints,
						    startConfig, endConfig);
      SplineDirectPathShPtr shPtr (ptr);
      SplineDirectPathWkPtr wkPtr (shPtr);

      if (ptr->init (wkPtr) != KD_OK) shPtr.reset ();
      return shPtr;
    }
    CkwsAbstractPathShPtr SplineDirectPath::clone () const
    {
      assert (0 && "SplineDirectPath is not clonable.");
      return CkwsAbstractPathShPtr ();
    }
    void SplineDirectPath::maxAbsoluteDerivative
    (double, double, std::vector<double>&) const
    {
      assert (0 && "You should not need to call this function.");
    }
    void SplineDirectPath::interpolate (double param, CkwsConfig& config) const
    {
      assert(param <= privateLength());

      CubicBSpline::vector_t result =
	(*spline_) (std::min(param, privateLength ()));
      for(size_type i = 0; i < roboptimToModel_.size (); ++i)
	config.dofValue(roboptimToModel_[i], result [i]);
    }

    double SplineDirectPath::computePrivateLength () const
    {
      return spline_->length ();
    }

    SplineDirectPath::SplineDirectPath
    (const CkwsPathShPtr& path, unsigned int nbControlPoints,
     std::vector<CjrlGikStateConstraint*> manifold,
     std::vector<CjrlGikStateConstraint*> goalConstraints,
     const CkwsConfig& startConfig, const CkwsConfig& endConfig) :
      CkwsDirectPath (startConfig, endConfig,
		      CkppSMLinearComponent::create ()),
      manifold_ (manifold), goalConstraints_ (goalConstraints),
      mask_ (std::vector <bool> (path->device ()->countDofs (), true)),
      spline_ (0)
    {
      hppDout (info, "");
      recomputeIndices ();
      bestFit (path, nbControlPoints);
    }

    ktStatus SplineDirectPath::init (const SplineDirectPathWkPtr& weakPtr)
    {
      hppDout (info, "");
      if (CkwsDirectPath::init (weakPtr) != KD_OK) return KD_ERROR;
      weakPtr_ = weakPtr;
      checkDirectPath ();
      return KD_OK;
    }

    void SplineDirectPath::checkDirectPath ()
    {
      const CkwsConfig& startConfig (privateStart ());
      const CkwsConfig& endConfig (privateEnd ());
      CkwsConfigShPtr config_0 = this->configAtParam (0.);
      CkwsConfigShPtr config_1 = this->configAtParam (1.);

      hppDout (info, "Initial configuration: " << *(config_0));
      hppDout (info, "Config at start      : " << startConfig);
      hppDout (info, "Final configuration: " << *(config_1));
      hppDout (info, "Config at end      : " << endConfig);
      assert (config_0->isEquivalent(startConfig));
      assert (config_1->isEquivalent(endConfig));
      hppDout (info, "assert passed.");

    }

    void SplineDirectPath::recomputeIndices ()
    {
      hppDout (info, "");
      roboptimToModel_.clear ();
      for (size_type iModel=0; iModel<mask_.size (); iModel++) {
	if (mask_[iModel]) roboptimToModel_.push_back (iModel);
      }
    }

    void SplineDirectPath::bestFit (CkwsPathConstShPtr path,
				    size_type nbControlPoints)
    {
      hppDout (info, "");
      size_type m = nbControlPoints;
      size_type n = roboptimToModel_.size ();
      using roboptim::CubicBSpline;
      using ::roboptim::Function;
      CkwsDeviceShPtr device (path->device ());
      spline_ = new CubicBSpline (Function::makeInterval
				  (0., path->length ()), n,
				  CubicBSpline::vector_t (m*n),
				  "Cubic B-spline: initial motion");

      // Interpolate input path at discretized way points.
      CubicBSpline::jacobian_t Bitj (m, m);
      CubicBSpline::jacobian_t Qitj (n, m);

      for (size_type j=0; j<m; j++) {
	double tj = j*path->length ()/(m-1);
	CkwsConfig config (device);
	path->getConfigAtDistance (tj, config);
	CubicBSpline::jacobian_t jac = spline_->variationConfigWrtParam (tj);
	hppDout (info, "spline_->variationConfigWrtParam (" << tj << ")="
		 << jac);
	for (size_type i=0; i<n; i++) {
	  try {
	    Qitj (i,j) = config.dofValue (roboptimToModel_[i]);
	  } catch (const std::exception& exc) {
	    hppDout (error, exc.what ());
	    hppDout (error, "i,j = " << i << "," << j);
	    hppDout (error, "config.size ()=" << config.size ());
	    hppDout (error, "m=" << m);
	    hppDout (error, "Qitj.size=" << Qitj.size1 () << ","
		     << Qitj.size2 ());
	    abort ();
	  }
	}
	for (size_type i=0; i<m; i++) {
	  try {
	    Bitj (i,j) = jac(0, n*i);
	  } catch (const std::exception& exc) {
	    hppDout (error, exc.what ());
	    hppDout (error, "i,j = " << i << "," << j);
	    hppDout (error, "m=" << m);
	    hppDout (error, "jac.size=" << jac.size1 () << "," << jac.size2 ());
	    hppDout (error, "Bitj.size=" << Bitj.size1 () << ","
		     << Bitj.size2 ());
	    abort ();
	  }
	}
      }
      hppDout (info, "Bitj = " << Bitj);
      hppDout (info, "Qitj = " << Qitj);
      CubicBSpline::jacobian_t inverse;
      MAL_INVERSE (Bitj, inverse, double);
      CubicBSpline::jacobian_t P = prod (Qitj,inverse);
      // Fill control point vector
      CubicBSpline::vector_t controlPoints (m*n);
      size_type idx=0;
      for (unsigned icp=0; icp<m; icp++) {
	for (unsigned dof=0; dof<n; dof++) {
	  try {
	    controlPoints (idx) = P(dof, icp);
	  } catch (const std::exception& exc) {
	    hppDout (error, exc.what ());
	    hppDout (error, "m=" << m);
	    hppDout (error, "n=" << n);
	    hppDout (error, "idx=" << idx);
	    hppDout (error, "controlPoints.size ()=" << controlPoints.size ());
	    abort ();
	  }
	  idx++;
	}
      }
      spline_->setParameters (controlPoints);
    }
  } // namespace roboptim
} // namespace hpp
