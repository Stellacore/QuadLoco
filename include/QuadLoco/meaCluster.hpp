//
// MIT License
//
// Copyright (c) 2024 Stellacore Corporation
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//


#pragma once


/*! \file
 * \brief Declarations for quadloco::mea::Cluster namespace
 *
 */


#include "QuadLoco/meaVector.hpp"

#include <iostream>
#include <sstream>
#include <string>


namespace quadloco
{

namespace mea
{

	//! \brief Characterize geometry of a collection of 2D measurements
	struct Cluster
	{
		//! Weighted centroid estimate
		mea::Vector theMeaCenter{};

		/*! \brief Centroid for collection of mea::Vector instances
		 *
		 * Object of type are expected to be compatible with
		 * \arg mea::Vector const & meaPnt = *iter;
		 */
		template <typename FwdIter>
		inline
		static
		mea::Vector
		meaCenter
			( FwdIter const & itBeg
			, FwdIter const & itEnd
			)
		{
			mea::Vector centroid{};
			img::Vector<double> pntSum{ 0., 0. };
			double wgtSum{ 0. };
			for (FwdIter iter{itBeg} ; itEnd != iter ; ++iter)
			{
				img::Vector<double> const & pnt = iter->location();
				double const sampCovar{ iter->covar().covarianceRMS() };
				double const sampWgt{ 1. / sampCovar };
				pntSum = pntSum + sampWgt * pnt;
				wgtSum += sampWgt;
			}
			if (0. < wgtSum)
			{
				double const meanCovar{ 1. / wgtSum };
				double const meanSigma{ std::sqrt(meanCovar) };
				img::Vector<double> const meanLoc{ meanCovar * pntSum };
				centroid = mea::Vector(meanLoc, meanSigma);
			}
			return centroid;
		}

	public:

		//! Default construction of a null (isValid() == false) instance
		inline
		explicit
		Cluster
			() = default;

		/*! \brief Construct description from collection of mea::Vector
		 *
		 * Object of type are expected to be compatible with
		 * \arg mea::Vector const & meaPnt = *iter;
		 */
		template <typename FwdIter>
		inline
		explicit
		Cluster
			( FwdIter const & itBeg
			, FwdIter const & itEnd
			)
			: theMeaCenter{ meaCenter(itBeg, itEnd) }
		{ }


		//! True if this instance contains viable data
		inline
		bool
		isValid
			() const
		{
			return theMeaCenter.isValid();
		}

		//! Estimated centroid of mea::Vector instances from ctor
		inline
		mea::Vector const &
		meaVectorCenter
			() const
		{
			return theMeaCenter;
		}

		//! Estimated uncertainty in centroid (from meaVectorCenter() instance)
		inline
		double
		centroidDeviationRMS
			() const
		{
			return meaVectorCenter().covar().deviationRMS();
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			oss
				<< "meaCenter: " << theMeaCenter
				;
			return oss.str();
		}

	}; // Cluster


} // [mea]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::mea::Cluster const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::mea::Cluster const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

