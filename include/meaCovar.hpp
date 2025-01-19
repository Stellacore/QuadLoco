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
 * \brief Declarations for quadloco::mea::Covar namespace
 *
 */


#include "opsEigen2D.hpp"
#include "opsmatrix.hpp"
#include "rasGrid.hpp"

#include <cmath>
#include <limits>
#include <iostream>
#include <sstream>
#include <string>


namespace quadloco
{

namespace mea
{

	//! \brief Covariance information for 2D data
	class Covar
	{
		//! Eigen decomposition of covariance matrix
		ops::Eigen2D theEig{};

		//! Diagonal covariance matrix corresponding with sigma
		inline
		static
		ops::Matrix
		fromSigma
			( double const & sigma
			)
		{
			double const covarMag{ sigma * sigma };
			return (covarMag * ops::matrix::identity(2u));
		}

	public:

		//! Construct default null instance (isValid() == false)
		inline
		explicit
		Covar
			() = default;

		//! Value construction
		inline
		explicit
		Covar
			( ops::Matrix const & covarMat
			)
			: theEig(covarMat)
		{ }

		//! Convenience ctor - create matrix with squared sigma on diagonals
		inline
		explicit
		Covar
			( double const & sigma
			)
			: Covar(Covar::fromSigma(sigma))
		{ }

		//! True if this instance contains viable data
		inline
		bool
		isValid
			() const
		{
			return theEig.isValid();
		}

		//! Estimated circular uncertainty (root-mean-square of semiaxes)
		inline
		double
		covarianceRMS
			() const
		{
			double rms{ std::numeric_limits<double>::quiet_NaN() };
			if (isValid())
			{
				double const lam1{ theEig.valueMin() };
				double const lam2{ theEig.valueMax() };
				rms = lam1 + lam2;
			}
			return rms;
		}

		//! Estimated circular uncertainty (root-mean-square of semiaxes)
		inline
		double
		deviationRMS
			() const
		{
			return std::sqrt(covarianceRMS());
		}

		//! Uncertainty ellipse semi-axis of shortest length
		inline
		img::Vector<double>
		semiAxisMin
			() const
		{
			img::Vector<double> semiAxis{};
			if (isValid())
			{
				semiAxis = std::sqrt(theEig.valueMin()) * theEig.vectorMin();
			}
			return semiAxis;
		}

		//! Uncertainty ellipse semi-axis of shortest length
		inline
		img::Vector<double>
		semiAxisMax
			() const
		{
			img::Vector<double> semiAxis{};
			if (isValid())
			{
				semiAxis = std::sqrt(theEig.valueMax()) * theEig.vectorMax();
			}
			return semiAxis;
		}

		//! Matrix values reconstituted from eigen decomposition
		inline
		ras::Grid<double>
		matrix
			() const
		{
			return theEig.matrix();
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
				oss << title << '\n';
			}
			using engabra::g3::io::fixed;
			ops::Matrix const coMat{ matrix() };
			oss
				<< fixed(coMat(0u, 0u), 6u, 6u)
				<< ' '
				<< fixed(coMat(0u, 1u), 6u, 6u)
				<< '\n'
				<< fixed(coMat(1u, 0u), 6u, 6u)
				<< ' '
				<< fixed(coMat(1u, 1u), 6u, 6u)
				;
			return oss.str();
		}

	}; // Covar


} // [mea]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::mea::Covar const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::mea::Covar const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

