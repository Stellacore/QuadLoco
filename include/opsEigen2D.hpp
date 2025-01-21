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
 * \brief Declarations for quadloco::ops::Eigen2D namespace
 *
 */


#include "imgVector.hpp"
#include "opsmatrix.hpp"
#include "rasGrid.hpp"

#include <Engabra>

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>


namespace quadloco
{

namespace ops
{

	//! \brief Eigen decomposition for a 2x2 matrix (no complex result support)
	struct Eigen2D
	{
		//! Minimum eigenvalue
		double theLamMin{ engabra::g3::null<double>() };

		//! Maximum eigenvalue
		double theLamMax{ engabra::g3::null<double>() };

		//! Eigenvector for minimum eigenvalue
		img::Vector<double> theVecMin{};

		//! Eigenvector for maximum eigenvalue
		img::Vector<double> theVecMax{};


		//! True if decomposition is well defined
		inline
		bool
		isValid
			() const
		{
			return
				(  engabra::g3::isValid(theLamMin)
				&& engabra::g3::isValid(theLamMax)
				);
		}

		//! True if absolute value of A and B are *both* less than machine eps
		inline
		static
		bool
		bothNearZero
			( double const valA
			, double const valB
			)
		{
			constexpr double eps{ std::numeric_limits<double>::epsilon() };
			return
				(  (std::abs(valA) < eps)
				&& (std::abs(valB) < eps)
				);
		}

		//! Default construction of a null (isValid() == false) instance
		inline
		explicit
		Eigen2D
			() = default;

		//! Perform eigen value/vector decomposition
		inline
		explicit
		Eigen2D
			( ras::Grid<double> const & mat2D
			)
		{
			double const det{ determinant2x2(mat2D) };
			double const trc{ trace2x2(mat2D) };

			double const radicand{ .25*trc*trc - det };
			if (! (radicand < 0.))
			{
				double const root{ std::sqrt(radicand) };
				double const lam0{ .5 * trc };
				theLamMin = lam0 - root;
				theLamMax = lam0 + root;

				double const & aa = mat2D(0, 0);
				double const & bb = mat2D(0, 1);
				double const & cc = mat2D(1, 0);
				double const & dd = mat2D(1, 1);
				if (bothNearZero(bb, cc)) // cc and bb near zero
				{
					theVecMin = { 1., 0. };
					theVecMax = { 0., 1. };
				}
				else
				if (std::abs(bb) < std::abs(cc)) // cc << bb (bb not zero)
				{
					theVecMin = { bb, theLamMin - aa };
					theVecMax = { bb, theLamMax - aa };
				}
				else // bb < cc (cc not zero)
				{
					theVecMin = { theLamMin - dd, cc };
					theVecMax = { theLamMax - dd, cc };
				}
				theVecMin = direction(theVecMin);
				theVecMax = direction(theVecMax);
			}
		}

		//! Smallest eigenvalue
		inline
		double const &
		valueMin
			() const
		{
			return theLamMin;
		}

		//! Largest eigenvalue
		inline
		double const &
		valueMax
			() const
		{
			return theLamMax;
		}

		//! Eigenvector corresponding with valueMin()
		inline
		img::Vector<double> const &
		vectorMin
			() const
		{
			return theVecMin;
		}

		//! Eigenvector corresponding with valueMax()
		inline
		img::Vector<double> const &
		vectorMax
			() const
		{
			return theVecMax;
		}

		//! Matrix values reconstituted from eigen decomposition
		inline
		ras::Grid<double>
		matrix
			() const
		{
			ops::Matrix mat(2u, 2u);
			double const & v11 = theVecMin[0];
			double const & v12 = theVecMin[1];
			double const & d1 = theLamMin;
			double const & v21 = theVecMax[0];
			double const & v22 = theVecMax[1];
			double const & d2 = theLamMax;
			mat(0u, 0u) = v11*d1*v11 + v12*d2*v12;
			mat(0u, 1u) = v11*d1*v21 + v12*d2*v22;
			mat(1u, 0u) = v21*d1*v11 + v22*d2*v12;
			mat(1u, 1u) = v21*d1*v21 + v22*d2*v22;
			return mat;
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
			using engabra::g3::io::fixed;
			oss
				<< "theLamMin: " << fixed(theLamMin)
				<< ' ' 
				<< "theVecMin: " << theVecMin
				<< '\n'
				<< "theLamMax: " << fixed(theLamMax)
				<< ' ' 
				<< "theVecMax: " << theVecMax
				;

			return oss.str();
		}

	}; // Eigen2D


} // [ops]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ops::Eigen2D const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::ops::Eigen2D const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

