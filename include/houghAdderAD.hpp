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
 * \brief Declarations for quadloco::hough::AdderAD
 *
 */


#include "datGrid.hpp"
#include "datRowCol.hpp"
#include "datSizeHW.hpp"
#include "datSizeMap.hpp"
#include "datSpan.hpp"
#include "datSpot.hpp"
#include "houghParmAD.hpp"
#include "pix.hpp"

#include <algorithm>
#include <numbers>


namespace quadloco
{

namespace hough
{

	//! Accumulation buffer for hough alpha,delta parameters
	class AdderAD
	{
		static constexpr double pi{ std::numbers::pi_v<double> };
		static constexpr double piTwo{ 2. * std::numbers::pi_v<double> };

		dat::SizeMap const theSizeMap;
		dat::Grid<float> theGridAD{};

		//! Add/subtract 2*pi if iAngle is under/over (min, max) range
		inline
		static
		double
		angleInRange
			( double const & iAngle
			, double const & min
			, double const & max
			)
		{
			double oAngle{ iAngle };
			// check for under value
			if (oAngle < min)
			{
				oAngle += piTwo;
			}
			else
			// check for over value
			if (! (oAngle < max))
			{
				oAngle -= piTwo;
			}
			return oAngle;
		}

	public:

		//! Construct accumulation grid with dimension adSize
		inline
		explicit
		AdderAD
			( dat::SizeHW const & adSize
			)
			: theSizeMap
				( adSize 
				, dat::Span{ -pi, pi }
				, dat::Span{ 0., piTwo }
				)
			, theGridAD(adSize)
		{
			std::fill(theGridAD.begin(), theGridAD.end(), 0.f);
		}

		//! Direct access to the accumulation grid
		inline
		dat::Grid<float> const &
		grid
			() const
		{
			return theGridAD;
		}

		//! Row/column in accumulation grid associated with ParmAD values
		inline
		dat::Spot
		datSpotFor
			( hough::ParmAD const & parmAD
			) const
		{
			// Get angles from parmAD and ensure within valid sizemap range
			dat::Spot const parmSpot
				{ angleInRange(parmAD.alpha(), -pi, pi)
				, angleInRange(parmAD.delta(), 0., piTwo)
				};
			return dat::Spot{ theSizeMap.gridSpotForAreaSpot(parmSpot) };
		}

		//! Row/column in accumulation grid associated with ParmAD values
		inline
		dat::RowCol
		datRowColFor
			( hough::ParmAD const & parmAD
			) const
		{
			dat::Spot const gridSpot{ datSpotFor(parmAD) };
			return cast::datRowCol(gridSpot);
		}

		inline
		hough::ParmAD
		houghParmADFor
			( dat::Spot const & gridSpot
			) const
		{
			hough::ParmAD parmAD{};
			if ( gridSpot.isValid()
			  && ( gridSpot[0] < (double)theGridAD.high())
			  && ( gridSpot[1] < (double)theGridAD.wide())
			   )
			{
				dat::Spot const areaSpot
					{ theSizeMap.areaSpotForGridSpot(gridSpot) };
				parmAD = hough::ParmAD{ areaSpot[0], areaSpot[1] };
			}
			return parmAD;
		}

		//! Incorporate parmAD values into theGridAD with gridMag as weight
		inline
		void
		add
			( hough::ParmAD const & parmAD
			, float const & gradMag
			)
		{
			if (pix::isValid(gradMag) && parmAD.isValid())
			{
				theGridAD(datRowColFor(parmAD)) += gradMag;
			}
		}

	}; // AdderAD


} // [hough]

} // [quadloco]

