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
#include "datMapSizeArea.hpp"
#include "datRowCol.hpp"
#include "datSizeHW.hpp"
#include "datSpan.hpp"
#include "datSpot.hpp"
#include "houghParmAD.hpp"
#include "pix.hpp"

#include <algorithm>
#include <cmath>
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

		dat::Grid<float> theGridAD{};
		dat::MapSizeArea const theMapSizeArea;
		double const theDeltaA{ std::numeric_limits<double>::quiet_NaN() };
		double const theDeltaD{ std::numeric_limits<double>::quiet_NaN() };

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

		//! Value wrapped into range [0,maxValue]
		inline
		static
		double
		wrappedValue
			( double const & anyValue
			, double const & maxValue
			)
		{
			double value{ anyValue };
			while (value < 0.)
			{
				value += maxValue;
			}
			while (! (value < maxValue))
			{
				value -= maxValue;
			}
			return value;
		}

	public:

		//! Useful size for AD accumulation grid for given pixel grid input
		inline
		static
		dat::SizeHW
		adSizeHintFor
			( dat::SizeHW const & hwPixGrid
			)
		{
			std::size_t const sizeAD{ hwPixGrid.perimeter() / 4u };
			return dat::SizeHW{ sizeAD, sizeAD };
		}


		//! Construct accumulation grid with dimension adSize
		inline
		explicit
		AdderAD
			( dat::SizeHW const & adSize
			)
			: theGridAD(adSize)
			, theMapSizeArea
				( theGridAD.hwSize() 
				, dat::Area
					{ dat::Span{ -pi, pi }
					, dat::Span{ 0., piTwo }
					}
				)
		{
			std::fill(theGridAD.begin(), theGridAD.end(), 0.f);
		}

		//! True if this instance contains valid data (is not null)
		inline
		bool
		isValid
			() const
		{
			return
				(  theMapSizeArea.isValid()
				&& theGridAD.isValid()
				);
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
		datSpotForAD
			( hough::ParmAD const & parmAD
			) const
		{
			// Get angles from parmAD and ensure within valid sizemap range
			dat::Spot const parmSpot
				{ angleInRange(parmAD.alpha(), -pi, pi)
				, angleInRange(parmAD.delta(), 0., piTwo)
				};
			return dat::Spot{ theMapSizeArea.gridSpotForAreaSpot(parmSpot) };
		}

		//! Row/column in accumulation grid associated with ParmAD values
		inline
		dat::RowCol
		datRowColForAD
			( hough::ParmAD const & parmAD
			) const
		{
			dat::Spot const gridSpot{ datSpotForAD(parmAD) };
			return cast::datRowCol(gridSpot);
		}

		//! Hough alpha,delta parameter values at location gridSpot
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
					{ theMapSizeArea.areaSpotForGridSpot(gridSpot) };
				parmAD = hough::ParmAD{ areaSpot[0], areaSpot[1] };
			}
			return parmAD;
		}

		//! Bell curve like weighting function exp(-|fracSpot|^2).
		inline
		double
		weightAt
			( dat::Vec2D<double> const & fracSpot
			, double const & sigma = 1.
			)
		{
			double const radius{ magnitude(fracSpot) };
			double const zVal{ radius / sigma };
			return std::exp(-zVal*zVal);
		}

		//! Grid RowCol location associated with grid spot
		inline
		dat::RowCol
		wrappedGridSpot
			( dat::Spot const & gridSpot
			) const
		{
			dat::RowCol rc{};

			if (gridSpot.isValid())
			{
				double const dHigh{ (double)theGridAD.high() };
				double const dubRow{ wrappedValue(gridSpot[0], dHigh) };

				double const dWide{ (double)theGridAD.wide() };
				double const dubCol{ wrappedValue(gridSpot[1], dWide) };

				rc = dat::RowCol
					{ static_cast<std::size_t>(std::floor(dubRow))
					, static_cast<std::size_t>(std::floor(dubCol))
					};

				/*
				dat::Spot const wrapSpot{ dubRow, dubCol };
				bool err{ false };
				if (! (rc.row() < theGridAD.high()))
				{
					err = true;
				}
				if (! (rc.col() < theGridAD.wide()))
				{
					err = true;
				}
				if (err)
				{
					std::cerr << "gridSpot: " << gridSpot << '\n';
					std::cerr << "theGridAD: " << theGridAD << '\n';
					std::cerr << "wrapSpot: " << wrapSpot << '\n';
					std::cerr << "rc: " << rc << '\n';
					exit(8);
				}
				*/
			}

			return rc;
		}

		//! Incorporate parmAD values into theGridAD with gridMag as weight
		inline
		void
		add
			( hough::ParmAD const & parmAD
			, float const & gradMag
			)
		{
			if (isValid() && pix::isValid(gradMag) && parmAD.isValid())
			{
				dat::Spot const spot00{ datSpotForAD(parmAD) };

				static dat::Spot const dSpotNN{ -1., -1. };
				static dat::Spot const dSpotZN{  0., -1. };
				static dat::Spot const dSpotPN{  1., -1. };

				static dat::Spot const dSpotNZ{ -1.,  0. };
				static dat::Spot const dSpotZZ{  0.,  0. };
				static dat::Spot const dSpotPZ{  1.,  0. };

				static dat::Spot const dSpotNP{ -1.,  1. };
				static dat::Spot const dSpotZP{  0.,  1. };
				static dat::Spot const dSpotPP{  1.,  1. };


				dat::Spot const spotNN{ spot00 + dSpotNN };
				dat::Spot const spotZN{ spot00 + dSpotZN };
				dat::Spot const spotPN{ spot00 + dSpotPN };

				dat::Spot const spotNZ{ spot00 + dSpotNZ };
				dat::Spot const spotZZ{ spot00 + dSpotZZ };
				dat::Spot const spotPZ{ spot00 + dSpotPZ };

				dat::Spot const spotNP{ spot00 + dSpotNP };
				dat::Spot const spotZP{ spot00 + dSpotZP };
				dat::Spot const spotPP{ spot00 + dSpotPP };

/*
std::cout << '\n';

std::cout << "spotNN: " << spotNN << '\n';
std::cout << "spotZN: " << spotZN << '\n';
std::cout << "spotPN: " << spotPN << '\n';

std::cout << "spotNZ: " << spotNZ << '\n';
std::cout << "spotZZ: " << spotZZ << '\n';
std::cout << "spotPZ: " << spotPZ << '\n';

std::cout << "spotNP: " << spotNP << '\n';
std::cout << "spotZP: " << spotZP << '\n';
std::cout << "spotPP: " << spotPP << '\n';
*/

				dat::RowCol const rcNN{ wrappedGridSpot(spotNN) };
				dat::RowCol const rcZN{ wrappedGridSpot(spotZN) };
				dat::RowCol const rcPN{ wrappedGridSpot(spotPN) };

				dat::RowCol const rcNZ{ wrappedGridSpot(spotNZ) };
				dat::RowCol const rcZZ{ wrappedGridSpot(spotZZ) };
				dat::RowCol const rcPZ{ wrappedGridSpot(spotPZ) };

				dat::RowCol const rcNP{ wrappedGridSpot(spotNP) };
				dat::RowCol const rcZP{ wrappedGridSpot(spotZP) };
				dat::RowCol const rcPP{ wrappedGridSpot(spotPP) };

/*
std::cout << '\n';

std::cout << "rcNN: " << rcNN << '\n';
std::cout << "rcZN: " << rcZN << '\n';
std::cout << "rcPN: " << rcPN << '\n';

std::cout << "rcNZ: " << rcNZ << '\n';
std::cout << "rcZZ: " << rcZZ << '\n';
std::cout << "rcPZ: " << rcPZ << '\n';

std::cout << "rcNP: " << rcNP << '\n';
std::cout << "rcZP: " << rcZP << '\n';
std::cout << "rcPP: " << rcPP << '\n';
*/


				dat::Spot const fracZZ{ spotZZ - cast::datSpot(rcZZ) };

/*
std::cout << '\n';
std::cout << "fracZZ: " << fracZZ << '\n';
*/

				/*
				dat::Spot const evalNN{ (fracZZ + dSpotNN) };
				dat::Spot const evalZN{ (fracZZ + dSpotZN) };
				dat::Spot const evalPN{ (fracZZ + dSpotPN) };

				dat::Spot const evalNZ{ (fracZZ + dSpotNZ) };
				dat::Spot const evalZZ{ (fracZZ + dSpotZZ) };
				dat::Spot const evalPZ{ (fracZZ + dSpotPZ) };

				dat::Spot const evalNP{ (fracZZ + dSpotNP) };
				dat::Spot const evalZP{ (fracZZ + dSpotZP) };
				dat::Spot const evalPP{ (fracZZ + dSpotPP) };
				*/

/*
std::cout << '\n';

std::cout << "evalNN: " << evalNN << '\n';
std::cout << "evalZN: " << evalZN << '\n';
std::cout << "evalPN: " << evalPN << '\n';

std::cout << "evalNZ: " << evalNZ << '\n';
std::cout << "evalZZ: " << evalZZ << '\n';
std::cout << "evalPZ: " << evalPZ << '\n';

std::cout << "evalNP: " << evalNP << '\n';
std::cout << "evalZP: " << evalZP << '\n';
std::cout << "evalPP: " << evalPP << '\n';
*/

				double const weightNN{ weightAt(fracZZ + dSpotNN) };
				double const weightZN{ weightAt(fracZZ + dSpotZN) };
				double const weightPN{ weightAt(fracZZ + dSpotPN) };

				double const weightNZ{ weightAt(fracZZ + dSpotNZ) };
				double const weightZZ{ weightAt(fracZZ + dSpotZZ) };
				double const weightPZ{ weightAt(fracZZ + dSpotPZ) };

				double const weightNP{ weightAt(fracZZ + dSpotNP) };
				double const weightZP{ weightAt(fracZZ + dSpotZP) };
				double const weightPP{ weightAt(fracZZ + dSpotPP) };

/*
std::cout << '\n';

std::cout << "weightNN: " << weightNN << '\n';
std::cout << "weightZN: " << weightZN << '\n';
std::cout << "weightPN: " << weightPN << '\n';

std::cout << "weightNZ: " << weightNZ << '\n';
std::cout << "weightZZ: " << weightZZ << '\n';
std::cout << "weightPZ: " << weightPZ << '\n';

std::cout << "weightNP: " << weightNP << '\n';
std::cout << "weightZP: " << weightZP << '\n';
std::cout << "weightPP: " << weightPP << '\n';
*/

				theGridAD(rcNN) += weightNN * gradMag;
				theGridAD(rcZN) += weightZN * gradMag;
				theGridAD(rcPN) += weightPN * gradMag;

				theGridAD(rcNZ) += weightNZ * gradMag;
				theGridAD(rcZZ) += weightZZ * gradMag;
				theGridAD(rcPZ) += weightPZ * gradMag;

				theGridAD(rcNP) += weightNP * gradMag;
				theGridAD(rcZP) += weightZP * gradMag;
				theGridAD(rcPP) += weightPP * gradMag;
			}
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
				<< " theGridAD: " << theGridAD
				<< '\n'
				<< "theMapSizeArea:\n" << theMapSizeArea
				;

			return oss.str();
		}


	}; // AdderAD


} // [hough]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::hough::AdderAD const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::hough::AdderAD const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

