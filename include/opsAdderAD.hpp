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
 * \brief Declarations for quadloco::ops::AdderAD
 *
 */


#include "imgSpan.hpp"
#include "imgSpot.hpp"
#include "pix.hpp"
#include "rasGrid.hpp"
#include "rasRowCol.hpp"
#include "rasSizeHW.hpp"
#include "sigParmAD.hpp"
#include "xfmMapSizeArea.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>


namespace quadloco
{

namespace ops
{

	//! Accumulation buffer for hough alpha,delta parameters
	class AdderAD
	{
		static constexpr double pi{ std::numbers::pi_v<double> };
		static constexpr double piTwo{ 2. * std::numbers::pi_v<double> };

		ras::Grid<float> theGridAD{};
		xfm::MapSizeArea const theMapSizeArea;
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
		ras::SizeHW
		adSizeHintFor
			( ras::SizeHW const & hwPixGrid
			)
		{
			std::size_t const sizeAD{ hwPixGrid.perimeter() / 4u };
			return ras::SizeHW{ sizeAD, sizeAD };
		}


		//! Construct accumulation grid with dimension adSize
		inline
		explicit
		AdderAD
			( ras::SizeHW const & adSize
			)
			: theGridAD(adSize)
			, theMapSizeArea
				( theGridAD.hwSize() 
				, img::Area
					{ img::Span{ -pi, pi }
					, img::Span{ 0., piTwo }
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
		ras::Grid<float> const &
		grid
			() const
		{
			return theGridAD;
		}

		//! Row/column in accumulation grid associated with ParmAD values
		inline
		img::Spot
		imgSpotForAD
			( sig::ParmAD const & parmAD
			) const
		{
			// Get angles from parmAD and ensure within valid sizemap range
			img::Spot const parmSpot
				{ angleInRange(parmAD.alpha(), -pi, pi)
				, angleInRange(parmAD.delta(), 0., piTwo)
				};
			return img::Spot{ theMapSizeArea.gridSpotForAreaSpot(parmSpot) };
		}

		//! Row/column in accumulation grid associated with ParmAD values
		inline
		ras::RowCol
		rasRowColForAD
			( sig::ParmAD const & parmAD
			) const
		{
			img::Spot const gridSpot{ imgSpotForAD(parmAD) };
			return cast::rasRowCol(gridSpot);
		}

		//! Hough alpha,delta parameter values at location gridSpot
		inline
		sig::ParmAD
		sigParmADFor
			( img::Spot const & gridSpot
			) const
		{
			sig::ParmAD parmAD{};
			if ( gridSpot.isValid()
			  && ( gridSpot[0] < (double)theGridAD.high())
			  && ( gridSpot[1] < (double)theGridAD.wide())
			   )
			{
				img::Spot const areaSpot
					{ theMapSizeArea.areaSpotForGridSpot(gridSpot) };
				parmAD = sig::ParmAD{ areaSpot[0], areaSpot[1] };
			}
			return parmAD;
		}

		//! Bell curve like weighting function exp(-|fracSpot|^2).
		inline
		double
		weightAt
			( img::Vector<double> const & fracSpot
			, double const & sigma = 1.
			)
		{
			double const radius{ magnitude(fracSpot) };
			double const zVal{ radius / sigma };
			return std::exp(-zVal*zVal);
		}

		//! Grid RowCol location associated with grid spot
		inline
		ras::RowCol
		wrappedGridSpot
			( img::Spot const & gridSpot
			) const
		{
			ras::RowCol rc{};

			if (gridSpot.isValid())
			{
				double const dHigh{ (double)theGridAD.high() };
				double const dubRow{ wrappedValue(gridSpot[0], dHigh) };

				double const dWide{ (double)theGridAD.wide() };
				double const dubCol{ wrappedValue(gridSpot[1], dWide) };

				rc = ras::RowCol
					{ static_cast<std::size_t>(std::floor(dubRow))
					, static_cast<std::size_t>(std::floor(dubCol))
					};

				/*
				img::Spot const wrapSpot{ dubRow, dubCol };
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
			( sig::ParmAD const & parmAD
			, float const & gradMag
			)
		{
			if (isValid() && pix::isValid(gradMag) && parmAD.isValid())
			{
				img::Spot const spot00{ imgSpotForAD(parmAD) };

				static img::Spot const dSpotNN{ -1., -1. };
				static img::Spot const dSpotZN{  0., -1. };
				static img::Spot const dSpotPN{  1., -1. };

				static img::Spot const dSpotNZ{ -1.,  0. };
				static img::Spot const dSpotZZ{  0.,  0. };
				static img::Spot const dSpotPZ{  1.,  0. };

				static img::Spot const dSpotNP{ -1.,  1. };
				static img::Spot const dSpotZP{  0.,  1. };
				static img::Spot const dSpotPP{  1.,  1. };


				img::Spot const spotNN{ spot00 + dSpotNN };
				img::Spot const spotZN{ spot00 + dSpotZN };
				img::Spot const spotPN{ spot00 + dSpotPN };

				img::Spot const spotNZ{ spot00 + dSpotNZ };
				img::Spot const spotZZ{ spot00 + dSpotZZ };
				img::Spot const spotPZ{ spot00 + dSpotPZ };

				img::Spot const spotNP{ spot00 + dSpotNP };
				img::Spot const spotZP{ spot00 + dSpotZP };
				img::Spot const spotPP{ spot00 + dSpotPP };

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

				ras::RowCol const rcNN{ wrappedGridSpot(spotNN) };
				ras::RowCol const rcZN{ wrappedGridSpot(spotZN) };
				ras::RowCol const rcPN{ wrappedGridSpot(spotPN) };

				ras::RowCol const rcNZ{ wrappedGridSpot(spotNZ) };
				ras::RowCol const rcZZ{ wrappedGridSpot(spotZZ) };
				ras::RowCol const rcPZ{ wrappedGridSpot(spotPZ) };

				ras::RowCol const rcNP{ wrappedGridSpot(spotNP) };
				ras::RowCol const rcZP{ wrappedGridSpot(spotZP) };
				ras::RowCol const rcPP{ wrappedGridSpot(spotPP) };

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


				img::Spot const fracZZ{ spotZZ - cast::imgSpot(rcZZ) };

/*
std::cout << '\n';
std::cout << "fracZZ: " << fracZZ << '\n';
*/

				/*
				img::Spot const evalNN{ (fracZZ + dSpotNN) };
				img::Spot const evalZN{ (fracZZ + dSpotZN) };
				img::Spot const evalPN{ (fracZZ + dSpotPN) };

				img::Spot const evalNZ{ (fracZZ + dSpotNZ) };
				img::Spot const evalZZ{ (fracZZ + dSpotZZ) };
				img::Spot const evalPZ{ (fracZZ + dSpotPZ) };

				img::Spot const evalNP{ (fracZZ + dSpotNP) };
				img::Spot const evalZP{ (fracZZ + dSpotZP) };
				img::Spot const evalPP{ (fracZZ + dSpotPP) };
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


} // [ops]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ops::AdderAD const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::ops::AdderAD const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

