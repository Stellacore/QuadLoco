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
 * \brief Top level file for quadloco::NM namespace
 *
 */


#include "cast.hpp"
#include "imgEdgel.hpp"
#include "imgGrad.hpp"
#include "opsfilter.hpp"
#include "rasChipSpec.hpp"
#include "rasgrid.hpp"
#include "rasGrid.hpp"
#include "raskernel.hpp"
#include "rasRowCol.hpp"
#include "rasSizeHW.hpp"

#include <algorithm>
#include <array>
#include <iterator>
#include <limits>
#include <numbers>
#include <vector>


namespace quadloco
{

/*! \brief Namespaced functions and utilities for quadloco::ops::EdgelGetter
 */
namespace ops
{

namespace grid
{
	/*! \brief Compute img::Grad for each pixel within specified ranges.
	 *
	 * For each pixel within specified row/col ranges, the gradient is
	 * computed from the 8 immediate neighbor pixels as a least squares
	 * linear fit. I.e. like fitting a geometric plane to the radiometric
	 * "surface" defined by the 9 neighborhood values.  The 4 corner
	 * samples are weighted by 1/sqrt(2) the weight of the 4 edge samples.
	 *
	 * Results are placed directly into an *externally allocated* output
	 * grid via the pointer ptGrads.
	 *
	 * NOTE: This function *assumes* the row/col begin/end ranges are
	 * valid and fall *entirely* inside the source image.
	 *
	 * The gradient computation may be expressed as a digital window with
	 * filter weights as follows.
	 *
	 * \verbatim
	 * ir2 = 1. / sqrt(2.);  // inverse root 2
	 * scl = 1. / (4.*ir2 + 2.*1.)
	 *
	 *                [ -ir2  -1.  -ir2 ]
	 * g[row] = scl * [   x    x     x  ]
	 *                [  ir2   1.   ir2 ]
	 *
	 *                [ -ir2   x    ir2 ]
	 * g[col] = scl * [  -1.   x     1. ]
	 *                [ -ir2   x    ir2 ]
	 *
	 * \endverbatim
	 */
	inline
	void
	fillGradientBy8x
		( ras::Grid<img::Grad> * const & ptGrads
		, ras::Grid<float> const & inGrid
		, ras::ChipSpec const & chipSpec
		)
	{
		ras::Grid<img::Grad> & grads = *ptGrads;

		std::size_t const rowNdxBeg{ chipSpec.srcRowBeg() };
		std::size_t const rowNdxEnd{ chipSpec.srcRowEnd() };
		std::size_t const colNdxBeg{ chipSpec.srcColBeg() };
		std::size_t const colNdxEnd{ chipSpec.srcColEnd() };

		for (std::size_t row{rowNdxBeg} ; row < rowNdxEnd ; ++row)
		{
			std::size_t const rowT{ row - 1u };
			std::size_t const & rowM = row;
			std::size_t const rowB{ row + 1u };

			for (std::size_t col{colNdxBeg} ; col < colNdxEnd ; ++col)
			{
				std::size_t const colL{ col - 1u };
				std::size_t const & colM = col;
				std::size_t const colR{ col + 1u };

				double const & TL = inGrid(rowT, colL);
				double const & TM = inGrid(rowT, colM);
				double const & TR = inGrid(rowT, colR);

				double const & ML = inGrid(rowM, colL);
				//double const & MM = inGrid(rowM, colM);
				double const & MR = inGrid(rowM, colR);

				double const & BL = inGrid(rowB, colL);
				double const & BM = inGrid(rowB, colM);
				double const & BR = inGrid(rowB, colR);

				// corner and edge filter values and (normalized) weights
				constexpr double fC{ 1./std::numbers::sqrt2_v<double> };
				constexpr double fE{ 1. };
				constexpr double wSum{ 4.*fC + 2.*fE };
				constexpr double wC{ fC / wSum };
				constexpr double wE{ fE / wSum };

				double const rowGrad
					{ wC * (BL - TL)
					+ wE * (BM - TM)
					+ wC * (BR - TR)
					};

				double const colGrad
					{ wC * (TR - TL)
					+ wE * (MR - ML)
					+ wC * (BR - BL)
					};

				grads(row,col) = img::Grad{ rowGrad, colGrad };
			}
		}
	}

	/*! \brief Gradient sub grid corresponding to ChipSpec from source Grid
	 *
	 * An edge gradient response is computed for every inGrid pixel that
	 * is within chipSpec.  The results - a smaller grid of size
	 * chipSpec.hwSize() - are copied into the return grid.
	 *
	 * The return grid is the same size as chipSpec and contains the
	 * edge response from the chipSpec-defined area within inGrid.
	 */
	inline
	ras::Grid<img::Grad>
	gradientSubGridBy8x
		( ras::Grid<float> const & inGrid
		, ras::ChipSpec const & chipSpec
		)
	{
		ras::Grid<img::Grad> grads;
		if (chipSpec.fitsInto(inGrid.hwSize()))
		{
			// allocate return structure
			ras::Grid<img::Grad> tmpFull{ inGrid.hwSize() };
			// no need to init (since unset values cropped away below)

			// compute gradients
			fillGradientBy8x(&tmpFull, inGrid, chipSpec);
			grads = ras::grid::subGridValuesFrom(tmpFull, chipSpec);
		}
		return grads;
	}


	/*! \brief Compute img::Grad for each pixel location (except at edges).
	 *
	 * This function determines appropriate row/col range limits and then
	 * runs fillGradientBy8x() for computation within the valid source
	 * image sub area.
	 */
	inline
	ras::Grid<img::Grad>
	gradientGridBy8x
		( ras::Grid<float> const & inGrid
		)
	{
		ras::Grid<img::Grad> grads;

		// check if there is enough room to process anything
		std::size_t const stepHalf{ 1u };
		std::size_t const stepFull{ 2u * stepHalf };
		if ((stepFull < inGrid.high()) && (stepFull < inGrid.wide()))
		{
			// allocate space
			ras::SizeHW const hwSize{ inGrid.hwSize() };
			grads = ras::Grid<img::Grad>(hwSize);

			// set border to null values
			static img::Grad const gNull{};
			ras::grid::fillBorder
				(grads.begin(), grads.hwSize(), stepHalf, gNull);

			//! determine start and end indices
			ras::ChipSpec const chipSpec
				{ ras::RowCol
					{ stepHalf
					, stepHalf
					}
				, ras::SizeHW
					{ (hwSize.high() - stepFull)
					, (hwSize.wide() - stepFull)
					}
				};

			// compute gradients
			fillGradientBy8x(&grads, inGrid, chipSpec);
		}

		return grads;
	}


	/*! \brief Compute img::Grad for each pixel location (except at edges).
	 *
	 * The stepHalf determine how wide an increment is used to estimate
	 * the edge in each direction. E.g., for a one dimensional signal
	 * array 'gVal', the gradient at location ndx is computed as:
	 * \verbatim
	 * grad1D = (gVal[ndx+stepHalf] - gVal[ndx-stepHalf]) / (2.*stepHalf)
	 * \endverbatim
	 *
	 * For 2D grid, the img::Grad components are computed similarly for
	 * each direction. Note that this is NOT a classic window computation
	 * but rather two indpenendent evaluations done in each index. E.g.,
	 * at location (row,col), the computed gradient is:
	 * \verbatim
	 * del = 2. * (double)stepHalf;
	 * gradValue[0] = (gVal(row+stepHalf, col) - gval(row-stepHalf,col)) / del
	 * gradValue[1] = (gVal(row, col+stepHalf) - gval(row,col-stepHalf)) / del
	 * \endverbatim
	 *
	 * NOTE: the stepHalf pixels around the boarder are set to null!!
	 */
	inline
	ras::Grid<img::Grad>
	gradientGridBy4x
		( ras::Grid<float> const & inGrid
		, std::size_t const & stepHalf = 1u
		)
	{
		ras::Grid<img::Grad> grads;

		// check if there is enough room to process anything
		std::size_t const stepFull{ 2u * stepHalf };
		if ((stepFull < inGrid.high()) && (stepFull < inGrid.wide()))
		{
			// allocate space
			ras::SizeHW const hwSize{ inGrid.hwSize() };
			grads = ras::Grid<img::Grad>(hwSize);

			//! Determine start and end indices
			std::size_t const rowNdxBeg{ stepHalf };
			std::size_t const rowNdxEnd{ rowNdxBeg + hwSize.high() - stepFull };
			std::size_t const colNdxBeg{ stepHalf };
			std::size_t const colNdxEnd{ colNdxBeg + hwSize.wide() - stepFull };

			// set border to null values
			static img::Grad const gNull{};
			ras::grid::fillBorder
				(grads.begin(), grads.hwSize(), stepHalf, gNull);

			float const scl{ 1.f / (float)stepFull };
			for (std::size_t row{rowNdxBeg} ; row < rowNdxEnd ; ++row)
			{
				std::size_t const rowM1{ row - stepHalf };
				std::size_t const rowP1{ row + stepHalf };
				for (std::size_t col{colNdxBeg} ; col < colNdxEnd ; ++col)
				{
					std::size_t const colM1{ col - stepHalf };
					std::size_t const colP1{ col + stepHalf };
					float const rowGrad
						{ scl * (inGrid(rowP1, col) - inGrid(rowM1, col)) };
					float const colGrad
						{ scl * (inGrid(row, colP1) - inGrid(row, colM1)) };

					grads(row,col) = img::Grad{ rowGrad, colGrad };
				}
			}
		}

		return grads;
	}


	//! Collection of all edgels associated with non-zero gradients.
	inline
	std::vector<img::Edgel>
	allEdgelsFrom
		( ras::Grid<img::Grad> const & gradGrid
			//!< Gradient grid from which to extract edgels (non-zero grads)
		)
	{
		std::vector<img::Edgel> pixEdgels{};
		pixEdgels.reserve(gradGrid.size());

		// could skip unset edge rows
		for (std::size_t row{0u} ; row < gradGrid.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < gradGrid.wide() ; ++col)
			{
				img::Grad const & grad = gradGrid(row, col);
				if (grad.isValid())
				{
					constexpr double tol
						{ std::numeric_limits<double>::epsilon() };
					if (tol < magnitude(grad))
					{
						// should be valid since grad is significant
						img::Edgel const edgel(img::Spot{ row, col }, grad);
						pixEdgels.push_back(edgel);
					}
				}
			}
		}

		return pixEdgels;
	}

	//! Functor for checking if a (row,col) location is inside padded boundary
	struct Inside
	{
		std::size_t theBegRow{ 0u };
		std::size_t theBegCol{ 0u };
		std::size_t theEndRow{ 0u };
		std::size_t theEndCol{ 0u };

		//! Construct to test inside hwSize within 'pad' cell border all around
		inline
		explicit
		Inside
			( ras::SizeHW const & hwSize
			, std::size_t const & pad
			)
		{
			bool const highNuf{ (2u*pad) < hwSize.high() };
			bool const wideNuf{ (2u*pad) < hwSize.wide() };
			if (highNuf && wideNuf)
			{
				theBegRow = pad;
				theBegCol = pad;
				theEndRow = hwSize.high() - pad;
				theEndCol = hwSize.wide() - pad;
			}
		}

		//! True if row,col is inside hwSize accounting for border pad border
		inline
		bool
		operator()
			( std::size_t const & row
			, std::size_t const & col
			) const
		{
			return
				(  (theBegRow < row) && (row < theEndRow)
				&& (theBegCol < col) && (col < theEndCol)
				);
		}

	};

	//! Collection edgels that have corroborating neighbors
	inline
	std::vector<img::Edgel>
	linkedEdgelsFrom
		( ras::Grid<img::Grad> const & gradGrid
			//!< Gradient grid from which to extract edgels (non-zero grads)
		, double const & supportMultiplier = 2.5
			//!< Degree of support (1: self only, 2: one other, etc)
		)
	{
		std::vector<img::Edgel> pixEdgels{};
		std::size_t const numElem{ gradGrid.size() };
		if (0u < numElem)
		{
			pixEdgels.reserve(numElem);

			// testing
			Inside const inFull(gradGrid.hwSize(), 2u);
			Inside const inEdge(gradGrid.hwSize(), 1u);

			std::size_t const rowLast{ gradGrid.high() - 1u };
			std::size_t const colLast{ gradGrid.wide() - 1u };

			// leave room to compute with neighbors
			// could skip unset edge rows
			for (std::size_t row{1u} ; row < rowLast ; ++row)
			{
				for (std::size_t col{1u} ; col < colLast ; ++col)
				{
					// gradient at center
					img::Grad const & gradCenter = gradGrid(row, col);
					if (gradCenter.isValid())
					{
						double const gMag{ magnitude(gradCenter) };
						constexpr double tol
							{ std::numeric_limits<double>::epsilon() };
						if (tol < gMag)
						{
							std::array<img::Grad const *, 8u> ptHoods
								{ &(gradGrid(row-1u, col-1u))
								, &(gradGrid(row-1u, col   ))
								, &(gradGrid(row-1u, col+1u))
								, &(gradGrid(row   , col-1u))
								//&(gradGrid(row   , col   ))
								, &(gradGrid(row   , col+1u))
								, &(gradGrid(row+1u, col-1u))
								, &(gradGrid(row+1u, col   ))
								, &(gradGrid(row+1u, col+1u))
								};
							img::Vector<double> hoodSum{ 0., 0. };
							double count{ 0. };
							for (img::Grad const * const & ptHood : ptHoods)
							{
								img::Grad const & hoodGrad = *ptHood;
								if (isValid(hoodGrad))
								{
									hoodSum = hoodSum + hoodGrad;
									count += 1.;
								}
							}

							// check for at least one valid 8-neighbor
							if (0. < count)
							{
								double const scl
									{ (8./count) // normalize to full hood size
									* (1./gMag) // unitize gradCenter in dot()
									};
								double const sumProj
									{ scl * dot(hoodSum, gradCenter) };
								double const hoodRatio{ sumProj / gMag };
								if (supportMultiplier < hoodRatio)
								{
									// assured to be valid at this point since
									// only processing non-trivial gradients
									// i.e., tol < magnitude(gradCenter) above
									img::Edgel const edgel
										(img::Spot{ row, col }, gradCenter);
									pixEdgels.push_back(edgel);
								}
							}
						}
					}
				}
			}
		}

		return pixEdgels;
	}

	//! Populate a grid with edgels magnitude data (only at edgel locations)
	inline
	ras::Grid<float>
	edgeMagGridFor
		( ras::SizeHW const & hwSize
		, std::vector<img::Edgel> const & edgels
		, std::size_t const & numToUse
		)
	{
		ras::Grid<float> grid(hwSize);
		constexpr float nan{ std::numeric_limits<float>::quiet_NaN() };
		std::fill(grid.begin(), grid.end(), nan);
		for (std::size_t nn{0u} ; nn < numToUse ; ++nn)
		{
			img::Edgel const & edgel = edgels[nn];
			if (! isValid(edgel))
			{
				std::cerr << "ERROR got invalid edgel were not expecting it\n";
				exit(9);
			}
			ras::RowCol const rc{ cast::rasRowCol(edgel.location()) };
			grid(rc) = edgel.magnitude();
		}

		return grid;
	}

	//! Populate a grid with edgels angles data (only at edgel locations)
	inline
	ras::Grid<float>
	edgeAngleGridFor
		( ras::SizeHW const & hwSize
		, std::vector<img::Edgel> const & edgels
		, std::size_t const & numToUse
		, float const & backgroundBias = -5.f
		)
	{
		ras::Grid<float> grid(hwSize);
		// angle values range from [-pi, pi), so start with a background
		// bias so that angle values are better distinguished.
		std::fill(grid.begin(), grid.end(), backgroundBias);
		for (std::size_t nn{0u} ; nn < numToUse ; ++nn)
		{
			img::Edgel const & edgel = edgels[nn];
			if (! isValid(edgel))
			{
				std::cerr << "ERROR got invalid edgel were not expecting it\n";
				exit(9);
			}
			ras::RowCol const rc{ cast::rasRowCol(edgel.location()) };
			double const angle{ edgel.angle() };
			grid(rc) = (float)angle;
		}

		return grid;
	}

	/*! \brief Apply func within hwBox moving across srcGrid
	 *
	 * A moving window, of size hwBox, moves across srcGrid. Every cell
	 * in srcGrid is visted other than the first/last hwBox.{high,wide}()
	 * rows/columns around the boarder (which are set to NaN in output).
	 *
	 * At each input cell in the source grid, boxFunc.reset() is called
	 * to allow function to reset behavior based on the new current
	 * source image cell values. Then boxFunc, is provided with assess to
	 * srcGrid information (via a call to boxFunc.consider() for every
	 * srcGrid cell in the hwBox window around current source cell. After
	 * calling consider() for every cell in hwBox, the evaluation function,
	 * boxFunc(), is called and the result is assigned to the output grid
	 * cell (at the same row,col location as the srcGrid cell then
	 * being evaluated).
	 *
	 * \note
	 * boxFunc must provide the following methods:
	 * \arg boxFunc{}; // default ctor
	 * \arg boxFunc.reset(srcValue)
	 * \arg boxFunc.consider(srcValue, boxRow, boxCol);
	 * \arg boxFunc();
	 *
	 * Example
	 * \snippet include/opsfilter.hpp DoxyExampleBoxFunc
	 */
	template <typename OutType, typename SrcType, typename BoxFunctor>
	inline
	ras::Grid<OutType>
	functionResponse
		( ras::Grid<SrcType> const & srcGrid
		, ras::SizeHW const & hwBox
		, BoxFunctor & boxFunc
		)
	{
		ras::Grid<OutType> outGrid;
		auto const isOdd
			{ [] (std::size_t const & val) { return (1 == val % 2u); } };

		// Are all conditions satisfied for (easy) filtering computation
		if ( srcGrid.isValid()
		  && hwBox.isValid()
		  && isOdd(hwBox.high())
		  && isOdd(hwBox.wide())
		  && ((hwBox.high() + 1u) < srcGrid.high())
		  && ((hwBox.wide() + 1u) < srcGrid.wide())
		   )
		{
			outGrid = ras::Grid<OutType>(srcGrid.hwSize());
			constexpr OutType nanOut
				{ std::numeric_limits<double>::quiet_NaN() };
			// TBD - maybe change to set explicitly only the edge values
			std::fill(outGrid.begin(), outGrid.end(), nanOut);

			int const halfHigh{ static_cast<int>(hwBox.high() / 2u) };
			int const halfWide{ static_cast<int>(hwBox.wide() / 2u) };

			int const wHigh{ (int)hwBox.high() };
			int const wWide{ (int)hwBox.wide() };

			// loop over active area of source/output grids
			int const srcRowEnd{ static_cast<int>(srcGrid.high()) - halfHigh };
			int const srcColEnd{ static_cast<int>(srcGrid.wide()) - halfWide };
			for (int srcRow{halfHigh} ; srcRow < srcRowEnd ; ++srcRow)
			{
				int const srcRow0{ srcRow - halfHigh };
				for (int srcCol{halfWide} ; srcCol < srcColEnd ; ++srcCol)
				{
					int const srcCol0{ srcCol - halfWide };

					OutType outVal{ nanOut };

					// reset filter to new source position
					SrcType const & refVal = srcGrid(srcRow, srcCol);
					if (engabra::g3::isValid(refVal))
					{
						boxFunc.reset(refVal);

						// integrate values over weighted window
						for (int wRow{0} ; wRow < wHigh ; ++wRow)
						{
							int const inRow{ srcRow0 + wRow };
							for (int wCol{0} ; wCol < wWide ; ++wCol)
							{
								int const inCol{ srcCol0 + wCol };

								// have functor consider this value
								SrcType const & srcVal = srcGrid(inRow, inCol);
								if (engabra::g3::isValid(srcVal))
								{
									boxFunc.consider
										( srcVal
										, static_cast<std::size_t>(wRow)
										, static_cast<std::size_t>(wCol)
										);
								}
								else
								{
									// upon encountering a null,
									// abandon entire window/box processing
									goto NextWindow;
								}
							}
						}

						outVal = boxFunc();
					}
					NextWindow:

					// update evolving return storage
					outGrid(srcRow, srcCol) = outVal;
				}
			}
		} // good inputs

		return outGrid;
	}

	//! \brief Result of running filter window over srcGrid.
	template <typename OutType, typename SrcType>
	inline
	ras::Grid<OutType>
	filtered
		( ras::Grid<SrcType> const & srcGrid
		, ras::Grid<OutType> const & filter
		)
	{
		ops::filter::WeightedSum<OutType, SrcType> bFunc{ &filter };
		return functionResponse
			<OutType, SrcType>
			(srcGrid, filter.hwSize(), bFunc);
	}

	//! \brief Result of a sum-square difference filter
	template <typename OutType, typename SrcType>
	inline
	ras::Grid<OutType>
	smoothGridFor
		( ras::Grid<SrcType> const & srcGrid
			//!< Input data
		, std::size_t const & halfSize
			//!< Halfsize for moving window
		, double const & sigma
			//!< Standard deviation of Gaussian to use for smoothing
		)
	{
		ras::Grid<OutType> const filter
			{ ras::kernel::gauss<OutType>(halfSize, sigma) };
		ops::filter::WeightedSum<OutType, SrcType> bFunc{ &filter };
		return functionResponse
			<OutType, SrcType>
			(srcGrid, filter.hwSize(), bFunc);
	}

	//! \brief Result of a sum-square difference filter
	template <typename OutType, typename SrcType>
	inline
	ras::Grid<OutType>
	sumSquareDiffGridFor
		( ras::Grid<SrcType> const & srcGrid
			//!< Input data
		, ras::SizeHW const & hwBox
			//!< Size of moving window
		)
	{
		ops::filter::SumSquareDiff<OutType> bFunc{};
		return functionResponse
			<OutType, SrcType>
			(srcGrid, hwBox, bFunc);
	}


} // grid

} // [ops]

} // [quadloco]

