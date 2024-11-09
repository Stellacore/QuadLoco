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
 * \brief Declaration of quadloco::prb::Quadness
 *
 */


#include "datGrid.hpp"
#include "datSpot.hpp"
#include "imgQuadTarget.hpp"
#include "pix.hpp"

#include <algorithm>
#include <limits>


namespace quadloco
{

/*! \brief Probability function namespace.
 */
namespace prb
{
	using Edge = std::array<double, 2u>;
	constexpr Edge sNullEdge
		{ std::numeric_limits<double>::quiet_NaN()
		, std::numeric_limits<double>::quiet_NaN()
		};


	//! TODO
	struct QuadStats
	{
		dat::Grid<Edge> const theEdgeGrads{};
		dat::Grid<double> const theEdgeMags{};
		dat::Span const theEdgeMagSpan{};

		inline
		static
		dat::Grid<double>
		edgeMagsFor
			( dat::Grid<Edge> const & grads
			)
		{
			dat::Grid<double> mags(grads.hwSize());
			using InIter = dat::Grid<Edge>::const_iterator;
			using OutIter = dat::Grid<double>::iterator;
			InIter inIter{ grads.cbegin() };
			for (OutIter outIter{mags.begin()} ; mags.end() != outIter
				; ++outIter, ++inIter)
			{
				Edge const & edge = *inIter;
				*outIter = std::hypot(edge[0], edge[1]);
			}
			return mags;
		}

		inline
		static
		dat::Grid<Edge>
		edgeGradsFor
			( dat::Grid<float> const & pixGrid
			)
		{
			dat::Grid<Edge> edgeGrid{};

			// need at least one useful pixel after eliminating edges
			if ((3u < pixGrid.high()) && (3u < pixGrid.wide()))
			{
				// allocate space
				edgeGrid = dat::Grid<Edge>(pixGrid.hwSize());

				// TODO - could be optimized to just set edges
				std::fill(edgeGrid.begin(), edgeGrid.end(), sNullEdge);

				// TODO - could be optimized
				std::size_t const rowEnd{ edgeGrid.high() - 1u };
				std::size_t const colEnd{ edgeGrid.wide() - 1u };
				for (std::size_t row{1u} ; row < rowEnd ; ++row)
				{
					for (std::size_t col{1u} ; col < colEnd ; ++col)
					{
						float const & pixTL = pixGrid(row-1u, col-1u);
						float const & pixTM = pixGrid(row-1u, col+0u);
						float const & pixTR = pixGrid(row-1u, col+1u);

						float const & pixML = pixGrid(row+0u, col-1u);
					//	float const & pixMM = pixGrid(row+0u, col+0u);
						float const & pixMR = pixGrid(row+0u, col+1u);

						float const & pixBL = pixGrid(row+1u, col-1u);
						float const & pixBM = pixGrid(row+1u, col+0u);
						float const & pixBR = pixGrid(row+1u, col+1u);

						// weight twice the differences across two pixel spans
						double const dRow
							{ .25 * (double)(pixBL - pixTL)
							+ .50 * (double)(pixBM - pixTM)
							+ .25 * (double)(pixBR - pixTR)
							};
						double const dCol
							{ .25 * (double)(pixTR - pixTL)
							+ .50 * (double)(pixMR - pixML)
							+ .25 * (double)(pixBR - pixBL)
							};
						// compute gradient normalized to one pixel spans
						Edge const edge{ .5 * dRow, .5 * dCol };

						edgeGrid(row, col) = edge;
					}
				}
			}

			return edgeGrid;
		}

		/*
		//! Generate stats from pixGrid
		inline
		static
		QuadStats
		from
			( dat::Grid<float> const & pixGrid
			)
		{
			return QuadStats{ edgeGradsFor(pixGrid), ... };
		}
		*/

		//! Construct stats for pixGrid data
		inline
		explicit
		QuadStats
			( dat::Grid<float> const & pixGrid
			)
			: theEdgeGrads{ edgeGradsFor(pixGrid) }
			, theEdgeMags{ edgeMagsFor(theEdgeGrads) }
			, theEdgeMagSpan{ pix::fullSpanFor(theEdgeMags) }
		{ }

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
			oss
				<< "  theEdgeGrads: " << theEdgeGrads
				<< '\n'
				<< "   theEdgeMags: " << theEdgeMags
				<< '\n'
				<< "theEdgeMagSpan: " << theEdgeMagSpan
				<< '\n';

			return oss.str();
		}


	}; // QuadStats

} // [prb]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::prb::QuadStats const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	/*
	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::prb::QuadStats const & item
		)
	{
		return item.isValid();
	}
	*/

} // [anon/global]


namespace quadloco
{

/*! \brief Probability function namespace.
 */
namespace prb
{

	//! Assessor to provide scores of pixel image likeness to a quad target
	struct Quadness
	{
		//! Detection parameters of suspected quad target image
		img::QuadTarget const theImgQuad{};


		//! True if this instance contains valid data
		inline
		bool
		isValid
			() const
		{
			return theImgQuad.isValid();
		}

		//! True if image is most likely a quad target
		inline
		bool
		isLikelyQuad
			( dat::Grid<float> const & pixGrid
				//!< Pixels maybe containing quad target (perspective)image
			, double const & confidenceFrac = .90
				//!< Confidence threshold to apply to decision
			) const
		{
			QuadStats const stats(pixGrid);

std::cout << '\n';
std::cout << stats.theEdgeGrads.infoStringContents
	("edgeGrid", "(%5.2f,%5.2f)")
	<< '\n';
std::cout << stats.theEdgeMags.infoStringContents
	("edgeMags", " %5.2f")
	<< '\n';
std::cout << '\n';
std::cout << "stats:\n" << stats << '\n';

//			hasBackFoldSymmetry(theImgQuad, pixGrid)
//			hasForeFoldSymmetry(theImgQuad, pixGrid)

			std::cout << "pixGrid: " << pixGrid << '\n';
			std::cout << "confidenceFrac: " << confidenceFrac << '\n';
			return {};//TODO
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
				<< "imgQuad: " << theImgQuad
				;

			return oss.str();
		}


	};


} // [prb]

} // [quadloco]


namespace
{

	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::prb::Quadness const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::prb::Quadness const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

