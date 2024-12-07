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
 * \brief Top level file for quadloco::sig::EdgeEval
 *
 */


#include "sigEdgeGrouper.hpp"
#include "sigCenterFitter.hpp"
#include "sigEdgeLine.hpp"
#include "sigItemWgt.hpp"

#include "angLikely.hpp"
#include "imgArea.hpp"
#include "opsgrid.hpp"
#include "rasGrid.hpp"
#include "sigEdgeInfo.hpp"
#include "sigQuadTarget.hpp"

#include <algorithm>
#include <array>
#include <vector>


namespace quadloco
{

namespace sig
{
	//! Attenuation power on dot product
	constexpr double sCosPower{ 10. };
	//! Multiply threshold for neighborhood gradient agreement
	constexpr double sLinkEdgeDist{ 2.50 };
	//! Use to estimate expected maximum number of strong edgels
	constexpr std::size_t sDiagMultiple{ 6u };
	//! Number of bins to use in angle direction peak detection
	constexpr std::size_t sNumAngleBins{ 32u };
	//! Expected minimum proximity of two adjacent radial edge rays
	constexpr double sRaySeparation{ 2.5 };
	//! Maximum misclosure distance between spot and associated edge line
	constexpr double sEdgeMissMax{ 2. };
	//! Relative uncertainty in opposing edge line angle
	constexpr double sEdgeLineAngleSigma{ 1./2. }; // about +/-30 degrees


	//! Clarification that ray is associated with raster edge gradient
	using EdgeRay = img::Ray;


	//! Candidate center point
	inline
	static
	SpotWgt
	intersectionOf
		( RayWgt const & rw1
		, RayWgt const & rw2
		)
	{
		CenterFitter fitter;
		fitter.addRay(rw1.item(), rw1.weight());
		fitter.addRay(rw2.item(), rw2.weight());
		return fitter.solutionSpotWeight();
	}



	//! \brief Table of probabilities for spot(row) and ray(col) combinations.
	class SpotRayProbs
	{
		//! (Pseudo)Probability of agreeemt between spots(row) and rays(col).
		ras::Grid<double> theProbTab;

		//! Compute pseudo-probability that edge ray is consistent with spot
		inline
		static
		ras::Grid<double>
		probTableFor
			( std::vector<SpotWgt> const & spotWgts
			, std::vector<RayWgt> const & rayWgts
			)
		{
			ras::Grid<double> probTab(spotWgts.size(), rayWgts.size());
			std::fill(probTab.begin(), probTab.end(), 0.);

			std::size_t const numSW{ spotWgts.size() };
			std::size_t const numRWs{ rayWgts.size() };

			// assess how well each spot agrees with all edge rays
			for (std::size_t ndxSW{0u} ; ndxSW < numSW ; ++ndxSW)
			{
				// access input spot data
				SpotWgt const & spotWgt = spotWgts[ndxSW];
				img::Spot const & spot = spotWgt.item();

				// consider all edge rays in "voting" on quality of the
				// candidate center spot location.
				for (std::size_t ndxRW{0u} ; ndxRW < numRWs ; ++ndxRW)
				{
					// access ray data
					RayWgt const & rayWgt = rayWgts[ndxRW];
					img::Ray const & ray = rayWgt.item();

					// determine collinearity gap (aka 'rejection') of
					// spot from current ray (misclosure distance)
					double const edgeMiss{ ray.distanceAlong(spot) };

					// assign pseudo-probability of collinearity
					double const arg{ (edgeMiss / 1.) };
					double const probCollin{ std::exp(-arg*arg) };

					probTab(ndxSW, ndxRW) = probCollin;
				}
			}
			return probTab;
		}

	public:

		//! Allocate and zero probability table.
		inline
		explicit
		SpotRayProbs
			( std::vector<SpotWgt> const & spotWgts
			, std::vector<RayWgt> const & rayWgts
			)
			: theProbTab{ probTableFor(spotWgts, rayWgts) }
		{ }

		//! rayNdx and prob weights for spotNdx (sorted: highest probs first)
		inline
		std::vector<NdxWgt>
		rayNdxWgtsFor
			( std::size_t const & spotNdx
			) const
		{
			std::vector<NdxWgt> rws;
			std::size_t const numRays{ theProbTab.wide() };
			for (std::size_t rayNdx{0u} ; rayNdx < numRays ; ++rayNdx)
			{
				double const & prob = theProbTab(spotNdx, rayNdx);
				NdxWgt const rw{ rayNdx, prob };
				rws.emplace_back(rw);
			}
			std::sort
				( rws.begin(), rws.end()
				, [] (NdxWgt const & nw1, NdxWgt const & nw2)
					// reverse compare order to put high probs at front
					{ return (nw2.weight() < nw1.weight()); }
				);
			return rws;
		}

	};


	//! \brief Evaluator of edgels that are likely part of quad target image
	class EdgeEval
	{
		std::vector<sig::EdgeInfo> const theEdgeInfos;

	// static functions

		//! Extract the largest magnitude edgels from grid
		inline
		static
		std::vector<img::Edgel>
		dominantEdgelsFrom
			( ras::Grid<img::Grad> const & gradGrid
			, double const & linkEdgeDist = sLinkEdgeDist
			, std::size_t const & diagMultiple = sDiagMultiple
			)
		{
			// get all strongly linked edgels
			std::vector<img::Edgel> edgels
				{ ops::grid::linkedEdgelsFrom(gradGrid, linkEdgeDist) };

			// conservative estimate of how many to search knowing that
			// a quad target should be consuming large part of grid
			double const diag{ gradGrid.hwSize().diagonal() };
			std::size_t const estNumToUse
				{ static_cast<std::size_t>((double)diagMultiple * diag) };
			std::size_t const numToUse{ std::min(estNumToUse, edgels.size()) };

			// move strongest edges near front
			std::partial_sort
				( edgels.begin(), edgels.begin()+numToUse
				, edgels.end()
				, [] (img::Edgel const & e1, img::Edgel const & e2)
					// sort in *desending* order
					{ return (e2.magnitude() < e1.magnitude()); }
				);

			// ignore less significant edges in return structure
			edgels.resize(numToUse);
			return edgels;
		}

		//! Determine (*combinatorially*) edgel radial edge (pseudo)likelihood
		inline
		static
		std::vector<sig::EdgeInfo>
		edgeInfosLikelyRadial
			( std::vector<img::Edgel> const & edgels
			)
		{
			// Individual edge properties
			std::vector<sig::EdgeInfo> edgeInfos;
			std::size_t const numElem{ edgels.size() };
			if (2u < numElem)
			{
				edgeInfos.reserve(numElem);

				// compute useful edgel properties suggesting
				// edgels likely to be on opposite radial edges
				for (std::size_t ndx1{0u} ; ndx1 < numElem ; ++ndx1)
				{
					// construct EdgeInfo tracking instance
					img::Edgel const & edgel = edgels[ndx1];
					edgeInfos.emplace_back(sig::EdgeInfo(edgel));

					// update just inserted edgel with prior edgels (below)
					sig::EdgeInfo & edgeInfo1 = edgeInfos.back();

					// compare from begin thru just *before* last item added
					std::size_t const numPrev{ edgeInfos.size() - 1u };
					for (std::size_t ndx2{0u} ; ndx2 < numPrev ; ++ndx2)
					{
						// combinatorially consider other edges and
						// update each with consideration of the other
						sig::EdgeInfo & edgeInfo2 = edgeInfos[ndx2];
						edgeInfo1.consider(edgeInfo2.edgel());
						edgeInfo2.consider(edgeInfo1.edgel());
					}
				}
			}

			// filter very low weight edges
// TODO - determine what is a low weight
			return edgeInfos;
		}

	public:

		// Process gradients into non-isolated edges
		inline
		explicit
		EdgeEval
			( ras::Grid<img::Grad> const & gradGrid
			)
			: theEdgeInfos
				{ edgeInfosLikelyRadial	
					(dominantEdgelsFrom(gradGrid))
				}
		{ }

		// TODO - should go someplace else - EdgeInfo functions maybe?
		//! Significant edgeInfo organized into a grid
		inline
		ras::Grid<float>
		edgeInfoGrid
			( ras::SizeHW const & hwSize
			) const
		{
			ras::Grid<float> eiGrid(hwSize);
			std::fill(eiGrid.begin(), eiGrid.end(), 0.f);
			for (sig::EdgeInfo const & edgeInfo : theEdgeInfos)
			{
				ras::RowCol const rowcol
					{ cast::rasRowCol(edgeInfo.edgel().start()) };
				eiGrid(rowcol) = (float)edgeInfo.consideredWeight();
			}
			return eiGrid;
		}

		//! Collection of edge elements being used for evaluation
		inline
		std::vector<img::Edgel>
		edgelsInUse
			() const
		{
			std::vector<img::Edgel> edgels;
			for (sig::EdgeInfo const & edgeInfo : theEdgeInfos)
			{
				edgels.emplace_back(edgeInfo.edgel());
			}
			return edgels;
		}

		/*! \brief Pseudo-probability that edge ray spots are NOT colocated.
		 *
		 * Spot locations should be separated by a couple pixels
		 * due to the multi-pixel span of gradient computation. (In practice
		 * resolution of the image will be a factor, but that's likely
		 * less than or comparable to gradient computation span).
		 *
		 * Consider two grid cells separated by one empty cell, the
		 * distance between them is 2 in a cardinal direction and 2.8
		 * in diagonal one. Therefore, one might expect a separation
		 * of less than this to be an artifact. E.g. less than 2-2.8
		 * is probably insignificant.
		 *
		 * Approaching another way, for a highly oblique target, two
		 * (oppositely directed) adjacent edge rays, could be separated
		 * by order of 2 or 2.8 pixels which should be considered
		 * significant. E.g. more than 2-2.8 is probably significant.
		 *
		 * Therefore, it's probably reasonable to associate a value of
		 * maybe 2.5 as a 50/50 proposition.  E.g. .50=exp(-arg^2)=P(arg)
		 * suggesting arg on the order of .83 or so.  For arg=delta/sigma
		 * this suggests sigma on the order of delta/.83. So e.g. for
		 * delta = 2-2.8, this suggests sigma value near 2.4 to 3.3
		 * (for using non-normalized pseudo-probability function, 'P(arg)').
		 */
		inline
		static
		double
		separationWeight
			( img::Ray const & ray1
			, img::Ray const & ray2
			, double const & sigma = sRaySeparation
			)
		{
			double const delta{ magnitude(ray2.start() - ray1.start()) };
			double const arg{ delta / sigma };
			double const wgtNear{ std::exp(-arg*arg) };
			double const wgtDistinct{ 1. - wgtNear };
			return wgtDistinct;
		}

		//! Spots (and weights) associated with pairwise edgeray intersections
		inline
		std::vector<SpotWgt>
		spotWeightsPairwise
			( std::vector<RayWgt> const & rayWgts
			, ras::SizeHW const & hwSize
			) const
		{
			std::vector<SpotWgt> spotWgts;
			img::Area const hwArea
				{ img::Span{ 0., static_cast<double>(hwSize.high())}
				, img::Span{ 0., static_cast<double>(hwSize.wide())}
				};

			// get edge rays likely associated with radial quad edges
			std::size_t const numEdges{ rayWgts.size() };
			std::size_t const & numUseRW = numEdges;

			// pair-wise intersection of all rays
			spotWgts.reserve((numUseRW * (numUseRW - 1u)) / 2u);
			for (std::size_t ndx1{0u} ; ndx1 < numUseRW ; ++ndx1)
			{
				RayWgt const & rw1 = rayWgts[ndx1]; // first ray
				for (std::size_t ndx2{ndx1+1u} ; ndx2 < numUseRW ; ++ndx2)
				{
					RayWgt const & rw2 = rayWgts[ndx2]; // second ray

					// intersect the two rays and check if intersection is...
					SpotWgt const tmpSpotWgt{ intersectionOf(rw1, rw2) };
					// ... defined
					if (tmpSpotWgt.isValid())
					{
						// ... within the hwSize shape (if one provided)
						if (hwArea.contains(tmpSpotWgt.item()))
						{
							// weight by geometric distinctiveness
							double const wgtDistinct
								{ separationWeight(rw1.item(), rw2.item()) };
							SpotWgt const useSpotWgt
								{ tmpSpotWgt.item()
								, wgtDistinct * tmpSpotWgt.theWeight
								};
							spotWgts.emplace_back(useSpotWgt);
						}
					}
				}
			}

			return spotWgts;
		}

		//! Update inSpotWgts weighting based on consensus of all edgerays
		inline
		std::vector<SpotWgt>
		spotWeightsConsensus
			( std::vector<SpotWgt> const & inSpotWgts
			, std::vector<RayWgt> const & rayWgts
			) const
		{
			std::vector<SpotWgt> spotWgts;
			spotWgts.reserve(inSpotWgts.size());

			SpotRayProbs const spotRayProbs(inSpotWgts, rayWgts);

			std::size_t const numSW{ inSpotWgts.size() };

			// assess how well each spot agrees with all edge rays
			for (std::size_t ndxSW{0u} ; ndxSW < numSW ; ++ndxSW)
			{
				// access input spot data
				SpotWgt const & inSpotWgt = inSpotWgts[ndxSW];
				img::Spot const & inSpot = inSpotWgt.item();
				double const & inWgt = inSpotWgt.weight();

				// get indices for most likely to be collinear rays
				std::vector<NdxWgt> const rayNdxWgts
					{ spotRayProbs.rayNdxWgtsFor(ndxSW) };

				// consider all edge rays in "voting" on quality of the
				// candidate center spot location.
				double voteTotal{ 0. };
				for (NdxWgt const & rayNdxWgt : rayNdxWgts)
				{
					std::size_t const & ndxRW = rayNdxWgt.item();
					double const & probCollin = rayNdxWgt.weight();

					double const & wgtRay = rayWgts[ndxRW].theWeight;

					// accumulate this pseudo probability into spot weight
					double const wgtVote{ wgtRay * probCollin };
					voteTotal += wgtVote;
				}

				// Create a return spot using vote total to modify prior weight
				double const updateWgt{ inWgt * voteTotal };
				spotWgts.emplace_back(SpotWgt{ inSpot, updateWgt });
			}
			return spotWgts;
		}

		//! Indices and weights for which rayWgts are most collinear with spot
		inline
		std::vector<NdxWgt>
		rayNdxWeights
			( img::Spot const & spotCandidate
			, double const & wgtCandidate
			, std::vector<RayWgt> const & rayWgts
			) const
		{
			std::vector<NdxWgt> colinRayNdxWgts;
			std::size_t const numRWs{ rayWgts.size() };
			colinRayNdxWgts.reserve(numRWs);

			// consider all edge rays in "voting" on quality of the
			// candidate center spot location.
			for (std::size_t ndxRW{0u} ; ndxRW < numRWs ; ++ndxRW)
			{
				// access ray data
				RayWgt const & rayWgt = rayWgts[ndxRW];
				img::Ray const & ray = rayWgt.item();
				double const & wgtRay = rayWgt.theWeight;

				// determine collinearity gap (aka 'rejection') of
				// spot from current ray (misclosure distance)
				double const edgeMiss{ ray.distanceAlong(spotCandidate) };

				if (edgeMiss < sEdgeMissMax)
				{
					// assign pseudo-probability of collinearity
					double const arg{ (edgeMiss / 1.) };
					double const probCollin{ std::exp(-arg*arg) };

					// pseudo probability for this spot solution
					double const wgt{ wgtCandidate * wgtRay * probCollin };

					// remember this ray index
					colinRayNdxWgts.emplace_back(NdxWgt{ ndxRW, wgt });
				}
			}

			return colinRayNdxWgts;
		}

		//! Spot/weights fit onto most collinear edge rays
		inline
		std::vector<QuadWgt>
		fitQuadWgtsFor
			( std::vector<QuadWgt> const & quadWgts
			, std::vector<RayWgt> const & rayWgts
			) const
		{
			std::vector<QuadWgt> fitQuadWgts;
			fitQuadWgts.reserve(quadWgts.size());
			for (QuadWgt const & quadWgt : quadWgts)
			{
				// start with nominal Quad signal (use to qualify rays)
				sig::QuadTarget const & srcQuad = quadWgt.item();
				double const & srcWgt = quadWgt.weight();

				// Determine ray weighting based on collinearity and
				// consistency with nominal srcQuad
				img::Spot const srcSpot{ srcQuad.centerSpot() };
				std::vector<NdxWgt> const rayNWs
					{ rayNdxWeights(srcSpot, srcWgt, rayWgts) };

				// require at least 4 radial edges for a quad target image
				if (3u < rayNWs.size())
				{
					CenterFitter fitter;
					for (NdxWgt const & rayNW : rayNWs)
					{
						std::size_t const & rayNdx = rayNW.item();
						double const & rayWgt = rayNW.weight();
						img::Ray const & ray = rayWgts[rayNdx].item();
						double const obsWgt{ rayWgt * srcWgt };
						fitter.addRay(ray, obsWgt);
					}

					SpotWgt const fitSpotWgt{ fitter.solutionSpotWeight() };
					sig::QuadTarget const fitSigQuad
						{ fitSpotWgt.item() // use fit center location
						, srcQuad.theDirX // keep src axis direction
						, srcQuad.theDirY // keep src axis direction
						};
					double const & sigma = fitSpotWgt.weight();
					// compute weight relative to 1-pix sigma
					double const wgt{ std::exp(-sigma*sigma) };
					QuadWgt const fitQuadWgt{ fitSigQuad, wgt };
					fitQuadWgts.emplace_back(fitQuadWgt);
				}
			}

			// sort with highest weight (most likely spot) first
			std::sort
				( fitQuadWgts.begin(), fitQuadWgts.end()
				, [] (QuadWgt const & qw1, QuadWgt const & qw2)
					// Reverse compare to sort with largest weights first
					{ return (qw2.weight() < qw1.weight()); }
				);

			return fitQuadWgts;
		}

		//! Search for combo of radLines fitting sig::QuadTarget
		inline
		std::vector<QuadWgt>
		quadWgtsFor
			( std::vector<EdgeLine> const & allRadLines
			, img::Spot const & centerSpot
			) const
		{
			std::vector<QuadWgt> quadWgts;

			// need at least four radial edges for a quad target
			std::size_t const numRad{ allRadLines.size() };
			if (! (3u < numRad))
			{
				return quadWgts;
			}

			// work with radial edge lines sorted by angle
			std::vector<EdgeLine> radLines{ allRadLines };
			std::sort
				( radLines.begin(), radLines.end()
				, [] (EdgeLine const & e1, EdgeLine const & e2)
					{ return (e1.angleOfLine() < e2.angleOfLine()); }
				);

/*
std::cout << "\n=========\n";
for (std::size_t ndx{0u} ; ndx < radLines.size() ; ++ndx)
{
	EdgeLine const & radLine = radLines[ndx];
	std::cout << "radLine[ndx:" << ndx << "] " << radLine.infoString() << '\n';
}
std::cout << "=========\n";
*/

			// find a positively directed edge (candidate for 'X' axis)
			std::size_t ndxCurr{0u};
			for ( ; ndxCurr < numRad ; ++ndxCurr)
			{
				EdgeLine const & radCurr = radLines[ndxCurr];
				if (0. < radCurr.turnMoment())
				{
					break;
				}
			}

			// properties of starting "+X" candidate
			EdgeLine const & radCurr = radLines[ndxCurr];
			img::Vector<double> const lineDirCurr{ radCurr.lineDirection() };

			// find next negatively directed edge (in cyclic order
			for (std::size_t nOff1{1u} ; nOff1 < numRad ; ++nOff1)
			{
				std::size_t const ndxNext{ (ndxCurr + nOff1) % numRad };
				EdgeLine const & radNext = radLines[ndxNext];
				img::Vector<double> const lineDirNext
					{ radNext.lineDirection() };

				// adjacent edge must be within pi rotation from first angle
				// (use positive outer product to handle phase wrap issues)
				if (0. < outer(lineDirCurr, lineDirNext))
				{
					if (radNext.isTurnDirOppositeTo(radCurr))
					{
						// find radLine most closely opposing Curr and Next
						NdxWgt const nwCurr
							{ radCurr.opposingNdxWgt
								(radLines, ndxCurr, sEdgeLineAngleSigma)
							};
						NdxWgt const nwNext
							{ radNext.opposingNdxWgt
								(radLines, ndxNext, sEdgeLineAngleSigma)
							};
						if (nwCurr.isValid() && nwNext.isValid())
						{
							std::size_t const & npiCurr = nwCurr.item();
							std::size_t const & npiNext = nwNext.item();
							double const & wgtCurr = nwCurr.weight();
							double const & wgtNext = nwNext.weight();
							double const wgtBoth{ wgtCurr * wgtNext };

/*
std::cout << '\n';
std::cout
	<< "radLines[ndxCurr:" << ndxCurr << "]:"
	<< ' ' << radLines[ndxCurr].infoString()
	<< '\n';
std::cout
	<< "radLines[ndxNext:" << ndxNext << "]:"
	<< ' ' << radLines[ndxNext].infoString()
	<< '\n';
std::cout
	<< "radLines[npiCurr:" << npiCurr << "]:"
	<< ' ' << radLines[npiCurr].infoString()
	<< ' ' << "wgtCurr: " << wgtCurr
	<< '\n';
std::cout
	<< "radLines[npiNext:" << npiNext << "]:"
	<< ' ' << radLines[npiNext].infoString()
	<< ' ' << "wgtNext: " << wgtNext
	<< '\n';
std::cout
	<< ' ' << "wgtBoth: " << wgtBoth
	<< '\n';
*/

							// "X" axis candidate
							img::Vector<double> const deltaCurr
								{ radLines[ndxCurr].lineDirection()
								- radLines[npiCurr].lineDirection()
								};
							// "Y" axis candidate
							img::Vector<double> const deltaNext
								{ radLines[ndxNext].lineDirection()
								- radLines[npiNext].lineDirection()
								};

							// compute average edge directions
							img::Vector<double> const dirX
								{ direction(deltaCurr) };
							img::Vector<double> const dirY
								{ direction(deltaNext) };

							// generate candidate signal
							sig::QuadTarget const sigQuad
								{ centerSpot, dirX, dirY };

							// combine opposing edge weights for signal wgt
							QuadWgt const quadWgt{ sigQuad, wgtBoth };
							quadWgts.emplace_back(quadWgt);
						}

					} // next is oppositely directed

				} // next is less than by away from curr
				else
				{
					break;
				}
			}

			return quadWgts;
		}

		//! \brief Associate quad target image signals with spot and ray data
		inline
		std::vector<QuadWgt>
		sigQuadEstimates
			( std::vector<SpotWgt> const & spotWgts
			, std::vector<RayWgt> const & rayWgts
			) const
		{
			std::vector<QuadWgt> allQWs;
			allQWs.reserve(spotWgts.size());

			// Collinearity probabilities for rays
			SpotRayProbs const spotRayProbs(spotWgts, rayWgts);
			std::size_t const numSW{ spotWgts.size() };


			// Attempt to generate sig::QuadTarget for each spot
			for (std::size_t ndxSW{0u} ; ndxSW < numSW ; ++ndxSW)
			{
				// access input spot data
				SpotWgt const & spotWgt = spotWgts[ndxSW];
				img::Spot const & spot = spotWgt.item();
			//	double const & inWgt = spotWgt.weight();

				// get indices for most likely to be collinear rays
				std::vector<NdxWgt> const rayNdxWgts
					{ spotRayProbs.rayNdxWgtsFor(ndxSW) };
				std::vector<EdgeLine> radLines;
				for (NdxWgt const rayNdxWgt : rayNdxWgts)
				{
					std::size_t const & rayNdx = rayNdxWgt.item();
					img::Ray const & ray = rayWgts[rayNdx].item();

					EdgeLine const radLine{ EdgeLine::from(spot, ray) };
					radLines.emplace_back(radLine);
				}

				std::vector<QuadWgt> const oneSpotQWs
					{ quadWgtsFor(radLines, spot) };
				allQWs.insert
					( allQWs.end()
					, oneSpotQWs.cbegin(), oneSpotQWs.cend()
					);
			}

			/*
			for (QuadWgt const & allQW : allQWs)
			{
				std::cout << "\nallQW: " << allQW.infoString() << '\n';
			}
			*/

			return allQWs;
		}


		//! Collection of center point candidates
		inline
		std::vector<QuadWgt>
		sigQuadWeights
			( ras::SizeHW const & hwSize = ras::SizeHW{}
				//!< If size isValid(), use it to filter candidate spots
			) const
		{

			// Define candidate edgerays and associated weights
			EdgeGrouper const edgeGrouper
				(theEdgeInfos, sNumAngleBins, sCosPower);
			std::vector<RayWgt> const rayWgts
				{ edgeGrouper.groupRayWeights(theEdgeInfos) };


constexpr bool showInfo{ true };
if (showInfo)
{
std::cout << "\n\n=============###############\n";
std::cout << infoStringFor(rayWgts, "rayWgt") << '\n';
}

			// Compute pairwise intersection of rays
			std::vector<SpotWgt> const pairSpotWgts
				{ spotWeightsPairwise(rayWgts, hwSize) };

if (showInfo)
{
std::cout << '\n';
std::cout << infoStringFor(pairSpotWgts, "pairSpotWgt") << '\n';
}

			// Spots with weighting based on consensus of all rays
			std::vector<SpotWgt> const qualSpotWgts
				{ spotWeightsConsensus(pairSpotWgts, rayWgts) };

if (showInfo)
{
std::cout << '\n';
std::cout << infoStringFor(qualSpotWgts, "qualSpotWgt") << '\n';
}

			// Check which spots are consistent with quad target geometry
			std::vector<QuadWgt> const quadWgts
				{ sigQuadEstimates(qualSpotWgts, rayWgts) };

if (showInfo)
{
std::cout << '\n';
std::cout << infoStringFor(quadWgts, "quadWgts") << '\n';
}

			//! Spot locations fit to most qualified rays
			std::vector<QuadWgt> const fitQuadWgts
				{ fitQuadWgtsFor(quadWgts, rayWgts) };

if (showInfo)
{
std::cout << '\n';
std::cout << infoStringFor(fitQuadWgts, "fitQuadWgts") << '\n';
}

if (showInfo)
{
std::cout << '\n';
std::cout << "theEdgeInfos.size: " << theEdgeInfos.size() << '\n';
std::cout << "rayWgts.size: " << rayWgts.size() << '\n';
std::cout << "pairSpotWgts.size: " << pairSpotWgts.size() << '\n';
std::cout << "qualSpotWgts.size: " << qualSpotWgts.size() << '\n';
std::cout << "quadWgts.size: " << quadWgts.size() << '\n';
std::cout << "fitQuadWgts.size: " << fitQuadWgts.size() << '\n';
std::cout << '\n';
}

			return fitQuadWgts;
		}


	}; // EdgeEval


} // [sig]

} // [quadloco]



