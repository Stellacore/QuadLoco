
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
 * \brief Top level file for quadloco::sig::EdgeEval
 *
 */


#include "angLikely.hpp"
#include "imgArea.hpp"
#include "opsgrid.hpp"
#include "rasGrid.hpp"
#include "sigEdgeInfo.hpp"

#include <algorithm>
#include <vector>


namespace quadloco
{

namespace sig
{
	constexpr double sCosPower = 10.; // attenuation power on dot product
	constexpr double sLinkEdgeDist = 2.50;
	constexpr std::size_t sDiagMultiple = 6u;
	constexpr std::size_t sNumAngleBins = 32u;


	//! Candidate radial edge ray (aligned with radiometric gradients)
	struct RayWgt
	{
		img::Ray theRay{};
		double theWeight{};

		//! True if this instance contains valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  theRay.isValid()
				&& engabra::g3::isValid(theWeight)
				);
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
				<< "ray: " << theRay
				<< ' '
				<< "wgt: " << engabra::g3::io::fixed(theWeight);
				;
			return oss.str();
		}


	}; // RayWgt


	//! Weighted spot location
	struct SpotWgt
	{
		img::Spot theSpot{};
		double theWeight{};

		//! Candidate center point
		inline
		static
		SpotWgt
		fromIntersectionOf
			( RayWgt const & rw1
			, RayWgt const & rw2
			)
		{
			img::Spot meetSpot{}; // null
			double meetWgt{ 0. };
			// access data: points on edge, edge dirs, and edge weights
			img::Vector<double> const & s1 = rw1.theRay.start();
			img::Vector<double> const & s2 = rw2.theRay.start();
			img::Vector<double> const & d1 = rw1.theRay.direction();
			img::Vector<double> const & d2 = rw2.theRay.direction();
			double const & wgtRay1 = rw1.theWeight;
			double const & wgtRay2 = rw2.theWeight;
			// coefficient matrix
			double const & fwd00 = d1[0];
			double const & fwd01 = d1[1];
			double const & fwd10 = d2[0];
			double const & fwd11 = d2[1];
			// determinant
			double const det{ fwd00*fwd11 - fwd01*fwd10 };
			if (std::numeric_limits<double>::epsilon() < std::abs(det))
			{
				// right hand side vector
				double const rhs0{ dot(d1, s1) };
				double const rhs1{ dot(d2, s2) };
				// inverse coefficient matrix
				double const & inv00 = fwd11;
				double const inv01{ -fwd01 };
				double const inv10{ -fwd10 };
				double const & inv11 = fwd00;
				// solve for intersection location
				double const scl{ 1. / det };
				meetSpot = img::Spot
					{ scl * (inv00*rhs0 + inv01*rhs1)
					, scl * (inv10*rhs0 + inv11*rhs1)
					};
				// use coefficient matrix determinant as intersection weight
				// combined with input data weights.
				double const wgtMeet{ std::abs(det) };
				meetWgt = wgtMeet * wgtRay1 * wgtRay2;
			}
			return SpotWgt{ meetSpot, meetWgt };
		}

		//! True if this instance contains valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  theSpot.isValid()
				&& engabra::g3::isValid(theWeight)
				);
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
				<< "spot: " << theSpot
				<< ' '
				<< "wgt: " << engabra::g3::io::fixed(theWeight);
				;
			return oss.str();
		}


	}; // SpotWgt

	//! Indices and weights for grouping EdgeInfo in association with an angle.
	struct EdgeGroup
	{
		//! Collection index and entry weight pair
		struct NdxWgt
		{
			std::size_t const theNdx;
			double const theWeight;

		}; // NdxWgt

		//! Index/Weights for assumed (external) EdgeInfo collection
		std::vector<NdxWgt> theNdxWgts;

		//! Ray best fitting (gradient-weighted average) edge direction
		inline
		img::Ray
		rayFit
			( std::vector<sig::EdgeInfo> const & edgeInfos
			, double * const ptWgt = nullptr
			) const
		{
			img::Ray ray{}; // null instance
			img::Vector<double> sumLoc{ 0., 0. };
			img::Vector<double> sumDir{ 0., 0. };
			double sumWgt{ 0. };
			for (NdxWgt const & ndxWgt : theNdxWgts)
			{
				std::size_t const & ndx = ndxWgt.theNdx;
				img::Edgel const & edgel = edgeInfos[ndx].edgel();
				double const & wgt = edgel.magnitude(); // gradient mag
				sumLoc = sumLoc + wgt * edgel.location();
				sumDir = sumDir + edgel.gradient();
				sumWgt += wgt;
			}
			if (0. < sumWgt)
			{
				img::Vector<double> const rayLoc{ (1./sumWgt) * sumLoc };
				img::Vector<double> const rayDir{ direction(sumDir) };
				ray = img::Ray{ rayLoc, rayDir };
				if (ptWgt)
				{
					*ptWgt = sumWgt;
				}
			}
			return ray;
		}

	}; // EdgeGroup


	//! \brief Estimate the association between EdgeInfo items and angle values.
	class GroupTable
	{
		std::vector<double> const theAngles;
		ras::Grid<double> const theNdxAngWeights;

		//! Collection of unitary directions corresponding with angle values.
		inline
		static
		std::vector<img::Vector<double> >
		directionsFor
			( std::vector<double> const & angles
			)
		{
			std::vector<img::Vector<double> > dirs;
			dirs.reserve(angles.size());
			for (double const & angle : angles)
			{
				dirs.emplace_back
					(img::Vector<double>{ std::cos(angle), std::sin(angle)} );
			}
			return dirs;
		}

		/*! \brief Create pseudo-probabilities of edges belonging to angles
		 *
		 * The table rows correspond to indices in the edge info struture
		 * and the table columns correspond with elements from the angles
		 * collection.
		 *
		 * Table values are an accumulation of weighted gradients, where
		 * the weight factor is computed based on proximity to angle
		 * directions.
		 *
		 * The maximum weight in a row - suggests the angle to which the
		 * row edgel is most closely algined (e.g. classification of
		 * edgels into angle categories).
		 *
		 * The larger weights down a column suggest which edgels are likely
		 * to associated with that particular angle (e.g. be on an edge
		 * with that direction).
		 */
		inline
		static
		ras::Grid<double>
		fillTable
			( std::vector<sig::EdgeInfo> const & edgeInfos
			, std::vector<double> const & angles
			, double const & cosPower = sCosPower // attenuation of dot product
			)
		{
			ras::Grid<double> tab(edgeInfos.size(), angles.size());
			std::fill(tab.begin(), tab.end(), 0.);

			std::vector<img::Vector<double> > const aDirs
				{ directionsFor(angles) };
			std::size_t const numEdges{ edgeInfos.size() };
			std::size_t const numAngles{ angles.size() };
			for (std::size_t eNdx{0u} ; eNdx < numEdges ; ++eNdx)
			{
				std::size_t const & row = eNdx;
				img::Edgel const & edgel = edgeInfos[eNdx].edgel();
				img::Vector<double> const eDir{ edgel.direction() };
				for (std::size_t aNdx{0u} ; aNdx < numAngles ; ++aNdx)
				{
					std::size_t const & col = aNdx;
					img::Vector<double> const & aDir = aDirs[aNdx];

					double const align{ dot(eDir, aDir) };
					if (.75 < align)
					{
						double const dirWgt{ std::powf(align, cosPower) };
						tab(row,col) += dirWgt * edgel.magnitude();
					}
				}
			}
			return tab;
		}

	public:

		//! Populate edge/angle weight table - ref: fillTable().
		inline
		GroupTable
			( std::vector<sig::EdgeInfo> const & edgeInfos
			, std::vector<double> const & peakAngles
			)
			: theAngles{ peakAngles }
			, theNdxAngWeights{ fillTable(edgeInfos, theAngles) }
		{ }

		//! Index/Weights from table classified by peakAngle (from ctor info)
		inline
		std::vector<EdgeGroup>
		edgeGroups
			() const
		{
			std::vector<EdgeGroup> edgeGroups;
			std::size_t const numGroups{ theNdxAngWeights.wide() };
			std::size_t const numElem{ theNdxAngWeights.high() };
			for (std::size_t col{0u} ; col < numGroups ; ++col)
			{
				std::vector<EdgeGroup::NdxWgt> ndxWgts;
				for (std::size_t row{0u} ; row < numElem ; ++row)
				{
					std::size_t const & ndx = row;
					double const & wgt = theNdxAngWeights(row, col);
					if (0. < wgt)
					{
						EdgeGroup::NdxWgt const ndxWgt{ ndx, wgt };
						ndxWgts.emplace_back(ndxWgt);
					}
				}
				edgeGroups.emplace_back(EdgeGroup{ ndxWgts });
			}
			return edgeGroups;
		}

		//! Description including contents of theNdxAngWeight table
		inline
		std::string
		infoStringContents
			( std::string const & title
				//!< Heading to print (along with size info)
			, std::string const & cfmt
				//!< A 'printf' style string to format each cell value
			) const
		{
			std::ostringstream oss;
			oss << theNdxAngWeights.infoStringContents(title, cfmt);
			oss << '\n';
			oss << "# angles: ";
			for (double const & angle : theAngles)
			{
				using engabra::g3::io::fixed;
				oss << ' ' << fixed(angle, 2u, 3u);
			}
			return oss.str();
		}

	}; // GroupTable


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
		edgeInfosFor
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
					sig::EdgeInfo edgeInfo1(edgel);
					edgeInfos.emplace_back(edgeInfo1);

					// compare from start until *before* last item just added
					std::size_t const numLast{ edgeInfos.size() - 1u };
					for (std::size_t ndx2{0u} ; ndx2 < numLast ; ++ndx2)
					{
						// combinatorially consider other edges and
						// update each with consideration of the other
						sig::EdgeInfo & edgeInfo2 = edgeInfos[ndx2];
						edgeInfo1.consider(edgeInfo2.edgel());
						edgeInfo2.consider(edgeInfo1.edgel());
					}
				}
			}
			return edgeInfos;
		}

	public:

		// Process gradients into non-isolated edges
		inline
		explicit
		EdgeEval
			( ras::Grid<img::Grad> const & gradGrid
			)
			: theEdgeInfos{ edgeInfosFor(dominantEdgelsFrom(gradGrid)) }
		{ }


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

		//! Determine most likely edgel angle directions (perp to radial edges)
		inline
		std::vector<double>
		peakAngles
			( std::size_t const & numAngBins = sNumAngleBins
			) const
		{
			std::vector<double> peaks;

			// create angle value accumulation (circular-wrapping) buffer
			ang::Likely angleProbs(numAngBins);
			for (sig::EdgeInfo const & edgeInfo : theEdgeInfos)
			{
				double const fwdAngle{ edgeInfo.consideredAngle() };
				double const weight{ edgeInfo.consideredWeight() };
				angleProbs.add(fwdAngle, weight, 2u);

				// also add opposite angle to create angle prob buffer
				// peaks that are symmetrically balanced (to faciliate 
				// edge group duo extraction below)
				double const revAngle
					{ ang::principalAngle(fwdAngle + ang::piOne()) };
				angleProbs.add(revAngle, weight, 2u);
			}

			// get peaks from angular accumulation buffer
			peaks = angleProbs.anglesOfPeaks();

/*
std::cout << angleProbs.infoString("angleProbs") << '\n';
std::cout << angleProbs.infoStringContents("angleProbs") << '\n';

std::vector<std::size_t> const peakMaxNdxs{ angleProbs.indicesOfPeaks() };
for (std::size_t const & peakMaxNdx : peakMaxNdxs)
{
	std::cout << "  peakMaxNdx: " << std::setw(4u) << peakMaxNdx << '\n';
}
std::cout << "--\n";

std::cout << "--\n";

for (double const & peakAngle : peaks)
{
	double const peakValue{ angleProbs.binSumAtAngle(peakAngle) };
	std::cout
		<< "  peakAngle: " << engabra::g3::io::fixed(peakAngle)
		<< "  peakValue: " << engabra::g3::io::fixed(peakValue)
		<< '\n';
}
*/

			return peaks;
		}

		//! \brief Weight table reflecting edgeInfo membership to angle group
		inline
		GroupTable
		groupTable
			() const
		{
			std::vector<double> const angles{ peakAngles() };
			return GroupTable(theEdgeInfos, angles);
		}

		//! Edge ray (aligned with gradients) candidates for radial edges
		inline
		std::vector<RayWgt>
		groupRayWeights
			() const
		{
			std::vector<RayWgt> rayWgts;
			GroupTable const groupTab{ groupTable() };
			std::vector<EdgeGroup> const groups{ groupTab.edgeGroups() };
			rayWgts.reserve(groups.size());
			for (EdgeGroup const & group : groups)
			{
				double wgt{ 0. };
				img::Ray const ray{ group.rayFit(theEdgeInfos, &wgt) };
				RayWgt const rayWgt{ ray, wgt };
				if (rayWgt.isValid())
				{
					rayWgts.emplace_back(rayWgt);
				}
			}
			std::sort
				( rayWgts.begin(), rayWgts.end()
				, [] (RayWgt const & rw1, RayWgt const & rw2)
					// reverse directions to sort largest weight first
					{ return rw2.theWeight < rw1.theWeight; }
				);
			return rayWgts;
		}

		//! Utility filter to determine if spots should be used.
		struct SpotFilter
		{
			bool theUseArea{ false };
			img::Area theArea{};

			/*! Determine when to use spots
			 *
			 * Use always if hwSize is not valid
			 * Otherwise, if hwSize is valid, only use spots inside the area.
			 */
			inline
			explicit
			SpotFilter
				( ras::SizeHW const & hwSize
				)
				: theUseArea{ hwSize.isValid() }
				, theArea
					{ img::Span{ 0., static_cast<double>(hwSize.high())}
					, img::Span{ 0., static_cast<double>(hwSize.wide())}
					}
			{ }

			//! True if spot should be utilized
			inline
			bool
			useSpot
				( img::Spot const & spot
				) const
			{
				return
					(  theUseArea
					&& theArea.contains(spot)
					);
			}

		};


		//! Collection of center point candidates
		inline
		std::vector<SpotWgt>
		centerSpotWeightsPairwiseA
			( ras::SizeHW const & hwSize = ras::SizeHW{}
				//!< If size isValid(), use it to filter candidate spots
			) const
		{
			std::vector<SpotWgt> spotWgts;
			std::vector<RayWgt> const rayWgts{ groupRayWeights() };
			std::size_t const numEdges{ rayWgts.size() };
			std::size_t const & numToUse = numEdges;
			spotWgts.reserve((numToUse * (numToUse - 1u)) / 2u);
			SpotFilter const spotFilter(hwSize);
			for (std::size_t ndx1{0u} ; ndx1 < numToUse ; ++ndx1)
			{
				RayWgt const & rw1 = rayWgts[ndx1];
				for (std::size_t ndx2{ndx1+1u} ; ndx2 < numToUse ; ++ndx2)
				{
					RayWgt const & rw2 = rayWgts[ndx2];
					SpotWgt const spotWgt
						{ SpotWgt::fromIntersectionOf(rw1, rw2) };
					if (spotWgt.isValid())
					{
						if (spotFilter.useSpot(spotWgt.theSpot))
						{
							spotWgts.emplace_back(spotWgt);
						}
					}
				}
			}
			std::sort
				( spotWgts.begin(), spotWgts.end()
				, [] (SpotWgt const & sw1, SpotWgt const & sw2)
					// reverse directions to sort largest weight first
					{ return sw2.theWeight < sw1.theWeight; }
				);
			return spotWgts;
		}

		//! Collection of center point candidates
		inline
		std::vector<SpotWgt>
		centerSpotWeights
			( ras::SizeHW const & hwSize = ras::SizeHW{}
				//!< If size isValid(), use it to filter candidate spots
			) const
		{
			std::vector<SpotWgt> spotWgts;
			SpotFilter const spotFilter(hwSize);

			// get edge rays likely associated with radial quad edges
			std::vector<RayWgt> const rayWgts{ groupRayWeights() };
			std::size_t const numEdges{ rayWgts.size() };
			std::size_t const & numUseRW = numEdges;
			// pair-wise intersection of all rays
			spotWgts.reserve((numUseRW * (numUseRW - 1u)) / 2u);
			for (std::size_t ndx1{0u} ; ndx1 < numUseRW ; ++ndx1)
			{
				RayWgt const & rw1 = rayWgts[ndx1];
				for (std::size_t ndx2{ndx1+1u} ; ndx2 < numUseRW ; ++ndx2)
				{
					RayWgt const & rw2 = rayWgts[ndx2];
					SpotWgt const spotWgt
						{ SpotWgt::fromIntersectionOf(rw1, rw2) };
					if (spotWgt.isValid())
					{
						if (spotFilter.useSpot(spotWgt.theSpot))
						{
							spotWgts.emplace_back(spotWgt);
						}
					}
				}
			}

			// assess how well each spot agrees with all edge rays
			std::size_t const numSW{ spotWgts.size() };
			for (std::size_t ndxSW{0u} ; ndxSW < numSW ; ++ndxSW)
			{
				SpotWgt & spotWgt = spotWgts[ndxSW];
				img::Spot const & spot = spotWgt.theSpot;
				double & wgtSpot = spotWgt.theWeight; // REPLACE this value

				// Replace pairwise (spotWgt.theWeight) from above
				// with a new spot weight computed with all edge rays
				wgtSpot = 0.;
				for (std::size_t ndxRW{0u} ; ndxRW < numUseRW ; ++ndxRW)
				{
					RayWgt const & rayWgt = rayWgts[ndxRW];
					img::Ray const & ray = rayWgt.theRay;
					double const & wgtRay = rayWgt.theWeight;

					double const edgeGap{ ray.distanceAlong(spot) };
					double const arg{ (edgeGap / 1.) };
					double const prob{ std::exp(-arg*arg) };
					// accumulate this pseudo probability into spot weight
					double const wgtVote{ wgtRay * prob };
					wgtSpot += wgtVote;
				}
			}

			// sort highest weight (most likely) spot first
			std::sort
				( spotWgts.begin(), spotWgts.end()
				, [] (SpotWgt const & sw1, SpotWgt const & sw2)
					// reverse directions to sort largest weight first
					{ return sw2.theWeight < sw1.theWeight; }
				);


			return spotWgts;
		}


	}; // EdgeEval


} // [sig]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sig::RayWgt const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::sig::RayWgt const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sig::SpotWgt const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::sig::SpotWgt const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

