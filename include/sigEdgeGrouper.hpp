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
 * \brief Declarations for quadloco::sig::EdgeGrouper namespace
 *
 */


#include "imgEdgel.hpp"
#include "imgRay.hpp"
#include "imgVector.hpp"
#include "rasGrid.hpp"
#include "sigEdgeInfo.hpp"
#include "sigItemWgt.hpp"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>


namespace quadloco
{

namespace sig
{

	//! Grouping of index/weights (into assumed external EdgeInfo collection)
	using GroupNWs = std::vector<NdxWgt>;


	//! \brief Estimate the association between EdgeInfo items and angle values.
	class EdgeGrouper
	{
		std::vector<AngleWgt> const theAngWgts;
		ras::Grid<double> const theNdxAngWeights;

		//! Collection of unitary directions corresponding with angle values.
		inline
		static
		std::vector<img::Vector<double> >
		directionsFor
			( std::vector<AngleWgt> const & peakAWs
			)
		{
			std::vector<img::Vector<double> > dirs;
			dirs.reserve(peakAWs.size());
			for (AngleWgt const & peakAW : peakAWs)
			{
				double const & angle = peakAW.item();
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
			, std::vector<AngleWgt> const & peakAWs
			, double const & cosPower
			)
		{
			ras::Grid<double> tab(edgeInfos.size(), peakAWs.size());
			std::fill(tab.begin(), tab.end(), 0.);

			std::vector<img::Vector<double> > const aDirs
				{ directionsFor(peakAWs) };
			std::size_t const numEdges{ edgeInfos.size() };
			std::size_t const numAngles{ peakAWs.size() };
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
		EdgeGrouper
			( std::vector<sig::EdgeInfo> const & edgeInfos
			, std::vector<AngleWgt> const & peakAWs
			, double const & cosPower
			)
			: theAngWgts{ peakAWs }
			, theNdxAngWeights{ fillTable(edgeInfos, theAngWgts, cosPower) }
		{ }

		//! Index/Weights from table classified by peakAngle (from ctor info)
		inline
		std::vector<GroupNWs>
		groupNdxWeights
			() const
		{
			std::vector<GroupNWs> eGroups;
			std::size_t const numGroups{ theNdxAngWeights.wide() };
			std::size_t const numElem{ theNdxAngWeights.high() };
			eGroups.reserve(numGroups);
			for (std::size_t col{0u} ; col < numGroups ; ++col)
			{
				std::vector<NdxWgt> ndxWgts;
				for (std::size_t row{0u} ; row < numElem ; ++row)
				{
					std::size_t const & ndx = row;
					double const & wgt = theNdxAngWeights(row, col);
					if (0. < wgt)
					{
						NdxWgt const ndxWgt{ ndx, wgt };
						ndxWgts.emplace_back(ndxWgt);
					}
				}
				eGroups.emplace_back(GroupNWs{ ndxWgts });
			}
			return eGroups;
		}

		//! Ray best fitting (gradient-weighted average) edge direction
		inline
		static
		RayWgt
		fitRayWeightFor
			( std::vector<NdxWgt> const & theNdxWgts
			, std::vector<sig::EdgeInfo> const & edgeInfos
			)
		{
			img::Ray ray{}; // null instance
			double wgt{ std::numeric_limits<double>::quiet_NaN() };
			img::Vector<double> sumLoc{ 0., 0. };
			img::Vector<double> sumDir{ 0., 0. };
			double sumWgt{ 0. };
			for (NdxWgt const & ndxWgt : theNdxWgts)
			{
				std::size_t const & ndx = ndxWgt.item();
				EdgeInfo const & edgeInfo = edgeInfos[ndx];
				img::Edgel const & edgel = edgeInfos[ndx].edgel();
				double const wgtRadial{ edgeInfo.consideredWeight() };
				double const & wgtGradMag = edgel.magnitude(); // gradient mag
				// double const wgtTotal{ wgtRadial * wgtGradMag };
				// NOTE: wgt on gradient is (wgtGradMag*direction==gradient()
				sumDir = sumDir + wgtRadial *              edgel.gradient();
				sumLoc = sumLoc + wgtRadial * wgtGradMag * edgel.location();
				sumWgt += wgtRadial * wgtGradMag;
			}
			if (0. < sumWgt)
			{
				img::Vector<double> const rayLoc{ (1./sumWgt) * sumLoc };
				img::Vector<double> const rayDir{ direction(sumDir) };
				ray = img::Ray{ rayLoc, rayDir };
				wgt = sumWgt;
			}
			RayWgt const rayWgt{ ray, wgt };
			return rayWgt;
		}

		//! Edge ray (aligned with gradients) candidates for radial edges
		inline
		std::vector<RayWgt>
		groupRayWeights
			( std::vector<sig::EdgeInfo> const & edgeInfos
			) const
		{
			std::vector<RayWgt> rayWgts;

			std::vector<GroupNWs> const groupNWs{ groupNdxWeights() };

			rayWgts.reserve(groupNWs.size());
			for (GroupNWs const & groupNW : groupNWs)
			{
				RayWgt const rayWgt
					{ fitRayWeightFor(groupNW, edgeInfos) };
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
			oss << "EdgeGrouper:angles:";
			for (AngleWgt const & angWgt : theAngWgts)
			{
				using engabra::g3::io::fixed;
				oss << "\n  " << angWgt.infoString();
			}
			return oss.str();
		}

	}; // EdgeGrouper


} // [sig]

} // [quadloco]

