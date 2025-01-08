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


#include "ang.hpp"
#include "imgEdgel.hpp"
#include "imgRay.hpp"
#include "imgVector.hpp"
#include "opsPeakAngles.hpp"
#include "rasGrid.hpp"
#include "sigEdgeInfo.hpp"
#include "sigItemWgt.hpp"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include <fstream>


namespace quadloco
{

namespace sig
{

	//! Grouping of index/weights (into assumed external EdgeInfo collection)
	using GroupNWs = std::vector<NdxWgt>;


	/*! \brief Estimate the association between EdgeInfo items and angle values.
	 *
	 * Given a collection of sig::EdgeInfo items (e.g. detected edgels)
	 * Use them to determine an angular histogram of edge directions.
	 * Edgels are grouped by association with peaks in the angle spectrum.
	 * Within each group, edgels are averaged into a mean edge ray that
	 * represents the group.
	 *
	 * For consuming code: just call the static function
	 * mainEdgeRayWeightsFor().
	 */
	class EdgeGrouper
	{
		/*
		* The angle peak results are stored in theAngWgts class member which
		* holds the angle value at each peak and the weight (relative height)
		* of each peak. This is done in peakAngleWeights().
		*
		* Determine which angle/weight most closely matches each input EdgeInfo
		* instance. This is done in probgridEdgeAngle() which returns a
		* table of pseudo-probabilities. Each pseudo-prob corresponds to how
		* well the input edgel (grid row) matches one of the angle peaks
		* (grid column).
		*/

		//! Peaks in angle histogram for edgel alignments
		std::vector<AngleWgt> const theAngWgts;

		//! Pseudo-probability values of edgels(rows) at each angle peak(cols).
		ras::Grid<double> const theProbGridEdgeAngle;

	public:

		//! Determine most likely edgel angle directions (perp to radial edges)
		inline
		static
		std::vector<AngleWgt>
		peakAngleWeights
			( std::vector<sig::EdgeInfo> const & edgeInfos
			, std::size_t const & numAngBins
			)
		{
			std::vector<AngleWgt> peakAWs;

			// create angle value accumulation (circular-wrapping) buffer
			ops::PeakAngles angleProbs(numAngBins);
			for (sig::EdgeInfo const & edgeInfo : edgeInfos)
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
			std::vector<double> const peakAngles{ angleProbs.anglesOfPeaks() };
			if (! peakAngles.empty())
			{
				double peakValueMax{ 0. };
				for (double const & peakAngle : peakAngles)
				{
					double const peakValue
						{ angleProbs.binSumAtAngle(peakAngle) };
					if (peakValueMax < peakValue)
					{
						peakValueMax = peakValue;
					}
				}
				for (double const & peakAngle : peakAngles)
				{
					double const peakValue
						{ angleProbs.binSumAtAngle(peakAngle) };
					double const peakRelValue{ peakValue / peakValueMax };
					AngleWgt const angleWgt
						{ peakAngle
						, peakRelValue
						};
					peakAWs.emplace_back(angleWgt);
				}
			}

/*
std::cout << angleProbs.infoString("angleProbs") << '\n';
std::cout << angleProbs.infoStringContents("angleProbs") << '\n';

std::vector<std::size_t> const peakMaxNdxs{ angleProbs.indicesOfPeaks() };
std::cout << "--\n";
for (std::size_t const & peakMaxNdx : peakMaxNdxs)
{
std::cout << "  peakMaxNdx: " << std::setw(4u) << peakMaxNdx << '\n';
}

std::cout << "--\n";
for (AngleWgt const & peakAW : peakAWs)
{
std::cout
	<< "  peakAngle: " << engabra::g3::io::fixed(peakAW.item())
	<< "  peakValue: " << engabra::g3::io::fixed(peakAW.weight())
	<< '\n';
}
std::cout << "--\n";
*/

			return peakAWs;
		}

	private:

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

	public:

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
		probGridEdgeAngle
			( std::vector<sig::EdgeInfo> const & edgeInfos
			, std::vector<AngleWgt> const & peakAWs
			, double const & alignSigmaEdgeAngle
			)
		{
			ras::Grid<double> tab(edgeInfos.size(), peakAWs.size());
			std::fill(tab.begin(), tab.end(), 0.);
			//
			// get directions for each peak in angle domain
			using DirVec = img::Vector<double>;
			std::vector<DirVec> const aDirs{ directionsFor(peakAWs) };
			//
			// for each edgeInfo, compute pseudo-prob relations to each peak
			std::size_t const numEdges{ edgeInfos.size() };
			std::size_t const numAngles{ peakAWs.size() };
			//
			// Convert weight from exponential argument to power of dot
			// product result (using 2nd order equivalance of power series)
			double const & sigma = alignSigmaEdgeAngle;
			double const cosPower{ std::sqrt(2./sigma/sigma) };
			//
			// edge indices in rows
			for (std::size_t eNdx{0u} ; eNdx < numEdges ; ++eNdx)
			{
				std::size_t const & row = eNdx;
				img::Edgel const & edgel = edgeInfos[eNdx].edgel();
				DirVec const eDir{ edgel.direction() };
				//
				// angle indices in columns
				for (std::size_t aNdx{0u} ; aNdx < numAngles ; ++aNdx)
				{
					std::size_t const & col = aNdx;
					DirVec const & aDir = aDirs[aNdx];
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

	private:

		//! Ray best fitting (gradient-weighted average) edge ray for a group
		inline
		static
		RayWgt
		meanEdgeRayWgtFor
			( std::vector<NdxWgt> const & edgeInfoNdxWgts
				//!< Indices into edgeInfos and associated edgel weight
			, std::vector<sig::EdgeInfo> const & edgeInfos
				//!< EdgeInfo data associated with indices in first arg
			)
		{
			img::Ray ray{}; // null instance
			double wgt{ std::numeric_limits<double>::quiet_NaN() };
			img::Vector<double> sumLoc{ 0., 0. };
			img::Vector<double> sumDir{ 0., 0. };
			double sumWgt{ 0. };
			for (NdxWgt const & edgeInfoNdxWgt : edgeInfoNdxWgts)
			{
				// access edgel data
				std::size_t const & ndx = edgeInfoNdxWgt.item();
				EdgeInfo const & edgeInfo = edgeInfos[ndx];
				img::Edgel const & edgel = edgeInfos[ndx].edgel();

				// compute weight for this edgel
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

		/*! Groups of edgels determined by alignments with angle peaks
		 *
		 * Return is a collection of groups, and each group is a collection
		 * of edgels that associate with a common angle orientation.
		 *
		 * Each GroupNWs element in the return collection represents a
		 * group of edgels (via NdxWgt's index into theEdgelInfos) that
		 * align with an angle peak (which overall group of the returned
		 * collection). The degree to which that edgel associates with
		 * the angle value is encoded in the pseudo-probability value
		 * of NdxWgt's weight value.
		 *
		 * This is primarily a staging result for subsequent determination
		 * of group averge rays (in mainEdgeRayWeights()).
		 */
		inline
		std::vector<GroupNWs>
		ndxWgtByAngle
			() const
		{
			std::vector<GroupNWs> eGroups;
			std::size_t const numAngPeaks{ theProbGridEdgeAngle.wide() };
			std::size_t const numEdgeInfos{ theProbGridEdgeAngle.high() };

			// create a group (of edgel ndxs) for each angle peak
			eGroups.reserve(numAngPeaks);

			// loop over angle peaks
			for (std::size_t col{0u} ; col < numAngPeaks ; ++col)
			{
				std::vector<NdxWgt> edgeInfoNdxWgts;

				// loop over input edgels (row is index into EdgeInfos)
				for (std::size_t row{0u} ; row < numEdgeInfos ; ++row)
				{
					std::size_t const & ndx = row;
					double const & wgt = theProbGridEdgeAngle(row, col);
					if (0. < wgt)
					{
						NdxWgt const edgeInfoNdxWgt{ ndx, wgt };
						edgeInfoNdxWgts.emplace_back(edgeInfoNdxWgt);
					}
				}
				eGroups.emplace_back(GroupNWs{ edgeInfoNdxWgts });
			}
			return eGroups;
		}

		/* \brief Split largeGroup into smaller ones with better aligned edgels.
		 *
		 * From large group, pick an arbitrary (e.g. first) index to
		 * begin the smaller group.
		 * 
		 * Then step through all edgels remaining in the largeGroup and
		 * move the aligned ones into the new small group.
		 *
		 */
		inline
		static
		std::vector<GroupNWs>
		splitOneGroup
			( GroupNWs const & largeGroup
			, std::vector<sig::EdgeInfo> const & edgeInfos
			, double const & tolCollin
			)
		{
			std::vector<GroupNWs> smallGroups;

			std::vector<bool> isDone(largeGroup.size(), false);
			std::vector<bool>::const_iterator itFind
				{ std::find(isDone.cbegin(), isDone.cend(), false) };
			std::size_t numLeft{ largeGroup.size() };
			while ((0u < numLeft) && (isDone.cend() != itFind))
			{
				GroupNWs smallGroup;
				smallGroup.reserve(numLeft); // conservative on size
				// get first one
				std::size_t const doNdx0
					{ (std::size_t)std::distance(isDone.cbegin(), itFind) };

				// get corresponding ndx/weight
				NdxWgt const & nw0 = largeGroup[doNdx0];
				std::size_t const & ndx0 = nw0.item();
				img::Edgel const & edgel0 = edgeInfos[ndx0].edgel();

				// add edgel0 to start small group
				smallGroup.emplace_back(nw0);
				isDone[doNdx0] = true;
				--numLeft;
				if (0u == numLeft)
				{
					break;
				}
				// (running average ray for small group
				img::Vector<double> sumStart{ edgel0.start() };
				img::Vector<double> sumDir{ edgel0.direction() };
				double sumCount{ 1. };

				// find the rest
				for (std::size_t doNdx1{doNdx0+1} ; doNdx1 < isDone.size()
					; ++doNdx1)
				{
					if (! isDone[doNdx1])
					{
						double const scl{ 1. / sumCount };
						img::Ray const aveRay
							{ scl * sumStart
							, direction(sumDir)
							};

						NdxWgt const & nw1 = largeGroup[doNdx1];
						std::size_t const & ndx1 = nw1.item();
						img::Edgel const & edgel = edgeInfos[ndx1].edgel();
						if (nearlyCollinear(edgel, aveRay, tolCollin))
						{
							smallGroup.emplace_back(nw1);
							isDone[doNdx1] = true;
							--numLeft;
							sumStart = sumStart + edgel.start();
							sumDir = sumDir + edgel.direction();
							sumCount += 1.;
							if (0u == numLeft)
							{
								break;
							}
						}
					}
					// else // skip already evaluated/consumed indices
				}

				// return small group from this run through 'isDone' list
				smallGroups.emplace_back(smallGroup);

				// find the next 'yet-to-be-processed' edgel
				itFind = std::find(isDone.cbegin(), isDone.cend(), false);
			}

			return smallGroups;
		}

		//! Split largeGroups into smaller groups with colinear edgels
		inline
		static
		std::vector<GroupNWs>
		splitByColin
			( std::vector<GroupNWs> const & largeGroups
			, std::vector<sig::EdgeInfo> const & edgeInfos
			, double const & tolCollin
			)
		{
			std::vector<GroupNWs> splitGroups;

			for (GroupNWs const & largeGroup : largeGroups)
			{
				std::vector<GroupNWs> const smallGroups
					{ splitOneGroup(largeGroup, edgeInfos, tolCollin) };
				splitGroups.insert
					( splitGroups.end()
					, smallGroups.cbegin(), smallGroups.cend()
					);
			}

			return splitGroups;
		}


		//! Populate edge/angle weight table - ref: probGridEdgeAngle().
		inline
		explicit
		EdgeGrouper
			( std::vector<sig::EdgeInfo> const & edgeInfos
			, std::vector<AngleWgt> const & peakAWs
			, double const & alignSigma
			)
			: theAngWgts{ peakAWs }
			, theProbGridEdgeAngle
				{ probGridEdgeAngle(edgeInfos, theAngWgts, alignSigma) }
		{ }

	// public:
	private:

		//! Construct a null instance
		inline
		explicit
		EdgeGrouper
			() = default;

		//! Edge ray (aligned with gradients) candidates for radial edges
		inline
		explicit
		EdgeGrouper
			( std::vector<sig::EdgeInfo> const & edgeInfos
			, std::size_t const & numAngleBins
			, double const & alignSigma
			)
			: EdgeGrouper
				( edgeInfos
				, peakAngleWeights(edgeInfos, numAngleBins)
				, alignSigma
				)
		{ }


		//! True if NdxAng grid is valid
		inline
		bool
		isValid
			() const
		{
			return
				(  (! theAngWgts.empty())
				&& theProbGridEdgeAngle.isValid()
				);
		}

	public:

		/*! \brief Mean edge rays for edgel groups consistent with quad target.
		 *
		 * Ref Explanation with mainEdgeRayWeights().
		 */
		inline
		static
		std::vector<RayWgt>
		mainEdgeRayWeightsFor
			( std::vector<sig::EdgeInfo> const & edgeInfos
			, std::size_t const & numAngleBins
			, double const & alignSigma
			)
		{
			EdgeGrouper const grouper(edgeInfos, numAngleBins, alignSigma);
			return grouper.mainEdgeRayWeights(edgeInfos);
		}

	private:

		/*! \brief Mean edge rays for each edgel grouping.
		 *
		 * Input edgels are associated into groups. For each group
		 * a mean edgel location and direction are estimated and
		 * associated with a quality weight.
		 *
		 * Each item in the return collection corresponds with one
		 * group of edgels and represents that group's mean location
		 * and mean direction (in the img::Ray part, RayWgt.item())
		 * along with a quality indictor (in the double part,
		 * RayWgt.weight()).
		 *
		 * NOTE: return RayWgt collection is *SORTED* from highest weight
		 *       at front and lowest weights at back;
		 *
		 * NOTE: the edgeInfos arg must be same one used for construction.
		 *       Alternatively, use static function mainEdgeRayWeightsFor().
		 */
		inline
		std::vector<RayWgt>
		mainEdgeRayWeights
			( std::vector<sig::EdgeInfo> const & edgeInfos
			) const
		{
			std::vector<RayWgt> rayWgts;

			// determine groups of edgels (with similar angle orientations)
			std::vector<GroupNWs> const groupByAngles{ ndxWgtByAngle() };

writeGroupNWs(groupByAngles, "groupNWs.dat", edgeInfos);

constexpr double tolCollin{ 10. };
			std::vector<GroupNWs> const groupNWs
				{ splitByColin(groupByAngles, edgeInfos, tolCollin) };
			if (! groupNWs.empty())
			{

				// determine representative edge-ray for each group
				rayWgts.reserve(groupNWs.size());
				for (GroupNWs const & groupNW : groupNWs)
				{
					// compute average edgeRay for this group
					RayWgt const rayWgt
						{ meanEdgeRayWgtFor(groupNW, edgeInfos) };
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
writeRayWgts(rayWgts, "rayWgts.dat");

			}
			return rayWgts;
		}

		//! Write group index data to file
		inline
		static
		void
		writeGroupNWs
			( std::vector<GroupNWs> const & groupNWss
			, std::string const & fname
			, std::vector<sig::EdgeInfo> const & edgeInfos
			)
		{
			std::ofstream ofs(fname);
			std::size_t idGroup{ 0u };
			ofs << "# location, gradient, idGroup, weight\n";
			for (GroupNWs const & groupNWs : groupNWss)
			{
				for (NdxWgt const & groupNW : groupNWs)
				{
					EdgeInfo const & ei = edgeInfos[groupNW.item()];
					ofs
						<< ei.edgel().location()
						<< ' '
						<< ei.edgel().gradient()
						<< ' '
						<< idGroup
						<< ' '
						<< groupNW.weight()
						<< '\n';
				}
				++idGroup;
			}
		}

		inline
		static
		void
		writeRayWgts
			( std::vector<RayWgt> const & rayWgts
			, std::string const & fname
			)
		{
			std::ofstream ofs(fname);
			for (RayWgt const & rayWgt : rayWgts)
			{
				ofs << rayWgt << '\n';
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
				<< "theAngWgts.size: " << theAngWgts.size()
				<< " "
				<< "theProbGridEdgeAngle: " << theProbGridEdgeAngle
				;

			return oss.str();
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
			oss << infoString(title);
			oss << '\n';
			oss << theProbGridEdgeAngle.infoStringContents(title, cfmt);
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


/*
namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sig::EdgeGrouper const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::sig::EdgeGrouper const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]
*/

