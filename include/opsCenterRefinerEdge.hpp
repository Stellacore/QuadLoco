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
 * \brief Declarations for quadloco::ops::CenterRefinerEdge namespace
 *
 */


#include "ang.hpp"
#include "cast.hpp"
#include "imgArea.hpp"
#include "imgEdgel.hpp"
#include "imgGrad.hpp"
#include "imgHit.hpp"
#include "imgRay.hpp"
#include "imgSpot.hpp"
#include "matEigen2D.hpp"
#include "meaVector.hpp"
#include "opsAngleTracker.hpp"
#include "opsgrid.hpp"
#include "prbStats.hpp"
#include "rasGrid.hpp"
#include "rasPeakRCV.hpp"
#include "valSpan.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <iterator>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>


namespace quadloco
{

namespace ops
{
	//! Compute 2D statistics on group of locations
	struct EdgeGroup
	{
		//! Weighted location sample
		struct Sample
		{
			//! Location of sample point
			img::Vector<double> theLoc{};

			//! Relative weight of this location
			double theWeight{ 0. };

		}; // Sample

		//! Collect samples
		std::vector<Sample> theSamps;

		//! Add weighted spot data to collection
		inline
		void
		add
			( img::Spot const & imgSpot
			, double const & weight
			)
		{
			theSamps.emplace_back(Sample{ imgSpot, weight });
		}

		//! Centroid of all samples in this group
		inline
		img::Vector<double>
		centroid
			() const
		{
			img::Vector<double> mean{};
			img::Vector<double> sumLocs{ 0., 0. };
			double sumWgts{ 0. };
			for (Sample const & samp : theSamps)
			{
				img::Vector<double> const & loc = samp.theLoc;
				double const & wgt = samp.theWeight;
				sumLocs = sumLocs + wgt * loc;
				sumWgts += wgt;
			}
			if (std::numeric_limits<double>::epsilon() < sumWgts)
			{
				mean = (1./sumWgts) * sumLocs;
			}
			return mean;
		}

		//! Semi axis of the longest scatter direction (+ or -)
		inline
		img::Vector<double>
		semiAxisMax
			( img::Vector<double> const & meanLoc
			) const
		{
			img::Vector<double> axisMag;

			// scatter matrix
			ras::Grid<double> scatter(2u, 2u);
			double sumWgts{ 0. };
			for (Sample const & samp : theSamps)
			{
				img::Vector<double> const & absLoc = samp.theLoc;
				img::Vector<double> const relLoc{ absLoc - meanLoc };
				double const & wgt = samp.theWeight;

				scatter(0u, 0u) += wgt * (relLoc[0] * relLoc[0]);
				scatter(0u, 1u) += wgt * (relLoc[0] * relLoc[1]);
			//	scatter(1u, 0u) += wgt * (relLoc[1] * relLoc[0]);
				scatter(1u, 1u) += wgt * (relLoc[1] * relLoc[1]);
				sumWgts += wgt;
			}
			scatter(0u, 0u) /= sumWgts;
			scatter(0u, 1u) /= sumWgts;
			scatter(1u, 0u) = scatter(0u, 1u); // symmetric
			scatter(1u, 1u) /= sumWgts;

			mat::Eigen2D const eig(scatter);
			axisMag = eig.valueMax() * eig.vectorMax();

			return axisMag;
		}

	}; // EdgeGroup


	//! Estimated center point given 4 edge locations and directions
	inline
	mea::Vector
	meaVectorFitFrom
		( std::array<img::Vector<double>, 4u> const & edgeLocs
			//!< locations of points on each radial edge
		, std::array<img::Vector<double>, 4u> const & edgeDirs
			//!< Corresponding (radial) directions
		)
	{
		mea::Vector meaVec{};

		// Assemble radial rays from EdgeGroup geometries
		mat::Matrix DtD(2u, 2u);
		img::Vector<double> Dts{ 0., 0. };
		std::fill(DtD.begin(), DtD.end(), 0.);
		for (std::size_t nGrp{0u} ; nGrp < 4u ; ++nGrp)
		{
			using Vec = img::Vector<double>;
			Vec const & si = edgeLocs[nGrp];
			Vec const & di = edgeDirs[nGrp];

			double const & di1 = di[0];
			double const & di2 = di[1];
			double const sidi{ outer(si, di) };

			// accumulate layer of normal system matrix
			DtD(0u, 0u) +=  di2*di2;
			DtD(0u, 1u) += -di2*di1;
			DtD(1u, 0u) += -di1*di2;
			DtD(1u, 1u) +=  di1*di1;

			// accumulate layer of right hand side vector
			Dts.theData[0u] +=  di2*sidi;
			Dts.theData[1u] += -di1*sidi;
		}

		mat::Matrix const invDtD{ mat::inverse2x2(DtD) };
		img::Vector<double> const solnPnt{ invDtD * Dts };

		if (solnPnt.isValid())
		{
			// For unexplained reason, the computations here
			// seem to be half a pixel short in each direction.
			// Therefore, add a half cell to recognize this
			img::Vector<double> const halfCell{ .5, .5 };
			img::Spot const fitLoc{ solnPnt + halfCell };
			//
			mat::Matrix const & covar = invDtD;
			meaVec = mea::Vector(fitLoc, covar);
		}

		return meaVec;
	}

	//! Pseudo-probabilty that ang1 and ang2 are oppositely directed
	inline
	static
	double
	probOpposite
		( img::Vector<double> const & dir1
		, img::Vector<double> const & dir2
		, double const & sigma = 1./4.
		)
	{
		img::Vector<double> const posDir1{  direction(dir1) };
		img::Vector<double> const posDir2{ -direction(dir2) };

		double const difMag{ magnitude(posDir1 - posDir2) };
		double const arg{ difMag / sigma };
		double const prob{ std::exp(-(arg*arg)) };

		return prob;
	}


	/*! \brief Refine center locations with line fitting (to edge magnitudes).
	 *
	 * Upon construction, gradient values are computed for the entire
	 * source image. These values are used for subsequent center candidate
	 * refinements.
	 *
	 */
	class CenterRefinerEdge
	{
		//! Gradient values for each source image cell.
		ras::Grid<img::Grad> const theGradGrid{};

		//! Angle probability data item
		struct AngProb
		{
			double theAng;
			double theProb;
		};

		//! Struct for tracking angles for candidate radial edge directions
		class PeakQuad
		{
			double theAngA1{ std::numeric_limits<double>::quiet_NaN() };
			double theAngB1{ std::numeric_limits<double>::quiet_NaN() };
			double theAngA2{ std::numeric_limits<double>::quiet_NaN() };
			double theAngB2{ std::numeric_limits<double>::quiet_NaN() };

			img::Vector<double> theDirA1{};
			img::Vector<double> theDirB1{};
			img::Vector<double> theDirA2{};
			img::Vector<double> theDirB2{};

		public:

			inline
			explicit
			PeakQuad
				() = default;

			//! establish angle pairing with angProbs (0,2) and (1,3)
			inline
			explicit
			PeakQuad
				( std::vector<AngProb> const & angProbs
					//!< Assumed to be *SORTED* in order of increasing angle!
				)
				: theAngA1{ angProbs[0].theAng }
				, theAngB1{ angProbs[1].theAng }
				, theAngA2{ angProbs[2].theAng }
				, theAngB2{ angProbs[3].theAng }
				, theDirA1{ std::cos(theAngA1), std::sin(theAngA1) }
				, theDirB1{ std::cos(theAngB1), std::sin(theAngB1) }
				, theDirA2{ std::cos(theAngA2), std::sin(theAngA2) }
				, theDirB2{ std::cos(theAngB2), std::sin(theAngB2) }
			{ }

			/*! \brief Direction with which relPos is most aligned
			 *
			 * Indices correspond with theDir[AB][12] as follows:
			 * \arg 0 : theDirA1
			 * \arg 1 : theDirB1
			 * \arg 2 : theDirA2  - should be near (theDirA1 + pi)
			 * \arg 3 : theDirB2  - should be near (theDirB1 + pi)
			 *
			 * Note that (to extend constructor argument is sorted by
			 * increasing angle value), these groups are ordered by
			 * monotonically increasing angle.
			 */
			inline
			std::size_t
			groupIndexFor
				( img::Vector<double> const & relPos
				) const
			{
				std::array<double, 4u> const dotVals
					{ dot(relPos, theDirA1)
					, dot(relPos, theDirB1)
					, dot(relPos, theDirA2)
					, dot(relPos, theDirB2)
					};
				using ItDot = std::array<double, 4u>::const_iterator;
				ItDot const itMax
					{ std::max_element(dotVals.cbegin(), dotVals.cend()) };
				std::iterator_traits<ItDot>::difference_type const itDist
					{ std::distance(dotVals.cbegin(), itMax) };
				return static_cast<std::size_t>(itDist);
			}

			//! Pseudo-probability that the angles form an "X" pattern
			inline
			double
			probability
				() const
			{
				double const probA{ probOpposite(theDirA1, theDirA2) };
				double const probB{ probOpposite(theDirB1, theDirB2) };
				return (probA * probB);
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
					<< "angA1: " << fixed(theAngA1)
					<< ' '
					<< "angA2: " << fixed(theAngA2)
					<< ' '
					<< "dirA1: " << theDirA1
					<< ' '
					<< "dirA2: " << theDirA2
					<< '\n'
					<< "angB1: " << fixed(theAngB1)
					<< ' '
					<< "angB2: " << fixed(theAngB2)
					<< ' '
					<< "dirB1: " << theDirB1
					<< ' '
					<< "dirB2: " << theDirB2
					;

				return oss.str();
			}
		};

		//! True if pattern in peakAngles is "center-like"
		inline
		PeakQuad
		peakQuadFor
			( ops::AngleTracker const & angleTracker
			) const
		{
			PeakQuad peakQuad{};

			// need at least four peaks and four valleys
			std::vector<double> const peakAngles
				{ angleTracker.anglesOfPeaks() };
			std::size_t const numPeaks{ peakAngles.size() };
			if (3u < numPeaks) // expect at least four
			{
				// create (circular) histogram of angle probabilities
				std::vector<std::size_t> const peakNdxs
					{ angleTracker.indicesOfPeaks() };
				std::vector<AngProb> angProbs;
				angProbs.reserve(peakNdxs.size());
				for (std::size_t const & peakNdx : peakNdxs)
				{
					double const angle{ angleTracker.angleAtIndex(peakNdx) };
					double const prob{ angleTracker.probAtIndex(peakNdx) };
					angProbs.emplace_back(AngProb{ angle, prob });
				}

				// there should be four strong, relatively isolated peaks
				// partially sort to put largest four peaks first
				std::partial_sort
					( angProbs.begin()
					, angProbs.begin() + 4
					, angProbs.end()
					, [] (AngProb const & ap1, AngProb const & ap2)
						// Note reverse compare to put larget elements first
						{ return (ap2.theProb < ap1.theProb); }
					);
				// then sort the four largest peaks by increasing angle
				std::sort
					( angProbs.begin()
					, angProbs.begin() + 4
					, [] (AngProb const & ap1, AngProb const & ap2)
						{ return (ap1.theAng < ap2.theAng); }
					);

				// compute stats for angle peaks vs angle background probs
				prb::Stats<float> statsBig{};
				prb::Stats<float> statsSml{};
				for (std::size_t nBig{0u} ; nBig < 4u ; ++nBig)
				{
					statsBig.consider(angProbs[nBig].theProb);
				}
				for (std::size_t nSml{4u} ; nSml < numPeaks ; ++nSml)
				{
					statsSml.consider(angProbs[nSml].theProb);
				}

				// since there are at least four candidates (if test above)
				// the 'big' stats are all valid
				double const minBig{ statsBig.min() };
				double const devBig{ statsBig.deviation() };

				// establish threshold of the smaller (e.g. noise) peaks
				// there may be arbitrary number of small peaks (if any)
				double maxSml{ statsSml.max() };
				if (! engabra::g3::isValid(maxSml))
				{
					maxSml = 0.;
				}
				double devSml{ statsSml.deviation() };
				if (! engabra::g3::isValid(devSml))
				{
					devSml = 0.;
				}

				// require big peaks to be "significantly" larger than small
				double const gotGap{ minBig - maxSml };
				double const signif{ devBig + devSml };
				if (signif < gotGap)
				{
					// assign probability for 4 strongest angle directions
					peakQuad = PeakQuad(angProbs);
				}

			} // (3u < numPeaks)

			return peakQuad;
		}

		//! Use edge values to estimate center location and uncertainty
		inline
		mea::Vector
		meaVectorCenter
			( PeakQuad const & peakQuad
			, std::vector<img::Edgel> const & edgels
			, img::Spot const & nomCenter
			, double const & edgeMagMax
			) const
		{
			mea::Vector meaVec{};

			// classify edgels by direction
			std::array<EdgeGroup, 4u> sampleGroups;
			for (img::Edgel const & edgel : edgels)
			{
				// Compare relative position with QuadPeak directions
				img::Vector<double> const relPos
					{ edgel.location() - nomCenter };
				std::size_t const ndxGrp{ peakQuad.groupIndexFor(relPos) };

				double const magSigma{ .25 * edgeMagMax };
				double const edgeMag{ edgel.magnitude() };
				double const arg{ (edgeMagMax - edgeMag) / magSigma };
				double const weight{ std::exp(-arg*arg) };
				sampleGroups[ndxGrp].add(edgel.location(), weight);
			}

			// centroids of each edge group - to provide point on edges
			std::array<img::Vector<double>, 4u> const edgeLocs
				{ sampleGroups[0].centroid()
				, sampleGroups[1].centroid()
				, sampleGroups[2].centroid()
				, sampleGroups[3].centroid()
				};

			// averge directions from opposite radial edges to get
			// oppositely consistent radial directions
			img::Vector<double> const aveDirA
				{ .5
				* (sampleGroups[0].semiAxisMax(edgeLocs[0])
				+ sampleGroups[2].semiAxisMax(edgeLocs[0]))
				};
			img::Vector<double> const aveDirB
				{ .5
				* (sampleGroups[1].semiAxisMax(edgeLocs[0])
				+ sampleGroups[3].semiAxisMax(edgeLocs[0]))
				};

			// check if radial edge statistica are valid
			if (aveDirA.isValid() && aveDirB.isValid())
			{
				// use opposing edge averages instead of individual ones
				std::array<img::Vector<double>, 4u> const edgeDirs
					{ aveDirA, aveDirB, aveDirA, aveDirB };

				// compute center fit to all four edges
				meaVec = meaVectorFitFrom(edgeLocs, edgeDirs);
			}

			return meaVec;
		}

	public:

		//! Compute and cache gradient values for use in other methods.
		inline
		explicit
		CenterRefinerEdge
			( ras::Grid<float> const & srcGrid
			)
			: theGradGrid{ ops::grid::gradientGridBy8x(srcGrid) }
		{ }

		/*! Use gradient grid values to estimate center point near to nomSpot
		 *
		 * Note: This assumes that nomSpot is sufficiently inside the
		 * gradient grid dimensions (e.g. at least searchRadius cells
		 * from each grid boarder.!!
		 */
		inline
		img::Hit
		imgHitNear
			( ras::RowCol const & rc0
			, std::size_t const & searchRadius
			) const
		{
			img::Hit hit{};
			if (1u < searchRadius)
			{
				// define processing work area
				int const r0{ static_cast<int>(rc0.row()) };
				int const c0{ static_cast<int>(rc0.col()) };
				int const delta{ static_cast<int>(searchRadius) };
				int const rowBeg{ r0 - delta };
				int const rowEnd{ r0 + delta + 1 };
				int const colBeg{ c0 - delta };
				int const colEnd{ c0 + delta + 1 };
				constexpr std::size_t iPi{ 3u }; // approx to pi - within 5% ;-)
				std::size_t const numPeri{ 2u * iPi * searchRadius };

				// evaluate probability that this point is a target center
				img::Spot const nomOrig{ cast::imgSpot(rc0) };

				//! Track edgels for subsequent use
				std::vector<img::Edgel> edgels;
				edgels.reserve(4u * searchRadius * searchRadius);

				// create angle direction accumluation buffer
				// (expecting four strong direction peaks - two pairs of
				// opposing directions)
				ops::AngleTracker angleTracker(numPeri);
				double edgeMagMax{ 0. };
				for (int row{rowBeg} ; row < rowEnd ; ++row)
				{
					for (int col{colBeg} ; col < colEnd ; ++col)
					{
						// TODO - can precompute angles for filter
						// value of gradient at this location

						// location in source
						img::Spot const loc{ (double)row, (double)col };
						img::Vector<double> const relSpot{ loc - nomOrig };

						// TODO Also enforce circle shape
						// gradient element location relative to nominal point
						// add some so that exact searchRadius pixels get used
						double const radius{ magnitude(relSpot) };
						double const radMax // small pad to include exact pix
							{ static_cast<double>(searchRadius + 1./1024.) };
						if ((0. < radius) && (radius < radMax))
						{
							// gradient in source
							img::Grad const & grad = theGradGrid(row, col);
							img::Edgel const edgel(loc, grad);
							edgels.emplace_back(edgel);

							// accumulation direction into angle historgram
							constexpr std::size_t binSigma{ 1u };
							double const edgeMag{ edgel.magnitude() };
							angleTracker.consider
								(edgel.angle(), edgeMag, binSigma);

							if (edgeMagMax < edgeMag)
							{
								edgeMagMax = edgeMag;
							}
						}
					}
				}

				// Determine pseudo probability that this is a legit center
				PeakQuad const peakQuad{ peakQuadFor(angleTracker) };
				double const quadProb{ peakQuad.probability() };
				if (engabra::g3::isValid(quadProb))
				{
					// fit center
					mea::Vector const meaCenter
						{ meaVectorCenter
							(peakQuad, edgels, nomOrig, edgeMagMax)
						};
					img::Spot const centerSpot{ meaCenter.location() };
					double const sigma{ meaCenter.deviationRMS() };
					hit = img::Hit(centerSpot, quadProb, sigma);
				}

			} // (1u < searchRadius)

			return hit;
		}

		//! Compute refine center point hits for all peakRCVs candidates
		inline
		std::vector<img::Hit>
		centerHits
			( std::vector<ras::PeakRCV> const & peakRCVs
				//!< Nominal center candidates
			, std::size_t const & searchRadius
				//!< Radius over which to consider edges
			) const
		{
			std::vector<img::Hit> hits;

			// active area for considering peaks
			std::size_t const hwMin{ 2u * searchRadius + 1u };
			if ((hwMin < theGradGrid.high()) && (hwMin < theGradGrid.wide()))
			{
				val::Span const rowSpan
					{ (double)searchRadius
					, (double)(theGradGrid.high() - searchRadius - 1u)
					};
				val::Span const colSpan
					{ (double)searchRadius
					, (double)(theGradGrid.wide() - searchRadius - 1u)
					};
				img::Area const liveArea{ rowSpan, colSpan };

				// process neighborhoods around each candidate center
				for (ras::PeakRCV const & peakRCV : peakRCVs)
				{
					ras::RowCol const & nomRC = peakRCV.theRowCol;
					img::Spot const nomCenter{ cast::imgSpot(nomRC) };
					if (liveArea.contains(nomCenter))
					{
						img::Hit const hit{ imgHitNear(nomRC, searchRadius) };
						if (hit.isValid())
						{
							hits.emplace_back(hit);
						}
					}
				}

			} // hw < theGradGrid sizes

			return hits;
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
				oss << title << '\n';
			}
			oss << "theGradGrid: " << theGradGrid << '\n';
			return oss.str();
		}


	}; // CenterRefinerEdge


} // [ops]

} // [quadloco]

namespace
{

	//! Put instance to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ops::CenterRefinerEdge const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

} // [anon/global]

