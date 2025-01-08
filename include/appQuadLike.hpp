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
 * \brief Declarations functions and utility for assessing "quadness"
 *
 */


#include "imgQuadTarget.hpp"
#include "imgSpot.hpp"
#include "prbStats.hpp"
#include "rasgrid.hpp"
#include "rasGrid.hpp"

#include <limits>
#include <vector>


namespace quadloco
{

namespace app
{
	//! Samples drawn from symmetry radii of quad squares
	template <typename Type>
	struct SquareRadiiSamples
	{
		// samples drawn from the signal flat areas (radially between edges)
		std::vector<Type> thePPs{};  // TR square
		std::vector<Type> theNPs{};  // TL square
		std::vector<Type> theNNs{};  // BL square
		std::vector<Type> thePNs{};  // BR square


		//! True if each radii contains at least this many samples
		inline
		bool
		isSignificant
			( std::size_t const & minNum = 2u
			) const
		{
			bool const tooFew
				{  (thePPs.size() < minNum)
				&& (theNPs.size() < minNum)
				&& (theNNs.size() < minNum)
				&& (thePNs.size() < minNum)
				};
			return (! tooFew);
		}

		//! Extract samples from grid based on imgQuad geometry
		inline
		static
		SquareRadiiSamples
		from
			( ras::Grid<Type> const & pixGrid
			, img::QuadTarget const & imgQuad
			)
		{
			std::vector<Type> ppVals{};
			std::vector<Type> npVals{};
			std::vector<Type> nnVals{};
			std::vector<Type> pnVals{};

			using Vector = quadloco::img::Vector<double>;

			Vector const & orig = imgQuad.theCenter;
			Vector const & xpDir = imgQuad.theDirX;
			Vector const & ypDir = imgQuad.theDirY;
			Vector const xnDir{ -xpDir };
			Vector const ynDir{ -ypDir };
			Vector const ppDir{ direction(xpDir + ypDir) }; // quad TR
			Vector const npDir{ direction(xnDir + ypDir) }; // quad TL
			Vector const pnDir{ direction(xpDir + ynDir) }; // quad BL
			Vector const nnDir{ direction(xnDir + ynDir) }; // quad BR

			// follow radial edge from center outward
			// start away from center (poor edge define)
			double const rad0{ 2. };
			// use diagonal as worst case radius (but expect less)
			double const maxRad
				{ std::hypot((double)pixGrid.high(), (double)pixGrid.wide()) };
			constexpr double dr{ 1. };

			std::size_t const maxSize{ (std::size_t)(std::ceil(maxRad / dr)) };
			ppVals.reserve(maxSize);
			npVals.reserve(maxSize);
			nnVals.reserve(maxSize);
			pnVals.reserve(maxSize);

			for (double rad{rad0} ; rad < maxRad ; rad += dr)
			{
				// radial edge locations
				Vector const xpLoc{ orig + rad * xpDir };
				Vector const xnLoc{ orig + rad * xnDir };
				Vector const ypLoc{ orig + rad * ypDir };
				Vector const ynLoc{ orig + rad * ynDir };

				// radial flat locations (midway between edges
				Vector const ppLoc{ .5 * (xpLoc + ypLoc) };
				Vector const npLoc{ .5 * (xnLoc + ypLoc) };
				Vector const nnLoc{ .5 * (xnLoc + ynLoc) };
				Vector const pnLoc{ .5 * (xpLoc + ynLoc) };

				// radial flat sample spots (cast from Vector)
				img::Spot const ppSpot{ ppLoc[0], ppLoc[1] };
				img::Spot const npSpot{ npLoc[0], npLoc[1] };
				img::Spot const nnSpot{ nnLoc[0], nnLoc[1] };
				img::Spot const pnSpot{ pnLoc[0], pnLoc[1] };


				std::size_t numValid{ 0u };

				Type const ppVal
					{ ras::grid::bilinValueAt<Type>(pixGrid, ppSpot) };
				if (img::isValidType(ppVal))
				{
					ppVals.emplace_back(ppVal);
					++numValid;
				}

				Type const npVal
					{ ras::grid::bilinValueAt<Type>(pixGrid, npSpot) };
				if (img::isValidType(npVal))
				{
					npVals.emplace_back(npVal);
					++numValid;
				}

				Type const nnVal
					{ ras::grid::bilinValueAt<Type>(pixGrid, nnSpot) };
				if (img::isValidType(nnVal))
				{
					nnVals.emplace_back(nnVal);
					++numValid;
				}

				Type const pnVal
					{ ras::grid::bilinValueAt<Type>(pixGrid, pnSpot) };
				if (img::isValidType(pnVal))
				{
					pnVals.emplace_back(pnVal);
					++numValid;
				}

				// If all four radial samples are invalid, then
				// likely have run past the size of the data grid
				if (0u == numValid)
				{
					break;
				}
			}

			return SquareRadiiSamples{ ppVals, npVals, nnVals, pnVals };
		}

		//! Catenation of all samples into a single array
		inline
		std::vector<Type>
		allSamps
			() const
		{
			std::vector<Type> allVals{};
			allVals.insert(allVals.end(), thePPs.cbegin(), thePPs.cend());
			allVals.insert(allVals.end(), theNPs.cbegin(), theNPs.cend());
			allVals.insert(allVals.end(), theNNs.cbegin(), theNNs.cend());
			allVals.insert(allVals.end(), thePNs.cbegin(), thePNs.cend());
			return allVals;
		}

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
			oss << "counts: "
				<< " PP: " << thePPs.size()
				<< " NP: " << theNPs.size()
				<< " NN: " << theNNs.size()
				<< " PN: " << thePNs.size()
				;
			return oss.str();
		}

	}; // SquareRadiiSamples


	//! Basic statistics for each square flat and all samples together
	template <typename Type>
	struct QuadSampleStats
	{
		prb::Stats<Type> thePP{};
		prb::Stats<Type> theNP{};
		prb::Stats<Type> theNN{};
		prb::Stats<Type> thePN{};
		prb::Stats<Type> theAll{};

		inline
		static
		QuadSampleStats
		from
			( SquareRadiiSamples<Type> const & samps
			)
		{
			// TODO Could be optimized (consider everything in one place

			prb::Stats<Type> ppStats{};
			ppStats.consider(samps.thePPs.cbegin(), samps.thePPs.cend());

			prb::Stats<Type> npStats{};
			npStats.consider(samps.theNPs.cbegin(), samps.theNPs.cend());

			prb::Stats<Type> nnStats{};
			nnStats.consider(samps.theNNs.cbegin(), samps.theNNs.cend());

			prb::Stats<Type> pnStats{};
			pnStats.consider(samps.thePNs.cbegin(), samps.thePNs.cend());

			prb::Stats<Type> allStats{};
			allStats.consider(samps.thePPs.cbegin(), samps.thePPs.cend());
			allStats.consider(samps.theNPs.cbegin(), samps.theNPs.cend());
			allStats.consider(samps.theNNs.cbegin(), samps.theNNs.cend());
			allStats.consider(samps.thePNs.cbegin(), samps.thePNs.cend());

			return QuadSampleStats
				{ ppStats, npStats, nnStats, pnStats, allStats };
		}

	}; // QuadSampleStats


	//! A (pseudo)probabilty values in pixGrid conform with image quad signal
	template <typename Type>
	inline
	double
	isQuadlike
		( ras::Grid<Type> const & pixGrid
		, img::QuadTarget const & imgQuad
		, std::ostream * const & ptMessages = nullptr
		)
	{
		double quadProb{ 0. };
		std::ostringstream strMsg;
		std::ostringstream strData;

		// sample grid along square symmetry radii
		SquareRadiiSamples<Type> const radSamps
			{ SquareRadiiSamples<Type>::from(pixGrid, imgQuad) };

		if (radSamps.isSignificant())
		{
			// compute statistics for the samples
			QuadSampleStats<Type> const stats
				{ QuadSampleStats<Type>::from(radSamps) };

			// use statistics to estimate "quadness"

			// check for variation of any kind (e.g. not null or const grid)
			Type const minAll{ stats.theAll.min() };
			Type const maxAll{ stats.theAll.max() };
			Type const rangeAll{ maxAll - minAll };

			strData << "@@@@ statsAll: " << stats.theAll.infoString() << '\n';
			strData << "@@@@ minAll: " << minAll << '\n';
			strData << "@@@@ maxAll: " << maxAll << '\n';
			strData << "@@@@ rangeAll: " << rangeAll << '\n';

			if (std::numeric_limits<Type>::epsilon() < std::abs(rangeAll))
			{
				Type const meanPP{ stats.thePP.mean() };
				Type const meanNP{ stats.theNP.mean() };
				Type const meanNN{ stats.theNN.mean() };
				Type const meanPN{ stats.thePN.mean() };
				Type const meanBack{ (Type)(.5 * (meanPP + meanNN)) };
				Type const meanFore{ (Type)(.5 * (meanNP + meanPN)) };

				// check overall signal
				Type const sigMag{ meanFore - meanBack };

				strData << "#### meanPP: " << meanPP << '\n';
				strData << "#### meanNP: " << meanNP << '\n';
				strData << "#### meanNN: " << meanNN << '\n';
				strData << "#### meanPN: " << meanPN << '\n';
				strData << "#### meanBack: " << meanBack << '\n';
				strData << "#### meanFore: " << meanFore << '\n';
				strData << "#### sigMag: " << sigMag << '\n';

				if (std::numeric_limits<Type>::epsilon() < std::abs(sigMag))
				{
					Type const invRange{ (Type)(1. / rangeAll) };
					Type const invSigMag{ (Type)(1. / sigMag) };

					// pseudoProb that signal is significant
					Type const fracSig{ invRange * (meanFore - meanBack) };

					// pseudoProb that foreground squares are uniform
					Type const argBack{ invSigMag * (meanPP - meanNN) };
					Type const argFore{ invSigMag * (meanNP - meanPN) };

					// NOTE: Code assumes 'real' type (e.g. not complex, etc)

					Type const probBack{ std::exp(-(argBack*argBack)) };
					Type const probFore{ std::exp(-(argFore*argFore)) };

					// pseudo-voodoo
					quadProb = fracSig * probBack * probFore;

					strData << "==== invRange: " << invRange << '\n';
					strData << "==== invSigMag: " << invSigMag << '\n';
					strData << "==== fracSig: " << fracSig << '\n';
					strData << "==== argBack: " << argBack << '\n';
					strData << "==== argFore: " << argFore << '\n';
					strData << "==== quadProb: " << quadProb << '\n';

				}
				else // std::abs(sigMag) < epsilon
				{
					strMsg << "small signal distinction\n";
					strMsg << "sigMag: "  << sigMag << '\n';
					strMsg << "meanBack: "  << meanBack << '\n';
					strMsg << "meanFore: "  << meanFore << '\n';
				}
			}
			else // std::abs(rangeAll) < epsilon
			{
				strMsg << "small overall data range\n";
				strMsg << "rangeAll: "  << rangeAll << '\n';
				strMsg << "maxAll: "  << maxAll << '\n';
				strMsg << "maxAll: "  << maxAll << '\n';
				strMsg << "statsAll: " << stats.theAll.infoString() << '\n';
			}
		}
		else // if (radSamps.isSignificant())
		{
			strMsg << "Insufficient number or radial samples\n";
			strMsg << "radSamps: " << radSamps.infoString() << '\n';
		}

		// if something went wrong, append data values to message string
		if (! strMsg.str().empty())
		{
			strMsg << "DataValues\n" << strData.str();
		}

		if (ptMessages)
		{
			(*ptMessages) << strMsg.str() << '\n';
		}

		return quadProb;
	}


} // [app]

} // [quadloco]

