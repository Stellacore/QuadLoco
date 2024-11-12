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


#include "datGrid.hpp"
#include "datSpot.hpp"
#include "imgQuadTarget.hpp"
#include "pixGradel.hpp"
#include "pixgrid.hpp"

#include <limits>
#include <vector>


namespace quadloco
{

namespace prb
{
	//! Tracking class to monitor min/max of multiple collections
	template <typename Type>
	class Stats
	{
		std::size_t theCount{ 0u };
		Type theSum{ 0 };
		Type theMin{ std::numeric_limits<Type>::max() };
		Type theMax{ std::numeric_limits<Type>::lowest() };

	public:

		//! Default construction (empty content) - use consider() to update
		inline
		Stats
			() = default;

		//! Construct by considering this collection
		template <typename FwdIter>
		inline
		Stats
			( FwdIter const & beg
			, FwdIter const & end
			)
		{
			consider(beg, end);
		}

		//! Smallest element considered
		inline
		Type const &
		min
			() const
		{
			return theMin;
		}

		//! Smallest element considered
		inline
		Type
		mean
			() const
		{
			Type result{ pix::null<Type>() };
			if (0u < theCount)
			{
				result = (Type)((1./(double)theCount) * theSum);
			}
			return result;
		}

		//! Largest element considered
		inline
		Type const &
		max
			() const
		{
			return theMin;
		}

		//! Adjust the current min/max values to accomodate all samps
		template <typename FwdIter>
		inline
		void
		consider
			( FwdIter const & beg
			, FwdIter const & end
			)
		{
			if (end != beg)
			{
				// update the sum
				theSum = std::accumulate(beg, end, theSum);
				theCount += (std::size_t)(std::distance(beg, end));
				// update the min/max
				std::pair<FwdIter, FwdIter>
					const itPair{ std::minmax_element(beg, end) };
				Type const & valMin = *(itPair.first);
				if (valMin < theMin)
				{
					theMin = valMin;
				}
				Type const & valMax = *(itPair.second);
				if (theMax < valMax)
				{
					theMax = valMax;
				}
			}
		}
	};


	/*
	//! Average of sample values
	template <typename Type>
	inline
	Type
	mean
		( std::vector<Type> const & samps
		)
	{
		Type ave{ pix::null<Type>() };
		constexpr Type zero{ 0 };
		Type const sum
			{ std::accumulate(samps.cbegin(), samps.cend(), zero) };
		if (! samps.empty()) // need at least one sample for average
		{
			double count{ (double)samps.size() };
			ave = (Type)((1./count) * sum);
		}
		return ave;
	}
	*/

	//! Standard deviation of sample values about given mean
	template <typename Type>
	inline
	Type
	deviation
		( std::vector<Type> const & samps
		, Type const & mean
		)
	{
		Type dev{ pix::null<Type>() };
		if (1u < samps.size()) // need at least one sample for average
		{
			Type sumSq{ 0 };
			for (std::size_t nn{0u} ; nn < samps.size() ; ++nn)
			{
				Type const dev{ samps[nn] - mean };
				sumSq = sumSq + (dev * dev);
			}
			double const dof{ (double)(samps.size() - 1u ) };
			dev = std::sqrt((Type)((1./dof) * sumSq));
		}
		return dev;
	}

	/*
	//! Average of sample values
	template <typename Type>
	inline
	std::pair<Type, Type>
	minmax
		( std::vector<Type> const & samps
		)
	{
		Stats<Type> minmax;
		minmax.consider(samps.cbegin(), samps.cend());
		return { minmax.min(), minmax.max() };
	}
	*/

	/*
	//! Sample statistics
	template <typename Type>
	struct SampStats
	{
		Stats<Type> theStats;
		Type theDev;

		//! Construct from sample values
		inline
		SampStats
			( std::vector<Type> const & samps
			)
			: theStats{ Stats<Type>(samps.cbegin(), samps.cend()) }
			, theDev{ deviation(samps, theStats.mean()) }
		{ }

	}; // SampStats
	*/


	//! Show information about sample values
	template <typename Type>
	inline
	void
	showSampsReal
		( std::vector<Type> const & samps
		, Stats<Type> const & stats
		, std::string const & title
		)
	{
		std::cout << title << '\n';
		std::cout << "---size: (" << samps.size() << ")\n";

		std::cout << "...";
		for (std::size_t nn{0u} ; nn < samps.size() ; ++nn)
		{
			std::cout << ' ' << samps[nn];
		}

		Type const dev{ deviation(samps, stats.mean()) };

		std::cout << '\n';
		std::cout << "...min: " << stats.min() << '\n';
		std::cout << "...ave: " << stats.mean() << '\n';
		std::cout << "...max: " << stats.max() << '\n';
		std::cout << "...dev: " << dev << '\n';
	}

	//! A (pseudo)probabilty values in pixGrid conform with image quad signal
	template <typename Type>
	inline
	double
	isQuadlike
		( dat::Grid<Type> const & pixGrid
		, img::QuadTarget const & imgQuad
		)
	{
		// samples drawn from the signal flat areas (radially between edges)
		std::vector<Type> ppVals{};
		std::vector<Type> npVals{};
		std::vector<Type> nnVals{};
		std::vector<Type> pnVals{};

		using namespace engabra::g3;
		Vector const & orig = imgQuad.theCenter;
		Vector const & xpDir = imgQuad.theDirX;
		Vector const & ypDir = imgQuad.theDirY;
		Vector const xnDir{ -xpDir };
		Vector const ynDir{ -ypDir };
		Vector const ppDir{ direction(xpDir + ypDir) }; // quad TR
		Vector const npDir{ direction(xnDir + ypDir) }; // quad TL
		Vector const pnDir{ direction(xpDir + ynDir) }; // quad BL
		Vector const nnDir{ direction(xnDir + ynDir) }; // quad BR

std::cout << "pixGrid: " << pixGrid << '\n';
std::cout << "imgQuad: " << imgQuad << '\n';

		// follow radial edge from center outward
		double const rad0{ 2. }; // start away from center (poor edge define)
		double const maxRad // way conservative max radius
			{ std::hypot((double)pixGrid.high(), (double)pixGrid.wide()) };
		constexpr double dr{ 1. };

		std::size_t const maxSize{ (std::size_t)(std::ceil(maxRad / dr)) };
		ppVals.reserve(maxSize);
		npVals.reserve(maxSize);
		nnVals.reserve(maxSize);
		pnVals.reserve(maxSize);

		for (double rad{rad0} ; rad < maxRad ; rad += dr)
		{
			// radial edge samples
			Vector const xpLoc{ orig + rad * xpDir };
			Vector const xnLoc{ orig + rad * xnDir };
			Vector const ypLoc{ orig + rad * ypDir };
			Vector const ynLoc{ orig + rad * ynDir };
			// radial flat samples
			Vector const ppLoc{ .5 * (xpLoc + ypLoc) };
			Vector const npLoc{ .5 * (xnLoc + ypLoc) };
			Vector const nnLoc{ .5 * (xnLoc + ynLoc) };
			Vector const pnLoc{ .5 * (xpLoc + ynLoc) };

			dat::Spot const ppSpot{ ppLoc[0], ppLoc[1] };
			dat::Spot const npSpot{ npLoc[0], npLoc[1] };
			dat::Spot const nnSpot{ nnLoc[0], nnLoc[1] };
			dat::Spot const pnSpot{ pnLoc[0], pnLoc[1] };

			// TODO - could probably break loop if all four are invalid

			Type const ppVal{ pix::grid::bilinValueAt<Type>(pixGrid, ppSpot) };
			if (isValid(ppVal))
			{
				ppVals.emplace_back(ppVal);
			}

			Type const npVal{ pix::grid::bilinValueAt<Type>(pixGrid, npSpot) };
			if (isValid(npVal))
			{
				npVals.emplace_back(npVal);
			}

			Type const nnVal{ pix::grid::bilinValueAt<Type>(pixGrid, nnSpot) };
			if (isValid(nnVal))
			{
				nnVals.emplace_back(nnVal);
			}

			Type const pnVal{ pix::grid::bilinValueAt<Type>(pixGrid, pnSpot) };
			if (isValid(pnVal))
			{
				pnVals.emplace_back(pnVal);
			}
		}

		std::vector<Type> allVals{};
		allVals.insert(allVals.end(), ppVals.cbegin(), ppVals.cend());
		allVals.insert(allVals.end(), npVals.cbegin(), npVals.cend());
		allVals.insert(allVals.end(), nnVals.cbegin(), nnVals.cend());
		allVals.insert(allVals.end(), pnVals.cbegin(), pnVals.cend());


		Stats<Type> allStats{};

		Stats<Type> ppStats{};
		ppStats.consider(ppVals.cbegin(), ppVals.cend());
		allStats.consider(ppVals.cbegin(), ppVals.cend());

		Stats<Type> npStats{};
		npStats.consider(npVals.cbegin(), npVals.cend());
		allStats.consider(npVals.cbegin(), npVals.cend());

		Stats<Type> nnStats{};
		nnStats.consider(nnVals.cbegin(), nnVals.cend());
		allStats.consider(nnVals.cbegin(), nnVals.cend());

		Stats<Type> pnStats{};
		pnStats.consider(pnVals.cbegin(), pnVals.cend());
		allStats.consider(pnVals.cbegin(), pnVals.cend());


std::cout << '\n';
showSampsReal(ppVals, ppStats, "\npp");
showSampsReal(npVals, npStats, "\nnp");
showSampsReal(nnVals, nnStats, "\nnn");
showSampsReal(pnVals, pnStats, "\npn");
showSampsReal(allVals, allStats, "\nall");



		// edge groups should exhibit rotational symmetry

		// flat groups should exhibit +-+- symmetry

		return .0; // TODO
	}


} // [prb]

} // [quadloco]

