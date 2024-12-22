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
 * \brief Declarations for quadloco::prb::Stats
 *
 */


#include "opsgrid.hpp"
#include "pix.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <utility>


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

	public: // static

		//! Standard deviation of sample values about given mean
		inline
		static
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
			return theMax;
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
					const itPair{ ops::grid::minmax_valid(beg, end) };
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

		//! Descriptive information about this instance
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
				<< "count: " << theCount
				<< ' '
				<< "sum: " << theSum
				<< ' '
				<< "min: " << min()
				<< ' '
				<< "mean: " << mean()
				<< ' '
				<< "max: " << max()
				;
			return oss.str();
		}

	}; // Stats

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


} // [prb]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	template <typename Type>
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::prb::Stats<Type> const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

} // [anon/global]

