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


#include "QuadLoco/pix.hpp"

#include <Engabra>

#include <algorithm>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>


namespace quadloco
{

namespace prb
{

	//! Tracking class to monitor min/max/ave/var of multiple collections
	template <typename Type>
	class Stats
	{
		std::size_t theCount{ 0u };
		Type theMin{ std::numeric_limits<Type>::max() };
		Type theMax{ std::numeric_limits<Type>::lowest() };

		Type theMean{ std::numeric_limits<Type>::quiet_NaN() };
		Type theVar{ 0 };

	public: // static

		//! Standard deviation of sample values about given mean
		inline
		static
		Type
		variance
			( std::vector<Type> const & samps
			, Type const & mean
			)
		{
			Type var{ pix::null<Type>() };
			if (1u < samps.size()) // need at least one sample for average
			{
				Type sumSq{ 0 };
				std::size_t count{ 0 };
				for (std::size_t nn{0u} ; nn < samps.size() ; ++nn)
				{
					Type const & value = samps[nn];
					if (engabra::g3::isValid(value))
					{
						Type const dev{ value - mean };
						sumSq = sumSq + (dev * dev);
						++count;
					}
				}
				double const dof{ (double)(count - 1u ) };
				var = static_cast<Type>(1./dof) * sumSq;
			}
			return var;
		}

		//! Standard deviation of sample values about given mean
		inline
		static
		Type
		deviation
			( std::vector<Type> const & samps
			, Type const & mean
			)
		{
			return std::sqrt(variance(samps, mean));
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

		/*! \brief Adjust the current min/max values to accomodate all samps
		 *
		 * NOTE: null values (NaN==value) are *NOT* considered
		 */
		inline
		void
		consider
			( Type const & value
			)
		{
			if (engabra::g3::isValid(value))
			{
				// update the count
				++theCount;

				// update running stats
				if (1u == theCount)
				{
					theMean = value;
					theVar = 0.; // initialize for recurrsive use next time
				}
				else
				{
					Type const prevMean{ theMean };
					Type const prevVar{ theVar };

					// Welford's method
					theMean = prevMean + (value - prevMean)/theCount; 
					theVar = prevVar + (value-prevMean)*(value - theMean);
				}

				// update the tracking stats
				if (value < theMin)
				{
					theMin = value;
				}
				if (theMax < value)
				{
					theMax = value;
				}
			}
		}

		/*! \brief Adjust the current min/max values to accomodate all samps
		 *
		 * NOTE: null values (NaN==value) are *NOT* considered
		 */
		template <typename FwdIter>
		inline
		void
		consider
			( FwdIter const & beg
			, FwdIter const & end
			)
		{
			for (FwdIter iter{beg} ; end != iter ; ++iter)
			{
				consider(*iter);
			}
		}

		//! True if this instance contains at least one data item
		inline
		bool
		isValid
			() const
		{
			return (0u < theCount);
		}

		//! Smallest element considered thus far
		inline
		Type
		min
			() const
		{
			Type result{ std::numeric_limits<Type>::quiet_NaN() };
			if (0u < theCount)
			{
				result = theMin;
			}
			return result;
		}

		//! Largest element considered thus far
		inline
		Type
		max
			() const
		{
			Type result{ std::numeric_limits<Type>::quiet_NaN() };
			if (0u < theCount)
			{
				result = theMax;
			}
			return result;
		}

		//! Difference max()-min()
		inline
		Type
		range
			() const
		{
			Type result{ std::numeric_limits<Type>::quiet_NaN() };
			if (0u < theCount)
			{
				result = max() - min();
			}
			return result;
		}

		//! Average of elements considered thus far
		inline
		Type
		mean
			() const
		{
			Type result{ std::numeric_limits<Type>::quiet_NaN() };
			if (0u < theCount)
			{
				result = theMean;
			}
			return result;
		}

		//! Variance of elements considered thus far
		inline
		Type
		variance
			() const
		{
			Type result{ std::numeric_limits<Type>::quiet_NaN() };
			if (1u < theCount)
			{
				result = theVar / static_cast<Type>(theCount - 1u);
			}
			return result;
		}

		//! Variance of elements considered thus far
		inline
		Type
		deviation
			() const
		{
			return std::sqrt(variance());
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
				oss << title << '\n';
			}
			using engabra::g3::io::fixed;
			oss
				<< " count: " << theCount
				<< '\n'
				<< "   min: " << fixed(min())
				<< '\n'
				<< "  mean: " << fixed(mean())
				<< '\n'
				<< "   max: " << fixed(max())
				<< '\n'
				<< "   dev: " << fixed(deviation())
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

