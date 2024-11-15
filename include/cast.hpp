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
 * \brief Top level file for quadloco::cast namespace
 *
 */


#include "pixSpot.hpp"
#include "pixGrad.hpp"
#include "datRowCol.hpp"
#include "datSpot.hpp"
#include "datVec2D.hpp"

#include <Engabra>

#include <cmath>



namespace quadloco
{

//! \brief Namespace for casting operations between data types
namespace cast
{
	//
	// To engabra::g3::Vector
	//

	//! Engabra Vector: [0,1] from (row(),col()) , [2] set to zero.
	inline
	engabra::g3::Vector
	vector
		( dat::RowCol const & rowcol
		)
	{
		return engabra::g3::Vector
			{ (double)rowcol.row(), (double)rowcol.col(), 0. };
	}

	//! Engabra Vector: [0,1] from spot[0,1], [2] set to zero.
	inline
	engabra::g3::Vector
	vector
		( dat::Spot const & spot
		)
	{
		return engabra::g3::Vector{ spot[0], spot[1], 0. };
	}

	//! Engabra Vector: [0,1] from vec[0,1], with output [2] set to zero.
	inline
	engabra::g3::Vector
	vector
		( dat::Vec2D<double> const & vec
		)
	{
		return engabra::g3::Vector{ vec[0], vec[1], 0. };
	}

	//! Engabra Vector: [0,1] from gradient element, [2] set to zero.
	inline
	engabra::g3::Vector
	vector
		( pix::Grad const & grad
		)
	{
		return engabra::g3::Vector{ grad[0], grad[1], 0. };
	}


	//
	// To dat::RowCol
	//

	//! Integral values that are the std::floor values of vec[0,1]
	inline
	dat::RowCol
	datRowCol
		( engabra::g3::Vector const & vec
		)
	{
		return dat::RowCol
			{ static_cast<std::size_t>(std::floor(vec[0]))
			, static_cast<std::size_t>(std::floor(vec[1]))
			};
	}

	//! Integral values that are the std::floor values of datSpot[0,1]
	inline
	dat::RowCol
	datRowCol
		( dat::Spot const & datSpot
		)
	{
		return dat::RowCol
			{ static_cast<std::size_t>(std::floor(datSpot[0]))
			, static_cast<std::size_t>(std::floor(datSpot[1]))
			};
	}

	//! Integral values that are the std::floor values of pixSpot[0,1]
	inline
	dat::RowCol
	datRowCol
		( pix::Spot const & pixSpot
		)
	{
		return dat::RowCol
			{ static_cast<std::size_t>(std::floorf(pixSpot[0]))
			, static_cast<std::size_t>(std::floorf(pixSpot[1]))
			};
	}

	//
	// To dat::Vec2D<Type>
	//

	//! The first two components, vec[0,1]
	template <typename Type>
	inline
	dat::Vec2D<Type>
	datVec2D
		( engabra::g3::Vector const & vec
		)
	{
		return dat::Vec2D<Type>
			{ vec[0]
			, vec[1]
			};
	}

	//
	// To dat::Spot
	//

	//! The first two components, vec[0,1]
	inline
	dat::Spot
	datSpot
		( engabra::g3::Vector const & vec
		)
	{
		return dat::Spot{ vec[0], vec[1] };
	}

	//! The first two components, vec[0,1]
	inline
	dat::Spot
	datSpot
		( dat::RowCol const & rowcol
		)
	{
		return dat::Spot
			{ static_cast<double>(rowcol.row())
			, static_cast<double>(rowcol.col())
			};
	}

	//! The first two components, vec[0,1]
	inline
	dat::Spot
	datSpot
		( pix::Spot const & pixSpot
		)
	{
		return dat::Spot
			{ static_cast<double>(pixSpot[0])
			, static_cast<double>(pixSpot[1])
			};
	}

	//
	// To pix::Spot
	//

	//! Float cast of the first two components, vec[0,1]
	inline
	pix::Spot
	pixSpot
		( engabra::g3::Vector const & vec
		)
	{
		return pix::Spot
			{ static_cast<float>(vec[0])
			, static_cast<float>(vec[1])
			};
	}

	//! Float cast of the first two components, vec[0,1]
	inline
	pix::Spot
	pixSpot
		( dat::RowCol const & rowcol
		)
	{
		return pix::Spot
			{ static_cast<float>(std::floor(rowcol.row()))
			, static_cast<float>(std::floor(rowcol.col()))
			};
	}

	//! Float cast of the first two components, vec[0,1]
	inline
	pix::Spot
	pixSpot
		( dat::Spot const & datSpot
		)
	{
		return pix::Spot
			{ static_cast<float>(datSpot[0])
			, static_cast<float>(datSpot[1])
			};
	}

	//
	// To pix::Grad
	//

	//! Float cast of the first two components, vec[0,1]
	inline
	pix::Grad
	pixGrad
		( engabra::g3::Vector const & vec
		)
	{
		return pix::Grad
			{ static_cast<float>(vec[0])
			, static_cast<float>(vec[1])
			};
	}

} // [cast]

} // [quadloco]

