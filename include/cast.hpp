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
#include "imgGrad.hpp"
#include "rasRowCol.hpp"
#include "imgSpot.hpp"
#include "imgVec2D.hpp"

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
		( ras::RowCol const & rowcol
		)
	{
		return engabra::g3::Vector
			{ (double)rowcol.row(), (double)rowcol.col(), 0. };
	}

	//! Engabra Vector: [0,1] from spot[0,1], [2] set to zero.
	inline
	engabra::g3::Vector
	vector
		( img::Spot const & spot
		)
	{
		return engabra::g3::Vector{ spot[0], spot[1], 0. };
	}

	//! Engabra Vector: [0,1] from vec[0,1], with output [2] set to zero.
	inline
	engabra::g3::Vector
	vector
		( img::Vec2D<double> const & vec
		)
	{
		return engabra::g3::Vector{ vec[0], vec[1], 0. };
	}

	//! Engabra Vector: [0,1] from gradient element, [2] set to zero.
	inline
	engabra::g3::Vector
	vector
		( img::Grad const & grad
		)
	{
		return engabra::g3::Vector{ grad[0], grad[1], 0. };
	}


	//
	// To ras::RowCol
	//

	//! Integral values that are the std::floor values of vec[0,1]
	inline
	ras::RowCol
	datRowCol
		( engabra::g3::Vector const & vec
		)
	{
		return ras::RowCol
			{ static_cast<std::size_t>(std::floor(vec[0]))
			, static_cast<std::size_t>(std::floor(vec[1]))
			};
	}

	//! Integral values that are the std::floor values of datSpot[0,1]
	template <typename Type>
	inline
	ras::RowCol
	datRowCol
		( img::Vec2D<Type> const & vec2D
		)
	{
		return ras::RowCol
			{ static_cast<std::size_t>(std::floor(vec2D[0]))
			, static_cast<std::size_t>(std::floor(vec2D[1]))
			};
	}

	//! Integral values that are the std::floor values of datSpot[0,1]
	inline
	ras::RowCol
	datRowCol
		( img::Spot const & datSpot
		)
	{
		return ras::RowCol
			{ static_cast<std::size_t>(std::floor(datSpot[0]))
			, static_cast<std::size_t>(std::floor(datSpot[1]))
			};
	}

	//! Integral values that are the std::floor values of pixSpot[0,1]
	inline
	ras::RowCol
	datRowCol
		( pix::Spot const & pixSpot
		)
	{
		return ras::RowCol
			{ static_cast<std::size_t>(std::floorf(pixSpot[0]))
			, static_cast<std::size_t>(std::floorf(pixSpot[1]))
			};
	}

	//
	// To img::Vec2D<Type>
	//

	//! The first two components, vec[0,1]
	template <typename Type>
	inline
	img::Vec2D<Type>
	datVec2D
		( engabra::g3::Vector const & vec
		)
	{
		return img::Vec2D<Type>
			{ vec[0]
			, vec[1]
			};
	}

	//
	// To img::Spot
	//

	//! The first two components, vec[0,1]
	inline
	img::Spot
	datSpot
		( engabra::g3::Vector const & vec
		)
	{
		return img::Spot{ vec[0], vec[1] };
	}

	//! The first two components, vec[0,1]
	inline
	img::Spot
	datSpot
		( ras::RowCol const & rowcol
		)
	{
		return img::Spot
			{ static_cast<double>(rowcol.row())
			, static_cast<double>(rowcol.col())
			};
	}

	//! The first two components, vec[0,1]
	inline
	img::Spot
	datSpot
		( pix::Spot const & pixSpot
		)
	{
		return img::Spot
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
		( ras::RowCol const & rowcol
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
		( img::Spot const & datSpot
		)
	{
		return pix::Spot
			{ static_cast<float>(datSpot[0])
			, static_cast<float>(datSpot[1])
			};
	}

	//
	// To img::Grad
	//

	//! Float cast of the first two components, vec[0,1]
	inline
	img::Grad
	pixGrad
		( engabra::g3::Vector const & vec
		)
	{
		return img::Grad
			{ static_cast<float>(vec[0])
			, static_cast<float>(vec[1])
			};
	}

} // [cast]

} // [quadloco]

