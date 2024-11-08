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
 * \brief Simulation namespace top level header.
 *
 */


#include "datGrid.hpp"
#include "imgCamera.hpp"
#include "objQuadTarget.hpp"
#include "simSampler.hpp"

#include <Rigibra>

#include <random>


namespace quadloco
{

/*! \brief Functions and utilities for supporting simulation
 */
namespace sim
{

	/*
	//! Estimate approximate image scale
	inline
	double
	approxPixPerObj
		( img::Camera const & camera
		, rigibra::Transform const & xCamWrtQuad
		)
	{
		using namespace engabra::g3;
		double scalePixPerObj{ null<double>() };
		// arbitrary object space displacement
		constexpr double objDelta{ 1./1024./1024. }; // should be reasonable
		static Vector const objPoInQuad{ zero<Vector>() };
		static Vector const objPxInQuad{ objDelta * e1 };
		static double const objMag{ magnitude(objPxInQuad - objPoInQuad) };
		// transform into camera exterior frame
		Vector const objPoInExt{ xCamWrtQuad(objPoInQuad) };
		Vector const objPxInExt{ xCamWrtQuad(objPxInQuad) };
		// corresponding image space displacement
		dat::Spot const imgPo{ camera.detectorSpotFor(objPoInExt) };
		dat::Spot const imgPx{ camera.detectorSpotFor(objPxInExt) };
		if (imgPo.isValid() && imgPx.isValid())
		{
			double const imgMag{ magnitude(imgPx - imgPo) };
			scalePixPerObj = (imgMag / objMag);
		}
		return scalePixPerObj;
	}
	*/

	//! Simulate an image through camera at position xCamWrtQuad
	inline
	void
	injectTargetInto
		( dat::Grid<float> * const & ptGrid
		, img::Camera const & camera
		, rigibra::Transform const & xCamWrtQuad
		, obj::QuadTarget const & objQuad
		)
	{
		dat::Grid<float> & grid = *ptGrid;
		constexpr double objDelta{ 1./1024. };

		double const & beg0 = objQuad.span0().theBeg;
		double const & end0 = objQuad.span0().theEnd;
		double const & beg1 = objQuad.span1().theBeg;
		double const & end1 = objQuad.span1().theEnd;

		Sampler const sampler(camera, xCamWrtQuad, objQuad);

		using namespace rigibra;
		for (std::size_t row{0u} ; row < grid.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < grid.wide() ; ++col)
			{
				double const subRow{ (double)row };
				double const subCol{ (double)col };
				dat::Spot const detSpot{ subRow, subCol };

				float const inten{ (float)sampler.intensityAt(detSpot) };
				if (engabra::g3::isValid(inten))
				{
					grid(row, col) = inten;
				}
			}
		}
	}

	//! Simulate an image through camera at position xCamWrtQuad
	inline
	dat::Grid<float>
	quadImage
		( img::Camera const & camera
		, rigibra::Transform const & xCamWrtQuad
		, obj::QuadTarget const & objQuad
		)
	{
		dat::Grid<float> grid(camera.theFormat);
		std::fill(grid.begin(), grid.end(), engabra::g3::null<float>());
		injectTargetInto(&grid, camera, xCamWrtQuad, objQuad);
		return grid;
	}


} // [sim]

} // [quadloco]

