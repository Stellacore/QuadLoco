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

#include <Rigibra>


namespace quadloco
{

/*! \brief Functions and utilities for supporting simulation
 */
namespace sim
{

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

	//! Intersection of ray with the Z=0 (e12) plane
	inline
	engabra::g3::Vector
	intersectE12
		( engabra::g3::Vector const rayBeg
		, engabra::g3::Vector const rayDir
		)
	{
		using namespace engabra::g3;
		double const sDotZ{ (rayBeg * e3).theSca[0] };
		double const dDotZ{ (rayDir * e3).theSca[0] };
		double const lambda{ -(sDotZ / dDotZ) };
		return (rayBeg + lambda*rayDir);
	}

	//! Funtor for sampling quad target given camera detector location
	class Sampler
	{
		img::Camera const theCamera{};
		rigibra::Transform const theCamWrtQuad{};
		obj::QuadTarget const theObjQuad{};

		// Cached objects
		rigibra::Transform const theQuadWrtCam{};

	public:

		//! Construct with information required to generate many samples
		inline
		explicit
		Sampler
			( img::Camera const & camera
			, rigibra::Transform const & xCamWrtQuad
			, obj::QuadTarget const & objQuad
			)
			: theCamera{ camera }
			, theCamWrtQuad{ xCamWrtQuad }
			, theObjQuad{ objQuad }
			, theQuadWrtCam{ rigibra::inverse(theCamWrtQuad) }
		{ }

		//! Sample theObjQuad near forward projection of detector location
		inline
		float
		intensityAt
			( dat::Spot const & detSpot
			) const
		{
			using engabra::g3::Vector;

			// shorthand
			Vector const & staInQuad = theCamWrtQuad.theLoc;
			rigibra::Attitude const & attQuadWrtExt = theQuadWrtCam.theAtt;

			// forward project detSpot through camera
			Vector const dirInExt{ theCamera.directionForDetSpot(detSpot) };
			// and then change exterior direction into expression quad frame
			Vector const dirInQuad{ attQuadWrtExt(dirInExt) };

			// intersect quad frame ray with quad plane
			Vector const pntInQuad{ intersectE12(staInQuad, dirInQuad) };

			// sample intensity from quad target
			dat::Spot const spotInQuad{ pntInQuad[0], pntInQuad[1] };

			//return the sample value
			return theObjQuad.intensityAt(spotInQuad);
		}

	}; // Sampler


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

		constexpr double objDelta{ 1./1024. };

		double const & beg0 = objQuad.span0().theBeg;
		double const & end0 = objQuad.span0().theEnd;
		double const & beg1 = objQuad.span1().theBeg;
		double const & end1 = objQuad.span1().theEnd;

		Sampler const sampler(camera, xCamWrtQuad, objQuad);

		using namespace rigibra;
		dat::Grid<float>::iterator itGrid{ grid.begin() };
		for (std::size_t row{0u} ; row < grid.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < grid.wide() ; ++col)
			{
				double const subRow{ (double)row };
				double const subCol{ (double)col };
				dat::Spot const detSpot{ subRow, subCol };

				float const quadInten{ sampler.intensityAt(detSpot) };
				*itGrid++ = quadInten;
			}
		}

		return grid;
	}


} // [sim]

} // [quadloco]

