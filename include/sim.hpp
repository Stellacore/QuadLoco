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

	//! Intersection of ray with the Z=0 (e12) plane
	inline
	engabra::g3::Vector
	intersectOnE12
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

		//! Provide a pseudo-random linear bias across the scene
		inline
		float
		noiseBias
			( dat::Spot const & spotInQuad
			) const
		{
			using namespace engabra::g3;
			// use lower left corner of quad as reference
			static double const bias0{ .1f };
			static double const biasPerMeter{ 0.50f };
			static Vector const biasDir{ direction(Vector{ 1.5, 1., .0}) };
			Vector const loc0
				{ theObjQuad.span0().theBeg
				, theObjQuad.span1().theBeg
				, 0.
				};
			Vector const locX{ spotInQuad[0], spotInQuad[1], 0. };
			double const distToX{ ((locX - loc0) * biasDir).theSca[0] };
			double const bias{ bias0 + biasPerMeter * distToX };
			float const inten{ static_cast<float>(bias) };
			return inten;
		}

		//! Dark current noise (Poisson based on temperature)
		inline
		float
		noiseDark
			() const
		{
			// intensity contribution per electron
			static float const intenPerE{ .01f };
			// noise generator
			static std::mt19937 gen(47989969u);

			// distribution proportional to temperature
			// NOTE: here assume constant temperature
			// expected electrons per pixel per second
			static std::size_t const expEpPpS{ 10u };
			static std::poisson_distribution<std::size_t> distro(expEpPpS);

			// noise electrons
			float const fNumE{ static_cast<float>(distro(gen)) };

			// dark intensity
			float const intenDark{ intenPerE * fNumE };

			return intenDark;
		}

		//! Shot noise (Poisson based on intensity)
		inline
		float
		noiseShot
			( float const & inten
			) const
		{
			static float const intenPerE{ .01f };
			static float const ePerInten{ 1.f / intenPerE };
			static std::mt19937 gen(96489979u);

			// distribution proportional to intensity
			float const fullNumE{ ePerInten * inten };
			float const rootNumE{ std::sqrtf(fullNumE) };
			std::poisson_distribution<std::size_t> distro(rootNumE);

			// noise electrons
			float const fNumE{ static_cast<float>(distro(gen)) };

			float const intenShot{ intenPerE * fNumE };
			return intenShot;
		}

		//! Sample theObjQuad near forward projection of detector location
		inline
		float
		intensityAt
			( dat::Spot const & detSpot
			) const
		{
			float intenSample{ engabra::g3::null<float>() };
			using engabra::g3::Vector;

			// shorthand
			Vector const & staInQuad = theCamWrtQuad.theLoc;
			rigibra::Attitude const & attQuadWrtExt = theQuadWrtCam.theAtt;

			// forward project detSpot through camera
			Vector const dirInExt{ theCamera.directionForDetSpot(detSpot) };
			// and then change exterior direction into expression quad frame
			Vector const dirInQuad{ attQuadWrtExt(dirInExt) };

			// intersect quad frame ray with quad plane
			Vector const pntInQuad{ intersectOnE12(staInQuad, dirInQuad) };

			// sample intensity from quad target
			dat::Spot const spotInQuad{ pntInQuad[0], pntInQuad[1] };

			// target signal intensity
			float const intenQuad{ theObjQuad.intensityAt(spotInQuad) };
			if (engabra::g3::isValid(intenQuad))
			{
				// illumination bias across target
				float const intenBias{ noiseBias(spotInQuad) };
				// dark current noise
				float const intenDark{ noiseDark() };
				// photon shot noise
				float const intenShot{ noiseShot(intenQuad + intenBias) };

				intenSample = (intenQuad + intenBias + intenDark + intenShot);
			}

			// return the sample value
			return intenSample;
		}

		//! Use subsampling with mutiple calls to intensityAt()
		inline
		float
		intensityNear
			( dat::Spot const & detSpot
			, std::size_t const & numSubSamps = 16u
			) const
		{
			float intensity{ engabra::g3::null<float>() };
			static std::mt19937 gen(48997969u);
			static std::uniform_real_distribution<double> distro(-.75, .75);
			float sum{ 0.f };
			float count{ 0.f };
			for (std::size_t nn{0u} ; nn < numSubSamps ; ++nn)
			{
				dat::Spot const useSpot
					{ detSpot + dat::Spot{ distro(gen), distro(gen) } };
				if (isValid(useSpot))
				{
					sum += intensityAt(useSpot);
					count += 1.f;
				}
			}
			if (0.f < count)
			{
				intensity = (1.f/count) * sum;
			}
			return intensity;
		}

	}; // Sampler


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

				float const inten{ sampler.intensityNear(detSpot) };
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

