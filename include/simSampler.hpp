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
 * \brief Declarations for simulation quadloco::sim::Sampler.
 *
 */


#include "datSpot.hpp"
#include "imgCamera.hpp"
#include "objQuadTarget.hpp"
#include "pixNoise.hpp"

#include <Engabra>
#include <Rigibra>

#include <random>


namespace quadloco
{

namespace sim
{

	//! Functor for sampling quad target given camera detector location
	class Sampler
	{
		//! Camera geometry to use in sampling process (simulated imaging)
		img::Camera const theCamera{};

		//! Orientation (pose) of camera exterior frame w.r.t. quad target
		rigibra::Transform const theCamWrtQuad{};

		//! Object space quad target (scene to be imaged)
		obj::QuadTarget const theObjQuad{};

		//! Simplistic digital imaging noise model
		pix::Noise const theNoiseModel{};

		// Cached objects

		//! Inverse of #theCamWrtQuad
		rigibra::Transform const theQuadWrtCam{};

		//! Intersection of ray with the Z=0 (e12) plane
		inline
		static
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
		double
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
			return bias;
		}

		//! Location on QuadTarget (in quad frame) associated with detSpot
		inline
		engabra::g3::Vector
		quadLocFor
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
			Vector const pntInQuad{ intersectOnE12(staInQuad, dirInQuad) };

			return pntInQuad;
		}

		//! Sample theObjQuad near forward projection of detector location
		inline
		double
		quadSignalFor
			( dat::Spot const & detSpot
			) const
		{
			double intenSample{ engabra::g3::null<double>() };

			engabra::g3::Vector const pntInQuad{ quadLocFor(detSpot) };
			if (isValid(pntInQuad))
			{
				// sample intensity from quad target
				dat::Spot const spotInQuad{ pntInQuad[0], pntInQuad[1] };

				// target signal intensity
				intenSample = theObjQuad.intensityAt(spotInQuad);
			}

			// return the sample value
			return intenSample;
		}

		//! Use subsampling with mutiple calls to intensityAt()
		inline
		double
		pureSignalIntensity
			( dat::Spot const & detSpot
			, std::size_t const & numSubSamps
			) const
		{
			double intensity{ engabra::g3::null<double>() };
			static std::mt19937 gen(48997969u);
			static std::normal_distribution<double> distro(-.75, .75);
			double sum{ 0. };
			double count{ 0. };
			for (std::size_t nn{0u} ; nn < numSubSamps ; ++nn)
			{
				dat::Spot const delta{ distro(gen), distro(gen) };
				dat::Spot const useSpot{ detSpot + delta };
				if (isValid(useSpot))
				{
					sum += quadSignalFor(useSpot);
					count += 1.;
				}
			}
			if (0. < count)
			{
				intensity = (1./count) * sum;
			}
			return intensity;
		}

		//! Use subsampling with mutiple calls to intensityAt()
		inline
		double
		intensityNear
			( dat::Spot const & detSpot
			, std::size_t const & numSubSamps = 16u
			) const
		{
			double intenSample{ engabra::g3::null<double>() };

			// initial supersampled ideal signal
			double const valueSignal
				{ pureSignalIntensity(detSpot, numSubSamps) };

			// add noise
			if (engabra::g3::isValid(valueSignal))
			{
				// since there is some signal, the detSpot must be
				// close to the quad target boundary.  I.e., intersection
				// with quad plane location may be just outside the
				// quad boundary, but is close enough to use for scene
				// illumination bias
				engabra::g3::Vector const pntInQuad{ quadLocFor(detSpot) };
				dat::Spot const spotInQuad{ pntInQuad[0], pntInQuad[1] };

				// illumination bias across target
				double const valueBias{ noiseBias(spotInQuad) };

				// combined scene effecs (signal plus illum)
				double const incidentValue{ valueSignal + valueBias };

				// combined noise (dark and shot)
				double const valueNoise
					{ theNoiseModel.valueFor(incidentValue) };

				// final recorded pixel value
				intenSample = incidentValue + valueNoise;
			}

			return intenSample;
		}

	}; // Sampler



} // [sim]

} // [quadloco]

