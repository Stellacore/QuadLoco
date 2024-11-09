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
#include "imgQuadTarget.hpp"
#include "objQuadTarget.hpp"
#include "pixNoise.hpp"

#include <Engabra>
#include <Rigibra>

#include <functional>
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

		// Sampling options
		bool const theUseSceneBias{ true };
		bool const theUseImageNoise{ true };


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

		//! Options to modify simulation sampling
		enum OptionFlags
		{
			  None           = 0x0000
			, UseSceneBias   = 0x0001
			, UseImageNoise  = 0x0002
		};

	private:

		//! True if testVal bit is set within haveBits
		inline
		static
		bool
		isSet
			( unsigned const & haveBits
			, OptionFlags const & testVal
			)
		{
			unsigned const setVal{ haveBits & (unsigned)testVal };
			bool const hasBit (0u != setVal);
			return hasBit;
		}

	public:

		//! Construct with information required to generate many samples
		inline
		explicit
		Sampler
			( img::Camera const & camera
			, rigibra::Transform const & xCamWrtQuad
			, obj::QuadTarget const & objQuad
			, unsigned const & optionsMask = (UseSceneBias | UseImageNoise)
			)
			: theCamera{ camera }
			, theCamWrtQuad{ xCamWrtQuad }
			, theObjQuad{ objQuad }
			, theQuadWrtCam{ rigibra::inverse(theCamWrtQuad) }
			, theUseSceneBias{ isSet(optionsMask, UseSceneBias) }
			, theUseImageNoise{ isSet(optionsMask, UseImageNoise) }
		{ }

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
				intenSample = theObjQuad.quadSignalAt(spotInQuad);
			}

			// return the sample value
			return intenSample;
		}

		//! Use subsampling to determine primary signal
		inline
		double
		pureSignalIntensity
			( dat::Spot const & detSpot
				//!< Location at which to evaluate object quad signal
			, std::size_t const & numOverSamps
				//!< Number of *ADDITIONAL* intra-pixel *OVER* samplings
			) const
		{
			double intensity{ engabra::g3::null<double>() };
			static std::mt19937 gen(48997969u);
			static std::normal_distribution<double> distro(.0, .5);
			double sum{ 0. };
			double count{ 0. };
			std::size_t const numSamps{ 1u + numOverSamps };
			for (std::size_t nn{0u} ; nn < numSamps ; ++nn)
			{
				// place first spot on exact pixel location
				constexpr dat::Spot halfSpot{ .5, .5 };
				dat::Spot delta{ 0., 0. };
				if (0u < nn)
				{
					delta = dat::Spot{ distro(gen), distro(gen) };
				}

				dat::Spot const useSpot{ detSpot + halfSpot + delta };
				if (isValid(useSpot))
				{
					double const qSig{ quadSignalFor(useSpot) };
					if (engabra::g3::isValid(qSig))
					{
						sum += qSig;
						count += 1.;
					}
				}
			}
			if (0. < count)
			{
				intensity = (1./count) * sum;
			}
			return intensity;
		}

		//! Geometry of perspective image created by quadImage()
		inline
		img::QuadTarget
		imgQuadTarget
			() const
		{
			using namespace engabra::g3;
			std::function<Vector(dat::Spot)> const vecFrom
				{ [] (dat::Spot const & spot)
					{ return Vector{ spot[0], spot[1], 0. }; }
				};
			Vector const centerInExt
				{ theCamWrtQuad(vecFrom(theObjQuad.centerSpot())) };
			Vector const xMidInExt
				{ theCamWrtQuad(vecFrom(theObjQuad.midSidePosX())) };
			Vector const yMidInExt
				{ theCamWrtQuad(vecFrom(theObjQuad.midSidePosY())) };

			dat::Spot const centerInDet
				{ theCamera.detectorSpotFor(centerInExt) };
			dat::Spot const xMidInDet
				{ theCamera.detectorSpotFor(xMidInExt) };
			dat::Spot const yMidInDet
				{ theCamera.detectorSpotFor(yMidInExt) };

			Vector const center{ vecFrom(centerInDet) };
			Vector const xDir{ direction(vecFrom(xMidInDet - centerInDet)) };
			Vector const yDir{ direction(vecFrom(yMidInDet - centerInDet)) };

			return img::QuadTarget{ center, xDir, yDir };
		}


		//! Use add noise to signal
		inline
		double
		intensityAt
			( dat::Spot const & detSpot
				//!< Location at which to simulate intensity
			, std::size_t const & numOverSamps
				//!< Number of *ADDITIONAL* intra-pixel *OVER* samplings
			) const
		{
			double intenSample{ engabra::g3::null<double>() };

			// initial supersampled ideal signal
			double const valueSignal
				{ pureSignalIntensity(detSpot, numOverSamps) };

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
				double valueBias{ 0. };
				if (theUseSceneBias)
				{
					valueBias = noiseBias(spotInQuad);
				}

				// combined scene effecs (signal plus illum)
				double const incidentValue{ valueSignal + valueBias };

				// combined noise (dark and shot)
				double valueNoise{ 0. };
				if (theUseImageNoise)
				{
					valueNoise = theNoiseModel.valueFor(incidentValue);
				}

				// final recorded pixel value
				intenSample = incidentValue + valueNoise;
			}

			return intenSample;
		}

	}; // Sampler



} // [sim]

} // [quadloco]

