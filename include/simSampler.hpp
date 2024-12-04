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


#include "cast.hpp"
#include "imgSpot.hpp"
#include "imgVector.hpp"
#include "objCamera.hpp"
#include "objQuadTarget.hpp"
#include "pixNoise.hpp"
#include "sigQuadTarget.hpp"

#include <Engabra>
#include <Rigibra>

#include <functional>
#include <iostream>
#include <random>
#include <sstream>
#include <string>


namespace quadloco
{

namespace sim
{

	//! Functor for sampling quad target given camera detector location
	class Sampler
	{
		//! Camera geometry to use in sampling process (simulated imaging)
		obj::Camera const theCamera{};

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
		bool const theAddSceneBias{ true };
		bool const theAddImageNoise{ true };


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
			, AddSceneBias   = 0x0001
			, AddImageNoise  = 0x0002
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
			( obj::Camera const & camera
			, rigibra::Transform const & xCamWrtQuad
			, obj::QuadTarget const & objQuad
			, unsigned const & optionsMask = (AddSceneBias | AddImageNoise)
			)
			: theCamera{ camera }
			, theCamWrtQuad{ xCamWrtQuad }
			, theObjQuad{ objQuad }
			, theQuadWrtCam{ rigibra::inverse(theCamWrtQuad) }
			, theAddSceneBias{ isSet(optionsMask, AddSceneBias) }
			, theAddImageNoise{ isSet(optionsMask, AddImageNoise) }
		{ }

		//! True if this instance contains valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  theCamera.isValid()
				&& rigibra::isValid(theCamWrtQuad)
				&& theObjQuad.isValid()
				);
		}

		//! Grid (pixel) format for simulation
		inline
		ras::SizeHW
		format
			() const
		{
			return theCamera.theFormat;
		}

		//! Location on QuadTarget (in quad frame) associated with detSpot
		inline
		engabra::g3::Vector
		quadLocFor
			( img::Spot const & detSpot
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
			( img::Spot const & spotInQuad
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
			( img::Spot const & detSpot
			) const
		{
			double intenSample{ engabra::g3::null<double>() };

			engabra::g3::Vector const pntInQuad{ quadLocFor(detSpot) };
			if (engabra::g3::isValid(pntInQuad))
			{
				// sample intensity from quad target
				img::Spot const spotInQuad{ pntInQuad[0], pntInQuad[1] };
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
			( img::Spot const & detSpot
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
				img::Spot const halfSpot{ .5, .5 };
				img::Spot delta{ 0., 0. };
				if (0u < nn)
				{
					delta = img::Spot{ distro(gen), distro(gen) };
				}

				img::Spot const useSpot{ detSpot + halfSpot + delta };
				if (::isValid(useSpot))
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
		sig::QuadTarget
		imgQuadTarget
			() const
		{
			using namespace engabra::g3;

			engabra::g3::Vector const centerInExt
				{ theCamWrtQuad(cast::engVector(theObjQuad.centerSpot())) };
			engabra::g3::Vector const xMidInExt
				{ theCamWrtQuad(cast::engVector(theObjQuad.midSidePosX())) };
			engabra::g3::Vector const yMidInExt
				{ theCamWrtQuad(cast::engVector(theObjQuad.midSidePosY())) };

			img::Vector const centerInDet
				{ theCamera.projectedSpotFor(centerInExt) };
			img::Vector const xMidInDet
				{ theCamera.projectedSpotFor(xMidInExt) };
			img::Vector const yMidInDet
				{ theCamera.projectedSpotFor(yMidInExt) };

			engabra::g3::Vector const center
				{ cast::engVector(centerInDet) };
			engabra::g3::Vector const xDir
				{ direction(cast::engVector(xMidInDet - centerInDet)) };
			engabra::g3::Vector const yDir
				{ direction(cast::engVector(yMidInDet - centerInDet)) };

			sig::QuadTarget const imgQuad{ center, xDir, yDir };
			return imgQuad;
		}


		//! Use add noise to signal
		inline
		double
		intensityAt
			( img::Spot const & detSpot
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
				img::Spot const spotInQuad{ pntInQuad[0], pntInQuad[1] };

				// illumination bias across target
				double valueBias{ 0. };
				if (theAddSceneBias)
				{
					valueBias = noiseBias(spotInQuad);
				}

				// combined scene effecs (signal plus illum)
				double const incidentValue{ valueSignal + valueBias };

				// combined noise (dark and shot)
				double valueNoise{ 0. };
				if (theAddImageNoise)
				{
					valueNoise = theNoiseModel.valueFor(incidentValue);
				}

				// final recorded pixel value
				intenSample = incidentValue + valueNoise;
			}

			return intenSample;
		}

		//! Descriptive information about this instance.
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
				<< "    theCamera: " << theCamera
				<< '\n'
				<< "theCamWrtQuad: " << theCamWrtQuad
				<< '\n'
				<< "   theObjQuad: " << theObjQuad
				<< '\n'
				<< "theNoiseModel: " << theNoiseModel
				;

			return oss.str();
		}

	}; // Sampler



} // [sim]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sim::Sampler const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::sim::Sampler const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

