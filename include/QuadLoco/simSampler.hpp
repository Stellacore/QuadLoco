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


#include "QuadLoco/imgQuadTarget.hpp"
#include "QuadLoco/imgSpot.hpp"
#include "QuadLoco/imgVector.hpp"
#include "QuadLoco/objCamera.hpp"
#include "QuadLoco/objQuadTarget.hpp"
#include "QuadLoco/pixNoise.hpp"

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
	//! Engabra Vector: [0,1] from spot[0,1], [2] set to zero.
	inline
	engabra::g3::Vector
	engVector
		( img::Spot const & spot
		)
	{
		return engabra::g3::Vector{ spot[0], spot[1], 0. };
	}


	//! Functor for sampling quad target given camera detector location
	class Sampler
	{

	public:

		//! Options to modify simulation sampling
		struct RenderOptions
		{
			bool const theAddSceneBias{ false };
			bool const theAddImageNoise{ false };
		};

	private:

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
		RenderOptions const theRenderOptions{};


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
			( obj::Camera const & camera
			, rigibra::Transform const & xCamWrtQuad
			, obj::QuadTarget const & objQuad
			, RenderOptions const & renderOptions =
				{ .theAddSceneBias = true
				, .theAddImageNoise = true
				}
			)
			: theCamera{ camera }
			, theCamWrtQuad{ xCamWrtQuad }
			, theObjQuad{ objQuad }
			, theQuadWrtCam{ rigibra::inverse(theCamWrtQuad) }
			, theRenderOptions{ renderOptions }
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
		img::Spot
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
			img::Spot const spot{ pntInQuad[0], pntInQuad[1] };

			return spot;
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
			img::Spot const spotInQuad{ quadLocFor(detSpot) };
			if (spotInQuad.isValid())
			{
				// sample intensity from quad target
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
			double sum{ 0. };
			double countGot{ 0. };
			double const countMin{ (double)(numOverSamps / 2u) };
			std::size_t const numSamps{ 1u + numOverSamps };
			for (std::size_t nn{0u} ; nn < numSamps ; ++nn)
			{
				// place first spot on exact pixel location
				static img::Spot const halfSpot{ .5, .5 };
				img::Spot delta{ 0., 0. };
				if (0u < nn)
				{
					static std::mt19937 gen(48997969u);
					static std::normal_distribution<double> distro(.0, .5);
					delta = img::Spot{ distro(gen), distro(gen) };
				}

				img::Spot const useSpot{ detSpot + halfSpot + delta };
				if (::isValid(useSpot))
				{
					double const qSig{ quadSignalFor(useSpot) };
					if (engabra::g3::isValid(qSig))
					{
						sum += qSig;
						countGot += 1.;
					}
				}
			}
			if (countMin < countGot)
			{
				intensity = (1./countGot) * sum;
			}
			return intensity;
		}

		//! Geometry of perspective image created by quadGrid()
		inline
		img::QuadTarget
		imgQuadTarget
			() const
		{
			using namespace engabra::g3;

			engabra::g3::Vector const centerInExt
				{ theCamWrtQuad(engVector(theObjQuad.centerSpot())) };
			engabra::g3::Vector const xMidInExt
				{ theCamWrtQuad(engVector(theObjQuad.midSidePosX())) };
			engabra::g3::Vector const yMidInExt
				{ theCamWrtQuad(engVector(theObjQuad.midSidePosY())) };

			using namespace quadloco::img;

			Vector<double> const centerInDet
				{ theCamera.projectedSpotFor(centerInExt) };
			Vector<double> const xMidInDet
				{ theCamera.projectedSpotFor(xMidInExt) };
			Vector<double> const yMidInDet
				{ theCamera.projectedSpotFor(yMidInExt) };

			Spot const center{ centerInDet };
			Vector<double> const xDir{ direction(xMidInDet - centerInDet) };
			Vector<double> const yDir{ direction(yMidInDet - centerInDet) };

			img::QuadTarget const simQuad{ center, xDir, yDir };
			return simQuad;
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
				img::Spot const spotInQuad{ quadLocFor(detSpot) };

				// illumination bias across target
				double valueBias{ 0. };
				if (theRenderOptions.theAddSceneBias)
				{
					valueBias = noiseBias(spotInQuad);
				}

				// combined scene effecs (signal plus illum)
				double const incidentValue{ valueSignal + valueBias };

				// combined noise (dark and shot)
				double valueNoise{ 0. };
				if (theRenderOptions.theAddImageNoise)
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

