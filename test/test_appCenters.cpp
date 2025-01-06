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


/*! \file
\brief Unit tests (and example) code for quadloco::app functions
*/


#include "appCenters.hpp"
#include "cast.hpp"
#include "imgSpot.hpp"
#include "io.hpp"
#include "objCamera.hpp"
#include "objQuadTarget.hpp"
#include "rasGrid.hpp"
#include "rasPeakRCV.hpp"
#include "simConfig.hpp"
#include "simRender.hpp"

#include <Engabra>
#include <Rigibra>

#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <sstream>


namespace quadloco
{

namespace sim
{
	//
	// Global simulation options
	//

	constexpr unsigned const sSamplerOptions
			{ Sampler::AddSceneBias | Sampler::AddImageNoise };
		//	{ Sampler::None };

	constexpr std::size_t const sSamplerOverNum
		//	{ 64u };  // reasonably good radiometric quality
			{  8u };
		//	{  0u };  // nearest neighbor - amiguity in edges

	constexpr unsigned const sQuadOptions
		//	{ obj::QuadTarget::WithTriangle | obj::QuadTarget::WithSurround };
		//	{ obj::QuadTarget::WithTriangle };
		//	{ obj::QuadTarget::WithSurround };
			{ obj::QuadTarget::None };


	constexpr std::size_t const sStaNumRollSteps // within half turn
			{ 4u };
		//	{ 1u };

	// nominal height above target (to avoid plane of target and behind it)
	constexpr double const sStaLocMinZ // [m]
			{ .250 };

	// start for each coordinate component
	constexpr double const sStaLocBegX // [m]
			{ .000 };
	constexpr double const sStaLocBegY // [m]
			{ .000 };
	constexpr double const sStaLocBegZ // [m]
			{ .000 };

	// increment for each coordinate component
	constexpr double const sStaLocDelX // [m]
			{ .250 };
	constexpr double const sStaLocDelY // [m]
			{ .250 };
	constexpr double const sStaLocDelZ // [m]
			{ .250 };

	// maximum (included) value for each coordinate component
	constexpr double const sStaLocMaxX // [m]
			{  .750 };
	constexpr double const sStaLocMaxY // [m]
			{  .250 };
	constexpr double const sStaLocMaxZ // [m]
			{  .000 };

	static std::vector<std::size_t> const sAppRingHalfSizes{ 5u, 3u };

	//
	// General configuration
	//

	//! Object space quad target
	inline
	obj::QuadTarget
	target
		()
	{
		constexpr double edgeMag{ .040 }; // [m]
		return obj::QuadTarget(edgeMag, sQuadOptions);
	}

	//! Camera with which to render
	inline
	obj::Camera
	camera
		()
	{
		// Ideal perspective camera by detector format and principal distance
		constexpr quadloco::ras::SizeHW format{ 800, 1200 };
		constexpr double pd{  600. };
		obj::Camera const cam{ format, pd };
		return cam;
	}

	//! Locations for camera to view target
	inline
	std::vector<engabra::g3::Vector>
	stationLocations
		()
	{
		std::vector<engabra::g3::Vector> locs;
		std::size_t const numPerX
			{ static_cast<std::size_t>(sStaLocMaxX / sStaLocDelX) };
		std::size_t const numPerY
			{ static_cast<std::size_t>(sStaLocMaxY / sStaLocDelY) };
		std::size_t const numPerZ
			{ static_cast<std::size_t>(sStaLocMaxZ / sStaLocDelZ) };
		locs.reserve(numPerX * numPerY * numPerZ); // overkill by about 2x
		for (double zRel{sStaLocBegZ}
			; ! (sStaLocMaxZ < zRel) ; zRel += sStaLocDelZ)
		{
			double const zz{ sStaLocMinZ + zRel };
			for (double yy{sStaLocBegY}
				; ! (sStaLocMaxY < yy) ; yy += sStaLocDelY)
			{
				for (double xx{sStaLocBegX}
					; ! (sStaLocMaxX < xx) ; xx += sStaLocDelX)
				{
					locs.emplace_back(engabra::g3::Vector{ xx, yy, zz });
				}
			}
		}
		return locs;
	}

	inline
	img::Spot
	centerSpotFor
		( obj::QuadTarget const & objQuad
		, std::shared_ptr<quadloco::sim::Config> const & ptConfig
		)
	{
		img::Spot spot{};
		if (ptConfig)
		{
			spot = ptConfig->imgSpotForTgtSpot(objQuad.centerSpot());
		}
		return spot;
	}

	inline
	ras::Grid<float>
	sourceGrid
		( std::shared_ptr<quadloco::sim::Config> const & ptConfig
		, std::size_t const & pad
		)
	{
		ras::Grid<float> grid{};
		if (ptConfig)
		{
			sim::Render const render{ *ptConfig, sSamplerOptions };
			img::ChipSpec const chipSpec{ ptConfig->chipSpecForQuad(pad) };
			grid = render.quadChip(sSamplerOverNum, chipSpec);
		}
		return grid;
	}


	struct Trial
	{
		std::size_t const theStaId{};
		std::size_t const theRollId{};

		std::shared_ptr<quadloco::sim::Config> thePtConfig{ nullptr };

		std::size_t const theChipPad{};
		ras::Grid<float> const theSrcGrid{};


		inline
		explicit
		Trial
			( std::size_t const & staId
			, std::size_t const & rollId
			, std::shared_ptr<quadloco::sim::Config> const & ptConfig
			, std::size_t const & chipPad = 10u
			)
			: theStaId{ staId }
			, theRollId{ rollId }
			, thePtConfig{ ptConfig }
			, theChipPad{ chipPad }
			, theSrcGrid{ sourceGrid(thePtConfig, theChipPad) }
		{ }


		inline
		img::Spot
		centerExp
			() const
		{
			img::Spot const centerInObj
				{ thePtConfig->theObjQuad.centerSpot() };
			img::Spot const centerInImg
				{ thePtConfig->imgSpotForTgtSpot(centerInObj) };

			img::ChipSpec const chipSpec
				{ thePtConfig->chipSpecForQuad(theChipPad) };
			img::Spot const chip0{ cast::imgSpot(chipSpec.theOrigRC) };

			img::Spot const centerInChp{ centerInImg - chip0 };
			return centerInChp;
		}

		inline
		std::string
		baseName
			() const
		{
			std::ostringstream oss;
			oss << "sim"
				<< std::setw(5u) << std::setfill('0') << theStaId
				<< '_'
				<< std::setw(1u) << theRollId
				;
			return oss.str();
		}

		inline
		ras::Grid<float> const &
		srcGrid
			() const
		{
			return theSrcGrid;
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
				<< "theStaId: " << theStaId
				<< ' '
				<< "theRollId: " << theRollId
				<< '\n'
				<< "theSrcGrid: " << theSrcGrid
				;
			return oss.str();
		}

	}; // Trial

	inline
	std::vector<std::shared_ptr<Trial> >
	manyTrials
		( std::size_t const & numRollSteps = sStaNumRollSteps
		)
	{
		std::vector<std::shared_ptr<Trial> > ptTrials;
		// start with an array of station locations
		std::vector<engabra::g3::Vector> const staLocs
			{ stationLocations() };

		constexpr double pi{ std::numbers::pi_v<double> };
		double const rollDelta{ pi / (double)numRollSteps };

		// size allowing for roll steps
		ptTrials.reserve(numRollSteps * staLocs.size());
		for (std::size_t nSta{0u} ; nSta < staLocs.size() ; ++nSta)
		{
			using namespace engabra::g3;
			using namespace rigibra;

			// get station locations from array
			Vector const & staLoc = staLocs[nSta];

std::cout << '\n';
std::cout << "staLoc: " << staLoc << '\n';
			for (std::size_t nRoll{0u} ; nRoll < numRollSteps; ++nRoll)
			{
				double const roll{ (double)nRoll * rollDelta };
				PhysAngle const physRoll{ roll * e12 };
				Attitude const attViewWrtLook(physRoll);

				// station geometry
				static Vector const tgtLoc{ zero<Vector>() }; // by assumption
				Vector const staDelta{ tgtLoc - staLoc };
				double const staRange{ magnitude(staDelta) };
				Vector const lookDir{ direction(staDelta) };

				// determine attitude to look at the target
				static Vector const camViewDir{ -e3 }; // by camera convention
				Spinor const spinToView{ camViewDir * lookDir };
				//
				PhysAngle const physAngle{ logG2(spinToView).theBiv };
				Attitude const attLookWrtRef(physAngle);

				// final view attitude is combined roll after pointing
				Attitude const staAtt(attViewWrtLook * attLookWrtRef);

				// use resulting transformation to create sim configuration
				Transform const xCamWrtTgt{ staLoc, staAtt };

				// construct simulation configuration
				std::shared_ptr<quadloco::sim::Config> const ptConfig
					{ std::make_shared<quadloco::sim::Config>
						(target(), camera(), xCamWrtTgt)
					};

				// assemble data into Trial structure for overall admin
				std::shared_ptr<Trial> const ptTrial
					{ std::make_shared<Trial>(nSta, nRoll, ptConfig) };
				ptTrials.emplace_back(ptTrial);

std::string const fileName{ (ptTrial->baseName() + ".pgm") };
(void)io::writeStretchPGM(fileName, ptTrial->srcGrid());

			}
		}
		return ptTrials;
	};


	struct Outcome
	{
		std::shared_ptr<Trial> const thePtTrial{};

		img::Spot const theGotCenter{};


		inline
		bool
		isValid
			() const
		{
			return ((bool)thePtTrial);
		}

		inline
		img::Spot
		centerExp
			() const
		{
			img::Spot spot{};
			if (isValid())
			{
				spot = thePtTrial->centerExp();
			}
			return spot;
		}

		inline
		img::Spot
		centerGot
			() const
		{
			return theGotCenter;
		}

		inline
		img::Spot
		centerDif
			() const
		{
			return cast::imgSpot(centerGot() - centerExp());
		}

		inline
		bool
		success
			( double const & magDifMax = 1./32.
			) const
		{
			bool okay{ isValid() };
			if (okay)
			{
				// if members exist, compare center finding at magDifMax
				double const magDifGot{ magnitude(centerDif()) };
				okay = (magDifGot < magDifMax);
			}
			return okay;
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
			if (thePtTrial)
			{
				oss << thePtTrial->infoString("thePtTrial");
			}
			oss
				<< '\n'
				<< "exp: " << centerExp()
				<< '\n'
				<< "got: " << centerGot()
				<< '\n'
				<< "dif: " << centerDif()
				<< '\n'
				;

			return oss.str();
		}

	}; // Outcome


	inline
	std::shared_ptr<Outcome>
	ptOutcomeFor
		( std::shared_ptr<Trial> const & ptTrial
		)
	{
		std::shared_ptr<Outcome> ptOutcome;
		if (ptTrial)
		{
			std::vector<ras::PeakRCV> const peaks
				{ app::multiSymRingPeaks
					(ptTrial->srcGrid(), sAppRingHalfSizes)
				};

			img::Spot gotCenter{};
			if (! peaks.empty())
			{
				gotCenter = cast::imgSpot(peaks.front().theRowCol);
			}
			ptOutcome = std::make_shared<Outcome>(ptTrial, gotCenter);
		}
		return ptOutcome;
	}

} // [sim]

} // [quadloco]

namespace
{
	//! Exercise multiSymRingPeaks() over range of simulated imagery
	void
	test1
		( std::ostream & oss
		)
	{
		using namespace quadloco;

		// generate many simulation cases
		std::vector<std::shared_ptr<sim::Trial> > const ptTrials
			{ sim::manyTrials() };
std::cout << "numTrials: " << ptTrials.size() << '\n';

		// process each simulated case and record result
		std::vector<std::shared_ptr<sim::Outcome> > ptOutcomeAlls;
		ptOutcomeAlls.reserve(ptTrials.size());
		for (std::shared_ptr<sim::Trial> const & ptTrial : ptTrials)
		{
			std::shared_ptr<sim::Outcome> const ptOutcome
				{ ptOutcomeFor(ptTrial) };
			if (ptOutcome)
			{
				ptOutcomeAlls.emplace_back(ptOutcome);
			}
		}
std::cout << "num ptOutcomeAlls: " <<  ptOutcomeAlls.size() << '\n';

		// assess results overall and note any unsuccessful cases
		std::ostringstream rptAll;
		std::vector<std::shared_ptr<sim::Outcome> > ptOutcomeErrs;
		ptOutcomeErrs.reserve(ptOutcomeAlls.size());
		for (std::shared_ptr<sim::Outcome> const & ptOutcomeAll : ptOutcomeAlls)
		{
			rptAll << ptOutcomeAll->infoString() << '\n';
			if (! ptOutcomeAll->success())
			{
				ptOutcomeErrs.emplace_back(ptOutcomeAll);
			}
		}
std::cout << "num ptOutcomeErrs: " <<  ptOutcomeErrs.size() << '\n';

		// report on unsuccessful trials
		std::ostringstream rptErr;
		for (std::shared_ptr<sim::Outcome> const & ptOutcomeErr : ptOutcomeErrs)
		{
			rptErr << ptOutcomeErr->infoString() << '\n';
		}

		// [DoxyExample01]
		// [DoxyExample01]

		bool hitErr{ false };
		if (ptTrials.empty())
		{
			oss << "Failure of ptTrials non-empty test\n";
			hitErr = true;
		}
		if (ptOutcomeAlls.empty())
		{
			oss << "Failure of ptOutcomes non-empty test\n";
			hitErr = true;
		}
		if (! rptErr.str().empty())
		{
			oss << "Failure of all good ptOutcome test\n";
			hitErr = true;
		}

		if (hitErr)
		{
			oss << rptErr.str() << '\n';
		}

	}

}

//! Standard test case main wrapper
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

//	test0(oss);
	test1(oss);

	if (oss.str().empty()) // Only pass if no errors were encountered
	{
		status = 0;
	}
	else
	{
		// else report error messages
		std::cerr << "### FAILURE in test file: " << __FILE__ << std::endl;
		std::cerr << oss.str();
	}
	return status;
}

