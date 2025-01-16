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
#include "opsCenterRefiner.hpp"
#include "rasChipSpec.hpp"
#include "rasGrid.hpp"
#include "rasPeakRCV.hpp"
#include "simConfig.hpp"
#include "simRender.hpp"
#include "sysTimer.hpp"

#include <Engabra>
#include <Rigibra>

#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <sstream>


namespace
{
	//
	// Test conditions
	//

	//! Define number of test cases to run
//#	define SmallTrial  // about 64
#	define LargeTrial  // about 5940

	/*! NOTE: this value depends (significantly) on amount of simulation noise
	 *
	 * This threshold can be lowered if oversampling (sSamplerOverNum) value
	 * is increased - which can increase simulation run time substantially.
	 */
	constexpr double sMaxPixErr{ 3./4. };

} // [anon]

namespace
{

	struct MinDelMax
	{
		double const theMin;
		double const theDel;
		double const theMax;

	}; // MinDelMax

	struct TrialSpec
	{
		MinDelMax theAzmMDM{};
		MinDelMax theVrtMDM{};
		MinDelMax theRngMDM{};

		inline
		static
		std::size_t
		estSizeFor
			( MinDelMax const & mdm
			)
		{
			double const mdmCount{ (mdm.theMax - mdm.theMin) / mdm.theDel };
			return (static_cast<std::size_t>(mdmCount) + 1u);
		}

		inline
		std::size_t
		estSizeAll
			() const
		{
			std::size_t const azmNum{ estSizeFor(theAzmMDM) };
			std::size_t const vrtNum{ estSizeFor(theVrtMDM) };
			std::size_t const rngNum{ estSizeFor(theRngMDM) };
			return (azmNum * vrtNum * rngNum);
		}

	};

} // [anon]


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

	// Ref: Function stationLocsPolar() to define station locations

	constexpr std::size_t const sStaNumRollSteps // within half turn
			{ 4u };
		//	{ 1u };

	static std::vector<std::size_t> const sAppRingHalfSizes{ 5u, 3u };


	constexpr double sPi{ std::numbers::pi_v<double> };

#if defined(SmallTrial)
	constexpr TrialSpec sTrialSmall
		{ MinDelMax // azmMinDelMax
			{ sPi * ( 0. / 180.)
			, sPi * (45. / 180.)
			, sPi * (45. / 180.)
			}
		, MinDelMax // vrtMinDelMax
			{ sPi * ( 0. / 180.)
			, sPi * (45. / 180.)
			, sPi * (45. / 180.)
			}
		, MinDelMax // rngMinDelMax
			{  .250 
			,  .500
			, 2.000
			}
		};
	static TrialSpec const sUseTrialSpec{ sTrialSmall };
#endif

#if defined(LargeTrial)
	constexpr TrialSpec sTrial5k
		{ MinDelMax // azmMinDelMax
			{ sPi * ( 0. / 180.)
			, sPi * ( 9. / 180.)
			, sPi * (90. / 180.)
			}
		, MinDelMax // vrtMinDelMax
			{ sPi * ( 0. / 180.)
			, sPi * ( 9. / 180.)
			, sPi * (72. / 180.)
			}
		, MinDelMax // rngMinDelMax
			{  .250 
			,  .125
			, 2.000
			}
		};
	static TrialSpec const sUseTrialSpec{ sTrial5k };
#endif


	//
	// Utilities
	//

	//! Polar coordinate decomposition of 3D location
	struct PolarDecomp
	{
		double const theAngleAzim{};
		double const theAngleVert{};
		double const theRangeDist{};

		inline
		static
		PolarDecomp
		from
			( engabra::g3::Vector const vec
			)
		{
			using namespace engabra::g3;
			double const magXY{ std::hypot(vec[0], vec[1]) };
			double const angleVert{ std::atan2(magXY, vec[2]) };
			double const angleAzim{ std::atan2(vec[1], vec[0]) };
			double const rangeDist{ magnitude(vec) };
			return { angleAzim, angleVert, rangeDist };
		}

		inline
		static
		PolarDecomp
		fromAVR
			( double const & azm
			, double const & vrt
			, double const & rng
			)
		{
			return { azm, vrt, rng };
		}


		inline
		engabra::g3::Vector
		vector
			() const
		{
			double const cosVrt{ std::sin(theAngleVert) };
			double const xDir{ cosVrt * std::cos(theAngleAzim) };
			double const yDir{ cosVrt * std::sin(theAngleAzim) };
			double const zDir{ std::cos(theAngleVert) };
			return
				{ theRangeDist * xDir
				, theRangeDist * yDir
				, theRangeDist * zDir
				};
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString  // PolarDecomp::
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			using engabra::g3::io::fixed;
			oss
				<< "Azm,Vrt,Rng:"
				<< ' ' << fixed(theAngleAzim, 2u, 4u)
				<< ' ' << fixed(theAngleVert, 2u, 4u)
				<< ' ' << fixed(theRangeDist, 3u, 3u)
				;

			return oss.str();
		}

	}; // PolarDecomp


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
	stationLocsPolar
		()
	{
		std::vector<engabra::g3::Vector> locs;

		TrialSpec const & trialSpec = sUseTrialSpec;

		double const & azmMin = trialSpec.theAzmMDM.theMin;
		double const & azmDel = trialSpec.theAzmMDM.theDel;
		double const & azmMax = trialSpec.theAzmMDM.theMax;

		double const & vrtMin = trialSpec.theVrtMDM.theMin;
		double const & vrtDel = trialSpec.theVrtMDM.theDel;
		double const & vrtMax = trialSpec.theVrtMDM.theMax;

		double const & rngMin = trialSpec.theRngMDM.theMin;
		double const & rngDel = trialSpec.theRngMDM.theDel;
		double const & rngMax = trialSpec.theRngMDM.theMax;

		std::cout << '\n';
		std::cout << "stationLocsPolar\n";
		std::cout << '\n';
		std::cout << "azmMin: " << azmMin << '\n';
		std::cout << "azmDel: " << azmDel << '\n';
		std::cout << "azmMax: " << azmMax << '\n';

		std::cout << '\n';
		std::cout << "vrtMin: " << vrtMin << '\n';
		std::cout << "vrtDel: " << vrtDel << '\n';
		std::cout << "vrtMax: " << vrtMax << '\n';

		std::cout << '\n';
		std::cout << "rngMin: " << rngMin << '\n';
		std::cout << "rngDel: " << rngDel << '\n';
		std::cout << "rngMax: " << rngMax << '\n';
		std::cout << '\n';
		std::cout << "estSizeAll(): " << trialSpec.estSizeAll() << '\n';

		locs.reserve(trialSpec.estSizeAll());

		for (double rng{rngMin} ; ! (rngMax < rng) ; rng += rngDel)
		{
			for (double azm{azmMin} ; ! (azmMax < azm) ; azm += azmDel)
			{
				for (double vrt{vrtMin} ; ! (vrtMax < vrt) ; vrt += vrtDel)
				{
					PolarDecomp const polarDecomp
						{ PolarDecomp::fromAVR(azm, vrt, rng) };
					locs.emplace_back(polarDecomp.vector());
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
			ras::ChipSpec const chipSpec{ ptConfig->chipSpecForQuad(pad) };
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

			ras::ChipSpec const chipSpec
				{ thePtConfig->chipSpecForQuad(theChipPad) };
			img::Spot const chip0{ cast::imgSpot(chipSpec.srcOrigRC()) };

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

		inline
		rigibra::Transform
		xformCamWrtRef
			() const
		{
			rigibra::Transform xform;
			if (thePtConfig)
			{
				xform = thePtConfig->theStaWrtQuad;
			}
			return xform;
		}

		inline
		engabra::g3::Vector
		stationLoc
			() const
		{
			engabra::g3::Vector staLoc;
			staLoc = xformCamWrtRef().theLoc;
			return staLoc;
		}

		inline
		PolarDecomp
		polarDecomp
			() const
		{
			engabra::g3::Vector const staLoc{ xformCamWrtRef().theLoc };
			PolarDecomp const polarAVR{ PolarDecomp::from(staLoc) };
			return polarAVR;
		}

		inline
		std::string
		infoStringConfig // Trial::
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			if (thePtConfig)
			{
				engabra::g3::Vector const staLoc{ xformCamWrtRef().theLoc };
				PolarDecomp const polarAVR{ polarDecomp() };
				oss
				//	<< "thePtConfig: " << thePtConfig->infoString()
					<< "station: " << staLoc
					<< ' '
					<< polarAVR.infoString()
					;
			}
			return oss.str();
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString // Trial::
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			oss
				<< "sta: " << std::setw(3u) << theStaId
				<< ' '
				<< "roll: " << std::setw(1u) << theRollId
				<< ' '
				<< "baseName: " << baseName()
				;
			if (thePtConfig)
			{
				oss
					<< "  "
					<< infoStringConfig()
					;
			}
		//	oss << "srcGrid: " << theSrcGrid ;
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
		//	{ stationLocsOnGrid() };
			{ stationLocsPolar() };

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

std::cout << "Simulating target for staLoc: " << staLoc << '\n';
			for (std::size_t nRoll{0u} ; nRoll < numRollSteps; ++nRoll)
			{
				double const roll{ (double)nRoll * rollDelta };
				PhysAngle const physRoll{ roll * e12 };
				Attitude const attViewWrtLook(physRoll);

				// station geometry
				static Vector const tgtLoc{ zero<Vector>() }; // by assumption
				Vector const staDelta{ tgtLoc - staLoc };
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
				using quadloco::sim::Config;
				std::shared_ptr<quadloco::sim::Config> const ptConfig
					{ std::make_shared<Config>
						(Config{ target(), camera(), xCamWrtTgt })
					};

				// assemble data into Trial structure for overall admin
				std::shared_ptr<Trial> const ptTrial
					{ std::make_shared<Trial>(nSta, nRoll, ptConfig) };
				ptTrials.emplace_back(ptTrial);

				/*
				if ("sim00152_1" == ptTrial->baseName())
				{
					std::cout << ptTrial->infoString() << '\n';
					std::string const fileName{ (ptTrial->baseName()+".pgm") };
					(void)io::writeStretchPGM(fileName, ptTrial->srcGrid());
				}
				*/
			}
		}
		return ptTrials;
	};


	struct Outcome
	{
		std::shared_ptr<Trial> const thePtTrial{ nullptr };

		img::Spot const theGotCenter{};


		inline
		bool
		isValid
			() const
		{
			return ((bool)thePtTrial && theGotCenter.isValid());
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
			( double const & magDifMax
			) const
		{
			bool okay{ isValid() };
			if (okay)
			{
				// if members exist, compare center finding at magDifMax
				double const magDifGot{ magnitude(centerDif()) };
				if (engabra::g3::isValid(magDifGot))
				{
					okay = (magDifGot < magDifMax);
				}
			}
			return okay;
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString // Outcome::
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
			using engabra::g3::io::fixed;
			oss
				<< '\n'
				<< "exp: " << centerExp()
				<< '\n'
				<< "got: " << centerGot()
				<< '\n'
				<< "dif: " << centerDif()
					<< "  mag: " << fixed(magnitude(centerDif()), 3u, 2u)
				<< '\n'
				;
			return oss.str();
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoStringDif // Outcome::
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			using engabra::g3::io::fixed;
			img::Spot const cDif{ centerDif() };
			oss
				<< "centerDif:"
				<< ' ' << fixed(cDif.row(), 3u, 2u)
				<< ' ' << fixed(cDif.col(), 3u, 2u)
				<< "  mag: " << fixed(magnitude(centerDif()), 3u, 2u)
				;

			return oss.str();
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoStringReport // Outcome::
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			PolarDecomp const polarAVR{ thePtTrial->polarDecomp() };
			oss
				<< thePtTrial->baseName()
				<< ' '
				<< thePtTrial->stationLoc()
				<< ' '
				<< infoStringDif()
				<< ' '
				<< polarAVR.infoString()
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
			ras::Grid<float> const & srcGrid = ptTrial->srcGrid();
			std::vector<ras::PeakRCV> const symPeakRCVs
				{ app::multiSymRingPeaks(srcGrid, sAppRingHalfSizes) };

			img::Spot gotCenter{};
			if (! symPeakRCVs.empty())
			{
// TODO which one?
				// refined center location based on half-turn symmetry
				ras::PeakRCV const & peak = symPeakRCVs.front();
				ras::RowCol const & nomCenterRC = peak.theRowCol;
				ops::CenterRefiner const refiner(&srcGrid);
				gotCenter = refiner.fitSpotNear(nomCenterRC);

// TODO which one?
				// qualified hits based on proximal edgel support
				/*
				ops::EdgeCenters const edgeCenters(srcGrid);
				std::vector<img::Hit> const qualHits
					{ edgeCenters.edgeQualifiedHits(symPeakRCVs) };
				gotCenter = qualHits.front().location();
				*/
			}

			ptOutcome = std::make_shared<Outcome>
				(Outcome{ ptTrial, gotCenter });

			/*
			if ("sim00152_1" == ptTrial->baseName())
			{
				std::ofstream ofs("sim00152_1_info.dat");
				ofs << "symPeakRCVs.size(): " << symPeakRCVs.size() << '\n';
				for (ras::PeakRCV const & peak : symPeakRCVs)
				{
					ofs << "...peak: " << peak << '\n';
				}
			}
			*/
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

std::cout << "starting simulation\n";
		sys::Timer timerSim{ "simulation" };
		// generate many simulation cases
		std::vector<std::shared_ptr<sim::Trial> > const ptTrials
			{ sim::manyTrials() };
		timerSim.stop();
std::cout << "numTrials: " << ptTrials.size() << '\n';

std::cout << "starting processing\n";
		sys::Timer timerProc{ "processing" };
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
		timerProc.stop();
std::cout << "num ptOutcomeAlls: " <<  ptOutcomeAlls.size() << '\n';

std::cout << "starting evaluation\n";
		// assess results overall and note any unsuccessful cases
		sys::Timer timerReport{ "report" };
		std::ostringstream rptAll;
		std::vector<std::shared_ptr<sim::Outcome> > ptOutcomeErrs;
		ptOutcomeErrs.reserve(ptOutcomeAlls.size());
		std::vector<std::shared_ptr<sim::Outcome> > ptOutcomeBads;
		ptOutcomeBads.reserve(ptOutcomeAlls.size());
		for (std::shared_ptr<sim::Outcome> const & ptOutcomeAll : ptOutcomeAlls)
		{
			rptAll
				<< ptOutcomeAll->infoStringReport()
				<< '\n';
			if (! ptOutcomeAll->isValid())
			{
				ptOutcomeBads.emplace_back(ptOutcomeAll);
			}
			else
			if (! ptOutcomeAll->success(sMaxPixErr))
			{
				ptOutcomeErrs.emplace_back(ptOutcomeAll);
			}
		}
		timerReport.stop();

		// report on unsuccessful trials
		std::ostringstream rptErr;
		for (std::shared_ptr<sim::Outcome> const & ptOutcomeErr : ptOutcomeErrs)
		{
			rptErr << ptOutcomeErr->infoString() << '\n';
		}
		std::ostringstream rptBad;
		for (std::shared_ptr<sim::Outcome> const & ptOutcomeBad : ptOutcomeBads)
		{
			// rptBad << ptOutcomeBad->infoString() << '\n';
			rptBad << ptOutcomeBad->thePtTrial->infoString() << '\n';
		}

		// [DoxyExample01]
		// [DoxyExample01]

		std::ostringstream msgTime;
		double const dAll{ (double)(ptOutcomeAlls.size()) };
		double const tPerEach{ timerProc.elapsed() / dAll };
		using engabra::g3::io::fixed;
		msgTime << "===============\n";
		msgTime << "   timer::Sim: " << timerSim << '\n';
		msgTime << "  timer::Proc: " << timerProc << '\n';
		msgTime << "timer::Report: " << timerReport << '\n';
		msgTime << "  ProcPerEach: " << fixed(tPerEach, 1u, 6u) << '\n';
		msgTime << "===============\n";

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
		if (! rptBad.str().empty())
		{
			oss << "Failure of all valid ptOutcome test\n";
			hitErr = true;
		}

		if (hitErr)
		{
			oss << "\n--- Null results\n" << '\n';
			oss << rptBad.str() << '\n';
			oss << "\n*** Error trials\n" << '\n';
			oss << rptErr.str() << '\n';
			oss << '\n';
			using engabra::g3::io::fixed;
			std::size_t const numGood
				{ ptOutcomeAlls.size() 
				- ptOutcomeErrs.size() 
				- ptOutcomeBads.size() 
				};
			oss << "       sMaxPixErr: " <<  fixed(sMaxPixErr, 3u, 3u) << '\n';
			oss << "num ptOutcomeAlls: " <<  ptOutcomeAlls.size() << '\n';
			oss << "num          Good: " <<  numGood << '\n';
			oss << "num          Errs: " <<  ptOutcomeErrs.size() << '\n';
			oss << "num          Null: " <<  ptOutcomeBads.size() << '\n';
			oss << msgTime.str() << '\n';
		}

		std::ofstream ofs("evalCenterFinding.dat");
		ofs << rptAll.str() << '\n';

		std::cout << msgTime.str() << '\n';

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

