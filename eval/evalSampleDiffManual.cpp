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
\brief Evaluate all chips in sample directory and report center finding results.
*/


#include "appCenters.hpp"

#include "cast.hpp"
#include "imgSpot.hpp"
#include "io.hpp"
#include "opsAllPeaks2D.hpp"
#include "opsSymRing.hpp"
#include "rasGrid.hpp"
#include "rasPeakRCV.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>


namespace eval
{
	//! Utility structure for tracking associated files
	struct FileInfo
	{
		std::filesystem::path theParentPath{};
		bool theHasPGM{ false };
		bool theHasMea{ false };

		inline
		bool
		hasAll
			() const
		{
			return (theHasPGM && theHasMea);
		}

		inline
		std::filesystem::path
		pathFor
			( std::string const & baseName
			, std::string const & ext
			) const
		{
			std::filesystem::path path{ theParentPath };
			path /= baseName;
			path += ext;
			return path;
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
				<< theParentPath.native()
				<< "  pgm: " << theHasPGM
				<< "  mea: " << theHasMea
				;
			return oss.str();
		}

	}; // FileInfo

	//! Combination of files needed for processing
	struct FileSet
	{
		std::string theSampleName{};
		std::filesystem::path thePathPGM;
		std::filesystem::path thePathMea;

		//! Common basename of input files
		inline
		std::string
		sampleName
			() const
		{
			return theSampleName;
		}

	}; // FileSet


	//! Collection of input valid/complete file path combinations
	inline
	std::vector<FileSet>
	fileSetsFrom
		( std::filesystem::path const & dirPath
		, std::set<std::string> const & sampNames
		)
	{
		std::vector<FileSet> fileSets;
		namespace fs = std::filesystem;

		using BaseName = std::string;
		std::map<BaseName, FileInfo> baseInfos;

		static std::string const extPGM(".pgm");
		static std::string const extMea(".meapoint");

		for (std::filesystem::path const & entry
			: std::filesystem::directory_iterator(dirPath))
		{
			if (fs::is_regular_file(entry))
			{
				std::string const ext{ entry.extension() };
				bool const isPGM{ (extPGM == ext) };
				bool const isMea{ (extMea == ext) };

				if (isPGM || isMea)
				{
					BaseName const baseName{ entry.stem() };
					fs::path const parent{ entry.parent_path() };
					fs::path basePath{ parent };
					basePath /= baseName;
					std::map<BaseName, FileInfo>::iterator itFind
						{ baseInfos.find(baseName) };

					// if not present already, insert a new FileInfo
					if (baseInfos.cend() == itFind)
					{
						FileInfo const fileInfo{ parent };
						itFind = baseInfos.emplace_hint
							( baseInfos.end()
							, std::make_pair(baseName, fileInfo)
							);
					}

					// update (now) existing FileInfo fields
					if (baseInfos.cend() != itFind)
					{
						FileInfo & fileInfo = itFind->second;
						if (isPGM)
						{
							fileInfo.theHasPGM = true;
						}
						else
						if (isMea)
						{
							fileInfo.theHasMea = true;
						}
					}

					std::string const stem{ entry.stem() };

				} // ext

			} // regular

		} // dir scan

		// prune for return of relevant values
		fileSets.reserve(baseInfos.size());
		for (std::map<BaseName, FileInfo>::value_type
			const & baseInfo : baseInfos)
		{
			std::string const & baseName = baseInfo.first;
			FileInfo const & fileInfo = baseInfo.second;
			if (fileInfo.hasAll())
			{
				bool useThisOne{ true };
				if (! sampNames.empty())
				{
					useThisOne = sampNames.contains(baseName);
				}
				if (useThisOne)
				{
					FileSet const fileSet
						{ baseName
						, fileInfo.pathFor(baseName, extPGM)
						, fileInfo.pathFor(baseName, extMea)
						};
					fileSets.emplace_back(fileSet);
				}
			}
		}

		return fileSets;
	}


	//! Processing results
	class Outcome
	{
		FileSet const theFileSet{};
		std::filesystem::path const theSaveDir{};
		quadloco::img::Spot const theExpPeakSpot{};
		quadloco::img::Spot const theGotPeakSpot{};

		quadloco::img::Spot const theDifPeakSpot{};
		double const theDifMag{};

	public:

		inline
		Outcome
			( FileSet const & fileSet
			, std::filesystem::path const & saveDir
			, quadloco::img::Spot const & expPeakSpot
			, quadloco::img::Spot const & gotPeakSpot
			)
			: theFileSet{ fileSet }
			, theSaveDir{ saveDir }
			, theExpPeakSpot{ expPeakSpot }
			, theGotPeakSpot{ gotPeakSpot }
			, theDifPeakSpot{ theGotPeakSpot - theExpPeakSpot }
			, theDifMag{ magnitude(theDifPeakSpot) }
		{ }

		//! Common basename of input FileSet
		inline
		std::string
		sampleName
			() const
		{
			return theFileSet.sampleName();
		}

		//! Magnitude of difference between got and expected locations
		inline
		double
		difMag
			() const
		{
			return theDifMag;
		}

		//! True if difMag() is strictly less than threshold
		inline
		bool
		goodWithin
			( double const & threshold = 1.
			) const
		{
			return (difMag() < threshold);
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
				oss << title << '\n';
			}
			using namespace quadloco;
			using engabra::g3::io::fixed;
			oss
				<< "  pgm: " << theFileSet.thePathPGM
				<< '\n'
				<< "  mea: " << theFileSet.thePathMea
				<< '\n'
				<< "  exp: " << theExpPeakSpot
				<< '\n'
				<< "  got: " << theGotPeakSpot
				<< '\n'
				<< "  dif: " << theDifPeakSpot
					<< ' ' << fixed(difMag(), 3u, 2u)
				;
			if (! goodWithin(1.))
			{
				oss << "   << Large Diff";
			}
			if (! theSaveDir.empty())
			{
				oss
					<< '\n'
					<< " save: " << theSaveDir << '\n'
					;
			}

			return oss.str();
		}


	}; // Outcome

} // [eval]


namespace
{

	//! Put item to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, eval::FileInfo const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! Put item to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, eval::Outcome const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

} // [anon]


namespace eval
{
	using namespace quadloco;

	inline
	std::string
	summaryString
		( std::vector<Outcome> const & outcomes
		)
	{
		std::ostringstream oss;
		oss << "No.evaluations: " << outcomes.size();
		for (Outcome const & outcome : outcomes)
		{
			using engabra::g3::io::fixed;
			oss << '\n';
			oss
				<< fixed(outcome.difMag(), 3u, 2u)
				<< "  "
				<< outcome.sampleName()
				;
		}

		return oss.str();
	}

	//! Generate a complete report string from collection of results
	inline
	std::string
	reportString
		( std::vector<Outcome> const & outcomes
		, bool const & showDetail = true
		, bool const & showSummary = true
		)
	{
		std::ostringstream rpt;
		if (showDetail)
		{
			rpt << "\n===== Samples\n";
			for (Outcome const & outcome : outcomes)
			{
				rpt
					<< "\n----- "
					<< outcome.sampleName()
					<< '\n'
					<< outcome
					<< '\n';
			}
		}
		if (showSummary)
		{
			rpt
				<< "\n===== Summary\n"
				<< summaryString(outcomes) << '\n';
		}
		return rpt.str();
	}


	//! Load first measurement from a ".meapoint" file assuming it is center
	inline
	img::Spot
	peakSpotLoadFrom
		( FileSet const & fileSet
		)
	{
		std::filesystem::path const & meaPath = fileSet.thePathMea;

		img::Spot peakSpot{};
		std::ifstream ifs(meaPath);
		if (ifs.good())
		{
			std::string name;
			double row, col;
			ifs >> name >> row >> col;
			if (! ifs.bad())
			{
				peakSpot = img::Spot{ row, col };
			}
		}
		return peakSpot;
	}

	//! Grid with sum of corresponding input cells.
	inline
	ras::Grid<float>
	operator+
		( ras::Grid<float> const & gridA
		, ras::Grid<float> const & gridB
		)
	{
		ras::Grid<float> sum;
		ras::SizeHW const hwSize{ gridA.hwSize() };
		if (hwSize == gridB.hwSize())
		{
			sum = ras::Grid<float>(hwSize);
			std::transform
				( gridA.cbegin(), gridA.cend()
				, gridB.cbegin()
				, sum.begin()
				, std::plus<>{}
				);
		}
		return sum;
	}

	//! Grid with product of corresponding input cells.
	inline
	ras::Grid<float>
	operator*
		( ras::Grid<float> const & gridA
		, ras::Grid<float> const & gridB
		)
	{
		ras::Grid<float> sum;
		ras::SizeHW const hwSize{ gridA.hwSize() };
		if (hwSize == gridB.hwSize())
		{
			sum = ras::Grid<float>(hwSize);
			std::transform
				( gridA.cbegin(), gridA.cend()
				, gridB.cbegin()
				, sum.begin()
				, std::multiplies<>{}
				);
		}
		return sum;
	}

// TODO extract core algorithm, redo and put in sane place (ops?), (app?)

	//! Run multiple ops::SymRings, combine results, return best response
	inline
	quadloco::img::Spot
	bestCenterFrom
		( ras::Grid<float> const & srcGrid
		, std::filesystem::path const & saveDir
		, FileSet const & fileSet
		)
	{
		using namespace quadloco;
		img::Spot bestSpot{};

//std::cout << "\n\n********* Restore sequentail filter sizes ******\n\n";
		std::vector<std::size_t> const halfSizes{ 5u, 3u };
//		std::vector<std::size_t> const halfSizes{ 5u };
		std::vector<ras::PeakRCV> const peakCombos
			{ app::multiSymRingPeaks(srcGrid, halfSizes) };

		// get largest peak
		if (! peakCombos.empty())
		{
			ras::PeakRCV const & peak = peakCombos.front();
			bestSpot = img::Spot
				{ static_cast<double>(peak.theRowCol.row())
				, static_cast<double>(peak.theRowCol.col())
				};
		}


		// optionally save intermediate data
		if ( std::filesystem::exists(saveDir)
		  && std::filesystem::is_directory(saveDir)
		   )
		{
			ras::Grid<float> peakGrid(srcGrid.hwSize());
			std::fill(peakGrid.begin(), peakGrid.end(), 0.f);
			for (ras::PeakRCV const & peakCombo : peakCombos)
			{
				peakGrid(peakCombo.theRowCol) = peakCombo.theValue;
			}

			std::string const baseName{ fileSet.sampleName() };
			std::filesystem::path peakPath{ saveDir };
			peakPath /= baseName;
			peakPath += ".pgm";
			bool const okaySave{ io::writeStretchPGM(peakPath, peakGrid) };
			if (! okaySave)
			{
				std::cerr
					<< "\n\n"
					<< "***** Error saving file: " << peakPath
					<< "\n\n";
			}
		}

		return bestSpot;
	}

	//! Result from load/process input files (optn'ly save intermediate data)
	inline
	Outcome
	processFileSet
		( FileSet const & fileSet
		, std::filesystem::path const & saveDir
		)
	{
		// load image chip
		std::filesystem::path const loadPath{ fileSet.thePathPGM };
		ras::Grid<std::uint8_t> const loadGrid{ io::readPGM(loadPath) };
		ras::Grid<float> const srcGrid
			{ ras::grid::realGridOf<float>(loadGrid, 0u) };

		// load expected measurements
		img::Spot const expPeakSpot{ peakSpotLoadFrom(fileSet) };

		// run center finder
		img::Spot const gotPeakSpot
			{ bestCenterFrom(srcGrid, saveDir, fileSet) };

		return Outcome{ fileSet, saveDir, expPeakSpot, gotPeakSpot };
	}

} // [eval]



/*! \brief Process all chip images from sample directory and report results.
 *
 * Arguments specify a load directory, loadDir. Activity proceeds as:
 * \arg Search all file basenames that have both ".pgm" and ".meapoint"
 *      files in loadDir/.
 * \arg Read the .meapoint to obtain expCenter location.
 * \arg Process the chip data to find best center estimate, gotCenter.
 * \arg Compute difCenter as difference of gotCenter less expCenter.
 * \arg Generate report of the {got,exp,dif}Center results.
 *
 * Optionally, if a saveDir argument is provided, intermediate results
 * are saved to files in saveDir/.
 */
int
main
	( int argc
	, char * argv[]
	)
{
	if (! (1 < argc))
	{
		std::cerr << '\n';
		std::cerr << "Process all chip samples in a directory\n";
		std::cerr << '\n';
		std::cerr << "Usage: <progname> <loadDir> [saveDir] [sampNames...]\n";
		std::cerr << '\n';
		std::cerr << 
			"  Program looks for corresponding file sets (.pgm and .meapoint)"
			"\nthat are in the loadDir. If all files are available, a"
			"\n'sample' is created and associated with the basename common"
			"\nto all files in the set. For each 'sample' (file set) center"
			"\nfinding is run and results are reported for the found center"
			"\ncompared with the one loaded from the .meapoint file."
			"\n"
			"\n  If saveDir is provided, intermediate results (e.g. peak grid)"
			"\nare saved to files in that directory (with same sampName)."
			"\n"
			"\n  If sampName(s) are provided, then only found samples matching"
			"\nthe sampName list are run"
			"\n\n"
			;
		return 1;
	}

	// Setup paths

	std::filesystem::path const appPath(argv[0]);
	std::filesystem::path const loadDir(argv[1]);
	std::filesystem::path saveDir{};
	std::set<std::string> sampNames{};
	if (2 < argc)
	{
		saveDir = std::filesystem::path(argv[2]);
	}
	if (3 < argc)
	{
		for (int ndx{3} ; ndx < argc ; ++ndx)
		{
			sampNames.insert(argv[ndx]);
		}
	}

	// Perform detection and generate report records

	std::vector<eval::Outcome> outcomes;
	std::vector<eval::FileSet> const fileSets
		{ eval::fileSetsFrom(loadDir, sampNames) };
	outcomes.reserve(fileSets.size());
	for (eval::FileSet const & fileSet : fileSets)
	{
		eval::Outcome const outcome
			{ eval::processFileSet(fileSet, saveDir) };
		outcomes.emplace_back(outcome);
	}

	// Report results

	constexpr bool showDetail{ false };
	constexpr bool showSummary{ true };
	std::string const report{ reportString(outcomes, showDetail, showSummary) };

	std::string const pad("  ");
	std::cout << '\n';
	std::cout << report << '\n';
	std::cout << '\n';
	std::cout << "application: "
		<< std::filesystem::canonical(appPath).native() << '\n';
	std::cout << pad << "loadDir: " << loadDir.native() << '\n';
	std::cout << pad << "saveDir: " << saveDir.native() << '\n';
	std::cout << '\n';

	return 0;
}


