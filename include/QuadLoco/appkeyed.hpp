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
 * \brief Declarations for quadloco::app::keyed namespace
 *
 */


#include "QuadLoco/appcenter.hpp"
#include "QuadLoco/cast.hpp"
#include "QuadLoco/imgHit.hpp"
#include "QuadLoco/imgSpot.hpp"
#include "QuadLoco/rasChipSpec.hpp"
#include "QuadLoco/rasgrid.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/rasRowCol.hpp"
#include "QuadLoco/rasSizeHW.hpp"

#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>


namespace quadloco
{

namespace app
{

namespace keyed
{
	//! Identifier for individual quad targets (e.g. for image with several)
	using QuadKey = std::string;

	/*! Load img::Spot locations from *.meaPnt file (row,col only)
	 *
	 * The .meapoint file format is collection of ascii records, one
	 * per image spot. Each record is of the form:
	 * \arg ID Row Col  Srr Src Scc
	 *
	 * Where
	 * \arg  ID: alpha-numeric ID
	 * \arg Row: Row location
	 * \arg Col: Column location
	 * \arg Srr: Row-Row covariance
	 * \arg Src: Row-Col cross variance (=Scr)
	 * \arg Scc: Col-Col covariance
	 *
	 * Note that all numeric fields (after the ID) are floating point
	 * values (e.g. expressing values with subpixel precision).
	 */
	inline
	std::map<QuadKey, ras::RowCol>
	keyMeaPointRCs
		( std::filesystem::path const & path
		)
	{
		std::map<QuadKey, ras::RowCol> keyRCs;
		std::ifstream ifs(path);
		std::string line;
		QuadKey key;
		double row;
		double col;
		while (ifs.good() && (! ifs.eof()))
		{
			std::getline(ifs, line);
			std::istringstream iss(line);
			iss >> key >> row >> col;
			if (! iss.bad())
			{
				img::Spot const spot{ row, col };
				ras::RowCol const rc{ cast::rasRowCol(spot) };
				if ((! key.empty()) && isValid(rc))
				{
					keyRCs.emplace_hint
						( keyRCs.end()
						, std::make_pair(key, rc)
						);
				}
			}
		}
		return keyRCs;
	}


	//! Creat ChipSpec instances for each source center location
	inline
	std::map<QuadKey, ras::ChipSpec>
	keyChipSpecsFor
		( std::map<QuadKey, ras::RowCol> const & keyCenterRCs
		, ras::SizeHW const & hwChip
		, ras::SizeHW const & hwFull
		)
	{
		// Generate crop areas centered on keyCenterRCs and inside source grid
		std::map<QuadKey, ras::ChipSpec> keySpecs;

		std::size_t const chipHalfHigh{ hwChip.high() / 2u };
		std::size_t const chipHalfWide{ hwChip.wide() / 2u };
		for (std::map<QuadKey, ras::RowCol>::value_type
			const & keyCenterRC : keyCenterRCs)
		{
			QuadKey const & key = keyCenterRC.first;
			std::size_t const & row0 = keyCenterRC.second.row();
			std::size_t const & col0 = keyCenterRC.second.col();
			if ( (chipHalfHigh < row0)
			  && (chipHalfWide < col0)
			   )
			{
				ras::RowCol const chipOrig
					{ (row0 - chipHalfHigh)
					, (col0 - chipHalfWide)
					};
				ras::ChipSpec const chipSpec{ chipOrig, hwChip };
				if (chipSpec.fitsInto(hwFull))
				{
					keySpecs.emplace_hint
						( keySpecs.end()
						, std::make_pair(key, chipSpec)
						);
				}
			}
		}

		return keySpecs;
	}

	//! Refined target center points near to nominal keyCenterRCs
	inline
	std::map<QuadKey, img::Hit>
	keyCenterHitsNearTo
		( std::map<QuadKey, ras::ChipSpec> const & keyChips
		, ras::Grid<std::uint8_t> const & loadGrid
		, std::vector<std::size_t> const & ringHalfSizes
		)
	{
		std::map<QuadKey, img::Hit> keyCenterHits{};

		// Extract refined center locations for each chip
		for (std::map<QuadKey, ras::ChipSpec>::value_type
			const & keyChip : keyChips)
		{
			QuadKey const & key = keyChip.first;
			ras::ChipSpec const & chipSpec = keyChip.second;

			// crop grid from source and convert to float
			ras::Grid<float> const srcGrid
				{ ras::grid::subGridValuesFrom<float>(loadGrid, chipSpec) };

			// find and refine center location
			img::Hit const chipHit
				{ center::refinedHitFrom(srcGrid, ringHalfSizes) };

			if (isValid(chipHit))
			{
				// adjust hit to reflect source grid coordinates
				img::Hit const imgHit
					{ chipSpec.fullSpotForChipSpot(chipHit.location())
					, chipHit.value()
					, chipHit.sigma()
					};
				keyCenterHits.emplace_hint
					( keyCenterHits.end()
					, std::make_pair(key, imgHit)
					);
			}
		}
		return keyCenterHits;
	}

} // [keyed]



} // [app]

} // [quadloco]

