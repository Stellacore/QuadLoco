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


#include "appcenter.hpp"
#include "cast.hpp"
#include "imgHit.hpp"
#include "imgSpot.hpp"
#include "rasChipSpec.hpp"
#include "rasgrid.hpp"
#include "rasGrid.hpp"
#include "rasRowCol.hpp"
#include "rasSizeHW.hpp"

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
	using QuadKey = std::size_t;

	//! Load img::Spot locations from *.meaPnt file (row,col only)
	inline
	std::map<QuadKey, ras::RowCol>
	keyMeaPointRCs
		( std::filesystem::path const & path
		)
	{
		std::map<QuadKey, ras::RowCol> keyRCs;
		std::ifstream ifs(path);
		std::string line;
		std::string strKey;
		std::set<std::string> strKeys{};
		double row;
		double col;
		while (ifs.good() && (! ifs.eof()))
		{
			std::getline(ifs, line);
			std::istringstream iss(line);
			iss >> strKey >> row >> col;
			if (! iss.bad())
			{
				strKeys.insert(strKey);
				std::size_t const key{ strKeys.size() };
				img::Spot const spot{ row, col };
				ras::RowCol const rc{ cast::rasRowCol(spot) };
				keyRCs.emplace_hint
					( keyRCs.end()
					, std::make_pair(key, rc)
					);
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
			std::size_t const & key = keyCenterRC.first;
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
		( std::map<QuadKey, ras::RowCol> const & keyCenterRCs
		, ras::SizeHW const & hwChip
		, ras::Grid<std::uint8_t> const & loadGrid
		)
	{
		std::map<QuadKey, img::Hit> keyCenterHits{};

		// Define chip specs centered on locations of interest
		std::map<QuadKey, ras::ChipSpec> const keyChips
			{ keyChipSpecsFor(keyCenterRCs, hwChip, loadGrid.hwSize()) };

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
			img::Hit const chipHit{ center::refinedHitFrom(srcGrid) };

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

