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
 * \brief Top level file for quadloco::io namespace
 *
 */


#include "pix.hpp"
#include "rasGrid.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>


namespace quadloco
{

/*! \brief Namespaced functions and utilities for supporting Input/Output.
 */
namespace io
{

	//! Write grid contents in PGM file format
	inline
	bool
	writePGM
		( std::filesystem::path const & pgmPath
		, ras::Grid<uint8_t> const & ugrid
		)
	{
		std::ofstream ofs
			( pgmPath
			, std::ios_base::out
			| std::ios_base::binary
			| std::ios_base::trunc
			);
		ofs << "P5\n" << ugrid.wide() << ' ' << ugrid.high() << " 255";
		ofs << '\n';
		for (std::size_t row{0u} ; row < ugrid.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < ugrid.wide() ; ++col)
			{
				ofs << ugrid(row, col);
			}
		}
		ofs << '\n';
		return (! ofs.fail());
	}

	//! Grid of image pixels retrieved from pgmPath
	inline
	ras::Grid<uint8_t>
	readPGM
		( std::filesystem::path const & pgmPath
		)
	{
		ras::Grid<uint8_t> ugrid{};
		std::ifstream ifs
			( pgmPath
			, std::ios_base::in
			| std::ios_base::binary
			);

		// read header info
		std::string magic;
		unsigned int high, wide;
		unsigned int maxPix;
		ifs >> magic;
		ifs >> wide >> high;
		ifs >> maxPix;

		// if valid header info, then read pixel values
		ras::SizeHW const hwSize{ high, wide };
		if (hwSize.isValid() && (255u == maxPix))
		{
			ugrid = ras::Grid<uint8_t>(hwSize);
			for (std::size_t row{0u} ; row < ugrid.high() ; ++row)
			{
				for (std::size_t col{0u} ; col < ugrid.wide() ; ++col)
				{
					ifs >> ugrid(row, col);
				}
			}
		}

		return ugrid;
	}

	//! Write 8-bit gray image based on maximal radiometric stretch of fGrid
	inline
	bool
	writeStretchPGM
		( std::filesystem::path const & pgmPath
		, ras::Grid<float> const & fGrid
		)
	{
		// compute spanning range of radiometry values
		sig::Span const fSpan{ pix::fullSpanFor(fGrid) };

		// stretch/compress fGrid values to fit into uGrid
		ras::Grid<uint8_t> const uGrid{ pix::uGrid8(fGrid, fSpan) };

		// write resulting uGrid
		return io::writePGM(pgmPath, uGrid);
	}

} // [io]

} // [quadloco]

