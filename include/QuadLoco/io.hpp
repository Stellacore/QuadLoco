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


#include "QuadLoco/pix.hpp"
#include "QuadLoco/rasgrid.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/valSpan.hpp"

#include <Engabra>

#include <cstdint>
#include <filesystem>
#include <limits>
#include <fstream>
#include <sstream>


namespace quadloco
{

/*! \brief Data Input/Output related code in namespace quadloco::io
 */
namespace io
{
	//! Remove everything after '#' character
	inline
	std::string
	stripped
		( std::string const & line
		)
	{
		return line.substr(0, line.find("#", 0));
	}

	//! Utility for handling PGM header data i/o
	struct Header
	{
		//! 'Magic' strint indicating PGM (i.e. "P5")
		std::string theMagic;
		//! Number of rows in image
		unsigned int theHigh;
		//! Number of columns in image
		unsigned int theWide;
		//! Largest pixel value to expect (e.g. 256, or 65536)
		unsigned int theMaxPix;

		//! Populate header from input stream (positioned at start of file)
		inline
		static
		Header
		fromStream
			( std::ifstream & ifs
			)
		{
			// read header info
			std::string magic{};
			unsigned int high{ 0u };
			unsigned int wide{ 0u };
			unsigned int maxPix{ 0u };
			std::string line;

			bool haveMagic{ false };
			bool haveHigh{ false };
			bool haveWide{ false };
			bool haveMaxPix{ false };
			bool haveAll{ false };

			while ( ifs.good() && (! ifs.eof()) && (! haveAll))
			{
				line.clear();
				std::getline(ifs, line);
				std::string const strip{ stripped(line) };

				if (! strip.empty())
				{
					std::istringstream iss(stripped(line));
					if (iss.good() && (! haveMagic))
					{
						iss >> magic;
						haveMagic = (! iss.fail()) && (! magic.empty());
					}

					if (iss.good() && (! haveWide))
					{
						iss >> wide;
						haveWide = (! iss.fail()) && (0u < wide);
					}

					if (iss.good() && (! haveHigh))
					{
						iss >> high;
						haveHigh = (! iss.fail()) && (0u < high);
					}

					if (iss.good() && (! haveMaxPix))
					{
						iss >> maxPix;
						haveMaxPix = (! iss.fail()) && (0u < maxPix);
					}

					haveAll
						=  haveMagic
						&& haveWide
						&& haveHigh
						&& haveMaxPix
						;
					if (haveAll)
					{
						break;
					}
				}
			}

			return Header(magic, high, wide, maxPix);
		}

		//! Value construction
		inline
		explicit
		Header
			( std::string const & magic
			, unsigned int const & high
			, unsigned int const & wide
			, unsigned int const & maxPix
			)
			: theMagic{ magic }
			, theHigh{ high }
			, theWide{ wide }
			, theMaxPix{ maxPix }
		{ }

		//! Populate values consistent with grid instance
		template <typename Type>
		inline
		explicit
		Header
			( ras::Grid<Type> const & grid
			)
			: theMagic("P5")
			, theHigh{ static_cast<unsigned int>(grid.high()) }
			, theWide{ static_cast<unsigned int>(grid.wide()) }
			, theMaxPix{ std::numeric_limits<Type>::max() }
		{ }

		//! A SizeHW with (theHigh,theWide) values in one place
		inline
		ras::SizeHW
		hwSize
			() const
		{
			return ras::SizeHW{ theHigh, theWide };
		}

		//! Put data to output file stream (assume positioned at start)
		inline
		void
		toStream
			( std::ofstream & ofs
			) const
		{
			ofs
				<< "P5"
				// NOTE switched order (wide,high) in PGM
				<< '\n' << theWide << ' ' << theHigh << ' ' << theMaxPix
				<< '\n';
		}

	}; // Header

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
		Header const hdr(ugrid);
		hdr.toStream(ofs);

		// write block of data to stream
		ofs.write(reinterpret_cast<char const *>(ugrid.cbegin()), ugrid.size());

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
		Header const hdr{ Header::fromStream(ifs) };
		std::string const magic{ hdr.theMagic };
		ras::SizeHW const hwSize{ hdr.hwSize() };
		unsigned int const maxPix{ hdr.theMaxPix };

		// if valid header info, then read pixel values
		if (hwSize.isValid() && (255u == maxPix))
		{
			ugrid = ras::Grid<uint8_t>(hwSize);

			// read block of data into image
			ifs.read(reinterpret_cast<char*>(ugrid.begin()), ugrid.size());
		}

		return ugrid;
	}

	//! Write 8-bit gray image based on maximal radiometric stretch of realGrid
	template <typename RealPix>
	inline
	bool
	writeStretchPGM
		( std::filesystem::path const & pgmPath
		, ras::Grid<RealPix> const & realGrid
		)
	{
		// compute spanning range of radiometry values
		val::Span const fSpan{ ras::grid::fullSpanFor(realGrid) };

		// stretch/compress realGrid values to fit into uGrid
		ras::Grid<uint8_t> const uGrid{ ras::grid::uGrid8(realGrid, fSpan) };

		// write resulting uGrid
		return io::writePGM(pgmPath, uGrid);
	}


	//! \brief Put grid coordinates to data file
	template <typename RealPix>
	inline
	bool
	writeAsciiFile
		( std::filesystem::path const & outPath
		, ras::Grid<RealPix> const & realGrid
		, std::string const & cfmtPerField
		, RealPix const & valueForNull
		)
	{
		std::ofstream ofs(outPath);
		std::size_t const high{ realGrid.hwSize().high() };
		std::size_t const wide{ realGrid.hwSize().wide() };
		ofs << "# High: " << high << '\n';
		ofs << "# Wide: " << wide << '\n';
		ofs << "# valueForNull: " << valueForNull << '\n';
		for (std::size_t row{0u} ; row < high ; ++row)
		{
			for (std::size_t col{0u} ; col < wide ; ++col)
			{
				RealPix const & gVal = realGrid(row,col);
				RealPix useVal{ valueForNull };
				if (engabra::g3::isValid(gVal))
				{
					useVal = gVal;
				}

				constexpr std::size_t bufSize{ 1024u };
				char buf[bufSize+1u]; // hopefully large enough
				std::snprintf(buf, bufSize, cfmtPerField.c_str(), useVal);
				ofs << ' ' << buf;

			}
			ofs << '\n';
		}
		return (! ofs.fail());
	}

} // [io]

} // [quadloco]

