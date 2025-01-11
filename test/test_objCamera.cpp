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
\brief Unit tests (and example) code for quadloco::obj::Camera
*/


#include "objCamera.hpp"

#include <Engabra>

#include <iostream>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// A default null camera
		quadloco::obj::Camera const nullCam{};
		bool const nullIsOkay{ (false == nullCam.isValid()) };

		// Ideal perspective camera by detector format and principal distance
		constexpr quadloco::ras::SizeHW format{ 128u, 256u };
		constexpr double pd{ 300. };
		quadloco::obj::Camera const camera{ format, pd };

		// output to stream
		std::ostringstream msg;
		msg << "camera: " << camera << '\n';

		// location relative to camera external frame
		// naming relative to 3D exterior frame
		// ('x' to right, 'y' up, 'z' toward viewer)
		engabra::g3::Vector const rgtPntInExt{  1.,  0., -30. };
		engabra::g3::Vector const lftPntInExt{ -1.,  0., -30. };
		engabra::g3::Vector const topPntInExt{  0.,  2., -30. };
		engabra::g3::Vector const botPntInExt{  0., -2., -30. };

		// project locations onto detector frame
		using quadloco::img::Spot;
		// naming relative to raster row/col as displayed on typical screen
		// ('row' down, 'y' to right, 'z' toward viewer)
		Spot const rgtOnDet{ camera.detectorSpotFor(rgtPntInExt) };
		Spot const lftOnDet{ camera.detectorSpotFor(lftPntInExt) };
		Spot const topOnDet{ camera.detectorSpotFor(topPntInExt) };
		Spot const botOnDet{ camera.detectorSpotFor(botPntInExt) };

		// check expected "on screen" geometry
		bool const okayRowLR{ (lftOnDet.row() == rgtOnDet.row()) };
		bool const okayColLR{ (lftOnDet.col() < rgtOnDet.col()) };
		bool const okayRowTB{ (topOnDet.row() < botOnDet.row()) };
		bool const okayColTB{ (topOnDet.col() == botOnDet.col()) };

	//	double const objDistLR{ magnitude(rgtPntInExt - lftPntInExt) };
	//	double const imgDistLR{ magnitude(rgtOnDet - lftOnDet) };

		// forward project detector spots from camera ...
		using engabra::g3::Vector;
		Vector const rgtDirInExt{ camera.directionForDetSpot(rgtOnDet) };
		Vector const lftDirInExt{ camera.directionForDetSpot(lftOnDet) };
		Vector const topDirInExt{ camera.directionForDetSpot(topOnDet) };
		Vector const botDirInExt{ camera.directionForDetSpot(botOnDet) };

		// ... and check if they are collinear with the object space points
		// (e.g. bivector product of direction (from origin) and pnt is zero)
		double const rgtErrMag
			{ engabra::g3::magnitude((rgtDirInExt * rgtPntInExt).theBiv) };
		double const lftErrMag
			{ engabra::g3::magnitude((lftDirInExt * lftPntInExt).theBiv) };
		double const topErrMag
			{ engabra::g3::magnitude((topDirInExt * topPntInExt).theBiv) };
		double const botErrMag
			{ engabra::g3::magnitude((botDirInExt * botPntInExt).theBiv) };

		// [DoxyExample01]

		if (! nullIsOkay)
		{
			oss << "Failure of nullIsOkay test\n";
			oss << "nullCam: " << nullCam << '\n';
		}

		if ( msg.str().empty())
		{
			oss << "Failure of op<<() test\n";
		}

		bool const allOkay{ okayRowLR && okayColLR && okayRowTB && okayColTB };
		if (! allOkay)
		{
			oss << "Failure of allOkay test\n";
			oss << "okayRowLR: " << std::boolalpha << okayRowLR << '\n';
			oss << "okayColLR: " << std::boolalpha << okayColLR << '\n';
			oss << "okayRowTB: " << std::boolalpha << okayRowTB << '\n';
			oss << "okayColTB: " << std::boolalpha << okayColTB << '\n';
			oss << "rgtOnDet: " << rgtOnDet << '\n';
			oss << "lftOnDet: " << lftOnDet << '\n';
			oss << "topOnDet: " << topOnDet << '\n';
			oss << "botOnDet: " << botOnDet << '\n';
		}

		using engabra::g3::io::fixed;
		if (! engabra::g3::nearlyEquals(rgtErrMag, 0.))
		{
			oss << "Failure of rgtErrMag misclosure test\n";
			oss << "rgtErrMag: " << fixed(rgtErrMag, 1u, 15u) << '\n';
		}
		if (! engabra::g3::nearlyEquals(lftErrMag, 0.))
		{
			oss << "Failure of lftErrMag misclosure test\n";
			oss << "lftErrMag: " << fixed(lftErrMag, 1u, 15u) << '\n';
		}
		if (! engabra::g3::nearlyEquals(topErrMag, 0.))
		{
			oss << "Failure of topErrMag misclosure test\n";
			oss << "topErrMag: " << fixed(topErrMag, 1u, 15u) << '\n';
		}
		if (! engabra::g3::nearlyEquals(botErrMag, 0.))
		{
			oss << "Failure of botErrMag misclosure test\n";
			oss << "botErrMag: " << fixed(botErrMag, 1u, 15u) << '\n';
		}

	}

}

//! Check behavior of NS
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	test0(oss);

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

