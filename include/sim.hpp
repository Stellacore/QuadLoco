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
 * \brief Top level file for quadloco::sim namespace
 *
 */


// #include ""//TODO


namespace quadloco
{

/*! \brief Simulation related code in namespace quadloco::sim
 */
namespace sim
{

	/*
	//! Estimate approximate image scale
	inline
	double
	approxPixPerObj
		( obj::Camera const & camera
		, rigibra::Transform const & xCamWrtQuad
		)
	{
		using namespace engabra::g3;
		double scalePixPerObj{ null<double>() };
		// arbitrary object space displacement
		constexpr double objDelta{ 1./1024./1024. }; // should be reasonable
		static Vector const objPoInQuad{ zero<Vector>() };
		static Vector const objPxInQuad{ objDelta * e1 };
		static double const objMag{ magnitude(objPxInQuad - objPoInQuad) };
		// transform into camera exterior frame
		Vector const objPoInExt{ xCamWrtQuad(objPoInQuad) };
		Vector const objPxInExt{ xCamWrtQuad(objPxInQuad) };
		// corresponding image space displacement
		img::Spot const imgPo{ camera.detectorSpotFor(objPoInExt) };
		img::Spot const imgPx{ camera.detectorSpotFor(objPxInExt) };
		if (imgPo.isValid() && imgPx.isValid())
		{
			double const imgMag{ magnitude(imgPx - imgPo) };
			scalePixPerObj = (imgMag / objMag);
		}
		return scalePixPerObj;
	}
	*/

} // [sim]

} // [quadloco]

