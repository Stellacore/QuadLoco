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
\brief Generate edge direction data as function of alhpa,delta parms.
*/


#include <Engabra>

#include <cmath>
#include <fstream>
#include <iostream>


namespace
{
	//! \brief Direction of image edge (pixel gradient dir) for alpha,delta
	inline
	engabra::g3::Vector
	gradientDirFor
		( double const & alpha
		, double const & delta
		)
	{
		using namespace engabra::g3;

		// line segment defined by end points on unit circle
		double const begAngle{ alpha };
		double const endAngle{ alpha + delta };
		Vector const begPnt{ cos(begAngle), sin(begAngle), 0. };
		Vector const endPnt{ cos(endAngle), sin(endAngle), 0. };

		// directed line tangent direction is between end points
		Vector const lineDir{ direction(endPnt - begPnt) };

		// edge direction is perpendicular to this (in *NEGATIVE* e12 dir)
		Vector const edgeDir{ (e12 * lineDir).theVec };

		return edgeDir;
	}

} // [anon]


/*! \brief Evaluate Hough space for the bounding circle reference convention.

Loop over Hough space locations with specified start and difference angles.
For each angle pair combination, determine the line segment running through
sample space and compute the associated edge pixel direction (perpendicular
to the line segment). Report the teo edge direction components as a function
of alpha,delta angle values.

Write result data to ascii file with records including:

	```
	Alpha Delta EdgeGrad[0] EdgeGrad[1]
	```
*/

int
main
	()
{
	constexpr double pi{ 1.*engabra::g3::pi };

	constexpr double begAlpha{ -pi };
	constexpr double endAlpha{  pi };
	constexpr double begDelta{ 0 };
	constexpr double endDelta{  2.*pi };

	constexpr double da{ 1./512. * (endAlpha - begAlpha) };

	// loop over length of arc on bounding circle
	static std::string const fname("/tmp/demoEdgeDir_foo.dat");
	std::ofstream ofs(fname);
	for (double delta{begDelta} ; delta < endDelta ; delta += da)
	{
		using engabra::g3::io::fixed;
		ofs << "\n\n# delta: = " << fixed(delta);

		// loop over starting point on bounding circle
		for (double alpha{begAlpha} ; alpha < endAlpha ; alpha += da)
		{
			engabra::g3::Vector const gradDir{ gradientDirFor(alpha, delta) };
			ofs
				<< fixed(alpha)
				<< ' ' 
				<< fixed(delta)
				<< ' ' 
				<< fixed(gradDir[0])
				<< ' ' 
				<< fixed(gradDir[0])
				<< '\n';
		}
	}

	std::cout
		<< "\nData written to file '" << fname << "'"
		<< "\nRecord format is"
		<< "\n <alpha> <delta> <gradDir[0]> <gradDir[1]>"
		<< "\n\n";

	return 0;
}


