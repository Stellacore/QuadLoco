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
	/*! \brief Projection of edge direction (for alpha,delta) onto posRefDir
	 *
	 * The parameters alpha,delta (location and arc on bounding circle)
	 * indirectly specify a line segment with a well defined edge
	 * direction. This function, returns the vector-vector dot product
	 * of the edge direction and the provided posRefDir direction.
	 */
	double
	edgeDirDotValueFor
		( double const & alpha
		, double const & delta
		, engabra::g3::Vector const & posRefDir
		)
	{
		using namespace engabra::g3;
		// line segment defined by end points on unit circle
		double const begAngle{ alpha };
		double const endAngle{ alpha + delta };
		Vector const begPnt{ cos(begAngle), sin(begAngle), 0. };
		Vector const endPnt{ cos(endAngle), sin(endAngle), 0. };
		// directed line tangent direction is between end points
		Vector const segDir{ direction(endPnt - begPnt) };
		double edgeDirDot{ 0. };
		if (isValid(segDir))
		{
			// edge direction is orthogonal to line segment dir (in R.H. sense)
			Vector const edgeDir{ (segDir * e12).theVec };
			// dot product with positive alignment direction
			edgeDirDot = (edgeDir * posRefDir).theSca[0];
		}
		return edgeDirDot;
	}

} // [anon]


/*! \brief Evaluate Hough space for the bounding circle reference convention.

Loop over Hough space locations with specified angle delta. For each
location determine the line segment running through sample space. For
each segment, compute the segment direction (SegDir) and dot that with
the positive reference axis. Assign the dot product value to the starting
Hough space location cell.

Write result data to ascii file with records including:

	```
	Alpha Delta EdgDirDotValue
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
	static engabra::g3::Vector const aRefDir{ engabra::g3::e2 };
	static engabra::g3::Vector const perpDir
		{ (aRefDir * engabra::g3::e12).theVec };

	// loop over length of arc on bounding circle
	static std::string const fname("foo.dat");
	std::ofstream ofs(fname);
	for (double delta{begDelta} ; delta < endDelta ; delta += da)
	{
		using engabra::g3::io::fixed;
		ofs << "\n\n# delta: = " << fixed(delta);

		// loop over starting point on bounding circle
		for (double alpha{begAlpha} ; alpha < endAlpha ; alpha += da)
		{
			// dot product between edge direction and aRefDir axis
			double const aRefDot
				{ edgeDirDotValueFor(alpha, delta, aRefDir) };
			double const perpDot
				{ edgeDirDotValueFor(alpha, delta, perpDir) };
			ofs
				<< fixed(alpha) << ' ' << fixed(delta)
				<< fixed(aRefDot)
				<< ' ' 
				<< fixed(perpDot)
				<< '\n';
		}
	}

	std::cout
		<< "\nData written to file '" << fname << "'"
		<< "\nRecord format is"
		<< "\n <alpha> <delta> <aRefDot> <perpDot>"
		<< "\nthe reference direction is: " << aRefDir
		<< "\nperpendicular direction is: " << perpDir
		<< "\n\n";

	return 0;
}


