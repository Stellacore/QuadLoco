// HEADER TODO


#include <Engabra>

#include <cmath>
#include <fstream>
#include <iostream>


namespace quadloco
{
	double
	edgeDirDotValueFor
		( double const & phi
		, double const & delta
		, engabra::g3::Vector const & posRefDir
		)
	{
		using namespace engabra::g3;
		// line segment defined by end points on unit circle
		double const begAngle{ phi };
		double const endAngle{ phi + delta };
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

} // [quadloco]


/*! \brief Evaluate Hough space for the bounding circle reference convention.

Loop over Hough space locations with specified angle delta. For each
location determine the line segment running through sample space. For
each segment, compute the segment direction (SegDir) and dot that with
the positive reference axis. Assign the dot product value to the starting
Hough space location cell.

Write result data to ascii file with records including:

	```
	Phi Delta EdgDirDotValue
	```
*/

int
main
	()
{
	constexpr double pi{ 1.*engabra::g3::pi };

	constexpr double begPhi{ -pi };
	constexpr double endPhi{  pi };
	constexpr double begDelta{ 0 };
	constexpr double endDelta{  2.*pi };

	constexpr double da{ 1./512. * (endPhi - begPhi) };
	static engabra::g3::Vector const posRefDir{ engabra::g3::e2 };

	static std::string const fname("foo.dat");
	std::ofstream ofs(fname);
	for (double delta{begDelta} ; delta < endDelta ; delta += da)
	{
		using engabra::g3::io::fixed;
		ofs << "\n\n# delta: = " << fixed(delta);
		for (double phi{begPhi} ; phi < endPhi ; phi += da)
		{
			double const edgDirDot
				{ quadloco::edgeDirDotValueFor(phi, delta, posRefDir) };
			ofs
				<< fixed(phi) << ' ' << fixed(delta)
				<< fixed(edgDirDot)
				<< '\n';
		}
	}

	std::cout
		<< 
		"\nData written to file '" << fname << "'"
		"\nRecord format is"
		"\n <phi> <delta> <edgeDirDot>"
		"\n(reference direction is: " << posRefDir
		<< "\n\n";

	return 0;
}


