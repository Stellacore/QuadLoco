// HEADER TODO


#include <Engabra>

#include <fstream>
#include <iostream>


/*! \brief Evaluate Hough space for the bounding circle reference convention.

Loop over Hough space locations with specified angle delta. For each
location determine the line segment running through sample space. For
each segment, compute the segment direction (SegDir) and dot that with
the positive reference axis. Assign the dot product value to the starting
Hough space location cell.

Write result data to ascii file with records including:

	```
	Phi Delta SigDirDotValue
	```
*/

int
main
	()
{
	constexpr double pi{ engabra::g3::pi };

	constexpr double begPhi{ -pi };
	constexpr double endPhi{  pi };
	constexpr double begDelta{ -pi };
	constexpr double endDelta{  pi };

	constexpr double da{ 1./128. * (endPhi - begPhi) };

	std::ofstream ofs("foo.dat");
	for (double delta{begDelta} ; delta < endDelta ; delta += da)
	{
		using engabra::g3::io::fixed;
		ofs << "\n\n# delta: = " << fixed(delta);
		for (double phi{begPhi} ; phi < endPhi ; phi += da)
		{
			double const dirVal{ phi + delta }; // TODO
			ofs
				<< fixed(delta) << ' ' << fixed(phi)
				<< fixed(dirVal)
				<< '\n';
		}
	}

	std::cout << "da: " << da << '\n';

	return 0;
}


