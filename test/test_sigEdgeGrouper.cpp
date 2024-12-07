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
\brief Unit tests (and example) code for quadloco::sig::EdgeGrouper
*/


#include "sigEdgeGrouper.hpp"

#include "imgEdgel.hpp"

#include <algorithm>
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
		// [DoxyExample00]

		using namespace quadloco;

		/*
		sig::EdgeGrouper const aNull{};
		bool const expIsValid{ false };
		bool const gotIsValid{ isValid(aNull) };

		// [DoxyExample00]

		if (! (gotIsValid == expIsValid))
		{
			oss << "Failure of gotIsValid aNull test\n";
			oss << "aNull: " << aNull << '\n';
		}
		*/
	}

	//! Fuctor to test if two edgels represent approximiately same edge
	struct NearlySameEdgel
	{
		double const theTol{ -1. };

		//! True directions are about the same and locations on same line
		inline
		bool
		operator()
			( quadloco::img::Ray const & ray1
			, quadloco::img::Ray const & ray2
			) const
		{
			using namespace quadloco::img;
			bool same{ false };
			Vector<double> const & dir1 = ray1.direction();
			Vector<double> const & dir2 = ray2.direction();
			double const dotPro{ dot(dir1, dir2) };
			if (0. < dotPro)
			{
				double const align{ outer(dir1, dir2) };
				if (std::abs(align) < theTol)
				{
					double const proj12{ ray1.distanceAlong(ray2.start()) };
					double const proj21{ ray2.distanceAlong(ray1.start()) };
					double const colinErr
						{ .5 * (std::abs(proj12) + std::abs(proj21)) };
					same = (colinErr < theTol);
				}
			}
			return same;
		}

	}; // NearlySameEdgel

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace quadloco;

		// A collection of edgels consistent with an (ideal) quad target
		using namespace quadloco::img;
		std::vector<Edgel> const edgels
			// (Note location units expected to have magnitude of pixels)
			{	// Radial edgels
			  Edgel{ Spot{  5.,  0. }, Grad{  0.,  1. } }  // 0
			, Edgel{ Spot{  0.,  5. }, Grad{ -1.,  0. } }  // 1
			, Edgel{ Spot{ -5.,  0. }, Grad{  0., -1. } }  // 2
			, Edgel{ Spot{  0., -5. }, Grad{  1.,  0. } }  // 3
				// Triangle corner edgels
			, Edgel{ Spot{  5.,  5. }, Grad{  1.,  1. } }  // 4
			, Edgel{ Spot{ -5., -5. }, Grad{ -1., -1. } }  // 5
				// Target limit edgels
			, Edgel{ Spot{  8.,  0. }, Grad{ -1.,  0. } }  // 6
			, Edgel{ Spot{  0.,  8. }, Grad{  0., -1. } }  // 7
			, Edgel{ Spot{ -8.,  0. }, Grad{  1.,  0. } }  // 8
			, Edgel{ Spot{  0., -8. }, Grad{  0.,  1. } }  // 9
			};

		// Simulate processing that detects these edgels
		std::vector<sig::EdgeInfo> edgeInfos;
		edgeInfos.reserve(edgels.size());
		for (Edgel const & edgel : edgels)
		{
			edgeInfos.emplace_back(sig::EdgeInfo(edgel));
		}
		// have each EdgeInfo consider all other edgels
		for (sig::EdgeInfo & edgeInfo : edgeInfos)
		{
			for (Edgel const & edgel : edgels)
			{
				// consideration of self should have no effect (zero weight)
				edgeInfo.considerOther(edgel);
			}
		}

		// Mean edge rays that are most consistent with quad target
		constexpr std::size_t numAngBins{ 32u };
		constexpr double alignSigma{ .45 };
		std::vector<sig::RayWgt> mainEdgeRayWgts
			{ sig::EdgeGrouper::mainEdgeRayWeightsFor
				(edgeInfos, numAngBins, alignSigma)
			};

		// [DoxyExample01]


		// The first four test case edgels should all be found
		std::vector<img::Ray> const expEdges
			{ edgels.begin(), edgels.begin()+4u };

		// check if each one is found
		std::vector<bool> wasFounds(expEdges.size(), false);
		NearlySameEdgel sameEdgel{ .001 };
		for (std::size_t nExp{0u} ; nExp < expEdges.size(); ++nExp)
		{
			img::Ray const & expEdge = expEdges[nExp];
			for (sig::RayWgt const & mainEdgeRayWgt : mainEdgeRayWgts)
			{
				img::Ray const & gotEdge = mainEdgeRayWgt.item();
				if (sameEdgel(expEdge, gotEdge))
				{
					wasFounds[nExp] = true;
					break;
				}
			}
		}
		bool hitErr{ false };
		for (std::size_t nExp{0u} ; nExp < expEdges.size(); ++nExp)
		{
			img::Ray const & expEdge = expEdges[nExp];
			img::Ray const & gotEdge = mainEdgeRayWgts[nExp].item();
			if (! wasFounds[nExp])
			{
				hitErr = true;
				oss << "Failure of find expected edge test\n";
				oss << "expEdge: " << expEdge << '\n';
			}
		}
		if (hitErr)
		{
			oss << "gotEdges\n";
			for (sig::RayWgt const & mainEdgeRayWgt : mainEdgeRayWgts)
			{
				img::Ray const & gotEdge = mainEdgeRayWgt.item();
				oss << "gotEdge: " << gotEdge << '\n';
			}
		}
	}

}

//! Standard test case main wrapper
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	test0(oss);
	test1(oss);

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

