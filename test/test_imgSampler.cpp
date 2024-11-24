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
\brief Unit tests (and example) code for quadloco::img::Sampler
*/


#include "imgSampler.hpp"

#include "imgEdgel.hpp"
#include "rasgrid.hpp" // TODO - temp
#include "sigParmAD.hpp"
#include "simgrid.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// generate grid (image) with a well defined edge
		quadloco::ras::SizeHW const hwSize{ 7u, 10u };
		quadloco::img::Edgel const expEdgel
			{ quadloco::img::Spot{ 3., 4. }
			, quadloco::img::Grad{ 2., 4. }
			};

		// [DoxyExample01]

		// simulate grid with a strongly defined intensity edge
		quadloco::ras::Grid<float> const pixGrid
			{ quadloco::sim::gridWithEdge(hwSize, expEdgel) };

		// collect gradient samples from image (one by one in this test)
		std::vector<quadloco::img::Edgel> pixEdgels;

		// sample gradient at various locations
		// NOTE: be sure pixGrid persists for scope of sampler
		quadloco::img::Sampler const sampler(& pixGrid);
		for (std::size_t row{0u} ; row < pixGrid.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < pixGrid.wide() ; ++col)
			{
				quadloco::ras::RowCol const rc{ row, col };
				quadloco::img::Grad const pixGrad{ sampler.pixGradAt(rc) };
				if (isValid(pixGrad)) // e.g. not too near image edge
				{
					quadloco::img::Edgel const edgel{ rc, pixGrad };
					pixEdgels.emplace_back(edgel);
				}
			}
		}

		// [DoxyExample01]

		std::size_t const gotSize{ pixEdgels.size() };
		std::size_t const expSize	
			{ pixGrid.size()
			- 2u * pixGrid.high()
			- 2u * pixGrid.wide()
			+ 4u
			};
		if (! (gotSize == expSize))
		{
			oss << "Failure of pixEdgels size test\n";
			oss << "exp: " << gotSize << '\n';
			oss << "got: " << expSize << '\n';
		}

		// fetch entire gradient grid
		quadloco::ras::Grid<quadloco::img::Grad> const grads
			{ quadloco::ras::grid::gradientGridFor(pixGrid) };
		// check if extracted samples match
		std::size_t sameCount{ 0u };
		for (quadloco::img::Edgel const & pixEdgel : pixEdgels)
		{
			quadloco::ras::RowCol const atRowCol
				{ quadloco::cast::rasRowCol(pixEdgel.location()) };
			quadloco::img::Grad const & expGrad = grads(atRowCol);
			quadloco::img::Grad const & gotGrad = pixEdgel.gradient();
			if (nearlyEquals(gotGrad, expGrad))
			{
				++sameCount;
			}
		}

		if (! (pixEdgels.size() == sameCount))
		{
			oss << "Failure of pixEdgels sameCount test\n";
			oss << "exp: " << pixEdgels.size() << '\n';
			oss << "got: " << sameCount << '\n';
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

//	test0(oss);
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

