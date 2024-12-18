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
\brief Unit tests (and example) code for quadloco::ops::Eigen2D
*/


#include "opsEigen2D.hpp"

#include "opsmatrix.hpp"
#include "rasGrid.hpp"

#include <Engabra>

#include <iostream>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace quadloco;

		// Grid structure is used for matrix data storage
		ras::Grid<double> matrix(2u, 2u);
		matrix(0, 0) =  1.25;
		matrix(0, 1) = -1.50;
		matrix(1, 0) =  matrix(0, 1);
		matrix(1, 1) =  2.125;

		// Create eigen decomposition instance
		ops::Eigen2D const eigen(matrix);

		// Get eigen values and vectors (here assuming for 2D matrix)
		double const gotValMin{ eigen.valueMin() };
		double const gotValMax{ eigen.valueMax() };
		img::Vector<double> const gotVecMin{ eigen.vectorMin() };
		img::Vector<double> const gotVecMax{ eigen.vectorMax() };

		// check values should be same as inputs
		img::Vector<double> const chkVecMin
			{ (1./gotValMin) * (matrix * gotVecMin) };
		img::Vector<double> const chkVecMax
			{ (1./gotValMax) * (matrix * gotVecMax) };

		bool const okayMin{ nearlyEquals(chkVecMin, gotVecMin) };
		bool const okayMax{ nearlyEquals(chkVecMax, gotVecMax) };

		// [DoxyExample01]

		if (! (okayMin && okayMax))
		{
			using engabra::g3::io::fixed;
			oss << "Failure of eigen relationship test\n";
			oss << "gotValMin: " << fixed(gotValMin) << '\n';
			oss << "gotValMax: " << fixed(gotValMax) << '\n';
			oss << "gotVecMin: " << gotVecMin << '\n';
			oss << "gotVecMax: " << gotVecMax << '\n';
			oss << "chkVecMin: " << chkVecMin << '\n';
			oss << "chkVecMax: " << chkVecMax << '\n';
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

