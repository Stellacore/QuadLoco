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
\brief Unit tests (and example) code for quadloco::prb::Histo
*/


#include "opsPeakFinder1D.hpp"
#include "prbHisto.hpp"
#include "valSpan.hpp"

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

		// Construct histogram
		prb::Histo hist
			( 100u  // Number of bins in accumulation array
			, val::Span{ 100., 200. } // Span of values to cover
			);

		// ... and populate with data

		// A first peak
		hist.addValue(100.0, 1.5); // at start of span
		// A second peak - average of three close data values
		// these values are close to each other relative to sigma magnitude
		hist.addValue(114.0, 1.5);
		hist.addValue(115.0, 1.5);
		hist.addValue(116.0, 1.5);
		// A third peak
		hist.addValue(199.99999, 1.5); // at end of span
		hist.addValue(200.00000, 1.5); // outside span (not used)

		// expected peak location results
		std::vector<double> const expPeakVals { 100., 115., 199.  };

		//std::cout << "hist: " << hist << '\n';
		//std::cout << hist.infoStringContents("histC") << '\n';

		// Extract peaks (local modes in histogram distribution)
		std::vector<double> gotPeakVals;
		std::vector<double> const & binProbs = hist.probabilities();
		ops::PeakFinder1D const peakFinder
			(binProbs.cbegin(), binProbs.cend(), ops::PeakFinder1D::Linear);
		std::vector<std::size_t> const peakNdxs{ peakFinder.peakIndices() };

		// Retrieve data values associated with bins containing peaks
		gotPeakVals.reserve(peakNdxs.size());
		for (std::size_t const & peakNdx : peakNdxs)
		{
			gotPeakVals.emplace_back(hist.valueAtIndex(peakNdx));
		}

		// [DoxyExample01]

		if (! (gotPeakVals.size() == expPeakVals.size()))
		{
			oss << "Failure of gotPeakVals.size() test\n";
			oss << "exp: " << expPeakVals.size() << '\n';
			oss << "got: " << gotPeakVals.size() << '\n';
			oss << "hist: " << hist << '\n';
		}
		else // same number of elements
		{
			auto const infoVals
				{ [] (std::vector<double> const & vals)
					{
						std::ostringstream oss;
						for (double const & val : vals)
						{
							oss << ' ' << val;
						}
						return oss.str();
					}
				};
			// assume return order is increasing value as with expPeakVals
			bool const sameValues
				{ std::equal
					( gotPeakVals.cbegin(), gotPeakVals.cend()
					, expPeakVals.cbegin()
					)
				};
			if (! sameValues)
			{
				oss << "Failure of sameValues test\n";
				oss << "exp: " << infoVals(expPeakVals) << '\n';
				oss << "got: " << infoVals(gotPeakVals) << '\n';
				oss << "hist: " << hist << '\n';
			//	oss << hist.infoStringContents("histContents") << '\n';
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

