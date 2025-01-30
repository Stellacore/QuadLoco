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
\brief Unit tests (and example) code for quadloco::mat::Eigen2D
*/


#include "QuadLoco/matEigen2D.hpp"

#include "QuadLoco/mat.hpp"
#include "QuadLoco/prbGauss1D.hpp"
#include "QuadLoco/rasGrid.hpp"

#include <Engabra>

#include <iostream>
#include <sstream>


namespace
{
	//! Check default operations
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		using namespace quadloco;

		mat::Eigen2D const aNull{};
		bool const gotIsValid{ isValid(aNull) };
		bool const expIsValid{ false };

		ras::Grid<double> matZero(2u, 2u);
		mat::Eigen2D const eigZero(matZero);
		double const gotValMinZero{ eigZero.valueMin() };
		double const gotValMaxZero{ eigZero.valueMax() };
		double const expValMinZero{ 0. };
		double const expValMaxZero{ 0. };


		// [DoxyExample00]

		using engabra::g3::io::fixed;

		if (! engabra::g3::nearlyEquals(gotValMinZero, expValMinZero))
		{
			oss << "Failure of gotValMinZero test\n";
			oss << "exp: " << fixed(expValMinZero) << '\n';
			oss << "got: " << fixed(gotValMinZero) << '\n';
			oss << "eigZero:\n" << eigZero << '\n';
		}
		if (! engabra::g3::nearlyEquals(gotValMaxZero, expValMaxZero))
		{
			oss << "Failure of gotValMaxZero test\n";
			oss << "exp: " << fixed(expValMaxZero) << '\n';
			oss << "got: " << fixed(gotValMaxZero) << '\n';
			oss << "eigZero:\n" << eigZero << '\n';
		}

		if (! (gotIsValid == expIsValid))
		{
			oss << "Failure of aNull validity test\n";
			oss << "exp: " << expIsValid << '\n';
			oss << "got: " << gotIsValid << '\n';
			oss << "aNull: " << aNull << '\n';
		}
	}

	//! Test decomposition via definition
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
		mat::Eigen2D const eigen(matrix);

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

		constexpr double tol{ 8. * std::numeric_limits<double>::epsilon() };
		bool const okayMin{ nearlyEquals(chkVecMin, gotVecMin, tol) };
		bool const okayMax{ nearlyEquals(chkVecMax, gotVecMax, tol) };

		// [DoxyExample01]

		if (! (okayMin && okayMax))
		{
			img::Vector<double> const difVecMin{ gotVecMin - chkVecMin };
			img::Vector<double> const difVecMax{ gotVecMax - chkVecMax };
			using engabra::g3::io::fixed;
			oss << "Failure of eigen relationship test\n";
			oss << "gotValMin: " << fixed(gotValMin) << '\n';
			oss << "gotValMax: " << fixed(gotValMax) << '\n';
			oss << "gotVecMin: " << gotVecMin << '\n';
			oss << "gotVecMax: " << gotVecMax << '\n';
			oss << "chkVecMin: " << chkVecMin << '\n';
			oss << "chkVecMax: " << chkVecMax << '\n';
			oss << "difVecMin: " << difVecMin << '\n';
			oss << "difVecMax: " << difVecMax << '\n';
		}

	}

	//! Test use with scatter data
	void
	test2
		( std::ostream & oss
		)
	{
		// [DoxyExample02]

		using namespace quadloco;

		using Vec2D = img::Vector<double>;
		Vec2D const expDir{ direction(Vec2D{ 3., -7. }) };
		constexpr double expSampSigma{ 1.75 };

		// generate scatter matrix using 
		ras::Grid<double> scatter(2u, 2u);
		std::fill(scatter.begin(), scatter.end(), 0.);

		// generate samples along a line (one eigenvalue is 0.)
		// and accumulate in scatter matrix
		prb::Gauss1D const gauss( 0., expSampSigma);
		double count{ 0. };
		double sumWgts{ 0. };
		double const lim{ 10. * expSampSigma };
		double const del{ (1./16.) * expSampSigma };
		for (double xx{-lim} ; xx < lim ; xx += del)
		{
			double const pdf{ gauss(xx) };
			double const weight{ 1000. * pdf };

			Vec2D const relLoc{ xx * expDir };
			scatter(0u, 0u) += weight * relLoc[0] * relLoc[0];
			scatter(0u, 1u) += weight * relLoc[0] * relLoc[1];
		// 	scatter(1u, 0u) += weight * relLoc[1] * relLoc[0];
			scatter(1u, 1u) += weight * relLoc[1] * relLoc[1];
			sumWgts += weight;
			++count;
		}
		scatter(1u, 0u) = scatter(0u, 1u); // is symmetric (and pos.def)

		// compute *sample* statistics

		// point location *sample* covariance is normalized weight matrix
		ras::Grid<double> const sampCovar{ (1./sumWgts) * scatter };

		// perform eigen decomposition of covariance matrix
		mat::Eigen2D const sampEig(sampCovar);

		// uncertainty (1-sigma) ellipsoid semiaxes are sqrt() of eigenvalues
		// by construction of test case, min eigenvalue is zero
		double const gotSampSigma{ std::sqrt(sampEig.valueMax()) };

		// estimate centroid fit statistics

		ras::Grid<double> const fitCovar{ (1./count) * sampCovar };
		double const expFitSigma{ (1./std::sqrt(count)) * expSampSigma };
		mat::Eigen2D const fitEig(fitCovar);
		double const gotFitSigma{ std::sqrt(fitEig.valueMax()) };

		// [DoxyExample02]

		constexpr double tol{ 8. * std::numeric_limits<double>::epsilon() };
		if (! engabra::g3::nearlyEquals(gotSampSigma, expSampSigma, tol))
		{
			double const difSampSigma{ gotSampSigma - expSampSigma };
			using namespace engabra::g3::io;
			oss << "Failure of gotSampSigma scatter matrix test\n";
			oss << "exp: " << fixed(expSampSigma) << '\n';
			oss << "got: " << fixed(gotSampSigma) << '\n';
			oss << "dif: " << enote(difSampSigma) << '\n';
		}
		if (! engabra::g3::nearlyEquals(gotFitSigma, expFitSigma, tol))
		{
			double const difFitSigma{ gotFitSigma - expFitSigma };
			using namespace engabra::g3::io;
			oss << "Failure of gotFitSigma scatter matrix test\n";
			oss << "exp: " << fixed(expFitSigma) << '\n';
			oss << "got: " << fixed(gotFitSigma) << '\n';
			oss << "dif: " << enote(difFitSigma) << '\n';
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
	test2(oss);

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

