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
\brief Experiment at locating center
*/


#include "cast.hpp"
#include "houghAdderAD.hpp"
#include "imgCamera.hpp"
#include "imgQuadTarget.hpp"
#include "io.hpp"
#include "objQuadTarget.hpp"
#include "pixEdgel.hpp"
#include "pixgrid.hpp"
#include "simRender.hpp"

#include <Engabra>
#include <Rigibra>

#include <algorithm>
#include <format>
#include <vector>


namespace quadloco
{
namespace sim
{
	using namespace quadloco;

	//! Simulate images for demonstration
	inline
	std::vector<dat::Grid<float> >
	quadImages
		( std::size_t const & numImages
		)
	{
		std::vector<dat::Grid<float> > grids;
		grids.reserve(10u);

		// configure camera (including image size)
		static dat::SizeHW const hwGrid{ 128u, 128u };
		static double const pd{ 200. };
		static img::Camera const camera{ hwGrid, pd };

		// quad target size
		static obj::QuadTarget const objQuad
			( .125
			, obj::QuadTarget::AddSurround
			| obj::QuadTarget::DoubleTriangle
			);

		// simulate a collection with slightly different perspectives
		for (std::size_t nn{0u} ; nn < numImages ; ++nn)
		{
			// orientation of camera
			using namespace rigibra;
			using namespace engabra::g3;
			double dist{ .15 + .2*(double)(nn) };
std::cout << "dist: " << dist << '\n';
			double const da{ .125 * (double)nn };
			double const dx{ dist * da };
			Transform const xCamWrtQuad
				{ Vector{ dx, -dx, dist }
				, Attitude{ PhysAngle{ BiVector{ da, da, 0. } } }
				};

			sim::Render const render(camera, xCamWrtQuad, objQuad);
			std::cout << "render: " << render << '\n';

			// simulate pixel image
			grids.emplace_back(render.quadImage());
		}

		return grids;
	}

} // [sim]

namespace pix
{
	using namespace quadloco;

	//! Compute edge elements from gradient grid
	inline
	std::vector<pix::Edgel>
	edgelsFromGradGrid
		( dat::Grid<pix::Grad> const & gradGrid
		)
	{
		std::vector<pix::Edgel> pixEdgels{};
		pixEdgels.reserve(gradGrid.size());

		// could skip unset edge rows
		for (std::size_t row{0u} ; row < gradGrid.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < gradGrid.wide() ; ++col)
			{
				pix::Grad const & grad = gradGrid(row, col);
				if (grad.isValid())
				{
					pix::Edgel const edgel(dat::RowCol{ row, col }, grad);
					pixEdgels.push_back(edgel);
				}
			}
		}

		return pixEdgels;
	}

	inline
	dat::Grid<float>
	edgeMagGridFor
		( dat::SizeHW const & hwSize
		, std::vector<pix::Edgel> const & pixEdgels
		, std::size_t const & numToUse
		)
	{
		dat::Grid<float> magGrid(hwSize);
		std::fill(magGrid.begin(), magGrid.end(), 0.f);
		for (std::size_t nn{0u} ; nn < numToUse ; ++nn)
		{
			pix::Edgel const & edgel = pixEdgels[nn];
			dat::RowCol const rc{ cast::datRowCol(edgel.location()) };
			magGrid(rc) = magnitude(edgel.gradient());
		}

		return magGrid;
	}


} // [pix]

} // [quadloco]


/*! \brief Find center location in simulated quad images
*/
int
main
	()
{
	constexpr std::size_t numImages{ 7u };
	std::vector<quadloco::dat::Grid<float> > const pixGrids
		{ quadloco::sim::quadImages(numImages) };

	// loop over sample images
	for (std::size_t nn{0u} ; nn < pixGrids.size() ; ++nn)
	{
		using namespace quadloco;

		dat::Grid<float> const & pixGrid = pixGrids[nn];

		// compute gradient at each pixel (interior to edge padding)
		dat::Grid<pix::Grad> const gradGrid
			{ pix::grid::gradientGridFor(pixGrid) };

		// assemble collection of edge elements from gradient grid
		std::vector<pix::Edgel> pixEdgels
			{ edgelsFromGradGrid(gradGrid) };

		// find the strongest edges
		std::sort
			( pixEdgels.rbegin(), pixEdgels.rend()
			, [] (pix::Edgel const & e1, pix::Edgel const & e2)
				{ return magnitude(e1.gradient()) < magnitude(e2.gradient()) ; }
			);

		// estimate a reasonable number of edge pixels to expect
		// (assume target nearly filling diagaonals and with 2 pix edges)
		double const diag{ pixGrid.hwSize().diagonal() };
		std::size_t const numToUse{ static_cast<std::size_t>(8. * diag) };

std::ofstream ofs("./lineSeg.dat");

		for (std::size_t ndx1{0u} ; ndx1 < numToUse ; ++ndx1)
		{
			pix::Edgel const & e1 = pixEdgels[ndx1];
			pix::Spot const & spot1 = e1.location();
			pix::Grad const & grad1 = e1.gradient();
			double const gMag1{ magnitude(grad1) };
			float const scale1{ (float)(1. / gMag1) };
			pix::Grad const dir1{ scale1 * grad1 };

			for (std::size_t ndx2{ndx1+1} ; ndx2 < numToUse ; ++ndx2)
			{
				pix::Edgel const & e2 = pixEdgels[ndx2];
				pix::Spot const & spot2 = e2.location();
				pix::Grad const & grad2 = e2.gradient();
				double const gMag2{ magnitude(grad2) };
				float const scale2{ (float)(1. / gMag2) };
				pix::Grad const dir2{ scale2 * grad2 };

				// check for edgels with opposite gradient directions
				double const dirDot{ -dot(dir1, dir2) };
				if (0. < dirDot)
				{
					// N=100 approximates exp(-(x/.1)^2) Gaussian
					// (arbitrarily) use cos()^N as weighting function
					double const wgtFacingDir{ std::pow(dirDot, 100.) };

					// separation of the two lines
					double const dist2from1{ dot((spot2 - spot1), dir1) };
					double const dist1from2{ dot((spot1 - spot2), dir2) };
					double const distBetween{ .5 * (dist2from1 + dist1from2) };

					constexpr double distSigma{ 1.5 }; // [pix]
					constexpr double distMaxBetween{ 3. * distSigma };
					if (distMaxBetween< distBetween)
					{
						continue;
					}

					// assign pseudo weight - higher for more colinear
					double const arg{ distBetween / distSigma };
					double const wgtLineGap{ std::exp(-arg*arg) };


					double const wgtRadial{ wgtFacingDir * wgtLineGap };

					// gradient intensity for use in accumulation weights
					double const wgtGradMag{ gMag1 * gMag2 };


					// mean gradient direction with +x component
//					pix::Grad meanGrad{ .5f * (grad1 - grad2) };
//					if (meanGrad[0] < 0.f)
//					{
//						meanGrad = pix::Grad{ -1.f * meanGrad };
//					}

					// mean gradient direction with +x component
					pix::Grad meanDir{ .5f * (dir1 - dir2) };
//					if (meanDir[0] < 0.f)
//					{
//						meanDir = pix::Grad{ -1.f * meanDir };
//					}

					// spot midway between edgels
					pix::Spot const meanSpot{ .5f * (spot1 + spot2) };


if (.15 < wgtRadial)
{
					double const angle1{ atan2(meanDir[1], meanDir[0]) };
					constexpr double pi{ std::numbers::pi_v<double> };
					double const angle2{ angle1 + pi };

using engabra::g3::io::fixed;
ofs
	<< ' ' << meanSpot  // 1,2
	<< ' ' << meanDir  // 3,4
	<< ' ' << fixed(angle1)  // 5
	<< ' ' << fixed(angle2)  // 6
	<< ' ' << fixed(wgtFacingDir)  // 7
	<< ' ' << fixed(wgtLineGap) // 8
	<< ' ' << fixed(wgtRadial) // 9 FacingDir*LineGap
	<< ' ' << fixed(wgtGradMag) // 10
	<< '\n';

}

//					pix::Edgel const meanEdgel{ meanSpot, meanGrad };
				}
			}
		}



		// draw strongest pixels into grid (for development feedback)
		dat::Grid<float> const magGrid
			(edgeMagGridFor(pixGrid.hwSize(), pixEdgels, numToUse));

		std::string const pixName{ std::format("./sample{:02d}pix.pgm", nn) };
		std::string const magName{ std::format("./sample{:02d}mag.pgm", nn) };
		io::writeStretchPGM(pixName, pixGrid);
		io::writeStretchPGM(magName, magGrid);

		std::cout << '\n';
		std::cout << "pixEdgels.size: " << pixEdgels.size() << '\n';
		std::cout << "Pix data written to '" << pixName << "'\n";
		std::cout << "Mag data written to '" << magName << "'\n";

break;

	}
	std::cout << '\n';

	return 0;
}


