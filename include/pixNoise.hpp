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


#pragma once


/*! \file
 * \brief Declarations for quadloco::pix::Noise
 *
 */


#include <random>


namespace quadloco
{

namespace pix
{
	//! Pseudo-Random-Number utilities
	namespace prn
	{
		//! Generateor to use for quadloco::pix
		static std::mt19937 sGen(47989969u);

	}; // [prn]


	//! A simplistic radiometric noise model for digital imagery
	struct Noise
	{
		//! Electrons per digital number value in readout
		static constexpr double theElectPerValue{ 70000. / 255. };

		//! Digital numbers per electron in a pixel
		static constexpr double theValuePerElect{ 1./theElectPerValue };


		//! Value associated with number of electrons
		inline
		double
		valueForNumE
			( std::size_t const & numE
			) const
		{
			return (theValuePerElect * (double)numE);
		}

		/*! Dark current electrons (ideally Poisson based on temperature)
		 *
		 * A more complete model would include:
		 * \arg Proportional to exposure time
		 * \arg Variance increases (radically) with increasing temperatur
		 *
		 */
		inline
		std::size_t
		eNumberDark
			( double const & expoSeconds = 1.
			) const
		{
			// distribution should be a (strong) function of temperature
			// NOTE: here assume constant temperature
			static std::size_t const ePerPixPerSec{ 10u };

			double const meanElect{ ePerPixPerSec * expoSeconds };
			std::poisson_distribution<std::size_t> distro(meanElect);

			// noise electrons
			return distro(prn::sGen);
		}

		//! Shot noise (Poisson on intensity peaking at sqrt of meanNumE)
		inline
		double
		eNumberShot
			( double const & intensityValue
			) const
		{
			double eShot{ 0. };
			if (0. < intensityValue)
			{
				double const fullNumE{ theElectPerValue * intensityValue };
				double const rootNumE{ std::sqrtf(fullNumE) };
				std::size_t const distroMean
					{ static_cast<std::size_t>(rootNumE) };

				// distribution proportional to meanIntensity
				std::poisson_distribution<std::size_t> distro(distroMean);

				// noise electrons
				eShot = distro(prn::sGen);
			}
			return eShot;
		}

		//! Dark current noise approximation (ignoring temperature dependency)
		inline
		double
		valueDark
			( double const & expoSeconds = 1.
			) const
		{
			double const pixVal{ valueForNumE(eNumberDark(expoSeconds)) };
			return pixVal;
		}

		//! Shot noise contribution relation to incident intensity
		inline
		double
		valueShot
			( double const & meanInValue
			) const
		{
			double const pixVal{ valueForNumE(eNumberShot(meanInValue)) };
			return pixVal;
		}

		//! Combined noise (dark + shot)
		inline
		double
		valueFor
			( double const & incidentValue
			, double const & expoSeconds = 1.
			) const
		{
			// dark current noise
			double const pixValDark{ valueDark(expoSeconds) };

			// photon shot noise
			double const pixValShot{ valueShot(incidentValue) };

			// combined
			return (pixValDark + pixValShot);
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			oss
				<< "theElectPerValue: " << theElectPerValue
				<< ' '
				<< "theValuePerElect: " << theValuePerElect
				;

			return oss.str();
		}


	}; // Noise


} // [pix]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::pix::Noise const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	/*
	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::pix::Noise const & item
		)
	{
		return item.isValid();
	}
	*/

} // [anon/global]

