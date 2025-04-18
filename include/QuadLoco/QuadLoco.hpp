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
 * \brief Main Project header file - includes all other public headers.
 *
 * This header includes all other project public headers so that
 * consuming code may (if desired) access all project capabilities as:
 * \code
 * #include <QuadLoco>
 * \endcode
 *
 * Includes the following project headers:
 * \snippet QuadLoco.hpp DoxyExample01
 *
 * This files also defines functions for project data adminstration
 * including
 * \arg projectVersion()
 * \arg sourceIdentity()
 */


// [DoxyExample01]

#include "QuadLoco/ang.hpp"
#include "QuadLoco/app.hpp"
#include "QuadLoco/cast.hpp"
#include "QuadLoco/img.hpp"
#include "QuadLoco/io.hpp"
#include "QuadLoco/mat.hpp"
#include "QuadLoco/mea.hpp"
#include "QuadLoco/obj.hpp"
#include "QuadLoco/ops.hpp"
#include "QuadLoco/pix.hpp"
#include "QuadLoco/prb.hpp"
#include "QuadLoco/ras.hpp"
#include "QuadLoco/sim.hpp"
#include "QuadLoco/sys.hpp"
#include "QuadLoco/val.hpp"
#include "QuadLoco/xfm.hpp"

// [DoxyExample01]


#include <string>


/*! \brief Top level project namespace.
 */
namespace quadloco
{

	//! String containing (git-derived) version information
	std::string
	projectVersion
		();

	//! String containing (git-derived) source commit information
	std::string
	sourceIdentity
		();


} // [quadloco]


