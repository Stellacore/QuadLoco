# QuadLoco - A fast, high precision, quadrant target center locator

This package provides a single algorithm focused on the accurate
location of the precise center of a quadrant target. The input image
chip is assumed to contain a prominent quad target (e.g. "detection"
or "extraction" of the quad target area is assumed done).

## Purpose

Quadrant targets are high contrast signal targets that are useful for
performing very precise location measurements in the context of
photogrammetry and computer vision applications.

The quad target pattern has characteristics that are conducive to high
accuracy center point location with high precision - often to within a
small fraction of a pixel.

The classic quadrant target is pattern of alternating light and dark
colored quadrants of a square. These and similar patterns are often 
visible in industrial settings, on spacecraft launch vehicles and
in precision metrology environments.

QuadLoco is a C++ open source software library that provides fast,
reliable and accurate location of the center point in digital image
regaions that are assumed to contain a quad target image.

In general practical applications, there are multiple phases:
* Detection (finding what features in an image are likely quadrant targets)
* Localization (finding center with subpixel precision and accuracy)
* Verification (using "find" solution to verify raster sample is a quad target)

This library focuses on localization.

## Usage

### Common case

There are two main algorithms that refine a nominal center location:
* SSD: Center point at best radiometric symmetry under 1/2 turn rotation
* Edge: Center point by intersecting 4 lines that best fit radial edge pixels

The SSD algorithm seems to be more robust (at least with simulated imagery)
and is the one recommended for general use.

Both algorithms operate on image/raster data contained in an instance of
quadloco::ras::Grid class. The file quadloco::ras::grid.hpp contains various
functions for creating and manipulating Grid instances. The file
quadloco::io.hpp contains functions for loading and saving Grid data for
a few very simple formats.

E.g.:
```
#include "QuadLoco/QuadLoco.hpp" // master include for entire project
...
{
	using namespace quadloco;

	// raster data loaded from 8-bit PGM file and cast to float type
	ras::Grid<std::uint8_t> const uGrid{ io::readPGM(srcPath) };
	ras::Grid<float> const srcGrid{ ras::grid::realGridOf<float>(uGrid) };
}
```

#### SSD Center Refinement

Starting with srcGrid similar to above, use class
* quadloco::ops::CenterRefinerSSD

Example code (from include/QuadLoco/opsCenterRefinerEdge.hpp)
```
{
	// refine center location based on half-turn symmetry
	using namespace quadloco;
	ras::PeakRCV const & peak = symPeakRCVs.front();
	ras::RowCol const & nomCenterRC = peak.theRowCol;
	ops::CenterRefinerSSD const ssdRefiner(&srcGrid);
	img::Hit const hit{ ssdRefiner.fitHitNear(nomCenterRC) };
	img::Spot const gotCenter{ hit.location() };
}
```

#### Edge Center Refinement

Starting with srcGrid similar to above, use class
* quadloco::ops::CenterRefinerEdge

Example code (from include/QuadLoco/opsCenterRefinerEdge.hpp)
```
{
	// refine center location based on edge response geometry
	using namespace quadloco;
	std::size_t const halfRadius{ 5u };
	ops::CenterRefinerEdge const edgeRefiner(srcGrid);
	std::vector<img::Hit> centerHits
		{ edgeRefiner.centerHits(symPeakRCVs, halfRadius) };
	if (! centerHits.empty())
	{
		// could just find the max prob or min uncertainty one
		std::sort(centerHits.rbegin(), centerHits.rend());
		img::Spot const gotCenter{ centerHits.front().location() };
	}
}
```


## Build

```
CC=/usr/bin/clang CXX=/usr/bin/clang++\
 \
 cmake\
 -DCMAKE_BUILD_TYPE=Release\
 -DCMAKE_PREFIX_PATH=/tmpLocal/\
 -DCMAKE_INSTALL_PREFIX=/tmpLocal/\
 ~/repos/QuadLoco\
 &&\
 cmake --build . --target all -j `nproc`\
 &&\
 ctest
 &&\
 cpack
```

## Project Concepts

### Quad Targets

* Quadrant target design

	- alternating quadrants of homogeous dark and light areas (e.g quadrants
	1 and 3 are dark while 2 and 4 are light or vice versa.

#### Terminology

Features

* Line - infinitely long and undirected

* Segment (of line) - finite section of a line that is assigned a direction.

	- Defined by a two points: a begin and an end
	- Segment Direction (SegDir): defined as unit vector from begin toward end
	- Edge Direction (EdgDir): righthand perpendicular to SegDir

* Quadrant

	- Invariant:
		-- Four individual Quadrant areas that form the target signal
	- Object Space:
		-- Each of four areas has same geometry
		-- Half-turn rotation symmetry
		-- Quarter turn radiometric antisymmetry
		-- right angle radial edge segments
		-- Point reflection symmetry of areas
	- Image Space:
		-- Half-turn rotation symmetry in perspective images
		-- Point reflection symmetry of areas

* Radiometric Convention

	- Foregraound - bright color (like continuation of surround)

	- Surround - foreground color (bright)

	- Background - dark color (treat like holes in the surround)

	- Signal - the two background quadrants. Defined by edges including

		-- edge: radial segments (with one end at center)

		-- edge: outer segments (perpendicular to the radial edges)

		-- area: background squares delimited by four edges each

* Border

	- Corners - two: defined by intersection of adjacent outer edge segments

		-- Outside Signal Corners

		-- Inside Signal Corners (end points of radial edges)

* Edge Relationships

	- The four radial edges point toward the vicinity of the center
	(they do not point exactly at the center because of radiometric
	effects such as blooming which displace radial edges transversly)

	- Each radial edge points toward a midside corner.

	- Adjacent edges (two radial edges that share a quadrant area in
	common.

	- Opposite edges are the the two radial edges that are approximately
	colinear with each other (albeit oppositely directed).

	- Opposite edges generally exhibit approximately equal and
	opposite transverse displacements as a function of radiometric
	effects.  Therefore, the midline halfway between opposite radial
	edges very nearly passes through the center.


* Center - theoretical point

	- For work here, the center is assumed to be part of the (pixel)
	data sample (e.g. center is visible in tentative selection window)

	- For practial purposes the image center can be defined as the
	point which simultaneously (e.g. in least squares sense) is closest
	to being colinear with all four radial edges.


### Target Image Properties

* A perspective image of the target has theoretically useful properties
that include:

	- approximately homogenous areas remain relatively homogenous
		-- variation due to illumination change
		-- radiometric noise from various sources

	- under true perspective projection, straight lines map into straight
	lines. For systems that have significant optical distortion either:
		-- algorithms should use corrected pixel coordinate locations
		-- detection/localization can be performed on a resampled image
		-- TODO/TBD - how/where to do this in design

	- there is a reasonable Signal/Noise ratio - specifically the variance
	of the foreground pixels and the variance of the background pixels is
	small compared with the distinction between background and foreground
	expected pixel values.


## General Approach

* Obtain a raster data sample from original image. E.g. by cropping pixels
from an image, or perhaps by performing a resampling operation to correct
for lens distortion.

	- Assume raster sample provides intensity values. Lowest values
	are associated with the (darker) background signal, and higher values
	are associated with the (lighter) foreground signal.

* Determine candidate center points

	- An annular filter is utilized to compute a pseudo-probability that
	the radiometric content of the annular area has half-turn symmetry.

	- Local maxima in the annular filter response are associated with
	candidate peak locations (with resolution of integer pixel location)

* Compute precise center

	- The strongest peaks are utilized as nominal locations about which
	to run an area based half-turn symmetry filter. This area filter is
	run on a small neighborhood around each candidate peak location.

	- The area filter response is used to compute a subpixel location
	for the filter response maximum.


## Software Components

### Project namespace headers

#### Namespace Summary

The project top level namespace is "quadloco". All declarations can be
accessed by including the top level file

* QuadLoco (or QuadLoco.hpp) - Project top level header - includes all others
e.g.
	```
	#include <QuadLoco>
	```

The top level "quadloco" namespace includes a number of sub-namespaces:

* app::  Application level utilities (a primary resource for consuming code)

* ang::  Angle data (e.g. Ring, ...)
* cast::  Casting functions
* img::  Image space - 2D data types (e.g. Spot, Grad, Area, ...
* io::  Input/Output basic capabilities
* mat::  Matrix operations support (very minimal)
* mea::  Measurements (values plus uncertainty information)
* obj::  Object space - 3D data type (e.g. QuadTarget, ...)
* ops::  Operations and processing (GridFilter, PeakFinder, ...)
* pix::  Picture element/radiometry (e.g.Noise, ...)
* prb::  Probability theory (e.g. Gauss1D, Stats, ...)
* ras::  Raster data (e.g. Grid, RowCol, SizeHW, ...)
* sim::  Simulation capabilities (e.g. Config, QuadTarget, grid, ...)
* sys::  System utilities (e.g. timing)
* val::  Value types (scalar data)
* xfm::  Transformations (e.g. MapSizeArea, ...)

#### Namespace Detail

Each project namespace has its own namespace header file that includes all
components of that namespace.

For details refer to the 'namespace' tab in project documenation created
by Doxygen during project build.

