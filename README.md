# QuadLoco - high precision quadrant target center locator

This package provides a single algorithm focused on the accurate
location of the precise center of a quadrant target.

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
reliable and accurate finding of the center point in digital images
quad targets.


There are multiple phases:
* Detection (finding what features in an image are likely quadrant targets)
* Localization (finding center with subpixel precision and accuracy)
* Verification (using "find" solution to verify raster sample is a quad target)

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

///--- TBD
* Compute edgel data from source image (keeping track of edge strengths
  and locations.

	- Apply corrections for optical distortions present in the camera
	system (if any)

* Utilize a specialized Hough detection to find candidates for the radial
  edges

	- Ideally for all four, but accept that one or two of the edges
	may contain relatively few pixels compred with the other two.
---/// TBD

* Estimate center point area

	- intersection of pairs that represent candidates for adjacent radial
	edges

* Classify pixels as belonging to radial edges or not

	- Determine consistency metric for each edgel w.r.t. each candidate
	radial edge.

* Select most promising quartet of radial edge candidates

* For the candidate target(s),

	- Assign weights to each edgel proportedly associated with the target

* Compute precise center

	- Utilize a least squares adjustment on edgels from the four radii


## Technical Concepts

* Sample extraction: Given nominal center point, extract sample data that:

	- represent a circular region of a perspective image (expected to contain
	quad target center and significant-enough" number of pixels on each fo
	the four radial edges.

* Radon transform - search for minima and maxima in integral signals?

* Hough detection - specialized version based on perimeter circle.

	- Utilize a circumscribing circle that contains the entire sample area.

		-- in practice extract sample from a circular region
		-- scale position data such that perimeter circle has unit radius

	- A line through the sample space intersects the bounding circle in
	two places:

		-- represent first intersection point as angle position on the circle
		-- represent the second point as signed angular difference from first


