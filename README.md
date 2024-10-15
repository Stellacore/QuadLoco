# QuadLoco - high precision quadrant target center locator

This package provides a single algorithm focused on the accurate
location of the precise center of a quadrant target.

## Purpose

Quadrant targets are useful for supporting very precise location measurements
in various photogrammetry and computer vision applications.

Classic quadrant target is pattern of alternating light and dark quadrants of
a square. Often seen on spacecraft and launch vehicles and robot development
and testing environments.

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

* (Line) Segment - finite section of a line that is assigned a direction.

	- Defined by a two points: a begin and an end
	- Segment Direction (SegDir): defined as unit vector from begin toward end
	- Edge Direction (EdgDir): righthand perpendicular to SegDir

* Quadrants
	- Four areas
	- Image Space:
		-- Half-turn rotation symmetry in perspective images
		-- Point reflection symmetry of areas
	- Object Space:
		-- Each of four areas has same geometry
		-- Half-turn rotation symmetry
		-- Quarter turn radiometric antisymmetry
		-- right angle radial edge segments
		-- Point reflection symmetry of areas

* Radiometric Convention

	- Foregraound - bright color (like continuation of surround)

	- Surround - foreground color (bright)

	- Background - dark color (treat like holes in the surround)

	- Signal - the two background quadrants

	- Square Areas (squares) - the four geometric quadrants of the target
	two of which are defined by the background signal area, and two
	of which are defined on top of the forground/surround areas that
	are geometrically symmetric with the background signal squares
	under a one-quarter turn symmetry.

* Border

	- Corners - two: defined by outer edge of background quadrant

		-- Outside Signal Corners

		-- Inside signal Corners

* Radial edges

	- Four of them defined by edges of background/signal quadrants
	which come close to eh target center.

	- Opposing edges (the two radial edges that are approximately
	colinear with each other (albeit oppositely directed)

	- Adjacent edges (two radial edges that share a quadrant area in
	common.

* Center - theoreticaly point

	- At mid point of the two inside signal corners

	- At point which is half-way between each pair of opposing edges

Notation:

* Phi: Start angle around bounding circle. Defines segement begin point.

	- Principal values in half open range [-pi,pi)

* Delta: Different angle added to Phi defines segment end point.

	- Principal values in half open range [0,2pi)

* Sample Plane: Sample plane: define in association with 'e12' bivector.

* Alignment Reference Dir: vector, 'a', associated with positive edge dir

	- e.g. 'e1' vector

### Image Properties

* Finder working with data from a perspective image - specifically

	- homogenous areas remain relatively homogenous
	- there is a reasonable Signal/Noise ratio
	- the edges are straight lines
		- if perspective image contains distortion, edgel positions
		should be corrected. TODO/TBD - how/where to do this in design

### Algorithm Assumptions

* Assume a full target is contained within the raster search area

	- there are at least a few (e.g. 3-5) pixels visible on each radial leg



## General Approach

* Define geometry for sampling original raster image

* Compute edgel data from source image (keeping track of edge strengths
  and locations.

	- Apply corrections for optical distortions present in the camera
	system (if any)

* Utilize a specialized Hough detection to find candidates for the radial
  edges

	- Ideally for all four, but accept that one or two of the edges
	may contain relatively few pixels compred with the other two.

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

* Hough detection - specialized version based on perimeter circle.

	- Utilize a circumscribing ciricle that contains the entire sample area.

		-- in practice extract sample from a circular region
		-- scale position data such that perimeter circle has unit radius

	- A line through the sample space intersects the bounding circle in
	two places:

		-- represent first intersection point as angle position on the circle
		-- represent the second point as signed angular difference from first


