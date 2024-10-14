# QuadLoco - Crazy fast quadrant target center locator


# Purpose

Quadrant targets are useful for supporting very precise location measurements
in various photogrammetry and computer vision applications.

Classic quadrant target is pattern of alternating light and dark quadrants of
a square. Often seen on spacecraft and launch vehicles and robot development
and testing environments.

There are multiple phases:
* Detection (finding what features in an image are likely quadrant targets)
* Localization (finding center with subpixel precision and accuracy)
* Verification (using "find" solution to verify raster sample is a quad target)


# Starting Concepts

* Quadrant target design

	- alternating quadrants of homogeous dark and light areas (e.g quadrants
	  1 and 3 are dark while 2 and 4 are light or vice versa.

* Finder working with data from a perspective image - specifically

	- homogenous areas remain relatively homogenous
	- there is a reasonable Signal/Noise ratio
	- the edges are straight lines
		- if perspective image contains distortion, edgel positions
		  should be corrected. TODO/TBD - how/where to do this in design

* Assume a full target is contained within the raster search area

	- there are at least a few (e.g. 3-5) pixels visible on each radial leg



