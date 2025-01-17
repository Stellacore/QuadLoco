#
# MIT License #
# Copyright (c) 2024 Stellacore Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

# list of all developer unit tests (or could GLOB for filenames)
set(unitTests

	_  # test template
	test_Version  # test project version info retrieval

	test_angRing  # wrap around data structures e.g. for angles
	test_appQuadLike  # probabilitic assessor of quad target pixel patterns
	test_cast  # data type conversion operations
	test_imgArea  # a 2D range of values
	test_imgEdgel  # individual pixel edge information (Spot and Grad)
	test_imgGrad  # pixel Gradent element type and functions
	test_imgHit  # a 2D location with associated significance and uncertainty
	test_imgQuadTarget  # represent perspective image of obj::QuadTarget
	test_imgRay  # a 2D geometric ray (start Spot and unit direction)
	test_imgSpot  # 'continuous' raster cell locations
	test_imgVector  # support for 2D vector operations
	test_io  # basic i/o support (e.g. pgm images)
	test_objCamera  # simplified ideal perspective camera
	test_objQuadTarget  # represent physical target object
	test_opsAllPeaks2D  # peak finding over a 2D grid
	test_opsCenterRefinerEdge  # sub-cell center refinement using local edges
	test_opsCenterRefinerSSD  # sub-cell center refinement using half-turn SSD
	test_opsEigen2D  # Eigen value decompositions
	test_opsFence  # bounding region determination
	test_opsgrid  # Edgel extraction
	test_opsPeakAngles  # angular value likelihood estimation
	test_opsPeakFinder1D  # peak finding over a 1D collection
	test_opsSymRing  # point reflection symmetry filter
	test_pix  # pixel image manipulations
	test_prbGauss1D  # Gaussian probability density functions in 1D
	test_prbHisto  # histogram data modeling
	test_prbStats  # statistics from running data stream
	test_rasChipSpec  # a 2D sub region of a larger area
	test_rasGrid  # general raster grid storage and access
	test_rasgrid  # pixel/grid functions (e.g. image processing)
	test_rasRowCol  # discete raster cell locations
	test_rasSizeHW  # basic "high/wide" area boundary (half open)
	test_simRender  # simulation of perspective images of quad target
	test_simSampler  # simulation of image intensity sampling
	test_sysTimer  # simple interval timer
	test_valSpan  # half open interval (include start, excludes end)
	test_xfmMapSizeArea  # raster cell to continous area mapping

	)

foreach(aUnitTest ${unitTests}) # loop over all tests

	add_executable(${aUnitTest} ${aUnitTest}.cpp)

	target_compile_options(
		${aUnitTest}
		PRIVATE
			$<$<CXX_COMPILER_ID:Clang>:${BUILD_FLAGS_FOR_CXX_CLANG}>
			$<$<CXX_COMPILER_ID:GNU>:${BUILD_FLAGS_FOR_CXX_GCC}>
			$<$<CXX_COMPILER_ID:MSVC>:${BUILD_FLAGS_FOR_CXX_VISUAL}>
		)

	target_include_directories(
		${aUnitTest}
		PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include # public interface
		PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src # project source
		PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}  # local test code includes
		)

	target_link_libraries(
		${aUnitTest}
		PRIVATE
			Engabra::Engabra
			Rigibra::Rigibra
			${thisProjLib}::${thisProjLib}
		)

	# add test to CTest collection
	add_test(
		${aUnitTest}  # cmake target
		${aUnitTest}  # command to run test (no command line arguments used)
		)

endforeach(aUnitTest)

