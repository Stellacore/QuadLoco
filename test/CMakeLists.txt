#
# MIT License
#
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

	test_cast  # data type conversion operations
	test_datArea  # a 2D range of values
	test_datChipSpec  # a 2D sub region of a larger area
	test_datCircle  # geometric circle (and line intersection)
	test_datFence  # bounding region determination
	test_datGrid  # general raster grid storage and access
	test_datMapSizeArea  # raster cell to continous area mapping
	test_datRing  # wrap around data structures e.g. for angles
	test_datRowCol  # discete raster cell locations
	test_datSizeHW  # basic "high/wide" area boundary (half open)
	test_datSpan  # half open interval (include start, excludes end)
	test_datSpot  # 'continuous' raster cell locations
	test_datVec2D  # support for 2D vector operations
	test_houghAdderAD  # accumulatio buffer for hough alpha,delta parms
	test_houghFindAD  # find maxima in hough::ParmAD space
	test_houghParmAD  # Hough "alpha,delta" parameter representation
	test_imgCamera  # simplified ideal perspective camera
	test_imgQuadTarget  # represent perspective image of obj::QuadTarget
	test_io  # basic i/o support (e.g. pgm images)
	test_objQuadTarget  # represent physical target object
	test_pixEdgel  # individual pixel edge information (Spot and Grad)
	test_pixGrad  # pixel Gradent element type and functions
	test_pixgrid  # pixel/grid functions (e.g. image processing)
	test_pix  # pixel image manipulations
	test_pixSampler  # pixel grid sampling functions
	test_pixSpot  # 'continuous' raster cell locations
	test_prbGauss1D  # Gaussian probability density functions in 1D
	test_prbquad  # probabilitic assessor of quad target pixel patterns
	test_simRender  # simulation of perspective images of quad target
	test_simSampler  # simulation of image intensity sampling

	)

foreach(aUnitTest ${unitTests}) # loop over all tests

	add_executable(${aUnitTest} ${aUnitTest}.cpp)

	target_compile_options(
		${aUnitTest}
		PRIVATE
			$<$<CXX_COMPILER_ID:Clang>:${BUILD_FLAGS_FOR_CLANG}>
			$<$<CXX_COMPILER_ID:GNU>:${BUILD_FLAGS_FOR_GCC}>
			$<$<CXX_COMPILER_ID:MSVC>:${BUILD_FLAGS_FOR_VISUAL}>
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

