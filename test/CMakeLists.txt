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

#	test_Quadlike  # assess the "quadness" of a pixel sampling
	test_imgCamera  # simplified ideal perspective camera
	test_datRowCol  # discete raster cell locations
	test_datSizeHW  # basic "high/wide" area boundary (half open)
	test_datGrid  # general raster grid storage and access
	test_simQuad  # simulation of perspective images of quad target

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
			${thisProjLib}::${thisProjLib}
		)

	# add test to CTest collection
	add_test(
		${aUnitTest}  # cmake target
		${aUnitTest}  # command to run test (no command line arguments used)
		)

endforeach(aUnitTest)

