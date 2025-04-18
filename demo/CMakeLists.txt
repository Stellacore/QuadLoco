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

set(ProgNames

	# _  # template for demo programs

	demoVersion

	demoEdgeDirAD
	demoSimQuadImage

	)

foreach(ProgName ${ProgNames})

	add_executable(
		${ProgName}
		${ProgName}.cpp
		)

	target_compile_options(
		${ProgName}
		PRIVATE
			$<$<CXX_COMPILER_ID:Clang>:${BUILD_FLAGS_FOR_CXX_CLANG}>
			$<$<CXX_COMPILER_ID:GNU>:${BUILD_FLAGS_FOR_CXX_GCC}>
			$<$<CXX_COMPILER_ID:MSVC>:${BUILD_FLAGS_FOR_CXX_VISUAL}>
		)

	target_include_directories(
		${ProgName}
		PUBLIC
			${CMAKE_SOURCE_DIR}/include/ # public interface
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/../src # project source
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}  # local test code includes
		)

	target_link_libraries(
		${ProgName}
		PRIVATE
			${thisProjLib}::${thisProjLib}
			Rigibra::Rigibra
			Engabra::Engabra
		)

endforeach(ProgName ${ProgNames})

