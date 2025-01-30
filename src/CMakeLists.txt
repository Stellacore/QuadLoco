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

##
## -- CMake build system description
##

message("### thisProjLib: ${thisProjLib}")


# ===
# === Library Components
# ===

# set(thisProjLibTarget "${thisProjLib}Lib")

set(srcFiles

	QuadLoco.cpp

	)

add_library(
	${thisProjLib}
	${srcFiles}
	)
add_library(
	${PROJECT_NAME}::${PROJECT_NAME}
	ALIAS
		${thisProjLib}
	)

target_sources(
	${thisProjLib}
	PUBLIC
		FILE_SET cppHeaders
			TYPE HEADERS
			BASE_DIRS
				../include/
			FILES
				../include/QuadLoco/ang.hpp
				../include/QuadLoco/angRing.hpp
				../include/QuadLoco/appcenter.hpp
				../include/QuadLoco/app.hpp
				../include/QuadLoco/appkeyed.hpp
				../include/QuadLoco/appQuadLike.hpp
				../include/QuadLoco/cast.hpp
				../include/QuadLoco/imgArea.hpp
				../include/QuadLoco/imgCircle.hpp
				../include/QuadLoco/imgEdgel.hpp
				../include/QuadLoco/imgGrad.hpp
				../include/QuadLoco/imgHit.hpp
				../include/QuadLoco/img.hpp
				../include/QuadLoco/imgQuadTarget.hpp
				../include/QuadLoco/imgRay.hpp
				../include/QuadLoco/imgSpot.hpp
				../include/QuadLoco/imgVector.hpp
				../include/QuadLoco/io.hpp
				../include/QuadLoco/matEigen2D.hpp
				../include/QuadLoco/matfunc.hpp
				../include/QuadLoco/mat.hpp
				../include/QuadLoco/mattype.hpp
				../include/QuadLoco/meaCluster.hpp
				../include/QuadLoco/meaCovar.hpp
				../include/QuadLoco/mea.hpp
				../include/QuadLoco/meaVector.hpp
				../include/QuadLoco/objCamera.hpp
				../include/QuadLoco/obj.hpp
				../include/QuadLoco/objQuadTarget.hpp
				../include/QuadLoco/opsAllPeaks2D.hpp
				../include/QuadLoco/opsAngleTracker.hpp
				../include/QuadLoco/opsCenterRefinerEdge.hpp
				../include/QuadLoco/opsCenterRefinerSSD.hpp
				../include/QuadLoco/opsFence.hpp
				../include/QuadLoco/opsfilter.hpp
				../include/QuadLoco/opsgrid.hpp
				../include/QuadLoco/ops.hpp
				../include/QuadLoco/opsPeakFinder1D.hpp
				../include/QuadLoco/opsSymRing.hpp
				../include/QuadLoco/pix.hpp
				../include/QuadLoco/pixNoise.hpp
				../include/QuadLoco/prbGauss1D.hpp
				../include/QuadLoco/prbHisto.hpp
				../include/QuadLoco/prb.hpp
				../include/QuadLoco/prbStats.hpp
				../include/QuadLoco/QuadLoco
				../include/QuadLoco/QuadLoco.hpp
				../include/QuadLoco/rasChipSpec.hpp
				../include/QuadLoco/rasgrid.hpp
				../include/QuadLoco/rasGrid.hpp
				../include/QuadLoco/ras.hpp
				../include/QuadLoco/raskernel.hpp
				../include/QuadLoco/rasPeakRCV.hpp
				../include/QuadLoco/rasRelRC.hpp
				../include/QuadLoco/rasRowCol.hpp
				../include/QuadLoco/rasSizeHW.hpp
				../include/QuadLoco/simConfig.hpp
				../include/QuadLoco/simgrid.hpp
				../include/QuadLoco/sim.hpp
				../include/QuadLoco/simRender.hpp
				../include/QuadLoco/simSampler.hpp
				../include/QuadLoco/sys.hpp
				../include/QuadLoco/sysTimer.hpp
				../include/QuadLoco/val.hpp
				../include/QuadLoco/valSpan.hpp
				../include/QuadLoco/xfm.hpp
				../include/QuadLoco/xfmMapSizeArea.hpp
	)

target_compile_options(
	${thisProjLib}
	PRIVATE
		$<$<CXX_COMPILER_ID:Clang>:${BUILD_FLAGS_FOR_CXX_CLANG}>
		$<$<CXX_COMPILER_ID:GNU>:${BUILD_FLAGS_FOR_CXX_GCC}>
		$<$<CXX_COMPILER_ID:MSVC>:${BUILD_FLAGS_FOR_CXX_VISUAL}>
	)

target_include_directories(
	${thisProjLib}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
		$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/ # local source headers
	)

target_link_libraries(
	${thisProjLib}
	PRIVATE
		Engabra::Engabra
		Rigibra::Rigibra
	)

##
## == Export CMake info for use of these targets by other CMake projects
##



## This is substituted into xConfig.cmake.in and used for install Exports
set(EXPORT_FILENAME ${PROJECT_NAME}Targets.cmake)

configure_package_config_file(
	${CMAKE_SOURCE_DIR}/cmake/xConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
	INSTALL_DESTINATION
		${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	)

write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
	VERSION
		${PROJECT_VERSION}
	COMPATIBILITY
		SameMajorVersion
	)

install(FILES
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
	DESTINATION
		${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	)


install(TARGETS
		${thisProjLib}
	EXPORT
		${PROJECT_NAME}Targets
	FILE_SET cppHeaders
	)

install(EXPORT
		${PROJECT_NAME}Targets
	FILE
		${EXPORT_FILENAME}  # ${PROJECT_NAME}Targets.cmake & xConfig.cmake.in
	NAMESPACE
		${PROJECT_NAME}::
	DESTINATION
		${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	)

