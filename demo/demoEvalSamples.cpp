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


/*! \file
\brief Evaluate all chips in sample directory and report center finding results.
*/


#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>


namespace
{
	//! Utility structure for tracking associated files
	struct FileInfo
	{
		std::filesystem::path theParentPath{};
		bool theHasPGM{ false };
		bool theHasMea{ false };

		inline
		bool
		hasAll
			() const
		{
			return (theHasPGM && theHasMea);
		}

		inline
		std::filesystem::path
		pathFor
			( std::string const & baseName
			, std::string const & ext
			) const
		{
			std::filesystem::path path{ theParentPath };
			path /= baseName;
			path += ext;
			return path;
		}

	}; // FileInfo

	//! Put item to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, FileInfo const & fileInfo
		)
	{
		ostrm
			<< fileInfo.theParentPath.native()
			<< "  pgm: " << fileInfo.theHasPGM
			<< "  mea: " << fileInfo.theHasMea
			;
		return ostrm;
	}

	//! Combination of files needed for processing
	struct FileSet
	{
		std::filesystem::path thePathPGM;
		std::filesystem::path thePathMea;

	}; // FileSet


	//! Collection of input valid/complete file path combinations
	inline
	std::vector<FileSet>
	fileSetsFrom
		( std::filesystem::path const & dirPath
		)
	{
		std::vector<FileSet> fileSets;
		namespace fs = std::filesystem;

		using BaseName = std::string;
		std::map<BaseName, FileInfo> baseInfos;

		static std::string const extPGM(".pgm");
		static std::string const extMea(".meapoint");

		for (std::filesystem::path const & entry
			: std::filesystem::directory_iterator(dirPath))
		{
			if (fs::is_regular_file(entry))
			{
				std::string const ext{ entry.extension() };
				bool const isPGM{ (extPGM == ext) };
				bool const isMea{ (extMea == ext) };

				if (isPGM || isMea)
				{
					BaseName const baseName{ entry.stem() };
					fs::path const parent{ entry.parent_path() };
					fs::path basePath{ parent };
					basePath /= baseName;
					std::map<BaseName, FileInfo>::iterator itFind
						{ baseInfos.find(baseName) };

					// if not present already, insert a new FileInfo
					if (baseInfos.cend() == itFind)
					{
						FileInfo const baseInfo{ parent };
						itFind = baseInfos.emplace_hint
							( baseInfos.end()
							, std::make_pair(baseName, baseInfo)
							);
					}

					// update (now) existing FileInfo fields
					if (baseInfos.cend() != itFind)
					{
						FileInfo & fileInfo = itFind->second;
						if (isPGM)
						{
							fileInfo.theHasPGM = true;
						}
						else
						if (isMea)
						{
							fileInfo.theHasMea = true;
						}
					}

					std::string const stem{ entry.stem() };

				} // ext

			} // regular

		} // dir scan

		// prune for return of relevant values
		fileSets.reserve(baseInfos.size());
		for (std::map<BaseName, FileInfo>::value_type
			const & baseInfo : baseInfos)
		{
			std::string const & baseName = baseInfo.first;
			FileInfo const & fileInfo = baseInfo.second;
			if (fileInfo.hasAll())
			{
				FileSet const fileSet
					{ fileInfo.pathFor(baseName, extPGM)
					, fileInfo.pathFor(baseName, extMea)
					};
				fileSets.emplace_back(fileSet);
			}
		}

		return fileSets;
	}


	//! Processing results
	struct DoneInfo
	{
		FileSet const theFileSet;
		std::filesystem::path const theSaveDir;

//		return DoneInfo{ pathPGM, pathMea, saveDir };
//		std::filesystem::path const & pathPGM = fileSet.thePathPGM;
//		std::filesystem::path const & pathMea = fileSet.thePathMea;
//std::filesystem::path const thePathPGM;
//std::filesystem::path const thePathMea;

		//! Common basename of input FileSet
		inline
		std::string
		sampleName
			() const
		{
			return theFileSet.thePathPGM.stem();
		}

	}; // DoneInfo


	//! Put item to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, DoneInfo const & doneInfo
		)
	{
		ostrm
			<< "  pgm: " << doneInfo.theFileSet.thePathPGM << '\n'
			<< "  mea: " << doneInfo.theFileSet.thePathMea << '\n'
			;
		if (! doneInfo.theSaveDir.empty())
		{
			ostrm
				<< " save: " << doneInfo.theSaveDir << '\n'
				;
		}
		return ostrm;
	}


	//! Result from load/process input files (optn'ly save intermediate data)
	inline
	DoneInfo
	processFileSet
		( FileSet const & fileSet
		, std::filesystem::path const & saveDir
		)
	{
		return DoneInfo{ fileSet, saveDir };
	}

	//! Generate a complete report string from collection of results
	inline
	std::string
	reportString
		( std::vector<DoneInfo> const & doneInfos
		)
	{
		std::ostringstream rpt;
		for (DoneInfo const & doneInfo : doneInfos)
		{
			rpt
				<< "\n======= "
				<< doneInfo.sampleName()
				<< '\n'
				<< doneInfo
				;
		}
		return rpt.str();
	}

} // [anon]


/*! \brief Process all chip images from sample directory and report results.
 *
 * Arguments specify a load directory, loadDir. Activity proceeds as:
 * \arg Search all file basenames that have both ".pgm" and ".meapoint"
 *      files in loadDir/.
 * \arg Read the .meapoint to obtain expCenter location.
 * \arg Process the chip data to find best center estimate, gotCenter.
 * \arg Compute difCenter as difference of gotCenter less expCenter.
 * \arg Generate report of the {got,exp,dif}Center results.
 *
 * Optionally, if a saveDir argument is provided, intermediate results
 * are saved to files in saveDir/.
 */
int
main
	( int argc
	, char * argv[]
	)
{
	if (! (1 < argc))
	{
		std::cerr << '\n';
		std::cerr << "Process all chip samples in a directory\n";
		std::cerr << '\n';
		std::cerr << "Usage: <progname> <loadDir> [saveDir]\n";
		std::cerr << '\n';
		return 1;
	}
	std::filesystem::path const appPath(argv[0]);
	std::filesystem::path const loadDir(argv[1]);
	std::filesystem::path saveDir{};
	if (2 < argc)
	{
		saveDir = std::filesystem::path(argv[2]);
	}

	// perform detection and generate report records
	std::vector<DoneInfo> doneInfos;
	std::vector<FileSet> const fileSets{ fileSetsFrom(loadDir) };
	doneInfos.reserve(fileSets.size());
	for (FileSet const & fileSet : fileSets)
	{
		DoneInfo const doneInfo{ processFileSet(fileSet, saveDir) };
		doneInfos.emplace_back(doneInfo);
	}

	// report results
	std::string const report{ reportString(doneInfos) };

	std::string const pad("  ");
	std::cout << '\n';
	std::cout << report << '\n';
	std::cout << '\n';
	std::cout << "application: "
		<< std::filesystem::canonical(appPath).native() << '\n';
	std::cout << pad << "loadDir: " << loadDir.native() << '\n';
	std::cout << pad << "saveDir: " << saveDir.native() << '\n';
	std::cout << '\n';

	return 0;
}


