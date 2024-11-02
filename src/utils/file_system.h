#pragma once

#include <vector>
#include <iostream>
#include <algorithm>
#include <string>
#include <sstream>

#include <sys/stat.h>
#ifdef WIN32
#include <direct.h>
#define NOMINMAX
#include "windows.h"
#include <commdlg.h>
#else
#include <unistd.h>
#include <dirent.h>
#endif

#ifndef S_ISDIR
#define S_ISDIR(mode)  (((mode) & S_IFMT) == S_IFDIR)
#endif

#ifndef S_ISREG
#define S_ISREG(mode)  (((mode) & S_IFMT) == S_IFREG)
#endif

namespace utils
{
	/** \brief Tools to handle std::string objects
	*/
	class StringTools
    {
	public:
		static void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " ");
		static std::string to_upper(const std::string& str);

		/** converts a value to a string with a given precistion */
		template <typename T>
		static std::string to_string_with_precision(const T val, const int n = 6);
		/** converts a double or a float to a string */
		template<typename T>
		static std::string real2String(const T r);
	};

	/** \brief This class implements different file system functions.
	*/
	class FileSystem
    {
	public:

		static std::string getFilePath(const std::string &path);
		static std::string getFileName(const std::string &path);
		static std::string getFileNameWithExt(const std::string &path);
		static std::string getFileExt(const std::string &path);
		static bool isRelativePath(const std::string &path);
		static int makeDir(const std::string &path);
		static int makeDirs(const std::string &path);
		static std::string normalizePath(const std::string &path);
		static bool fileExists(const std::string& fileName);
		static std::string getProgramPath();
		static bool copyFile(const std::string &source, const std::string &dest);
		static bool isFile(const std::string &path);
		static bool isDirectory(const std::string &path);
		static bool getFilesInDirectory(const std::string& path, std::vector<std::string> &res);
#ifdef WIN32
		/** Open windows file dialog.\n
		* dialogType 0 = Open file dialog\n
		* dialogType 1 = Save file dialog\n
		*/
		static const std::string fileDialog(int dialogType,
			const std::string &initialDir,
			const std::string &filter);
#endif
	};

	template <typename T>
	std::string StringTools::to_string_with_precision(const T val, const int n)
	{
		std::ostringstream ostr;
		ostr.precision(n);
		ostr << std::fixed << val;
		return ostr.str();
	}

	template<typename T>
	std::string StringTools::real2String(const T r)
	{
		std::string str = to_string_with_precision(r,20);
		str.erase(str.find_last_not_of('0') + 1, std::string::npos);
		str.erase(str.find_last_not_of('.') + 1, std::string::npos);
		return str;
	}

}
