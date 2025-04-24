#include "file_system.h"
#include <algorithm>
#include <cstring>

namespace utils {
    void StringTools::tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters) {
        std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
        std::string::size_type pos = str.find_first_of(delimiters, lastPos);

        while (std::string::npos != pos || std::string::npos != lastPos)
        {
            tokens.push_back(str.substr(lastPos, pos - lastPos));
            lastPos = str.find_first_not_of(delimiters, pos);
            pos = str.find_first_of(delimiters, lastPos);
        }
    }

    std::string StringTools::to_upper(const std::string& str) {
        std::string res = str;
        std::transform(res.begin(), res.end(), res.begin(), ::toupper);
        return res;
    }

    std::string FileSystem::getFilePath(const std::string &path) {
        std::string npath = normalizePath(path);

        std::string result = npath;
        size_t i = result.rfind('.', result.length());
        if (i != std::string::npos)
        {
            result = result.substr(0, i);
        }
        size_t p1 = result.rfind('\\', result.length());
        size_t p2 = result.rfind('/', result.length());
        if ((p1 != std::string::npos) && (p2 != std::string::npos))
            result = result.substr(0, std::max(p1, p2));
        else if (p1 != std::string::npos)
            result = result.substr(0, p1);
        else if (p2 != std::string::npos)
            result = result.substr(0, p2);
        return result;
    }

    std::string FileSystem::getFileName(const std::string &path) {
        std::string npath = normalizePath(path);

        std::string result = npath;
        size_t i = result.rfind('.', result.length());
        if (i != std::string::npos)
        {
            result = result.substr(0, i);
        }
        size_t p1 = result.rfind('\\', result.length());
        size_t p2 = result.rfind('/', result.length());
        if ((p1 != std::string::npos) && (p2 != std::string::npos))
            result = result.substr(std::max(p1, p2) + 1, result.length());
        else if (p1 != std::string::npos)
            result = result.substr(p1 + 1, result.length());
        else if (p2 != std::string::npos)
            result = result.substr(p2 + 1, result.length());
        return result;
    }

    std::string FileSystem::getFileNameWithExt(const std::string &path) {
        std::string npath = normalizePath(path);

        std::string result = npath;
        size_t p1 = result.rfind('\\', result.length());
        size_t p2 = result.rfind('/', result.length());
        if ((p1 != std::string::npos) && (p2 != std::string::npos))
            result = result.substr(std::max(p1, p2) + 1, result.length());
        else if (p1 != std::string::npos)
            result = result.substr(p1 + 1, result.length());
        else if (p2 != std::string::npos)
            result = result.substr(p2 + 1, result.length());
        return result;
    }

    std::string FileSystem::getFileExt(const std::string &path) {
        std::string npath = normalizePath(path);

        std::string result = npath;
        size_t i = result.rfind('.', result.length());
        if (i != std::string::npos)
        {
            result = result.substr(i + 1, result.length());
        }
        return result;
    }

    bool FileSystem::isRelativePath(const std::string &path) {
        std::string npath = normalizePath(path);

        // Windows
        size_t i = npath.find(":");
        if (i != std::string::npos)
            return false;
        else if (npath[0] == '/')
            return false;
        return true;
    }

    int FileSystem::makeDir(const std::string &path) {
        std::string npath = normalizePath(path);

        struct stat st;
        int status = 0;

        if (stat(path.c_str(), &st) != 0)
        {
#if WIN32
            status = _mkdir(path.c_str());
#else
            status = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
            if (status != 0 && errno != EEXIST)
                status = -1;
        }
        else if (!(S_IFDIR & st.st_mode))
        {
            errno = ENOTDIR;
            status = -1;
        }

        return status;
    }

    int FileSystem::makeDirs(const std::string &path) {
        char *pp;
        char *sp;
        int  status;
#ifdef WIN32
        char *copyOfPath = _strdup(path.c_str());
#else
        char *copyOfPath = strdup(path.c_str());
#endif

        status = 0;
        pp = copyOfPath;
        pp = pp + 3;		// Cut away Drive:
        while ((status == 0) && (((sp = strchr(pp, '/')) != 0) || ((sp = strchr(pp, '\\')) != 0)))
        {
            if (sp != pp)
            {
                *sp = '\0';
                status = makeDir(copyOfPath);
                *sp = '/';
            }
            pp = sp + 1;
        }
        if (status == 0)
            status = makeDir(path);
        free(copyOfPath);
        return status;
    }

    std::string FileSystem::normalizePath(const std::string &path) {
        if (path.size() == 0)
            return path;
        std::string result = path;
        std::replace(result.begin(), result.end(), '\\', '/');
        std::vector<std::string> tokens;
        StringTools::tokenize(result, tokens, "/");
        result = "";
        if (path[0] == '/')
            result = "/";
        result = result + tokens[0];
        for (unsigned int i = 1; i < tokens.size(); i++)
            result = result + "/" + tokens[i];

        return result;
    }

    bool FileSystem::fileExists(const std::string& fileName) {
        if (FILE *file = fopen(fileName.c_str(), "r"))
        {
            fclose(file);
            return true;
        }
        else
            return false;
    }

    std::string FileSystem::getProgramPath() {
        char buffer[1000];
#ifdef WIN32
        GetModuleFileName(NULL, buffer, 1000);
#else
        char szTmp[32];
        sprintf(szTmp, "/proc/%d/exe", getpid());
        int bytes = std::min((int)readlink(szTmp, buffer, 1000), 999);
        buffer[bytes] = '\0';
#endif
        std::string::size_type pos = std::string(buffer).find_last_of("\\/");
        return std::string(buffer).substr(0, pos);

    }

    bool FileSystem::copyFile(const std::string &source, const std::string &dest) {
        const size_t bufferSize = 8192;
        char buffer[bufferSize];
        size_t size;

        FILE* sourceFile = fopen(source.c_str(), "rb");
        FILE* destFile = fopen(dest.c_str(), "wb");

        if ((sourceFile == NULL) || (destFile == NULL))
            return false;

        while (size = fread(buffer, 1, bufferSize, sourceFile))
        {
            fwrite(buffer, 1, size, destFile);
        }

        fclose(sourceFile);
        fclose(destFile);

        return true;
    }

    bool FileSystem::isFile(const std::string &path) {
        struct stat st;
        if (!stat(path.c_str(), &st))
            return S_ISREG(st.st_mode);
        return false;
    }

    bool FileSystem::isDirectory(const std::string &path) {
        struct stat st;
        if (!stat(path.c_str(), &st))
            return S_ISDIR(st.st_mode);
        return false;
    }

    bool FileSystem::getFilesInDirectory(const std::string& path, std::vector<std::string> &res) {
#ifdef WIN32
        std::string p = path + "\\*";
        WIN32_FIND_DATA data;
        HANDLE hFind = FindFirstFile(p.c_str(), &data);
        if (hFind  != INVALID_HANDLE_VALUE)
        {
            do
            {
                res.push_back(data.cFileName);
            }
            while (FindNextFile(hFind, &data) != 0);
            FindClose(hFind);
            return true;
        }
        return false;
#else
        DIR* dir = opendir(path.c_str());
        if (dir != NULL)
        {
            struct dirent *dp;
            while ((dp = readdir(dir)) != NULL)
                res.push_back(dp->d_name);
            closedir(dir);
            return true;
        }
        return false;
#endif
    }

#ifdef WIN32
    const std::string FileSystem::fileDialog(
        int dialogType,
        const std::string &initialDir,
        const std::string &filterName,
        const std::string &filter) {
        std::string initDir = normalizePath(initialDir);
        std::replace(initDir.begin(), initDir.end(), '/', '\\');
        OPENFILENAME ofn;       // common dialog box structure
        char fileNameBuffer[512];
        fileNameBuffer[0] = '\0';

        const std::string filterWithEscape = filterName + " (*." + filter + ")" + '\0' + "*." + filter + '\0';

        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.lpstrFile = fileNameBuffer;
        ofn.nMaxFile = sizeof(fileNameBuffer);
        ofn.lpstrFilter = filterWithEscape.c_str();
        ofn.nFilterIndex = 1;
        ofn.lpstrInitialDir = (LPSTR)initDir.c_str();
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;

        if (dialogType == 0)
        {
            if (GetOpenFileName(&ofn))
                return std::string(fileNameBuffer);
        }
        else
        {
            if (GetSaveFileName(&ofn))
                return std::string(fileNameBuffer);
        }
        return "";
    }
#endif

}