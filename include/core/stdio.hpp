/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017-2026
 * Created by Leonardo Parisi (leonardo.parisi[at]gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef _H_MPL_IO_STDIO_H_
#define _H_MPL_IO_STDIO_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>

#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <filesystem>
#include <iostream>

#include <sys/dir.h>   // DIR
#include <libgen.h>    // basename
#include <fcntl.h>     // open
#include <unistd.h>    // write read close
#include <stdarg.h>    // va_list ...
#include <sys/stat.h>  // stat
#include <wordexp.h>
#include <pwd.h>       // getpwuid


//*****************************************************************************/
// mpl::io
//*****************************************************************************/
namespace mpl::io {
  
  //*****************************************************************************/
  // util
  //*****************************************************************************/
  namespace util {
    
    //*****************************************************************************/
    // resolveBasicPath
    //*****************************************************************************/
    inline void resolveBasicPath(const char * srcPath, char * dst) {

      // Caso 1: path assoluto
      if(srcPath[0] == '/') {
        strcpy(dst, srcPath);
        return;
      }

      // Caso 2: home (~)
      if(srcPath[0] == '~') {
          
        struct passwd * pw = getpwuid(getuid());
          
        if(!pw) abort(); // caso estremamente improbabile

        strcpy(dst, pw->pw_dir);

          // "~qualcosa" non è standard: gestiamo solo "~" o "~/..."
        if(srcPath[1] == '/')
          strcat(dst, srcPath + 1); // aggiunge "/..."

        return;
          
      }

      // Caso 3: path relativo
      if(getcwd(dst, PATH_MAX) == nullptr) abort();

      strcat(dst, "/");
      strcat(dst, srcPath);
      
    }
  
    //*****************************************************************************/
    // removeJunk
    //*****************************************************************************/
    inline void removeJunk(char * begin, char * end) {
      while(*end!=0) { *begin++ = *end++; }
      *begin = 0;
    }
  
    //*****************************************************************************/
    // manualPathFold
    //*****************************************************************************/
    inline char * manualPathFold(char * path) {
       
      char *s, *priorSlash;
       
      while((s=strstr(path, "/../"))!=NULL) {
        *s = 0;
        if((priorSlash = strrchr(path, '/'))==NULL) { /* oops */ *s = '/'; break; }
        removeJunk(priorSlash, s+3);
      }
       
      while((s=strstr(path, "/./"))!=NULL) { removeJunk(s, s+2); }
      while((s=strstr(path, "//"))!=NULL)  { removeJunk(s, s+1); }
       
      s = path + (strlen(path)-1);
       
      if(s!=path && *s=='/') { *s=0; }
       
      return path;
       
    }
    
    //****************************************************************************/
    // skipWhite
    //****************************************************************************/
    inline const char * skipWhite(const std::string & line){
      
      for(size_t i=0; i<line.length(); ++i)
        if(!isspace(line[i])) return &line[i];
      
      return &line[line.length()-1];
      
    }
    
  } // end namespace util
  

  //****************************************************************************/
  // remove()
  //****************************************************************************/
  inline bool remove(const std::string & path) {
  
    namespace fs = std::filesystem;

    std::error_code ec;

    if(!fs::exists(path, ec)) {
      fprintf(stderr, "mpl::io::remove() error: the path '%s' does not exist\n", path.c_str());
      return false;
    }

    if(fs::is_directory(path, ec)) {
      // Rimuove la directory e tutto il suo contenuto
      fs::remove_all(path, ec);
    } else {
      // Rimuove un singolo file
      fs::remove(path, ec);
    }

    if(ec) {
      fprintf(stderr, "mpl::io::remove() error: cannot remove '%s': %s\n", path.c_str(), ec.message().c_str());
      return false;
    }

    return true;
  
  }

  //****************************************************************************/
  // isToSkip
  //****************************************************************************/
  inline bool isToSkip(const std::string & line){

    const std::string & ptr = util::skipWhite(line);

    if(ptr.length() == 1 && std::isspace(line[0])) return true;

    if(ptr.length() < 1) return true;

    if(ptr[0] == '#') return true;

    return false;

  }

  //****************************************************************************/
  // isToSkip (overload su C-string, senza allocazioni)
  //****************************************************************************/
  inline bool isToSkip(const char * line){

    // salto gli spazi iniziali
    while(*line != '\0' && std::isspace((unsigned char)*line)) ++line;

    // riga vuota o composta solo da spazi
    if(*line == '\0') return true;

    // riga di commento
    if(*line == '#') return true;

    return false;

  }
  
  //*****************************************************************************/
  // expandPath
  //*****************************************************************************/
  inline void expandPath(const char * srcPath, char * destPath) {
    
    char buff[PATH_MAX+1];

    // Risolve ~, path relativi e path assoluti
    util::resolveBasicPath(srcPath, buff);

    // Normalizza: realpath se possibile, altrimenti manualPathFold
    if(realpath(buff, destPath) == nullptr)
      strcpy(destPath, util::manualPathFold(buff));
    
  }

  //*****************************************************************************/
  // expandPath
  //*****************************************************************************/
  inline void expandPath(char * path) {

    char buff[PATH_MAX+1];

    expandPath(path, buff);
     
    strcpy(path, buff);

  }
  
  //*****************************************************************************/
  // expandPath
  //*****************************************************************************/
  inline void expandPath(std::string & path) {

    char buff[PATH_MAX+1];

    expandPath(path.c_str(), buff); 

    path = buff;

  }
  
  //*****************************************************************************/
  // expandPath
  //*****************************************************************************/
  inline std::string expandPath(const std::string & path) {

    char buff[PATH_MAX+1];

    expandPath(path.c_str(), buff);

    return std::string(buff);
    
  }

  //*****************************************************************************/
  // expandPath
  //*****************************************************************************/
  inline void expandPath(const std::string & srcPath, std::string & destPath) {

    char buff[PATH_MAX+1];

    expandPath(srcPath.c_str(), buff); 

    destPath = buff;

   }
  
  //*****************************************************************************/
  // expandPath
  //*****************************************************************************/
  inline void expandPath(const std::string & srcPath, char * destPath) {

    expandPath(srcPath.c_str(), destPath);

   }

  //*****************************************************************************/
  // getParts
  //*****************************************************************************/
  inline std::vector<std::string> getParts(const std::filesystem::path& path) {
    
    std::vector<std::string> parts;
    
    for(const auto & part : path)
      parts.push_back(part.string());
    
    return parts;
    
  }



  //*****************************************************************************/
  // basename
  //*****************************************************************************/
  inline std::string basename(const std::string & filename) {

    return std::filesystem::path(filename).filename().string();

  }

  //*****************************************************************************/
  // cp
  //*****************************************************************************/
  inline void _cp(std::string srcPath, std::string dstPath) {

    // http://stackoverflow.com/questions/10195343/copy-a-file-in-a-sane-safe-and-efficient-way
    // Fastest of the variants benchmarked on 110 files of 37.8Mb each (~42s).

    expandPath(srcPath);
    expandPath(dstPath);

    std::ifstream src(srcPath, std::ios::binary);
    if(!src.good()){
      fprintf(stderr, "mpl::io::cp() error: cannot open source file '%s': %s\n", srcPath.c_str(),  strerror(errno));
      abort();
    }

    std::ofstream dst(dstPath, std::ios::binary);
    if(!dst.good()){
      fprintf(stderr, "mpl::io::cp() error: cannot open destination file '%s': %s\n", dstPath.c_str(),  strerror(errno));
      abort();
    }

    dst << src.rdbuf();

  }


  //*****************************************************************************/
  // openTempFile
  //*****************************************************************************/
  inline FILE * openTempFile(std::string & outPath, const char * mode) {
    
    char path[] = "/tmp/tmpfileXXXXXX";
      
    // le XXXXXX vengono modificate a caso
    int fd = mkstemp(path);
      
    if(fd == -1) {
      perror("mpl::io::openTempFile() error: cannot create the temporary file");
      return nullptr;
    }

    outPath = path;  // Salva il percorso del file temporaneo
    
    FILE * file = fdopen(fd, "w");  // Converte il file descriptor in FILE*

    if(!file) {
      perror("mpl::io::openTempFile() error: cannot open the temporary file");
      close(fd);
    }

    return file;
    
  }

  //*****************************************************************************/
  // openTempFile
  //*****************************************************************************/
  inline FILE * openTempFile(const char * mode) {
    
    std::string outPath;

    return openTempFile(outPath, mode);
    
  }


  //*****************************************************************************/
  // isDirectory
  //*****************************************************************************/
  inline bool isDirectory(const std::string & path) {

    std::error_code ec;

    bool result = std::filesystem::is_directory(path, ec);

    if(ec) {
      fprintf(stderr, "mpl::io::isDirectory() error: cannot stat the path '%s': %s\n", path.c_str(), ec.message().c_str());
      abort();
    }

    return result;

  }

  //*****************************************************************************/
  // exists
  //*****************************************************************************/
  inline bool exists(std::string path) {
    
    expandPath(path);
    
    return std::filesystem::exists(path);
    
  }

  //*****************************************************************************/
  // isFile
  //*****************************************************************************/
  inline bool isFile(const std::string & path){

    std::error_code ec;

    bool result = std::filesystem::is_regular_file(path, ec);

    if(ec) {
      fprintf(stderr, "mpl::io::isFile() error: cannot stat the path '%s': %s\n", path.c_str(), ec.message().c_str());
      abort();
    }

    return result;

  }

  //****************************************************************************/
  // cp
  //****************************************************************************/
  inline void cp(const std::string & inputFile, const std::string & outputFolder, const std::string & outputFileName) {

    // Mi creo la la path
    std::string ouputFile = outputFolder + "/" + outputFileName;

    // Copio il file di traiettorie
    _cp(inputFile, ouputFile);

  }

  //****************************************************************************/
  // cp
  //****************************************************************************/
  inline void cp(const std::string & inputFile, const std::string & outputFolder) {

    std::string ouputFile = outputFolder + "/" + basename(inputFile);

    // Copio il file di traiettorie
    _cp(inputFile, ouputFile);

  }

  //*****************************************************************************/
  // openf
  //*****************************************************************************/
  inline FILE * open(const char * filepath, const char * mode) {
    
    char absolutePath[PATH_MAX];
    
    expandPath(filepath, absolutePath);
        
    //printf("filepath %s | absolutePath %s\n", filepath, absolutePath);
        
    FILE * file = fopen(absolutePath, mode);
    
    if(file==NULL) {
      fprintf(stderr, "mpl::io::open() error: cannot open file '%s': %s\n", absolutePath, strerror(errno));
      abort();
    }
    
    return file;

  }

  //*****************************************************************************/
  // open
  //*****************************************************************************/
  inline FILE * open(const std::string & filepath, const std::string & mode) {
    
    return open(filepath.c_str(), mode.c_str());
    
  }
  
  //*****************************************************************************/
  // close
  //*****************************************************************************/
  inline void close(FILE * file) {
    
    if(file != NULL) {
      fclose(file);
    } else { /* fprintf(stderr, "no file to close\n");*/ }
    
  }
  
  //*****************************************************************************/
  // dirname
  //*****************************************************************************/
  inline std::string dirname(const std::string & filename) {

    std::string parent = std::filesystem::path(filename).parent_path().string();

    // No directory component: fall back to the current directory.
    return parent.empty() ? "." : parent;

  }
  
  //*****************************************************************************/
  // extension
  //*****************************************************************************/
  inline std::string extension(const std::string & filename){

    // std::filesystem returns the extension including the leading dot
    // (or empty if there is none): strip the dot to keep the bare suffix.
    std::string ext = std::filesystem::path(filename).extension().string();

    if(!ext.empty() && ext[0] == '.') ext.erase(0, 1);

    return ext;

  }
  
  //*****************************************************************************/
  // name
  //*****************************************************************************/
  inline const std::string name(const std::string & filename) {
    
    std::string str = basename(filename);
    
    size_t lastindex = str.find_last_of(".");
    
    return str.substr(0, lastindex);
    
  }
  
  //*****************************************************************************/
  // subdirName
  //*****************************************************************************/
  inline const std::string subdirName(const std::string & filePath) {
    
    if(filePath.empty() == false) {
      
      size_t toPos = filePath.find_last_of('/') - 1;
      
      if(toPos != std::string::npos) {
        
        size_t fromPos = filePath.find_last_of('/', toPos);
        
        if(fromPos != std::string::npos) {
          return filePath.substr(fromPos + 1, toPos - fromPos);
        }
        
      }
      
    }
    
    return "";
    
  }
  
  //*****************************************************************************/
  // subdir
  //*****************************************************************************/
  inline void subdir(const std::string & path, std::vector<std::string> & dirList){

    namespace fs = std::filesystem;

    std::string dirPath = expandPath(path);

    std::error_code ec;

    fs::directory_iterator it(dirPath, ec), end;

    if(ec){
      fprintf(stderr, "mpl::io::subdir() error: cannot open the directory '%s': %s\n", dirPath.c_str(), ec.message().c_str());
      abort();
    }

    for(; it != end; it.increment(ec)) {

      const fs::directory_entry & node = *it;

      const std::string name = node.path().filename().string();

      // Keep only sub-directories whose name does not start with a dot.
      if(name[0] != '.' && node.is_directory(ec))
        dirList.push_back(name);

    }

    std::sort(dirList.begin(), dirList.end());

  }
 

  //*****************************************************************************/
  // getFilePath
  //*****************************************************************************/
  inline std::string getFilePath(FILE * file) {

      if(!file) return "";

      int fd = fileno(file);
      if(fd == -1) return "";

#if defined(__APPLE__) || defined(MACOSX)
      // macOS: no /proc, use fcntl(F_GETPATH)
      char realPath[PATH_MAX];
      if(fcntl(fd, F_GETPATH, realPath) != -1)
        return std::string(realPath);
#else
      // Linux: resolve the /proc/self/fd symlink
      char path[PATH_MAX];
      snprintf(path, sizeof(path), "/proc/self/fd/%d", fd);

      char realPath[PATH_MAX];
      ssize_t len = readlink(path, realPath, sizeof(realPath) - 1);
      if(len != -1) {
          realPath[len] = '\0';
          return std::string(realPath);
      }
#endif

      return "";
  }
  
  //*****************************************************************************/
  // ls
  //*****************************************************************************/
  inline void ls(const std::string & path, std::vector<std::string> & filesList, const std::string & fileExtension){

    namespace fs = std::filesystem;

    std::string dirPath = expandPath(path);

    std::error_code ec;

    fs::directory_iterator it(dirPath, ec), end;

    if(ec){
      fprintf(stderr, "mpl::io::ls() error: cannot open the directory '%s': %s\n", dirPath.c_str(), ec.message().c_str());
      abort();
    }

    for(; it != end; it.increment(ec)) {

      const fs::directory_entry & node = *it;

      const std::string name = node.path().filename().string();

      // Skip names starting with a dot.
      if(name.empty() || name[0] == '.') continue;

      // Keep only regular files (symlinks to regular files included).
      if(!node.is_regular_file(ec)) continue;

      // Filter by extension (unless the filter is '*').
      if(fileExtension[0] != '*' && extension(name) != fileExtension) continue;

      filesList.push_back(dirPath + "/" + name);

    }

    std::sort(filesList.begin(), filesList.end());

  }
  
  //*****************************************************************************/
  // mkdir
  //*****************************************************************************/
  inline int dirmk(const char * format, ...){

    namespace fs = std::filesystem;

    char path[PATH_MAX];

    va_list ap;

    va_start(ap, format);

    vsnprintf(path, PATH_MAX, format, ap);

    va_end(ap);

    expandPath(path, path);

    std::error_code ec;

    // If the path already exists it must be a directory.
    if(fs::exists(path, ec)) {

      if(!fs::is_directory(path, ec)) {
        fprintf(stderr, "mpl::io::dirmk() error: '%s' exists but is not a directory\n", path);
        return -1;
      }

    } else {

      // Create the directory and all the missing intermediate ones.
      fs::create_directories(path, ec);

      if(ec) {
        fprintf(stderr, "mpl::io::dirmk() error: cannot create directory '%s': %s\n", path, ec.message().c_str());
        return -1;
      }

    }

    // Enforce the 0755 permissions (owner rwx, group/others rx).
    fs::perms mode = fs::perms::owner_all   |
                     fs::perms::group_read  | fs::perms::group_exec |
                     fs::perms::others_read | fs::perms::others_exec;

    fs::permissions(path, mode, ec);

    if(ec) {
      fprintf(stderr, "mpl::io::dirmk() error: chmod('%s') failed: %s\n", path, ec.message().c_str());
      return -1;
    }

    return 0;

  }
  
  inline int dirmk(const std::string & path){ return dirmk(path.c_str()); }

//*****************************************************************************/
// areFilesEqual
//*****************************************************************************/
inline bool areFilesEqual(const std::string & file1, const std::string & file2) {

  std::ifstream f1(file1, std::ios::binary | std::ios::ate);
  std::ifstream f2(file2, std::ios::binary | std::ios::ate);

  if(!f1.good() || !f2.good()) return false;

  // Different sizes cannot be equal.
  if(f1.tellg() != f2.tellg()) return false;

  f1.seekg(0); f2.seekg(0);

  return std::equal(std::istreambuf_iterator<char>(f1), std::istreambuf_iterator<char>(),
                    std::istreambuf_iterator<char>(f2));

}

} // namespace mpl::io


#endif // _H_MPL_IO_STDIO_H_
