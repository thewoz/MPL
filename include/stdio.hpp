/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017
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

#ifndef _H_MPL_STDIO_H_
#define _H_MPL_STDIO_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>

#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <filesystem>

#include <sys/dir.h>   // DIR
#include <libgen.h>    // basename
#include <fcntl.h>     // open
#include <unistd.h>    // write read close
#include <stdarg.h>    // va_list ...
#include <sys/stat.h>  // stat
#include <wordexp.h>
#include <pwd.h>       // getpwuid


//*****************************************************************************
// mpl::io
//*****************************************************************************
namespace mpl::io {
  
  //*****************************************************************************
  // util
  //*****************************************************************************
  namespace util {
    
    //*****************************************************************************
    // appendCwd
    //*****************************************************************************
    inline void appendCwd(const char * path, char * dst) {
      
      if(path[0]=='/') {
        strcpy(dst, path);
      } else if(path[0]=='.') {
        if(getcwd(dst, PATH_MAX)==NULL) {
          fprintf(stderr, "error getcwd(): (%d) %s\n", errno, strerror(errno));
          abort();
        }
        strcat(dst, "/");
        strcat(dst, path);
      } else if(path[0]=='~') {
        struct passwd * passwdEnt = getpwuid(getuid());
        strcpy(dst, passwdEnt->pw_dir);
        strcat(dst, &path[1]);
      } else {
        dst[0] = '/'; dst[0] = '\0';
        strcat(dst, path);
      }
      
    }
    
    //*****************************************************************************
    // removeJunk
    //*****************************************************************************
    inline void removeJunk(char * begin, char * end) {
      while(*end!=0) { *begin++ = *end++; }
      *begin = 0;
    }
    
    //*****************************************************************************
    // manualPathFold
    //*****************************************************************************
    inline char * manualPathFold(char * path) {
      
      char *s, *priorSlash;
      
      while ((s=strstr(path, "/../"))!=NULL) {
        *s = 0;
        if ((priorSlash = strrchr(path, '/'))==NULL) { /* oops */ *s = '/'; break; }
        removeJunk(priorSlash, s+3);
      }
      
      while ((s=strstr(path, "/./"))!=NULL) { removeJunk(s, s+2); }
      while ((s=strstr(path, "//"))!=NULL) { removeJunk(s, s+1); }
      
      s = path + (strlen(path)-1);
      
      if (s!=path && *s=='/') { *s=0; }
      
      return path;
      
    }
    
    //****************************************************************************
    // skipWhite
    //****************************************************************************
    inline const char * skipWhite(const std::string & line){
      
      for(uint32_t i=0; i<line.length(); ++i)
        if(!isspace(line[i])) return &line[i];
      
      return &line[line.length()-1];
      
    }
    
  } // end namespace util

  

  
  //****************************************************************************
  // isToSkip
  //****************************************************************************
  inline bool isToSkip(const std::string & line){
    
    const std::string & ptr = util::skipWhite(line);
    
    if(ptr.length() == 1 && std::isspace(line[0])) return true;
    
    if(ptr.length() < 1) return true;
    
    if(ptr[0] == '#') return true;
    
    return false;
    
  }
  
  //*****************************************************************************
  // expandPath
  //*****************************************************************************
  inline void expandPath(const char * srcPath, char * destPath) {
    
    char buff[PATH_MAX+1];
    
    wordexp_t p;
    
    if(wordexp(srcPath, &p, 0)==0){
      util::appendCwd(p.we_wordv[0], buff);
      wordfree(&p);
    } else {
      util::appendCwd(srcPath, buff);
    }
    
    if(realpath(buff, destPath)==NULL)
      strcpy(destPath, util::manualPathFold(buff));
    
  }
  
  //*****************************************************************************
  // expandPath
  //*****************************************************************************
  //void expandPath(char * path) { expandPath(path, path); }

   //*****************************************************************************
   // expandPath
   //*****************************************************************************
   inline void expandPath(char * path) {

     char buff[PATH_MAX+1];

     expandPath(path, buff);

     strcpy(path, buff);

   }
  
  //*****************************************************************************
  // expandPath
  //*****************************************************************************
  inline void expandPath(std::string & path) {

    char buff[PATH_MAX+1];

    expandPath(path.c_str(), buff); 

    path = buff;

  }
  
  //*****************************************************************************
  // expandPath
  //*****************************************************************************
  inline std::string expandPath(const std::string & path) {

    char buff[PATH_MAX+1];

    expandPath(path.c_str(), buff);

    return std::string(buff);
    
  }

  //*****************************************************************************
  // expandPath
  //*****************************************************************************
  inline void expandPath(const std::string & srcPath, std::string & destPath) {

    char buff[PATH_MAX+1];

    expandPath(srcPath.c_str(), buff); 

    destPath = buff;

   }
  
  //*****************************************************************************
  // expandPath
  //*****************************************************************************
  inline void expandPath(const std::string & srcPath, char * destPath) {

    expandPath(srcPath.c_str(), destPath);

   }

  //*****************************************************************************
  // getParts
  //*****************************************************************************
  inline std::vector<std::string> getParts(const std::filesystem::path& path) {
    
    std::vector<std::string> parts;
    
    for(const auto & part : path)
      parts.push_back(part.string());
    
    return parts;
    
  }

//
//
//namespace fs = std::filesystem;
//
//int main() {
//    fs::path p = "/AAA/BBB/CCC/DDD/EEE/FFF";
//
//    std::vector<fs::path> parts;
//    for (auto& part : p) {
//        parts.push_back(part);
//    }
//
//    if (parts.size() >= 2) {
//        std::cout << parts[parts.size() - 2].string() << "\n"; // EEE
//        std::cout << parts[parts.size() - 1].string() << "\n"; // FFF
//    }
//}


  //*****************************************************************************
  // basename
  //*****************************************************************************
  inline const std::string basename(const std::string & filename) {
    
    const char * p = strrchr(filename.c_str(), '/');
    
    if(p) return std::string(p + 1);
    else  return filename;
    
    return filename;
    
  }

  //*****************************************************************************
  // extension
  //*****************************************************************************
  inline const char * extension(const char * filename){
    
    const char * p = strrchr(filename, '.');
    
    return  p ? p + 1 : (char *) filename;
    
  }

  //*****************************************************************************
  // cp
  //*****************************************************************************
  inline void _cp(std::string srcPath, std::string dstPath) {
    
    // http://stackoverflow.com/questions/10195343/copy-a-file-in-a-sane-safe-and-efficient-way
    
    // Tested on 110 files 37.8Mb each
#if(0) // 179s
    
    expandPath(srcPath);
    expandPath(dstPath);
    
    char buf[BUFSIZ];
    
    size_t size;
    
    int src  = open(srcPath.c_str(), O_RDONLY, 0);
    int dest = open(dstPath.c_str(), O_WRONLY | O_CREAT /*| O_TRUNC*/, 0644);
    
    while((size = read(src, buf, BUFSIZ)) > 0) {
      if(write(dest, buf, size) < 0){
        fprintf(stderr, "error in copy '%s' to '%s': %s\n", srcPath.c_str(), dstPath.c_str(), strerror(errno));
        abort();
      }
    }
    
    close(src);
    close(dest);
#endif
   
#if(1) //42s
    expandPath(srcPath);
    expandPath(dstPath);
    
    std::ifstream src(srcPath, std::ios::binary);
    if(!src.good()){
      fprintf(stderr, "std::io::cp() error in open source file '%s': %s\n", srcPath.c_str(),  strerror(errno));
      abort();
    }
    
    std::ofstream dst(dstPath, std::ios::binary);
    if(!dst.good()){
      fprintf(stderr, "std::io::cp() error in open destination file '%s': %s\n", dstPath.c_str(),  strerror(errno));
      abort();
    }
    
    dst << src.rdbuf();
    
#endif
  
#if(0) //61s
    std::filesystem::copy_file(srcPath, dstPath, std::filesystem::copy_options::skip_existing);
#endif
    
  }


  //*****************************************************************************
  // openTempFile
  //*****************************************************************************
  FILE * openTempFile(std::string & outPath, const char * mode) {
    
    char path[] = "/tmp/tmpfileXXXXXX";
      
    // le XXXXXX vengono modificate a caso
    int fd = mkstemp(path);
      
    if(fd == -1) {
      perror("Errore nella creazione del file temporaneo");
      return nullptr;
    }

    outPath = path;  // Salva il percorso del file temporaneo
    
    FILE * file = fdopen(fd, "w");  // Converte il file descriptor in FILE*

    if(!file) {
      perror("Errore nell'apertura del file");
      close(fd);
    }

    return file;
    
  }

  //*****************************************************************************
  // openTempFile
  //*****************************************************************************
  FILE * openTempFile(const char * mode) {
    
    std::string outPath;

    return openTempFile(outPath, mode);
    
  }


  //*****************************************************************************
  // isDirectory
  //*****************************************************************************
  inline bool isDirectory(const std::string & path) {
    
    struct stat s;
     
    if(stat(path.c_str(),&s) == 0) {
      
      if(s.st_mode & S_IFDIR) return true;
      
    } else {
      fprintf(stderr, "cannot open the path '%s': %s\n", path.c_str(), strerror(errno));
      abort();
    }
    
    return false;
    
  }

  //*****************************************************************************
  // exists
  //*****************************************************************************
  inline bool exists(std::string path) {
    
    expandPath(path);
    
    return std::filesystem::exists(path);
    
  }

  //*****************************************************************************
  // isFile
  //*****************************************************************************
  inline bool isFile(const char * path){
    
    struct stat s;
    
    if(stat(path,&s) == 0) {
      
      if(s.st_mode & S_IFREG) return true;

    } else {
      fprintf(stderr, "cannot open the path '%s': %s\n", path, strerror(errno));
      abort();
    }
    
    return false;
    
  }

  //****************************************************************************/
  // cp
  //****************************************************************************/
  inline void cp(std::string inputFile, std::string outputFolder, std::string outputFileName) {
  
    // Mi creo la la path
    std::string ouputFile = outputFolder + "/" + outputFileName;
  
    // Copio il file di traiettorie
    _cp(inputFile, ouputFile);
  
  }

  //****************************************************************************/
  // cp
  //****************************************************************************/
  inline void cp(std::string inputFile, std::string outputFolder) {

    std::string ouputFile = outputFolder;

    ouputFile += "/" + basename(inputFile);

    // Copio il file di traiettorie
    _cp(inputFile, ouputFile);

  }

  //*****************************************************************************
  // openf
  //*****************************************************************************
  inline FILE * open(const char * filepath, const char * mode) {
    
    char absolutePath[PATH_MAX];
    
    expandPath(filepath, absolutePath);
        
    //printf("filepath %s | absolutePath %s\n", filepath, absolutePath);
        
    FILE * file = fopen(absolutePath, mode);
    
    if(file==NULL){
      fprintf(stderr, "error in opening file '%s': %s\n", absolutePath, strerror(errno));
      abort();
    }
    
    return file;

  }

  //*****************************************************************************
  // open
  //*****************************************************************************
  inline FILE * open(const std::string & filepath, const std::string & mode) {
    
    return open(filepath.c_str(), mode.c_str());
    
  }
  
  //*****************************************************************************
  // close
  //*****************************************************************************
  inline void close(FILE * file) {
    
    if(file != NULL){
      fclose(file);
    } else { /* fprintf(stderr, "no file to close\n");*/ }
    
  }
  
  //*****************************************************************************
  // dirname
  //*****************************************************************************
  inline std::string dirname(const std::string & filename) {
    
    size_t lastindex = filename.find_last_of("/");
    
    if(lastindex == std::string::npos) return "./";
    
    return filename.substr(0, lastindex+1);
    
  }
  
  //*****************************************************************************
  // basename
  //*****************************************************************************
  inline const char * basename(const char * filename) {
    
    const char * p = strrchr(filename, '/');
    
    return p ? p + 1 : (char *) filename;
    
  }
  
  //*****************************************************************************
  // extension
  //*****************************************************************************
  inline const std::string extension(const std::string & filename){
    
    const char * p = strrchr(filename.c_str(), '.');
    
    if(p) return std::string(p + 1);
    else  return filename;
    
    return filename;
    
  }
  
  //*****************************************************************************
  // name
  //*****************************************************************************
  inline const std::string name(const std::string & filename) {
    
    std::string str = basename(filename);
    
    size_t lastindex = str.find_last_of(".");
    
    return str.substr(0, lastindex);
    
  }
  
  //*****************************************************************************
  // subdirName
  //*****************************************************************************
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
  
  //*****************************************************************************
  // subdir
  //*****************************************************************************
  inline void subdir(const std::string & path, std::vector<std::string> & dirList){
    
    char dirPath[PATH_MAX] = {'\0', };
    
    if(path[0]=='~'){
      snprintf(dirPath, PATH_MAX, "%s%s", getenv("HOME"), &path[1]);
    }  else {
      strcpy(dirPath, path.c_str());
    }
    
    DIR * dir;
    
    if((dir = opendir(dirPath)) == NULL){
      fprintf(stderr, "cannot open the directory '%s': (%d) %s\n", dirPath, errno, strerror(errno));
      abort();
    }
    
    struct dirent * node;
    
    while((node = readdir(dir)) != NULL) {

      // Check whether it is a regular file or not.
      if((node->d_type == DT_DIR || node->d_type == DT_UNKNOWN) && node->d_name[0] != '.')
        dirList.push_back(node->d_name);
        
    }
    
    std::sort(dirList.begin(), dirList.end());
    closedir(dir);
  }
 

  //*****************************************************************************
  // getDirectoryPath
  //*****************************************************************************
  std::string getDirectoryPath(const std::string& filePath) {
    return std::filesystem::path(filePath).parent_path().string();
  }

  //*****************************************************************************
  // getFilePath
  //*****************************************************************************
  std::string getFilePath(FILE * file) {
    
      if(!file) return "";
      
      int fd = fileno(file);
      if (fd == -1) return "";

      char path[PATH_MAX];
      snprintf(path, sizeof(path), "/proc/self/fd/%d", fd);

      char realPath[PATH_MAX];
      ssize_t len = readlink(path, realPath, sizeof(realPath) - 1);
      if(len != -1) {
          realPath[len] = '\0';
          return std::string(realPath);
      }
      
      return "";
  }
  
  //*****************************************************************************
  // ls
  //*****************************************************************************
  inline void ls(const std::string & path, std::vector<std::string> & filesList, const std::string & fileExtension){
    
    char dirPath[PATH_MAX] = {'\0', };
    
    if(path[0]=='~'){
      snprintf(dirPath, PATH_MAX, "%s%s", getenv("HOME"), &path[1]);
    }  else {
      strcpy(dirPath, path.c_str());
    }
    
    DIR * dir;
    
    if((dir = opendir(dirPath)) == NULL){
      fprintf(stderr, "cannot open the directory '%s': (%d) %s\n", dirPath, errno, strerror(errno));
      abort();
    }
    
    struct dirent * node;
    
    char tmpStr[PATH_MAX];
    
    errno = ENOENT;
    
    while((node = readdir(dir)) != NULL) {
      
      // Check whether it is a regular file or not.
      if(node->d_type != DT_REG && node->d_type != DT_LNK && node->d_type != DT_UNKNOWN)
        continue;

      // Filter the name by extention
#if defined(__APPLE__) || defined(MACOSX)
      if(node->d_namlen == 0 || (fileExtension[0]!='*' && strcmp(extension(node->d_name), fileExtension.c_str()) != 0))
        continue;
#else
      if(strlen(node->d_name) == 0 || (fileExtension[0]!='*' && strcmp(extension(node->d_name), fileExtension.c_str()) != 0))
        continue;
#endif

      // Skip name starting with the dot
      if(node->d_name[0] == '.') continue;
      
      snprintf(tmpStr, PATH_MAX, "%s/%s", dirPath, node->d_name);

      filesList.push_back(tmpStr);
      
      errno = ENOENT;

    }    

    std::sort(filesList.begin(), filesList.end());

    closedir(dir);

  }
  
  //*****************************************************************************
  // mkdir
  //*****************************************************************************
  inline int dirmk(const char * format, ...){
    
    char path[PATH_MAX];
    
    va_list ap;
    
    va_start(ap, format);
    
    vsnprintf(path, PATH_MAX, format, ap);
    
    va_end(ap);
    
    struct stat sb;
    
    expandPath(path, path);
    
    char *p, *npath;
    
    int mode = S_IEXEC | S_IREAD | S_IRGRP | S_IROTH | S_IWRITE | S_IXOTH | S_IXGRP;
    
    if(stat(path, &sb) == 0){
      
      if(S_ISDIR(sb.st_mode) == 0){
        fprintf(stderr, "'%s': file exists but is not a directory\n", path);
        return -1;
      }
      
      if(chmod(path, mode)){
        fprintf(stderr, "chmod() %s\n(%d) %s\n", path, errno, strerror(errno));
        return -1;
      }
      
      return 0;
      
    }
    
    char * tmpstr = (char *) malloc(1 + strlen(path));
    
    if(tmpstr == NULL){
      fprintf (stderr, "malloc: out of virtual memory\n");
      return -1;
    }
    
    npath = (char *)strcpy(tmpstr, (path));    /* So we can write to it. */
    
    /* Check whether or not we need to do anything with intermediate dirs. */
    
    /* Skip leading slashes. */
    p = npath;
    
    while(*p == '/')
      p++;
    
    while((p = strchr (p, '/'))){
      
      *p = '\0';
      
      if(stat(npath, &sb) != 0){
        
        int err = mkdir(npath, mode);
        
        if(err && errno != EEXIST){
          fprintf(stderr, "cannot create directory '%s': (%d) %s\n", npath, errno, strerror(errno));
          free(npath);
          return -1;
        }
        
      } else if(S_ISDIR(sb.st_mode) == 0) {
        fprintf(stderr, "'%s': file exists but is not a directory\n", npath);
        free(npath);
        return -1;
      }
      
      *p++ = '/';   /* restore slash */
      
      while(*p == '/')
        p++;
      
    }
    
    /* Create the final directory component. */
    
    int err = mkdir(npath, mode);
    
    if(stat(npath, &sb) && err && errno != EEXIST) {
      fprintf(stderr, "cannot create directory '%s': (%d) %s\n", npath, errno, strerror(errno));
      free(npath);
      return err;
    }
    
    free(npath);
    
    return 0;
    
  }
  
  inline int dirmk(const std::string & path){ return dirmk(path.c_str()); }

//*****************************************************************************
// areFilesEqual
//*****************************************************************************
bool areFilesEqual(const std::string & file1, const std::string & file2) {
  std::string command = "cmp -s \"" + file1 + "\" \"" + file2 + "\"";
  return std::system(command.c_str()) == 0;
}

} /* namespace mpl::io */


#endif /* _H_MPL_STDIO_H_ */
