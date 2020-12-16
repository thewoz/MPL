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

#include <sys/dir.h>   // DIR
#include <libgen.h>    // basename
#include <fcntl.h>     // open
#include <unistd.h>    // write read close
#include <stdarg.h>    // va_list ...
#include <sys/stat.h>  // stat
#include <wordexp.h>
#include <pwd.h>       // getpwuid


/*****************************************************************************/
// mpl::io
/*****************************************************************************/
namespace mpl::io {
  
  /*****************************************************************************/
  // util
  /*****************************************************************************/
  namespace util {
    
    /*****************************************************************************/
    // appendCwd
    /*****************************************************************************/
    inline void appendCwd(const char * path, char * dst) {
      
      if(path[0]=='/'){
        strcpy(dst, path);
      } else if(path[0]=='.'){
        if(getcwd(dst, PATH_MAX)==NULL){
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
    
    /*****************************************************************************/
    // removeJunk
    /*****************************************************************************/
    inline void removeJunk(char * begin, char * end) {
      while(*end!=0) { *begin++ = *end++; }
      *begin = 0;
    }
    
    /*****************************************************************************/
    // manualPathFold
    /*****************************************************************************/
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
    
  } // end namespace util

  /*****************************************************************************/
  // expandPath
  /*****************************************************************************/
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
  
  /*****************************************************************************/
  // expandPath
  /*****************************************************************************/
  //void expandPath(char * path) { expandPath(path, path); }

  /*****************************************************************************/
   // expandPath
   /*****************************************************************************/
   inline void expandPath(char * path) {

     char buff[PATH_MAX+1];

     expandPath(path, buff);

     strcpy(path, buff);

   }
  
  /*****************************************************************************/
  // expandPath
  /*****************************************************************************/
  inline void expandPath(std::string & path) {

    char buff[PATH_MAX+1];

    expandPath(path.c_str(), buff); 

    path = buff;

  }
  
  /*****************************************************************************/
  // expandPath
  /*****************************************************************************/
  inline void expandPath(const std::string & srcPath, std::string & destPath) {

    char buff[PATH_MAX+1];

    expandPath(srcPath.c_str(), buff); 

    destPath = buff;

   }
  
  /*****************************************************************************/
  // expandPath
  /*****************************************************************************/
  inline void expandPath(const std::string & srcPath, char * destPath) {

    expandPath(srcPath.c_str(), destPath);

   }

  /*****************************************************************************/
  // cp
  /*****************************************************************************/
  inline void cp(const std::string & _srcPath, const std::string & _dstPath, size_t BUFFER_SIZE = BUFSIZ) {
    
    // http://stackoverflow.com/questions/10195343/copy-a-file-in-a-sane-safe-and-efficient-way
    
    char srcPath[PATH_MAX];
    char dstPath[PATH_MAX];
    
    expandPath(_srcPath, srcPath);
    expandPath(_dstPath, dstPath);
    
    char buf[BUFFER_SIZE];
    
    size_t size;
    
    int src  = open(srcPath, O_RDONLY, 0);
    int dest = open(dstPath, O_WRONLY | O_CREAT /*| O_TRUNC*/, 0644);
    
    while((size = read(src, buf, BUFFER_SIZE)) > 0) {
      if(write(dest, buf, size) < 0){
        fprintf(stderr, "error in copy '%s' to '%s': %s\n", srcPath, dstPath, strerror(errno));
        abort();
      }
    }
    
    close(src);
    close(dest);
    
  }
  
  /*****************************************************************************/
  // openf
  /*****************************************************************************/
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
  
  /*****************************************************************************/
  // openf
  /*****************************************************************************/
  inline FILE * open(const std::string & filepath, const std::string & mode) {
    
    return open(filepath.c_str(), mode.c_str());
    
  }
  
  /*****************************************************************************/
  // close
  /*****************************************************************************/
  inline void close(FILE * file) {
    
    if(file != NULL){
      fclose(file);
    } else { /* fprintf(stderr, "no file to close\n");*/ }
    
    
  }
  
  /*****************************************************************************/
  // dirname
  /*****************************************************************************/
  inline std::string dirname(const std::string & filename) {
    
    size_t lastindex = filename.find_last_of("/");
    
    if(lastindex == std::string::npos) return "./";
    
    return filename.substr(0, lastindex);
    
  }
  
  /*****************************************************************************/
  // basename
  /*****************************************************************************/
  inline const char * basename(const char * filename) {
    
    const char * p = strrchr(filename, '/');
    
    return p ? p + 1 : (char *) filename;
    
  }
  
  
  /*****************************************************************************/
  // basename
  /*****************************************************************************/
  inline const std::string basename(const std::string & filename) {
    
    const char * p = strrchr(filename.c_str(), '/');
    
    if(p) return std::string(p + 1);
    else  return filename;
    
    return filename;
    
  }
  
  /*****************************************************************************/
  // extension
  /*****************************************************************************/
  inline const char * extension(const char * filename){
    
    const char * p = strrchr(filename, '.');
    
    return  p ? p + 1 : (char *) filename;
    
  }
  
  /*****************************************************************************/
  // extension
  /*****************************************************************************/
  inline const std::string extension(const std::string & filename){
    
    const char * p = strrchr(filename.c_str(), '.');
    
    if(p) return std::string(p + 1);
    else  return filename;
    
    return filename;
    
  }
  
  /*****************************************************************************/
  // name
  /*****************************************************************************/
  inline const std::string name(const std::string & filename) {
    
    std::string str = basename(filename);
    
    size_t lastindex = str.find_last_of(".");
    
    return str.substr(0, lastindex);
    
  }
  
  
  /*****************************************************************************/
  // subdirName
  /*****************************************************************************/
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
  
  
  
  
  /*****************************************************************************/
  // isDirectory
  /*****************************************************************************/
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
  
  
  /*****************************************************************************/
  // isFile
  /*****************************************************************************/
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
  
  /*****************************************************************************/
  // subdir
  /*****************************************************************************/
  inline void subdir(const std::string & path, std::vector<std::string> & dirList){
    
    char dirPath[PATH_MAX] = {'\0', };
    
    if(path[0]=='~'){
      sprintf(dirPath, "%s%s", getenv("HOME"), &path[1]);
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
      if(node->d_type == DT_DIR && node->d_name[0] != '.')
        dirList.push_back(node->d_name);
      
    }
    
    std::sort(dirList.begin(), dirList.end());
    
    closedir(dir);
    
  }
  
  
  /*****************************************************************************/
  // ls
  /*****************************************************************************/
  inline void ls(const std::string & path, std::vector<std::string> & filesList, const std::string & fileExtension){
    
    char dirPath[PATH_MAX] = {'\0', };
    
    if(path[0]=='~'){
      sprintf(dirPath, "%s%s", getenv("HOME"), &path[1]);
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

      // Filter the name
#if defined(__APPLE__) || defined(MACOSX)
      if(node->d_namlen == 0 || (fileExtension[0]!='*' && strcmp(extension(node->d_name), fileExtension.c_str()) != 0))
        continue;
#else
      if(strlen(node->d_name) == 0 || (fileExtension[0]!='*' && strcmp(extension(node->d_name), fileExtension.c_str()) != 0))
        continue;
#endif

      sprintf(tmpStr, "%s/%s", dirPath, node->d_name);

      filesList.push_back(tmpStr);
      
      errno = ENOENT;

    }    

    std::sort(filesList.begin(), filesList.end());
    
    closedir(dir);

  }
  
  
  /*****************************************************************************/
  // mkdir
  /*****************************************************************************/
  inline int dirmk(const char * format, ...){
    
    char path[PATH_MAX];
    
    va_list ap;
    
    va_start(ap, format);
    
    vsprintf(path, format, ap);
    
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
        fprintf(stderr, "%s: (%d) %s\n", path, errno, strerror(errno));
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

  
} /* namespace mpl::io */


#endif /* _H_MPL_STDIO_H_ */
