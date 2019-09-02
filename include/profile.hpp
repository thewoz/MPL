/*
 * MIT License
 *
 * Copyright © 2019
 * Created by Leonardo Parisi (leonardo.parisi[at]gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _H_MPL_PROFILE_H_
#define _H_MPL_PROFILE_H_

#include <cstdlib>
#include <cstdio>

#include <cerrno>

#include <cstdarg> // va_list ...

#include <climits>
#include <cstring>

#include <ctime>

#include <sys/time.h>
#include <sys/resource.h>

#include <vector>

/*****************************************************************************/
// namespace
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // profile
  /*****************************************************************************/
  class profile {
    
    private:
    
      struct data_t {
        
        timeval tv1;
        
        char str[PATH_MAX];
        
      };
    
      static std::vector<data_t> timer;
    
      static bool onScreenTime;
    
      static FILE * logTimeFile;
      static FILE * logMemoryFile;
    
      static char str[PATH_MAX];
    
      static rusage usageData;
    
      profile() {  }
    
    
    public:
    
      enum { ON, OFF };
    
      //***************************************************************************************************//
      // close
      //***************************************************************************************************//
      static void close() {
        
        if(logTimeFile != NULL){
          fflush(logTimeFile);
          fclose(logTimeFile);
        }
        
        if(logMemoryFile != NULL){
          fflush(logMemoryFile);
          fclose(logMemoryFile);
        }
        
      }
    
    
      //***************************************************************************************************//
      // setOutputOnScreen
      //***************************************************************************************************//
      static inline void setOutputOnScreen(bool mode) {
        
        onScreenTime = mode;
        
      }
    
    
      //***************************************************************************************************//
      // initTime
      //***************************************************************************************************//
      static void initTime(const char * format = NULL, ...) {
        
        if(format != NULL) {
          
          va_list ap;
          
          va_start(ap, format);
          
          vsprintf(str, format, ap);
          
          va_end(ap);
          
          logTimeFile = fopen(str, "w");
          
          if(logTimeFile == NULL) {
            fprintf(stderr, "Error, something went wrong with in fopen(%s): %s\n", str, strerror(errno));
            exit(EXIT_FAILURE);
          }
          
        }
        
        //onScreenTime = mode;
        
      }
    
      //***************************************************************************************************//
      // initMemory
      //***************************************************************************************************//
      static void initMemory(const char * format = NULL, ... ) {
        
        if(format != NULL) {
          
          va_list ap;
          
          va_start(ap, format);
          
          vsprintf(str, format, ap);
          
          va_end(ap);
          
          logMemoryFile = fopen(str, "w");
          
          if(logMemoryFile == NULL) {
            fprintf(stderr, "Error, something went wrong with in fopen(%s): %s\n", str, strerror(errno));
            exit(EXIT_FAILURE);
          }
          
        }
        
      }
    
    
      //***************************************************************************************************//
      // start
      //***************************************************************************************************//
      static void start(const char *format, ... ) {
        
        data_t tmpProfileTime;
        
        va_list ap;
        
        va_start(ap, format);
        
        vsprintf(tmpProfileTime.str, format, ap);
        
        va_end(ap);
        
        //printf("AAA %s\n", tmpProfileTime.str);
        
        gettimeofday(&tmpProfileTime.tv1, NULL);
        
        timer.push_back(tmpProfileTime);
        
      }
    
    
      //***************************************************************************************************//
      // enlapse
      //***************************************************************************************************//
      static double enlapse() {
        
        timeval tv2;
        
        gettimeofday(&tv2, NULL);
        
        timeval tv1 = timer.back().tv1;
        
        return (((double)(tv2.tv_usec - tv1.tv_usec) / 1000000.0L) + ((double) (tv2.tv_sec - tv1.tv_sec)));
        
      }
    
    
      //***************************************************************************************************//
      // done
      //***************************************************************************************************//
      static double done() {
        
        timeval tv2;
        
        gettimeofday(&tv2, NULL);
        
        timeval tv1 = timer.back().tv1;
        
        double sec = (((double)(tv2.tv_usec - tv1.tv_usec) / 1000000.0L) + ((double) (tv2.tv_sec - tv1.tv_sec)));
        
        if(onScreenTime == ON) {
          fprintf(stdout, "%s done in %.03f sec\n", timer.back().str, sec);
          fflush(stdout);
        }
        
        if(logTimeFile != NULL) {
          fprintf(logTimeFile, "%s done in %.08f sec\n", timer.back().str, sec);
          fflush(logTimeFile);
        }
        
        timer.pop_back();
        
        return sec;
        
      }
    
    
      //***************************************************************************************************//
      // memoryPeakGB
      //***************************************************************************************************//
      static double memoryPeakGB() {
        
  #if defined(__APPLE__)
        
        return -1;
        
  #endif
        
        FILE* file = fopen("/proc/self/status", "r");
        
        if(file == NULL) {
          fprintf(stderr, "Error, something went wrong with in fopen(/proc/self/status): %s\n", strerror(errno));
          exit(EXIT_FAILURE);
        }
        
        char line[PATH_MAX];
        
        uint32_t memoryKB;
        
        while(fgets(line, 128, file) != NULL){
          
          char * pch = strstr(line, "VmHWM");
          
          if(pch != NULL){
            
            char dummy1[PATH_MAX]; char dummy2[PATH_MAX];
            
            sscanf(line, "%s %d %s", dummy1, &memoryKB, dummy2);
            
            break;
            
          }
          
        }
        
        fclose(file);
        
        return ( memoryKB / 1000.0 ) / 1000.0;
        
        /*
         struct rusage rusage;
         
         getrusage(RUSAGE_SELF, &rusage);
         
         #if defined(__APPLE__)
         (size_t)rusage.ru_maxrss;
         #else
         (size_t)(rusage.ru_maxrss * 1024L);
         #endif
         */
      }
    
    
    
      //***************************************************************************************************//
      // logMemory
      //***************************************************************************************************//
      static void logMemory(const char *format, ... ) {
        
        getrusage(RUSAGE_SELF, &usageData);
        
        va_list ap;
        
        va_start(ap, format);
        
        vsprintf(str, format, ap);
        
        va_end(ap);
        
        size_t peak, current;
        
        getMemUsage(current, peak);
        
        /*
         if(onScreenMemory) {
         #ifdef __APPLE__
         fprintf(stdout, "%s %.02fMB VmHWM %.02fMB VmRSS %.02fMB\n\n", str, (usageData.ru_maxrss/1024.0)/1024.0, peak/1024.0, current/1024.0);
         #else
         fprintf(stdout, "%s %.02fMB VmHWM %.02fMB VmRSS %.02fMB\n\n", str, (usageData.ru_maxrss/1024.0), peak/1024.0, current/1024.0);
         #endif
         fflush(stdout);
         }
         */
        
        if(logMemoryFile != NULL) {
  #ifdef __APPLE__
          fprintf(logMemoryFile, "%s %.02fMB VmHWM %.02fMB VmRSS %.02fMB\n\n", str, (usageData.ru_maxrss/1024.0)/1024.0, peak/1024.0, current/1024.0);
  #else
          fprintf(logMemoryFile, "%s %.02fMB VmHWM %.02fMB VmRSS %.02fMB\n\n", str, (usageData.ru_maxrss/1024.0), peak/1024.0, current/1024.0);
  #endif
          fflush(logMemoryFile);
        }
        
      }
    
    
    private:
    
      //***************************************************************************************************//
      // memUsage
      //***************************************************************************************************//
      static void getMemUsage(size_t & currentUsage, size_t & peakUsage) {
        
        FILE* file = fopen("/proc/self/status", "r");
        
        if(file == NULL) {
          fprintf(stderr, "Error, something went wrong with in fopen(/proc/self/status): %s\n", strerror(errno));
          exit(EXIT_FAILURE);
        }
        
        char line[PATH_MAX];
        
        while(fgets(line, 128, file) != NULL){
          
          if(strncmp(line, "VmHWM:", 7) == 0)
            peakUsage = parseLine(line);
          
          if(strncmp(line, "VmRSS:", 6) == 0)
            currentUsage = parseLine(line);
          
        }
        
        fclose(file);
        
      }
    
    
      //***************************************************************************************************//
      // parseLine
      //***************************************************************************************************//
      static inline size_t parseLine(char* line) {
        
        size_t i = strlen(line);
        
        while (*line < '0' || *line > '9') line++;
        
        line[i-3] = '\0';
        
        i = atoi(line);
        
        return i;
        
      }
    
  };
  
  FILE * profile::logTimeFile = NULL;
  FILE * profile::logMemoryFile = NULL;
  std::vector<profile::data_t> profile::timer = std::vector<profile::data_t>();
  rusage profile::usageData;
  char profile::str[PATH_MAX];
  bool profile::onScreenTime = true;
  
} /* namespace */

#endif /* _H_MPL_PROFILE_H_ */








/*
 #include <sys/resource.h>
 
 struct rusage usage;
 
 getrusage(RUSAGE_SELF, &usage);
 
 double ru_utime = (((double)(usage.ru_utime.tv_usec) / 1000000.0L) + ((double) (usage.ru_utime.tv_sec)));
 double ru_stime = (((double)(usage.ru_stime.tv_usec) / 1000000.0L) + ((double) (usage.ru_stime.tv_sec)));
 
 printf("ru_utime: %f\n", ru_utime);
 printf("ru_stime: %f\n", ru_stime);
 printf("tot: %f\n", ru_utime+ru_stime);
 printf("max RAM used (MB): %ld\n", usage.ru_maxrss/1024);
 */


//***************************************************************************************************//
// profile_memory_c
//***************************************************************************************************////
// process_mem_usage(double &, double &) - takes two doubles by reference,
// attempts to read the system-dependent data for a process' virtual memory
// size and resident set size, and return the results in KB.
//
// On failure, returns 0.0, 0.0
/*
 void process_mem_usage(double& vm_usage, double& resident_set){
 using std::ios_base;
 using std::ifstream;
 using std::string;
 
 vm_usage     = 0.0;
 resident_set = 0.0;
 
 // 'file' stat seems to give the most reliable results
 //
 std::ifstream stat_stream("/proc/self/stat",ios_base::in);
 
 // dummy vars for leading entries in stat that we don't care about
 //
 std::string pid, comm, state, ppid, pgrp, session, tty_nr;
 std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
 std::string utime, stime, cutime, cstime, priority, nice;
 std::string O, itrealvalue, starttime;
 
 // the two fields we want
 //
 unsigned long vsize;
 long rss;
 
 stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
 >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
 >> utime >> stime >> cutime >> cstime >> priority >> nice
 >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
 
 stat_stream.close();
 
 long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
 vm_usage     = vsize / 1024.0;
 resident_set = rss * page_size_kb;
 }
 */
