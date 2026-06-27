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

#ifndef _H_MPL_CORE_LOG_H_
#define _H_MPL_CORE_LOG_H_

#include <cerrno>
#include <climits>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

//*****************************************************************************/
// namespace mpl
//*****************************************************************************/
namespace mpl {

  //***************************************************************************/
  // class log
  //***************************************************************************/
  // Logger statico: stampa su stdout/stderr e, opzionalmente, su un file di
  // log aperto con init(). Tutte le funzioni di stampa usano il formato
  // printf-style.
  //***************************************************************************/
  class log {

  public:

    enum { ON, OFF };

  private:

    inline static std::FILE * output   = nullptr;
    inline static bool        onScreen = ON;

    log() = delete;

    //*************************************************************************/
    // emit() - stampa 'format' su 'primary' (se non nullptr) e sul file di
    //          log (se aperto). 'ap' resta valido per il chiamante grazie a
    //          va_copy.
    //*************************************************************************/
    static void emit(std::FILE * primary, const char * format, va_list ap) {

      if(primary != nullptr) {
        va_list cp;
        va_copy(cp, ap);
        std::vfprintf(primary, format, cp);
        va_end(cp);
      }

      if(output != nullptr) {
        va_list cp;
        va_copy(cp, ap);
        std::vfprintf(output, format, cp);
        va_end(cp);
      }

      flush();

    }

  public:

    //*************************************************************************/
    // getOutput / setOutputOnScreen
    //*************************************************************************/
    static std::FILE * getOutput() { return output; }

    static void setOutputOnScreen(bool mode) { onScreen = mode; }

    //*************************************************************************/
    // init() - apre (o riapre) il file di log.
    //*************************************************************************/
    static void init(const char * format, ...) {

      char filename[PATH_MAX];

      va_list ap;
      va_start(ap, format);
      std::vsnprintf(filename, PATH_MAX, format, ap);
      va_end(ap);

      if(output != nullptr) std::fclose(output);

      output = std::fopen(filename, "w");

      if(output == nullptr) {
        std::fprintf(stderr, "mpl::log::init() error: cannot open '%s': %s\n",
                     filename, std::strerror(errno));
        std::exit(EXIT_FAILURE);
      }

    }

    //*************************************************************************/
    // msn() - messaggio informativo (stdout se onScreen + file di log).
    //*************************************************************************/
    static void msn(const char * format, ...) {

      va_list ap;
      va_start(ap, format);
      emit(onScreen == ON ? stdout : nullptr, format, ap);
      va_end(ap);

    }

    //*************************************************************************/
    // warning() - messaggio non fatale (stderr + file di log).
    //*************************************************************************/
    static void warning(const char * format, ...) {

      va_list ap;
      va_start(ap, format);
      emit(stderr, format, ap);
      va_end(ap);

    }

    //*************************************************************************/
    // error() - messaggio fatale (stderr + file di log), poi termina.
    //*************************************************************************/
    static void error(const char * format, ...) {

      va_list ap;
      va_start(ap, format);
      emit(stderr, format, ap);
      va_end(ap);

      std::exit(EXIT_FAILURE);

    }

    //*************************************************************************/
    // flush / close
    //*************************************************************************/
    static void flush() {

      std::fflush(stdout);
      std::fflush(stderr);

      if(output != nullptr) std::fflush(output);

    }

    static void close() { if(output != nullptr) std::fclose(output); }

  };

} // namespace mpl

#endif // _H_MPL_CORE_LOG_H_
