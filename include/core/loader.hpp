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


#ifndef _H_MPL_IO_LOADER_H_
#define _H_MPL_IO_LOADER_H_

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cctype>

#include <vector>
#include <string>
#include <sstream>
#include <limits>
#include <algorithm>
#include <functional>
#include <charconv>
#include <type_traits>

#include <mpl/core/stdio.hpp>

#ifndef LINE_MAX
  #define LINE_MAX 2048
#endif

//****************************************************************************/
// namespace mpl
//****************************************************************************/
namespace mpl {
  
  //****************************************************************************/
  // class loader
  //****************************************************************************/
  class loader {
    
  public:
    
    //****************************************************************************/
    // data
    //****************************************************************************/
    class data {
      
      private:
      
        // vettore dove vengono salvati i valori delle colonne lette
        std::vector<std::string> arguments;
      
      
      public:
      
        std::string line;

      
        //****************************************************************************/
        // add
        //****************************************************************************/
        void add(const std::string & argument) { arguments.push_back(argument); }

        // aggiunge un argomento da [begin, begin+len) senza copie intermedie
        void add(const char * begin, size_t len) { arguments.emplace_back(begin, len); }

        //****************************************************************************/
        // clear / reserve - per riusare lo stesso oggetto tra una riga e l'altra
        //****************************************************************************/
        void clear() { arguments.clear(); }
        void reserve(size_t n) { arguments.reserve(n); }

        //****************************************************************************/
        // get
        //****************************************************************************/
        template <typename T>
        T get(size_t index) const {

          if(index >= arguments.size()){
            fprintf(stderr, "mpl::loader::get() error: the requested argument (%zu) does not exist\n", index);
            abort();
          }

          const std::string & str = arguments[index];

          T value{};

          if constexpr (std::is_integral_v<T>) {
            // veloce e senza allocazioni
            std::from_chars(str.data(), str.data() + str.size(), value);
          } else if constexpr (std::is_floating_point_v<T>) {
            // libc++ su macOS non supporta from_chars floating: uso strtod
            value = static_cast<T>(strtod(str.c_str(), nullptr));
          } else {
            // fallback per tipi non numerici
            std::stringstream iss(str);
            iss >> value;
          }

          return value;

        }
      
      }; // class data
    
  private:

    //****************************************************************************/
    // nextToken
    //
    // Estrae il prossimo token (separato da spazi) a partire da *cursor.
    // Ritorna false a fine riga; altrimenti [begin, end) delimita il token
    // e cursor viene avanzato. Non copia ne' alloca nulla.
    //****************************************************************************/
    static inline bool nextToken(char *& cursor, char *& begin, char *& end) {

      // salto gli spazi prima del token
      while(*cursor != '\0' && std::isspace((unsigned char)*cursor)) ++cursor;

      // fine riga
      if(*cursor == '\0') return false;

      begin = cursor;

      // avanzo fino al prossimo spazio
      while(*cursor != '\0' && !std::isspace((unsigned char)*cursor)) ++cursor;

      end = cursor;

      return true;

    }

    //****************************************************************************/
    // getColsToRead
    //****************************************************************************/
    void getColsToRead(const std::string & colsFormat, std::vector<size_t> & columns){
      
      std::stringstream iss(colsFormat);
      
      std::size_t value;

      while(iss.good()){

        iss >> value;

        columns.push_back(value-1);
        
      }
      
    }
 
  public:
    
    //****************************************************************************/
    // loader
    //****************************************************************************/
    loader(){};
    
    //****************************************************************************/
    // loader
    //****************************************************************************/
    loader(const std::string & inputFile, const std::string & columnsToRead, std::function<void(const loader::data & arguments)> fillerFun = nullptr) {
      
      (*this)(inputFile, columnsToRead, fillerFun);
      
    }
    
    //****************************************************************************/
    // load
    //****************************************************************************/
    void load(const std::string & inputFile, const std::string & columnsToRead, std::function<void(const loader::data & arguments)> fillerFun = nullptr) {
      
      (*this)(inputFile, columnsToRead, fillerFun);
      
    }
    
    //****************************************************************************/
    // operator ()
    //****************************************************************************/
    void operator () (const std::string & inputFile, const std::string & columnsToRead, std::function<void(const loader::data & arguments)> fillerFun = nullptr) {

      // se non viene passato un filler uso quello virtuale chiamandolo
      // direttamente nel loop (niente std::function/std::bind di mezzo)
      const bool useVirtual = !fillerFun;

      // apro il file in lettura
      FILE * input = io::open(inputFile, "r");
      
      // vettore che contiene i numeri delle colonne da leggere
      std::vector<size_t> columns;
            
      // riempio il vettore delle colonne da leggere
      getColsToRead(columnsToRead, columns);
      
      // variabile di appoggio di lettura di una linea del file
      char line[LINE_MAX];

      // oggetto riusato tra una riga e l'altra per evitare riallocazioni
      loader::data arguments;
      arguments.reserve(columns.size());

      // ciclo su tutte le linee del file
      while(fgets(line, LINE_MAX, input)){

        // se la linea e vuota o va saltata la salto
        if(mpl::io::isToSkip(line)) continue;

        // ripulisco gli argomenti della riga precedente (mantenendo la capacita')
        arguments.clear();

        arguments.line = line;

        // variabile che segna il numero di argomenti letti sulla singola linea
        size_t read = 0;

        // variabile per scorrere le colonne da salvare
        size_t colIndex = 0;

        char * cursor = line;
        char * begin  = nullptr;
        char * end    = nullptr;

        // ciclo su tutte le colonne della riga
        while(nextToken(cursor, begin, end)){

          // se la colonna non va salvata
          if(read != columns[colIndex]) { ++read; continue; }

          // salvo l'argomento letto (senza copie intermedie)
          arguments.add(begin, (size_t)(end - begin));

          // se ho letto tutte le colonne che mi interessavano
          if(read == columns.back()) break;

          // incremento il contatore delle colonne da salvare lette
          ++colIndex;

          // incremento il contatore delle colonne lette
          ++read;

        } // ciclo su tutte le colonne del file

        // se non ho letto tutte le colonne che mi aspettavo
        if(read != columns.back()) { fprintf(stderr, "mpl::loader::operator() error: bad number of columns in \"%s\"\n", inputFile.c_str()); abort(); }

        // fillo i dati
        if(useVirtual) filler(arguments); else fillerFun(arguments);

      } // ciclo su tutte le righe del file
      
      io::close(input);

    }
    
    //****************************************************************************/
    // operator ()
    //****************************************************************************/
    template <typename T>
    inline void operator () (const std::string & inputFile, const std::string & columnsToRead, T & container, void(T::*fillerFun)(const loader::data & arguments)) {
      (*this)(inputFile, columnsToRead, [&container, fillerFun](const loader::data & a){ (container.*fillerFun)(a); });
    }
    
    //****************************************************************************/
    // operator ()
    //****************************************************************************/
    template <typename T>
    inline void operator () (const std::string & inputFile, const std::string & columnsToRead, T * container, void(T::*fillerFun)(const loader::data & arguments)){
      (*this)(inputFile, columnsToRead, [container, fillerFun](const loader::data & a){ (container->*fillerFun)(a); });
    }
    
    //****************************************************************************/
    // minmax
    //****************************************************************************/
    template <typename T>
    void minmax(const std::string & inputFile, size_t column, T & min, T & max){
      
      // apro il file in lettura
      FILE * input = io::open(inputFile, "r");
      
      // variabile di appoggio di lettura di una linea del file
      char line[PATH_MAX];
      
      // variabile di appoggio per un singolo argomento letto
      T value;
      
      min = std::numeric_limits<T>::max();
      max = std::numeric_limits<T>::lowest();
      
      column -= 1;

      // ciclo su tutte le linee del file
      while(fgets(line, PATH_MAX, input)){

        // se la linea e vuota o va saltata la salto
        if(mpl::io::isToSkip(line)) continue;

        // variabile che segna il numero di argomenti letti sulla singola linea
        size_t read = 0;

        char * cursor = line;
        char * begin  = nullptr;
        char * end    = nullptr;

        bool found = false;

        // ciclo su tutte le colonne della riga
        while(nextToken(cursor, begin, end)){

          // se la colonna non va salvata
          if(read++ != column) continue;

          // begin e' delimitato da spazi: strtod si ferma da solo
          value = static_cast<T>(strtod(begin, nullptr));

          if(min > value) min = value;
          if(max < value) max = value;

          found = true;

          break;

        } // ciclo su tutte le colonne del file

        // se non ho letto la colonna che mi aspettavo
        if(!found) { fprintf(stderr, "mpl::loader::minmax() error: column not found\n"); abort(); }

      } // ciclo su tutte le righe del file
      
      io::close(input);

    }
    
    //****************************************************************************/
    // max
    //****************************************************************************/
    template <typename T>
    void max(const std::string & inputFile, size_t column, T & max){ T min; minmax(inputFile, column, min, max); }
    
    //****************************************************************************/
    // min
    //****************************************************************************/
    template <typename T>
    void min(const std::string & inputFile, size_t column, T & min){ T max; minmax(inputFile, column, min, max); }
    
    //****************************************************************************/
    // tune
    //****************************************************************************/
    int tune(const std::string & inputFile) {
      
      FILE * input = io::open(inputFile, "r");

      // variabile di appoggio di lettura di una linea del file
      char line[PATH_MAX];
      
      int column = 0;
      
      // ciclo su tutte le linee del file
      while(fgets(line, PATH_MAX, input)){
                
        // se la linea e vuota o va saltata la salto
        if(mpl::io::isToSkip(line)) continue;

        char * cursor = line;
        char * begin  = nullptr;
        char * end    = nullptr;

        // conto le colonne della riga
        while(nextToken(cursor, begin, end)) column++;

      }
      
      io::close(input);

      return column;
      
    }
    
  protected:
    
    //****************************************************************************/
    // loader
    //****************************************************************************/
    virtual void filler(const loader::data & arguments) { };
    
  }; // class load
  
  //****************************************************************************/
  // get
  //****************************************************************************/
  template <>
  inline const std::string & loader::data::get(size_t index) const {

    if(index >= arguments.size()){
      fprintf(stderr, "mpl::loader::get() error: the requested argument (%zu) does not exist\n", index);
      abort();
    }
    
    return arguments[index];
    
  }
  
} // namespace mpl

#endif // _H_MPL_IO_LOADER_H_
