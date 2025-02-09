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

#ifndef _H_MPL_CPLEX_H_
#define _H_MPL_CPLEX_H_

#include <cstdlib>
#include <cstdio>

#include <sys/time.h>

#include <ilcplex/ilocplex.h>

//*****************************************************************************/
// namespace cplex
//*****************************************************************************/
namespace mpl::cplex {
  
  //*****************************************************************************/
  // namespace utils
  //*****************************************************************************/
  namespace utils {
    
    //*****************************************************************************/
    // data_t
    //*****************************************************************************/
    struct data_t {
      
    private:
      
      data_t() = delete;
      
      IloModel model;
      
      IloEnv environment;
      
      IloNumVarArray variables;
      IloNumArray coefficients;
      IloRangeArray constrains;
      
      IloObjective objective;
      
      std::size_t size;
      
    public:
      
      data_t(std::size_t _size) {
        
        size = _size;
        
        model        = IloModel(environment);
        
        variables    = IloNumVarArray(environment, size, 0, 1, IloNumVar::Bool);
        
        coefficients = IloNumArray(environment, size);
        
        constrains   = IloRangeArray(environment);
        
      }
      
      ~data_t() { environment.end(); }
      
      inline IloNumVarArray & getVariables()    { return variables;    }
      inline IloEnv         & getEnvironment()  { return environment;  }
     // inline IloNumArray    & getCoefficients() { return coefficients; }
     // inline IloRangeArray  & getConstrains()   { return constrains;   }
     // inline IloObjective   & getObjective()    { return objective;    }
      inline IloModel       & getModel()        { return model;        }

      inline std::ostream & getNullStream() { return environment.getNullStream(); }
      
      inline std::size_t getSize() { return size; }
      
      inline void addConstrain(IloExpr & expression, double value){ constrains.add(IloRange(environment, value, expression)); }
      
      inline void setCoefficient(std::size_t index, double value) { coefficients[index] = value; }
      
      inline void setCoefficients(double value, std::size_t from = 0, std::size_t to = std::numeric_limits<std::size_t>::max()) {
        
        if(to == std::numeric_limits<std::size_t>::max()) to = size;
        
        for(std::size_t i=from; i<to; ++i) coefficients[i] = value;
        
      }
      
      inline IloNumVar getVariable(std::size_t index) { return variables[index]; }
      
//      inline void setLinearCoefs() {
//        
//        // Create object function
//        objective = IloObjective(environment, 0.0, IloObjective::Minimize);
//        
//        objective.setLinearCoefs(variables, coefficients);
//        
//      }
      
      void setupModel() {
        
        // create objective
        objective = IloObjective(environment, 0.0, IloObjective::Minimize);
        
        objective.setLinearCoefs(variables, coefficients);
  
        // add objective
        model.add(objective);
        
        // add constrains
        model.add(constrains);
        
      }
      
    };
  
    
    //*******************************************************************************************************************************************/
    // Minimize cover set
    //*******************************************************************************************************************************************/
    //
    //    output_file_name:
    //          The file name output if NULL the log of cplex will be not saved
    //
    //    time_limit:
    //          The maximum time, in seconds, for computations before termination
    //
    //    relative_optimality_tolerance:
    //          Relative optimality tolerance guarantees that a solution lies within a certain percentage of the optimal solution
    //
    //    absolute_optimality_tolerance:
    //          Absolute optimality tolerance guarantees that a solution lies within a certain absolute range of the optimal solution
    //
    //*******************************************************************************************************************************************/
    bool minimize(data_t & data, std::vector<bool> & solution, const char * output_file_name = NULL, double time_limit = 1.0e+75, double relative_optimality_tolerance = 1.0e-04, double absolute_optimality_tolerance = 1.0e-06) {
      
      
      if(0) {
        IloEnv env;
        IloObjective obj;
        IloNumVarArray var(env);
        IloRangeArray rng(env);
        IloCplex cplex;
        IloModel model;
        cplex.importModel(model, "model.lp" , obj, var, rng);
        cplex.extract(model);
        cplex.solve();
        IloNumArray vals(env);
        cplex.getValues(vals, var);
        env.out() << "Solution status = " << cplex.getStatus() << std::endl;
        env.out() << "Solution value = " << cplex.getObjValue() << std::endl;
        env.out() << "Values = " << vals << std::endl;
      }
      
      static int counter = 0;
      
//      // Create object function
//      IloObjective objective(data.getEnvironment(), 0.0, IloObjective::Minimize);
//      
//      objective.setLinearCoefs(data.getVariables(), data.getCoefficients());
//      
//      // Create cplex model
//      IloModel model(data.getEnvironment());
//      
//      model.add(data.getObjective());
//      model.add(data.getConstrains());
      
      data.setupModel();
      
      // Setup clex solver
      IloCplex cplex(data.getModel());

      //cplex.setParam(IloCplex::TiLim,   time_limit); // tolto da me ora
      // cplex.setParam(IloCplex::Threads, 4);
      //cplex.setParam(IloCplex::EpGap,   relative_optimality_tolerance); // tolto da me ora
      //cplex.setParam(IloCplex::EpAGap,  absolute_optimality_tolerance); // tolto da me ora
      
#if(0)
      
      cplex.setParam(IloCplex::PopulateLim, 3);
      cplex.setParam(IloCplex::SolnPoolAGap, 0);
      //cplex.setParam(IloCplex::SolnPoolGap, 1);
      //cplex.setParam(IloCplex::SolnPoolCapacity, 3);
      cplex.setParam(IloCplex::SolnPoolIntensity, 1);
      
      printf("IloCplex::TiLim             %e\n",   cplex.getParam(IloCplex::TiLim));
      printf("IloCplex::NodeLim           %lld\n", cplex.getParam(IloCplex::NodeLim));
      printf("IloCplex::Threads           %d\n",   cplex.getParam(IloCplex::Threads));
      printf("IloCplex::EpGap             %e\n",   cplex.getParam(IloCplex::EpGap));
      printf("IloCplex::EpAGap            %e\n",   cplex.getParam(IloCplex::EpAGap));
      printf("IloCplex::PopulateLim       %d\n",   cplex.getParam(IloCplex::PopulateLim));
      printf("IloCplex::SolnPoolAGap      %e\n",   cplex.getParam(IloCplex::SolnPoolAGap));
      printf("IloCplex::SolnPoolGap       %e\n",   cplex.getParam(IloCplex::SolnPoolGap));
      printf("IloCplex::SolnPoolCapacity  %d\n",   cplex.getParam(IloCplex::SolnPoolCapacity));
      printf("IloCplex::SolnPoolIntensity %d\n",   cplex.getParam(IloCplex::SolnPoolIntensity));
      
#endif
      
      std::ofstream logFile;
      
      if(output_file_name!=NULL) { logFile.open(output_file_name, std::ofstream::out); cplex.setOut(logFile); }
      else cplex.setOut(data.getNullStream());
            
      //if(counter == 11881 || counter == 11882) cplex.setOut(std::cout);
      
      // timeval tv1; gettimeofday(&tv1, NULL);
      
      IloBool statusSolve = cplex.solve();
      
      if(!statusSolve) {
        //static int errorNum = 0;
        //std::string path = "/Users/thewoz/Desktop/model" + std::to_string(errorNum) + ".lp";
        //cplex.exportModel(path.c_str());
        fprintf(stderr, "error: cplex solve()\n");
        return false;
      }
      
//      if(!statusSolve) {
//        
//        fprintf(stderr, "error: cplex il modello non è stato risolto correttamente.\n");
//        
//        IloCplex::CplexStatus cplexStatus = cplex.getCplexStatus();
//
//        IloAlgorithm::Status status = cplex.getStatus();
//        
//        printf("%d) CplexStatus: %d Status: %d \n", counter, cplexStatus, status);
//        
//        try {
//          
//          IloNumArray values(data.getEnvironment());
//          cplex.getValues(values, data.getVariables());
//        
//        } catch (const IloException & e) {
//          
//          std::cerr << "Errore nel recupero dei valori: " << e.getMessage();
//          std::cout << "Variabili dichiarate: " << data.getVariables().getSize() << std::endl;
//          std::cout << "Variabili attive nel modello: " << cplex.getNcols() << std::endl;
//          
//          auto vars = data.getVariables();
//          
//          for (int i=0; i<vars.getSize(); ++i) {
//            try {
//              double value = cplex.getValue(vars[i]); // Ottieni il valore della variabile
//            } catch (const IloException & e) {
//              std::cout << e.getMessage();
//            }
//          }
//          
//        }
//        
//        counter++;
//        
//        return false;
//        
//      }
//      
//      counter++;
      
      //timeval tv2; gettimeofday(&tv2, NULL);

      //double solve_time = (((double)(tv2.tv_usec - tv1.tv_usec) / 1000000.0L) + ((double) (tv2.tv_sec - tv1.tv_sec)));
      
      //fprintf(stderr, "%lu %f\n", data.getSize(), solve_time); fflush(stderr);
      
      // non so se serve vistoil controllo di sopra
      if(cplex.getStatus() != IloAlgorithm::Optimal){
        fprintf(stderr, "error: cplex soluzione ottimale non trovata.\n");
        return false;
      }
      
      if(output_file_name!=NULL) logFile.close();
      
#if(0)
      
      if(cplex.populate()) {
        
        double pop_time = ((double)(clock() - start)/(double)CLK_TCK) - sol_time;
        
        long solutionsNum = cplex.getSolnPoolNsolns();
        
        printf("\nNumber of solutions found %ld in %f %f \n\n", solutionsNum, sol_time, pop_time);
        
        for(int k=0; k<solutionsNum; k++){
          printf("Solution %d energy %f\n\n", k+1, cplex.getObjValue(k));
          
#if(0)
          IloNumArray values(data.getEnvironment());
          
          cplex.getValues(values, data.getVariables(), k);
          
          printf("%d %d %d\n", (int)fabs(values[0]), (int)fabs(values[1]), (int)fabs(values[2]));
          printf("%d %d %d\n", (int)fabs(values[3]), (int)fabs(values[4]), (int)fabs(values[5]));
          printf("%d %d %d\n", (int)fabs(values[6]), (int)fabs(values[7]), (int)fabs(values[8]));
          
          printf("\n");
          
          printf("%d %d %d\n", (int)fabs(values[ 9]), (int)fabs(values[10]), (int)fabs(values[11]));
          printf("%d %d %d\n", (int)fabs(values[12]), (int)fabs(values[13]), (int)fabs(values[14]));
          
          printf("\n");printf("\n");printf("\n");
#endif
          
        }
        
      }
      
#endif
      
      
      IloNumArray values(data.getEnvironment());
      
      cplex.getValues(values, data.getVariables());
     
      solution.resize(data.getSize(), false);
      
      for(std::size_t i=0; i<data.getSize(); ++i){
        if(values[i] > 0) solution[i] = true;
      }
      
      values.end();
      
      return true;
      
    }
    
  } /* namespace utils */
  
  
  //*****************************************************************************/
  // minimizeSoft
  //*****************************************************************************/
  bool minimizeSoft(std::vector<double> & coefficient, std::vector< std::vector<std::size_t> > & constrains, std::vector<bool> & solution, const char * output_file_name = NULL, double time_limit = 1.0e+75, double relative_optimality_tolerance = 1.0e-04, double absolute_optimality_tolerance = 1.0e-06) {
    
    cplex::utils::data_t data(coefficient.size());
    
    for(std::size_t i=0; i<coefficient.size(); ++i)
      data.setCoefficient(i, coefficient[i]);
    
    //data.setLinearCoefs();
    
    for(std::size_t i=0; i<constrains.size(); ++i){
      
      IloExpr expression(data.getEnvironment());
      
      if(constrains[i].size() != 0) {
        
        for(std::size_t j=0; j<constrains[i].size()-1; ++j)
          expression += data.getVariable(constrains[i][j]);
        
        expression -= data.getVariable(constrains[i][constrains[i].size()-1]);
        
      } else { printf("error point without constrains\n"); abort(); }
      
      data.addConstrain(expression, 0);
      
      expression.end();
      
    }
    
    return cplex::utils::minimize(data, solution, output_file_name);
    
  }
  
  //*****************************************************************************/
  // minimizeHard
  //*****************************************************************************/
  bool minimizeHard(std::vector<double> & coefficient, std::vector< std::vector<std::size_t> > & constrains, std::vector<bool> & solution, const char * output_file_name = NULL, double time_limit = 1.0e+75, double relative_optimality_tolerance = 1.0e-04, double absolute_optimality_tolerance = 1.0e-06) {
        
    cplex::utils::data_t data(coefficient.size());
    
    for(std::size_t i=0; i<coefficient.size(); ++i)
      data.setCoefficient(i, coefficient[i]);
    
    //data.setLinearCoefs();
    
    for(std::size_t i=0; i<constrains.size(); ++i){
      
      IloExpr expression(data.getEnvironment());
      
      for(std::size_t j=0; j<constrains[i].size(); ++j)
        expression += data.getVariable(constrains[i][j]);
      
      data.addConstrain(expression, 1);
      
      expression.end();
      
    }

    return cplex::utils::minimize(data, solution, output_file_name);
    
  }
  
} /* namespace cplex */


#endif /* _H_MPL_CPLEX_H_ */


