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

#include <ilcplex/ilocplex.h>

/*****************************************************************************/
// namespace cplex
/*****************************************************************************/
namespace mpl::cplex {
  
  /*****************************************************************************/
  // namespace utils
  /*****************************************************************************/
  namespace utils {
    
    /*****************************************************************************/
    // data_t
    /*****************************************************************************/
    struct data_t {
      
    private:
      
      data_t() = delete;
      
      IloEnv environment;
      
      IloNumVarArray variables;
      IloNumArray coefficients;
      IloRangeArray constrains;
      
      IloObjective objective;
      
      std::size_t size;
      
    public:
      
      data_t(std::size_t _size) {
        
        size = _size;
        
        variables    = IloNumVarArray(environment, size, 0, 1, IloNumVar::Bool);
        
        coefficients = IloNumArray(environment, size);
        
        constrains   = IloRangeArray(environment);
        
      }
      
      ~data_t() { environment.end(); }
      
      inline IloNumVarArray & getVariables()    { return variables;    }
      inline IloEnv         & getEnvironment()  { return environment;  }
      inline IloNumArray    & getCoefficients() { return coefficients; }
      inline IloRangeArray  & getConstrains()   { return constrains;   }
      inline IloObjective   & getObjective()    { return objective;    }
      
      inline std::ostream & getNullStream() { return environment.getNullStream(); }
      
      inline std::size_t getSize() { return size; }
      
      inline void addConstrain(IloExpr & expression, double value){ constrains.add(IloRange(environment, value, expression)); }
      
      inline void setCoefficient(std::size_t index, double value) { coefficients[index] = value; }
      
      inline void setCoefficients(double value, std::size_t from = 0, std::size_t to = std::numeric_limits<std::size_t>::max()) {
        
        if(to == std::numeric_limits<std::size_t>::max()) to = size;
        
        for(std::size_t i=from; i<to; ++i) coefficients[i] = value;
        
      }
      
      inline IloNumVar getVariable(std::size_t index) { return variables[index]; }
      
      inline void setLinearCoefs() {
        
        // Create object function
        objective = IloObjective(environment, 0.0, IloObjective::Minimize);
        
        objective.setLinearCoefs(variables, coefficients);
        
      }
      
    };
    
    /*******************************************************************************************************************************************/
    // Minimize cover set
    /*******************************************************************************************************************************************/
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
    /*******************************************************************************************************************************************/
    bool minimize(data_t & data, std::vector<bool> & solution, const char * output_file_name = NULL, double time_limit = 1.0e+75, double relative_optimality_tolerance = 1.0e-04, double absolute_optimality_tolerance = 1.0e-06) {
      
      // Create object function
      IloObjective objective(data.getEnvironment(), 0.0, IloObjective::Minimize);
      
      objective.setLinearCoefs(data.getVariables(), data.getCoefficients());
      
      // Create cplex model
      IloModel model(data.getEnvironment());
      
      model.add(data.getObjective());
      model.add(data.getConstrains());
      
      // Setup clex solver
      IloCplex cplex(model);
      
      
      cplex.setParam(IloCplex::TiLim,   time_limit);
      // cplex.setParam(IloCplex::Threads, 4);
      cplex.setParam(IloCplex::EpGap,   relative_optimality_tolerance);
      cplex.setParam(IloCplex::EpAGap,  absolute_optimality_tolerance);
      
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
      
      //cplex.setOut(data.getNullStream());
      
      //time_t start = clock();
      
      cplex.solve();
      
      //double sol_time = (double)(clock() - start)/(double)CLK_TCK;
      
      if(cplex.getStatus() != IloAlgorithm::Optimal){
        fprintf(stderr, "error: cplex terminate with an error.\n");
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
  
  
  /*****************************************************************************/
  // minimizeSoft
  /*****************************************************************************/
  bool minimizeSoft(std::vector<double> & coefficient, std::vector< std::vector<std::size_t> > & constrains, std::vector<bool> & solution, const char * output_file_name = NULL, double time_limit = 1.0e+75, double relative_optimality_tolerance = 1.0e-04, double absolute_optimality_tolerance = 1.0e-06) {
    
    cplex::utils::data_t data(coefficient.size());
    
    for(std::size_t i=0; i<coefficient.size(); ++i)
      data.setCoefficient(i, coefficient[i]);
    
    data.setLinearCoefs();
    
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
  
  /*****************************************************************************/
  // minimizeHard
  /*****************************************************************************/
  bool minimizeHard(std::vector<double> & coefficient, std::vector< std::vector<std::size_t> > & constrains, std::vector<bool> & solution, const char * output_file_name = NULL, double time_limit = 1.0e+75, double relative_optimality_tolerance = 1.0e-04, double absolute_optimality_tolerance = 1.0e-06) {
        
    cplex::utils::data_t data(coefficient.size());
    
    for(std::size_t i=0; i<coefficient.size(); ++i)
      data.setCoefficient(i, coefficient[i]);
    
    data.setLinearCoefs();
    
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


