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

#ifndef _H_MPL_GUROBI_H_
#define _H_MPL_GUROBI_H_

#include <cstdlib>
#include <cstdio>

#include <fstream>
#include <limits>
#include <memory>
#include <vector>

#include <gurobi_c++.h>

namespace mpl::gurobi {

  namespace utils {

    struct data_t {

    private:

      data_t() = delete;

      GRBEnv environment;
      GRBModel model;

      std::vector<GRBVar> variables;
      std::vector<double> coefficients;

      std::size_t size;

    public:

      explicit data_t(std::size_t _size) :
      environment(true),
      model(environment),
      variables(),
      coefficients(_size, 0.0),
      size(_size) {

        environment.set(GRB_IntParam_OutputFlag, 0);
        environment.start();

        variables.reserve(size);
        for(std::size_t i=0; i<size; ++i)
          variables.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY));

      }

      inline std::vector<GRBVar> & getVariables() { return variables; }
      inline GRBModel            & getModel()     { return model; }

      inline std::size_t getSize() { return size; }

      inline void addConstrain(const GRBLinExpr & expression, double value) { model.addConstr(expression >= value); }

      inline void setCoefficient(std::size_t index, double value) { coefficients[index] = value; }

      inline void setCoefficients(double value, std::size_t from = 0, std::size_t to = std::numeric_limits<std::size_t>::max()) {

        if(to == std::numeric_limits<std::size_t>::max()) to = size;

        for(std::size_t i=from; i<to; ++i) coefficients[i] = value;

      }

      inline GRBVar getVariable(std::size_t index) { return variables[index]; }

      void setupModel() {

        GRBLinExpr objective = 0.0;
        for(std::size_t i=0; i<size; ++i)
          objective += coefficients[i] * variables[i];

        model.setObjective(objective, GRB_MINIMIZE);

      }

    };

    bool minimize(data_t & data, std::vector<bool> & solution, const char * output_file_name = NULL, double time_limit = 1.0e+75, double relative_optimality_tolerance = 1.0e-04, double absolute_optimality_tolerance = 1.0e-06) {

      data.setupModel();

      GRBModel & model = data.getModel();

      model.set(GRB_DoubleParam_TimeLimit, time_limit);
      model.set(GRB_DoubleParam_MIPGap, relative_optimality_tolerance);
      model.set(GRB_DoubleParam_MIPGapAbs, absolute_optimality_tolerance);

      if(output_file_name != NULL) model.set(GRB_StringParam_LogFile, output_file_name);

      try {
        model.optimize();
      } catch (const GRBException & e) {
        fprintf(stderr, "error: gurobi optimize() (%d - %s)\n", e.getErrorCode(), e.getMessage().c_str());
        return false;
      }

      int status = model.get(GRB_IntAttr_Status);
      if(status != GRB_OPTIMAL) {
        fprintf(stderr, "error: gurobi soluzione ottimale non trovata (status=%d).\n", status);
        return false;
      }

      solution.assign(data.getSize(), false);

      for(std::size_t i=0; i<data.getSize(); ++i) {
        if(data.getVariable(i).get(GRB_DoubleAttr_X) > 0.0)
          solution[i] = true;
      }

      return true;

    }

  } /* namespace utils */


  bool minimizeSoft(std::vector<double> & coefficient, std::vector< std::vector<std::size_t> > & constrains, std::vector<bool> & solution, const char * output_file_name = NULL, double time_limit = 1.0e+75, double relative_optimality_tolerance = 1.0e-04, double absolute_optimality_tolerance = 1.0e-06) {

    utils::data_t data(coefficient.size());

    for(std::size_t i=0; i<coefficient.size(); ++i)
      data.setCoefficient(i, coefficient[i]);

    for(std::size_t i=0; i<constrains.size(); ++i) {

      GRBLinExpr expression = 0.0;

      if(constrains[i].size() != 0) {

        for(std::size_t j=0; j<constrains[i].size()-1; ++j)
          expression += data.getVariable(constrains[i][j]);

        expression -= data.getVariable(constrains[i][constrains[i].size()-1]);

      } else { printf("error point without constrains\n"); abort(); }

      data.addConstrain(expression, 0.0);

    }

    return utils::minimize(data, solution, output_file_name, time_limit, relative_optimality_tolerance, absolute_optimality_tolerance);

  }

  bool minimizeHard(std::vector<double> & coefficient, std::vector< std::vector<std::size_t> > & constrains, std::vector<bool> & solution, const char * output_file_name = NULL, double time_limit = 1.0e+75, double relative_optimality_tolerance = 1.0e-04, double absolute_optimality_tolerance = 1.0e-06) {

    utils::data_t data(coefficient.size());

    for(std::size_t i=0; i<coefficient.size(); ++i)
      data.setCoefficient(i, coefficient[i]);

    for(std::size_t i=0; i<constrains.size(); ++i) {

      GRBLinExpr expression = 0.0;

      for(std::size_t j=0; j<constrains[i].size(); ++j)
        expression += data.getVariable(constrains[i][j]);

      data.addConstrain(expression, 1.0);

    }

    return utils::minimize(data, solution, output_file_name, time_limit, relative_optimality_tolerance, absolute_optimality_tolerance);

  }

} /* namespace gurobi */

#endif /* _H_MPL_GUROBI_H_ */
