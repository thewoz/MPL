#include <iostream>
#include <string>
#include <vector>

#ifdef MPL_ENABLE_CPLEX
#include <optimization/cplex.hpp>
#endif

#ifdef MPL_ENABLE_GUROBI
#include <optimization/gurobi.hpp>
#endif

namespace {

bool runExample(bool use_hard_constraints, const std::string & solver_name) {
  std::vector<double> coefficients = {1.0, 2.0, 3.0};

  std::vector<std::vector<std::size_t>> constraints;
  if(use_hard_constraints) {
    constraints = {
      {0, 1},
      {1, 2}
    };
  } else {
    constraints = {
      {0, 1, 2},
      {0, 2, 1}
    };
  }

  std::vector<bool> solution;

#ifdef MPL_ENABLE_CPLEX
  if(solver_name == "cplex") {
    bool ok = use_hard_constraints
      ? mpl::cplex::minimizeHard(coefficients, constraints, solution)
      : mpl::cplex::minimizeSoft(coefficients, constraints, solution);

    if(!ok) return false;
  }
#endif

#ifdef MPL_ENABLE_GUROBI
  if(solver_name == "gurobi") {
    bool ok = use_hard_constraints
      ? mpl::gurobi::minimizeHard(coefficients, constraints, solution)
      : mpl::gurobi::minimizeSoft(coefficients, constraints, solution);

    if(!ok) return false;
  }
#endif

  if(solution.empty()) {
    std::cerr << "Solver non disponibile o nome non valido: " << solver_name << '\n';
    return false;
  }

  std::cout << "Solver: " << solver_name << '\n';
  std::cout << "Tipo vincoli: " << (use_hard_constraints ? "hard" : "soft") << '\n';
  std::cout << "Soluzione: ";
  for(bool v : solution)
    std::cout << (v ? 1 : 0) << ' ';
  std::cout << '\n';

  return true;
}

} // namespace

int main(int argc, char ** argv) {
  if(argc < 2) {
    std::cerr << "Uso: " << argv[0] << " <cplex|gurobi> [hard|soft]\n";
    return 1;
  }

  std::string solver_name = argv[1];
  bool use_hard_constraints = true;

  if(argc >= 3) {
    std::string mode = argv[2];
    if(mode == "soft") use_hard_constraints = false;
    else if(mode != "hard") {
      std::cerr << "Secondo argomento non valido. Usa hard o soft.\n";
      return 1;
    }
  }

  return runExample(use_hard_constraints, solver_name) ? 0 : 2;
}
