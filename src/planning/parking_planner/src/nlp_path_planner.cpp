// Copyright 2020 Embotech AG, Zurich, Switzerland. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This file contains the implementation of the NLP part of the path planner, in
// particular the code calling the NLP solver. The actual problem solved is described
// in generate_nlp_planner_solver.cpp. The latter code is a standalone program that,
// when compiled, generates some callbacks for IPOPT in the form of shared library code.
// The code here then loads those callbacks from the shared library and uses them with
// IPOPT to solve the problem numerically.


#include <casadi/casadi.hpp>

#include <iostream>
#include <vector>

#include "parking_planner/configuration.hpp"
#include "parking_planner/nlp_path_planner.hpp"
#include "parking_planner/nlp_adapters.hpp"

// This header is generated by generate_nlp_planner_solver.cpp
#include "nlp_cpp_info.hpp"

namespace autoware
{

namespace motion
{

namespace planning
{

namespace parking_planner
{

// Since CasADi requires bounds for constraints to be
//     something <= the function <= something else
// and we need to be able to implement constraints of the form
//     the function <= something
// we need to mimmick the behavior of "no lower bound" by picking a large number. The number here
// has been picked heuristically as "large enough".
constexpr auto LARGE_NEGATIVE_NUMBER = -1.0e4;

// Since the number of obstacles in the problem has to be constant and picked at compile time,
// the obstacle constraints have to be populated with dummy values when there are fewer obstacles
// in the problem than are supported. The value here defines how far away from the starting state
// the dummy obstacle is placed. This value is in meters and should be far enough away so as to
// not interfere with the maneuver, but also close enough not to lead to numerical issues. The
// value has been chosen through experimentation and is in meters.
constexpr auto DUMMY_OBSTACLE_DISTANCE = 2e2;


// --- Adapter functionality ----------------------------------------------------------
// This is not in nlp_adapters.hpp because it is only used for constructing concrete
// valued objects for the problem instance, but not for generating the solver.
static std::vector<NLPObstacleStageVariables<double>>
make_nlpobstacle_stage_variables(const double lambda_guess, const double mu_guess)
{
  // First build stage variables
  std::vector<NLPObstacleStageVariables<double>> stage_variables{};
  stage_variables.reserve(HORIZON_LENGTH);

  const auto make_guess_vector = [](auto len, auto value) {
      std::vector<double> guess{};
      for (std::size_t j = {}; j < len; ++j) {
        guess.push_back(value);
      }
      return guess;
    };

  for (std::size_t k = {}; k < HORIZON_LENGTH; ++k) {
    stage_variables.push_back(
      NLPObstacleStageVariables<double>(
        make_guess_vector(NLPObstacleStageVariables<double>::get_lambda_length(), lambda_guess),
        make_guess_vector(NLPObstacleStageVariables<double>::get_mu_length(), mu_guess) )
    );
  }

  return stage_variables;
}

static NLPObstacle<double> create_nlpobstacle(
  const Polytope2D<double> polytope,
  const double lambda_guess, const double mu_guess)
{
  auto stage_variables = make_nlpobstacle_stage_variables(lambda_guess, mu_guess);

  const auto & polytope_halfplanes = polytope.get_halfplanes();
  std::vector<Halfplane2D<double>> halfplanes{};
  halfplanes.insert(
    halfplanes.end(), polytope_halfplanes.begin(),
    polytope_halfplanes.end());

  // Make sure we don't attempt to solve a problem we cannot solve
  if (MAX_HYPERPLANES_PER_OBSTACLE < polytope_halfplanes.size() ) {
    throw std::domain_error{"Cannot handle this many halfplanes"};
  }

  // Fill the remaining halfplanes with copies of the first one
  for (std::size_t k = {}; k < (MAX_HYPERPLANES_PER_OBSTACLE - polytope_halfplanes.size()); ++k) {
    halfplanes.push_back(polytope_halfplanes[0]);
  }

  return NLPObstacle<double>(stage_variables, halfplanes);
}

// Create a dummy obstacle for the given initial guesses of lambda and mu, as
// well as a vehicle state. The vehicle state is used to ensure that the
// dummy obstacle is far enough away from the vehicle to not matter for the
// path planning.
static NLPObstacle<double> create_dummy_nlpobstacle(
  const double lambda_guess, const double mu_guess,
  const VehicleState<double> & current_state
)
{
  const auto x_vehicle = current_state.get_x();
  const auto y_vehicle = current_state.get_y();
  constexpr auto L = DUMMY_OBSTACLE_DISTANCE;  // shorthand
  Point2D<double> p1(x_vehicle + L - 1, y_vehicle + L - 1);
  Point2D<double> p2(x_vehicle + L - 1, y_vehicle + L + 1);
  Point2D<double> p3(x_vehicle + L + 1, y_vehicle + L + 1);
  Point2D<double> p4(x_vehicle + L + 1, y_vehicle + L - 1);
  Polytope2D<double> dummy_polyhedron({p1, p2, p3, p4});
  return create_nlpobstacle(dummy_polyhedron, lambda_guess, mu_guess);
}


// Just used for returning both types of bounds
struct ConstraintBounds
{
  std::vector<double> lower;
  std::vector<double> upper;
};

ConstraintBounds create_constraint_bounds()
{
  std::vector<double> upper_bounds{};
  upper_bounds.reserve(NUMBER_OF_EQUALITIES + NUMBER_OF_INEQUALITIES);
  std::vector<double> lower_bounds{};
  lower_bounds.reserve(NUMBER_OF_EQUALITIES + NUMBER_OF_INEQUALITIES);
  for (std::size_t k = {}; k < NUMBER_OF_EQUALITIES; k++) {
    lower_bounds.push_back(0.0);
    upper_bounds.push_back(0.0);
  }
  for (std::size_t k = {}; k < NUMBER_OF_INEQUALITIES; k++) {
    lower_bounds.push_back(LARGE_NEGATIVE_NUMBER);
    upper_bounds.push_back(0.0);
  }

  return ConstraintBounds{lower_bounds, upper_bounds};
}

std::vector<NLPObstacle<double>> create_obstacles_from_polyhedra(
  const std::vector<Polytope2D<double>> & obstacles,
  const VehicleState<double> & current_state
)
{
  if (obstacles.size() > MAX_NUMBER_OF_OBSTACLES) {
    throw std::length_error{"Too many obstacles given as input"};
  }

  // First put all the actual obstacles in the list
  std::vector<NLPObstacle<double>> nlp_obstacle_list{};
  nlp_obstacle_list.reserve(MAX_NUMBER_OF_OBSTACLES);
  for (const auto & obstacle : obstacles) {
    const auto nlp_obstacle = create_nlpobstacle(obstacle, 0.1, 0.1);
    nlp_obstacle_list.push_back(nlp_obstacle);
  }

  // Fill the rest of the positions with dummy obstacles
  for (std::size_t k = {}; k < (MAX_NUMBER_OF_OBSTACLES - obstacles.size() ); k++) {
    const auto nlp_obstacle = create_dummy_nlpobstacle(0.1, 0.1, current_state);
    nlp_obstacle_list.push_back(nlp_obstacle);
  }

  return nlp_obstacle_list;
}

NLPPathPlanner::NLPPathPlanner(
  const NLPCostWeights<double> & cost_weights,
  const VehicleState<double> & lower_state_bounds,
  const VehicleState<double> & upper_state_bounds,
  const VehicleCommand<double> & lower_command_bounds,
  const VehicleCommand<double> & upper_command_bounds
)
: m_cost_weights(cost_weights)
{
  m_lower_state_bounds = lower_state_bounds;
  m_upper_state_bounds = upper_state_bounds;
  m_lower_command_bounds = lower_command_bounds;
  m_upper_command_bounds = upper_command_bounds;
}

template<typename T>
static bool are_vectors_close(std::vector<T> vector1, std::vector<T> vector2, double tolerance)
{
  for (std::size_t k = {}; k < vector1.size(); ++k) {
    if (std::abs(vector1[k] - vector2[k]) > tolerance) {
      return false;
    }
  }
  return true;
}

bool NLPPathPlanner::check_trajectory(
  const Trajectory<double> & trajectory,
  const VehicleState<double> & initial_state,
  const VehicleState<double> & goal_state,
  const std::vector<Polytope2D<double>> & obstacles,
  const BicycleModelParameters<double> & model_parameters,
  const double tolerance
) const
{
  const auto model = BicycleModel<double, double>(model_parameters);

  // NOTE I'm aware the below closures serialize the same thing a bunch of times,
  // but it's more readable this way and A* + NLP take orders of magnitude more
  // computational effort anyway.

  // Create a single-step dynamics checking function
  const auto check_dynamics =
    [&model, tolerance](auto next_state, auto current_state,
      auto current_command) -> bool {
      const auto computed_state = model.integrated_dynamics(
        current_state, current_command,
        INTEGRATION_STEP_SIZE, NUMBER_OF_INTEGRATION_STEPS);
      const auto next_serialized = next_state.serialize();
      const auto computed_serialized = computed_state.serialize();
      if (!are_vectors_close(next_serialized, computed_serialized, tolerance) ) {
        return false;
      }
      return true;
    };

  // Create a single-step bounds checking function
  const auto check_bounds = [](auto current, auto lower, auto upper) -> bool {
      const auto current_serialized = current.serialize();
      const auto lower_serialized = lower.serialize();
      const auto upper_serialized = upper.serialize();

      for (std::size_t k = {}; k < current_serialized.size(); ++k) {
        if ((current_serialized[k] < lower_serialized[k]) ||
          (current_serialized[k] > upper_serialized[k]) )
        {
          return false;
        }
      }
      return true;
    };

  const auto check_absence_of_collisions = [&obstacles, &model](auto state) {
      for (const auto & obstacle : obstacles) {
        if (obstacle.intersects_with(model.compute_bounding_box(state)) ) {
          return false;
        }
      }
      return true;
    };


  // Go through trajectory checking dynamics and bounds
  for (std::size_t k = {}; k < trajectory.size() - 1; k++) {
    if (!check_dynamics(
        trajectory[k + 1].get_state(), trajectory[k].get_state(),
        trajectory[k].get_command() ) )
    {
      std::cout << "dynamics violated in step " << k << std::endl;
      return false;
    }

    if (!check_bounds(trajectory[k].get_state(), m_lower_state_bounds, m_upper_state_bounds)) {
      std::cout << "state bounds violated in step " << k << std::endl;
      return false;
    }

    if (!check_bounds(
        trajectory[k].get_command(), m_lower_command_bounds,
        m_upper_command_bounds))
    {
      std::cout << "command bounds violated in step " << k << std::endl;
      return false;
    }

    if (!check_absence_of_collisions(trajectory[k].get_state()) ) {
      std::cout << "collision in step " << k << std::endl;
      return false;
    }
  }

  // Check that initial and final state are what we expect them to be
  if (!are_vectors_close(
      trajectory[0].get_state().serialize(),
      initial_state.serialize(), tolerance) ||
    !are_vectors_close(
      trajectory.back().get_state().serialize(),
      goal_state.serialize(), tolerance)  )
  {
    return false;
  }

  return true;
}

// --- Actual planning functionality --------------------------------------------------
NLPResults NLPPathPlanner::plan_nlp(
  const VehicleState<double> & current_state,
  const VehicleState<double> & goal_state,
  const Trajectory<double> & initial_guess,
  const std::vector<Polytope2D<double>> & obstacles,
  const BicycleModelParameters<double> & model_parameters
) const
{
  // Assemble solver inputs
  std::vector<NLPObstacle<double>> nlp_obstacles{};
  try {
    nlp_obstacles = create_obstacles_from_polyhedra(obstacles, current_state);
  } catch (const std::length_error & e) {
    std::cout << e.what() << std::endl;
    return NLPResults{Trajectory<float64_t>{}, casadi::Dict{}};
  }
  auto p = assemble_parameter_vector(
    current_state, goal_state, model_parameters, nlp_obstacles, m_cost_weights);
  auto vars_and_bounds = assemble_variable_vector_and_bounds(
    initial_guess, nlp_obstacles,
    m_lower_state_bounds, m_upper_state_bounds, m_lower_command_bounds, m_upper_command_bounds);
  auto constraint_bounds = create_constraint_bounds();

  // NOTE set print_level to higher for debugging purposes. The hessian approximation has to be
  // kept the same as in generate_nlp_planner_solver, because different settings lead to different
  // callbacks being created.
  casadi::Dict ipopt_options = {{"hessian_approximation", "limited-memory"}, {"print_level", 0},
    {"max_iter", 500}};
  casadi::Function solver = casadi::nlpsol(
    "solver", "ipopt", SHARED_LIBRARY_DIRECTORY + "/" +
    "libparking_planner_callbacks.so", {{"ipopt", ipopt_options}});

  // Call solver with the assembled data
  casadi::DMDict res = solver(
    casadi::DMDict{
            {"x0", vars_and_bounds.variables},
            {"p", p},
            {"ubg", constraint_bounds.upper},
            {"lbg", constraint_bounds.lower},
            {"ubx", vars_and_bounds.upper_bounds},
            {"lbx", vars_and_bounds.lower_bounds},
          });

  // Get some info about the solve
  auto stats = solver.stats();

  // Extract and return results
  std::vector<double> elements = res["x"].get_elements();
  auto resulting_trajectory = disassemble_variable_vector(elements, HORIZON_LENGTH);
  return NLPResults{resulting_trajectory, stats};
}

}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware
