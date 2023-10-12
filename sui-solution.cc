/**
 * @file sui-solution.cc
 * @brief Implementation of search strategies for solving a problem using various algorithms.
 *
 * This file contains the implementation of three search strategies: Breadth-First Search (BFS),
 * Depth-First Search (DFS), and A* Search. These strategies are used to find a solution of FreeCell game
 *
 * @author Mikuláš Brázda (xbrazd21@stud.fit.vutbr.cz), Hynek Šabacký (xsabac02@stud.fit.vutbr.cz)
 * @date 10.10.2023
 */

#include "search-strategies.h"
#include <queue>
#include <set>
#include <stack>
#include <memory>
#include <algorithm>
#include "memusage.h"

const unsigned MEM_LIMIT = 50'000'000;

struct bfsNode {
	std::shared_ptr<SearchState> state;
	std::shared_ptr<SearchAction> action;	

	std::shared_ptr<bfsNode> prevNode;
	bfsNode(const SearchState state) : state(std::make_shared<SearchState>(state)), action(nullptr), prevNode(nullptr) {}
	bfsNode(const SearchState state, const SearchAction action, const bfsNode prevNode) : 
	state(std::make_shared<SearchState>(state)), 
	action(std::make_shared<SearchAction>(action)), 
	prevNode(std::make_shared<bfsNode>(prevNode)) {}
	bfsNode() : state(nullptr), action(nullptr), prevNode(nullptr) {}
};

struct dfsNode {
	SearchState state;
	SearchAction action;
	int depth;
};

struct AStarNode {
	std::shared_ptr<SearchState> state;
	std::shared_ptr<SearchAction> action;

	int depth;	
	double f_value;

	std::shared_ptr<AStarNode> prevNode;
	
	AStarNode(const SearchState state) : state(std::make_shared<SearchState>(state)), action(nullptr), depth(0), f_value(0.0), prevNode(nullptr) {}
	AStarNode(const SearchState state, const SearchAction action, const int depth, const double h_value, const AStarNode prevNode) : 
	state(std::make_shared<SearchState>(state)), 
	action(std::make_shared<SearchAction>(action)), 
	depth(depth + 1),
	f_value((depth+1.0) + h_value),
	prevNode(std::make_shared<AStarNode>(prevNode)) {}
	AStarNode() : state(nullptr), action(nullptr), depth(0), f_value(0.0), prevNode(nullptr) {}

	// guarantees that the states will be ordered by f_value in frontier
	bool operator<(const AStarNode& other) const { 
        return f_value < other.f_value;
    }
};

/**
 * @brief Solve a problem using Breadth-First Search (BFS).
 *
 * This method uses the Breadth-First Search algorithm to find a solution to a problem
 * starting from the given initial state.
 *
 * @param init_state The initial state of the problem.
 * @return A vector of SearchAction objects representing the sequence of actions to reach the solution.
 */
std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	std::queue<bfsNode> frontier;
	std::set<SearchState> explored;

	if (init_state.isFinal())
		return {};

	frontier.push(init_state);

	while (!frontier.empty() and (getCurrentRSS() + MEM_LIMIT < mem_limit_)) {
		bfsNode current = frontier.front();
		frontier.pop();


		if (explored.find(*current.state) == explored.end()) {
			std::shared_ptr<SearchAction> newAction;

			for (const SearchAction &action : current.state->actions()) {
				SearchState newState(action.execute(*current.state));
				
				bfsNode newNode(newState, action, current); 
				if (newNode.state->isFinal()) {
					std::vector<SearchAction> actions = {};

					while (newNode.prevNode) {
						actions.push_back(*newNode.action);
						newNode = *newNode.prevNode;
					}
					//return reversed actions because we are going from final state to initial state
					std::reverse(actions.begin(), actions.end());
					return actions;
				}
				frontier.push(newNode);
			}
			explored.insert(*current.state);
		}
	}

	return {};
}

/**
 * @brief Solve a problem using Depth-First Search (DFS).
 *
 * This method uses the Depth-First Search algorithm to find a solution to a problem
 * starting from the given initial state.
 *
 * @param init_state The initial state of the problem.
 * @return A vector of SearchAction objects representing the sequence of actions to reach the solution.
 */
std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	std::stack<dfsNode> frontier;
	std::vector<SearchAction> solution;
	int size;

	if (init_state.isFinal())
		return {};

	frontier.push({init_state, init_state.actions()[0], 0});

	while (!frontier.empty() ) {
		dfsNode current(frontier.top());
		frontier.pop();

		while ((size = solution.size()) >= current.depth && current.depth > 0) {
			solution.pop_back();
		}
		
		if (current.depth > 0)
			solution.push_back(current.action);


		for (auto &action : current.state.actions()) {  
			auto newState = action.execute(current.state);

			if (newState.isFinal()) {
				solution.push_back(action);
				return solution;
			}
			dfsNode newNode({newState, action, current.depth + 1});

			if (newNode.depth < depth_limit_) {
				frontier.push(newNode);
			}
		}
	}

	return {};
}

/**
 * @brief Calculate a lower bound for the distance in a game state.
 *
 * This method calculates a lower bound for the distance in a game state using a heuristic
 * inspired by https://ai.dmi.unibas.ch/papers/paul-helmert-icaps2016wshsdip.pdf. 
 * The heuristic considers 1-suit cycles and cards out of home. 1-suit cycles happen
 * when two cards of the same color are in the same stack and the card on top has a lower value.

 * @param state The game state for which to calculate the lower bound.
 * @return The calculated lower bound for the distance in the game state.
 */
double StudentHeuristic::distanceLowerBound(const GameState &state) const {

    // identify 1-suit cycles
	int cycleCount = 0;
	for (auto stack : state.stacks) {
		std::vector<Card> stackCards = stack.storage();

		for (unsigned int i = 0; i<stackCards.size(); i++) {
			auto cardX = stack.storage()[i];
			auto stackCardsRest = std::vector<Card>(stackCards.begin() + i, stackCards.end());

			for (auto cardY : stackCardsRest) {
				if (cardY.color == cardX.color && cardY.value > cardX.value)
					cycleCount++;
			}
		}
	}
	
	int cards_out_of_home = king_value * colors_list.size();
    for (const auto &home : state.homes) {
        auto opt_top = home.topCard();
        if (opt_top.has_value())
            cards_out_of_home -= opt_top->value;
    }

	// to normalize the heuristic so that cycleCount has bigger weight
	return cycleCount*3.0+cards_out_of_home;
}

/**
 * @brief Solve a problem using A* Search with a heuristic.
 *
 * This method uses the A* Search algorithm to find a solution to a problem
 * starting from the given initial state and using a provided heuristic.
 *
 * @param init_state The initial state of the problem.
 * @return A vector of SearchAction objects representing the sequence of actions to reach the solution.
 */
std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	std::multiset<AStarNode> frontier;
	std::map<SearchState, double> explored;
	std::vector<SearchAction> solution;

	if (init_state.isFinal())
		return {};

	frontier.insert(init_state);
	
	while (!frontier.empty() and (getCurrentRSS() + MEM_LIMIT < mem_limit_)) {
		AStarNode current = *(frontier.begin());
		frontier.erase(frontier.begin());

		if (explored.find(*current.state) == explored.end() || explored.find(*current.state)->second > current.f_value) {
			for (const SearchAction &action : current.state->actions()) {
				SearchState newState(action.execute(*current.state));
				AStarNode newNode(newState, action, compute_heuristic(newState, *(this->heuristic_)), current.depth, current); 
				if (newNode.state->isFinal()) {
					std::vector<SearchAction> actions = {};

					while (newNode.prevNode) {
						actions.push_back(*newNode.action);
						newNode = *newNode.prevNode;
					}
					//return reversed actions because we are going from final state to initial state
					std::reverse(actions.begin(), actions.end());
					return actions;
				}
				frontier.insert(newNode);
			}
			explored[*current.state] = current.f_value;
		} 
	}

	return {};
}
