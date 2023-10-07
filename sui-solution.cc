#include "search-strategies.h"
#include <queue>
#include <set>
#include <stack>
#include <memory>
#include <algorithm>

struct bfsMapping {
	std::shared_ptr<SearchState> state;
	std::shared_ptr<SearchAction> action;	

	std::shared_ptr<bfsMapping> prevNode;
	bfsMapping(const SearchState state) : state(std::make_shared<SearchState>(state)), action(nullptr), prevNode(nullptr) {}
	bfsMapping(const SearchState state, const SearchAction action, const bfsMapping prevNode) : 
	state(std::make_shared<SearchState>(state)), 
	action(std::make_shared<SearchAction>(action)), 
	prevNode(std::make_shared<bfsMapping>(prevNode)) {}
	bfsMapping() : state(nullptr), action(nullptr), prevNode(nullptr) {}
};

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	std::queue<bfsMapping> frontier;
	std::set<SearchState> explored;

	frontier.push(init_state);

	while (!frontier.empty()) {
		bfsMapping current = frontier.front();
		frontier.pop();
	
		if (current.state->isFinal()) {
			std::vector<SearchAction> actions = {};

			while (current.prevNode) {
				actions.push_back(*current.action);
				current = *current.prevNode;
			}
			//return reversed actions because we are going from final state to initial state
			std::reverse(actions.begin(), actions.end());
			
			return actions;
		}

		if (explored.find(*current.state) == explored.end()) {
			std::shared_ptr<SearchAction> newAction;

			for (const SearchAction &action : current.state->actions()) {
				SearchState newState(action.execute(*current.state));
				bfsMapping newNode(newState, action, current); 
				frontier.push(newNode);
			}
			explored.insert(*current.state);
		}
	}

	return {};
}


struct dfsMapping {
	SearchState state;
	SearchAction action;
	int depth;
};

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	std::stack<dfsMapping> frontier;
	std::set<SearchState> explored;
	std::vector<SearchAction> solution;

	SearchState working_state(init_state);
	frontier.push({working_state, working_state.actions()[0], 0});

	while (!frontier.empty()) {
		SearchState current(frontier.top().state);
		SearchAction action(frontier.top().action);
		int depth = frontier.top().depth;
		frontier.pop();

		while (solution.size() >= depth && depth != 0) {
			solution.pop_back();
		}

		solution.push_back(action);
		
		if (current.isFinal()) {
			return solution;
		}

		if (depth >= depth_limit_) {
			continue;
		}

		if (explored.find(current) == explored.end()) {
			for (const SearchAction &action : current.actions()) {
				auto newState = action.execute(current);
				frontier.push({newState, action, depth + 1});
			}
			explored.insert(current);
		}
	}

	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

struct AStarFrontierItem {
	std::shared_ptr<SearchState> state;
	std::shared_ptr<SearchAction> action;

	int depth;	
	double f_value;

	std::shared_ptr<AStarFrontierItem> prevNode;

	AStarFrontierItem(const SearchState state) : state(std::make_shared<SearchState>(state)), action(nullptr), depth(0), f_value(0.0), prevNode(nullptr) {}
	AStarFrontierItem(const SearchState state, const SearchAction action, const int depth, const double h_value, const AStarFrontierItem prevNode) : 
	state(std::make_shared<SearchState>(state)), 
	action(std::make_shared<SearchAction>(action)), 
	f_value((depth+1.0) + h_value),
	depth(depth + 1),
	prevNode(std::make_shared<AStarFrontierItem>(prevNode)) {}
	AStarFrontierItem() : state(nullptr), action(nullptr), f_value(MAXFLOAT), depth(0), prevNode(nullptr) {}

	bool operator<(const AStarFrontierItem& other) const { //guarantees that the states will be ordered by f_value in frontier
        return f_value < other.f_value;
    }
};

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	std::multiset<AStarFrontierItem> frontier;
	std::set<SearchState> explored;
	std::vector<SearchAction> solution;

	frontier.insert(init_state);

	while (!frontier.empty()) {
		AStarFrontierItem current = *(frontier.begin());
		frontier.erase(frontier.begin());

		if (current.state->isFinal()) {
			std::vector<SearchAction> actions = {};

			while (current.prevNode) {
				actions.push_back(*current.action);
				current = *current.prevNode;
			}
			//return reversed actions because we are going from final state to initial state
			std::reverse(actions.begin(), actions.end());
			
			return actions;
		}

		if (explored.find(*current.state) == explored.end()) {
			for (const SearchAction &action : current.state->actions()) {
				SearchState newState(action.execute(*current.state));
				AStarFrontierItem newNode(newState, action, compute_heuristic(newState, *(this->heuristic_)), current.depth, current); 
				frontier.insert(newNode);
			}
			explored.insert(*current.state);
		}
	}

	return {};
}
