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

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
