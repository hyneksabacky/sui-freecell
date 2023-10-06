#include "search-strategies.h"
#include <queue>
#include <set>
#include <stack>

struct bfsMapping {
	SearchState prevState;
	SearchAction action;
	bool head;
};

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	std::queue<SearchState> frontier;
	std::vector<SearchAction> solution;
	std::set<SearchState> explored;
	std::map<SearchState, std::vector<SearchAction>> stateMap;

	SearchState working_state(init_state);
	frontier.push(working_state);

	stateMap[working_state] = {};

	while (!frontier.empty()) {
		SearchState current(frontier.front());
		frontier.pop();
		
		if (current.isFinal()) {
			return stateMap[current];
		}

		if (explored.find(current) == explored.end()) {
			for (const SearchAction &action : current.actions()) {
				auto newState = action.execute(current);
				frontier.push(newState);
				
				auto newActions = stateMap[current];
				newActions.push_back(action);
				stateMap[newState] = newActions;
			}
			explored.insert(current);
		}
	}

	return {};
}

// std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
// 	std::queue<SearchState> frontier;
// 	std::vector<SearchAction> solution;
// 	std::set<SearchState> explored;
// 	std::map<SearchState, bfsMapping> stateMap;

// 	SearchState working_state(init_state);
// 	frontier.push(working_state);

// 	stateMap[working_state] = {working_state, working_state.actions()[0], true};

// 	while (!frontier.empty()) {
// 		SearchState current(frontier.front());
// 		frontier.pop();
		
// 		if (current.isFinal()) {
			
// 			std::vector<SearchAction> actions = {};
// 			bfsMapping currentMapping = stateMap[current];
// 			SearchState = SearchState
// 			while (!currentMapping.head) {
// 				auto newMapping(stateMap[currentMapping.prevState]);
// 				actions.push_back(currentMapping.action);
// 				currentMapping = stateMap[currentMapping.prevState];
// 			}
// 			return actions;
// 		}

// 		if (explored.find(current) == explored.end()) {
// 			for (const SearchAction &action : current.actions()) {
// 				auto newState = action.execute(current);
// 				frontier.push(newState);
// 				stateMap[newState] = {current, action, false};
// 			}
// 			explored.insert(current);
// 		}
// 	}

// 	return {};
// }


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
