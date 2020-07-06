/* This hpp file defines MaxSuccess Greedy search on a given labeled graph
with specified start and goal set */

#include <vector>
#include <iostream>
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <string> // std::string, std::to_string
#include <cstdlib> // std::rand, std::srand
#include <iterator>

#include "Graph.hpp"
#include "MaxSuccessGreedySolver.hpp"
#include "Timer.hpp"

MaxSuccessGreedySolver_t::MaxSuccessGreedySolver_t(Graph_t &g)
{
	// initialize the start & goalSet
	m_start = g.getStart();
	m_goalSet = g.getGoalSet();
	m_targetPoses = g.getTargetPoses();
	for (int i=0; i < m_goalSet.size(); i++)
	{
		m_goalmap[m_goalSet[i]] = m_targetPoses[i];
	}
	m_labelWeights = g.getLabelWeights();
	countObsAndGoalHypos();
	// computeGoalMean(g); // heuristics
	std::vector<int> temp_goalIdxes = m_goalhypos; // for the start, all the goals are available

	m_G = std::vector<float>(g.getnNodes(), std::numeric_limits<float>::max());
	m_G[m_start] = 0.0;

	for (auto const &gg: m_goalSet)
	{
		m_GForGoals[gg] = std::numeric_limits<float>::max();
		m_highestSuccessForGoals[gg] = -1.0;
		m_expandedForGoals[gg] = false;
	}
	m_highestSuccess = std::vector<float>(g.getnNodes(), -1.0);
	// std::cout << "The results coming from compute Reachability: " << computeReach(temp_goalIdxes) << "\n";
	m_highestSuccess[m_start] = computeSurvival({}) * computeReach(temp_goalIdxes);

	m_open.push( new MaxSuccGreedyNode_t(m_start, 0.0,  {}, computeSurvival({}), 
		temp_goalIdxes, computeReach(temp_goalIdxes), false, nullptr) );

	m_expanded = std::vector<bool>(g.getnNodes(), false);


	m_isFailure = false;

	// std::map<int, bool> m_expandedForGoals;
	// // // print m_expandedForGoals
	// std::cout << "m_expandedForGoals\n";
	// std::vector<int> currentIDs{301, 302, 303, 304, 305, 306, 307, 308, 309};

	// for (auto const &cids: currentIDs)
	// {
	// 	if (cids % 2 == 0)
	// 	{
	// 		m_expandedForGoals[cids] = 1;
	// 	}
	// 	else
	// 	{
	// 		m_expandedForGoals[cids] = 0;
	// 	}
 
	// }

	// for (auto const &cids: currentIDs)
	// {
	// 	std::cout << "getIndex: " << getIndex(cids) << "\n";
	// 	std::cout << "expand or not: " << m_expandedForGoals[getIndex(cids)] << "\n\n";
	// 	// std::cout << "expand or not: " << m_expandedForGoals[cids] << "\n\n";

	// }

}

MaxSuccessGreedySolver_t::~MaxSuccessGreedySolver_t()
{
	while (!m_open.empty())
	{
		MaxSuccGreedyNode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }
}

void MaxSuccessGreedySolver_t::MSGreedy_search(Graph_t &g)
{
	float neighbor_g;
	std::vector<int> neighborLabels;
	float neighborSurvival;
	std::vector<int> neighbor_goalIdxes;
	float neighbor_reachability;
	float neighbor_successValue;

	while (!m_open.empty())
	{
		MaxSuccGreedyNode_t *current = m_open.top();
		m_open.pop();


		// Now check if the current node has been expanded
		// If it is a non-goal node, check m_expanded
		// If it is a goal node, check m_expandedForGoals
		if (current->m_isGoal == false and m_expanded[current->m_id] == true)
		{
			// No need to put it into the closed list
			delete current;
			continue;
		}
		if (current->m_isGoal == true and m_expandedForGoals[current->m_id] == true)
		{
			// No need to put it into the closed list
			delete current;
			continue;		
		}

		// std::cout << "pop out current node: " << current->m_id << "  " << current->m_f << "\n";
		// std::cout << "[ ";
		// for (auto const &l: current->m_labels)
		// {
		// 	std::cout << l << " ";
		// }
		// std::cout << "]  ,";
		// std::cout << "[ ";
		// for (auto const &gi : current->m_goalIndexes)
		// {
		// 	std::cout << gi << " ";
		// }
		// std::cout << "]\n";
		// if (current->m_parent != nullptr)
		// {
		// 	std::cout << current->m_survival << ", " << current->m_reachability << ", " 
		// 					<< current->m_successValue << ", " << current->m_isGoal 
		// 												<< ", " << (current->m_parent)->m_id << "\n";
		// }
		// else
		// {
		// 	std::cout << current->m_survival << ", " << current->m_reachability << ", " 
		// 					<< current->m_successValue << ", " << current->m_isGoal 
		// 												<< ", " << "start" << "\n";
		// }

		// // std::cout << "press any key to continue...\n";
		// // std::cin.get();
		m_closed.push_back(current);
		if (current->m_isGoal == false)
		{
			m_expanded[current->m_id] = true;			
		}
		else
		{
			m_expandedForGoals[current->m_id] = true;
		}

		if (current->m_isGoal == true)
		{
			std::cout << "MSG PATH FOUND\n";
			back_track_path(); // construct your path
			pathToTrajectory(g);
			m_goalLabels = current->m_labels;
			m_survivability = current->m_survival;
			m_goalIdxReached = current->m_goalIndexes;
			m_pathCost = current->m_g;

			m_pathSuccessValue = current->m_successValue;
			// std::cout << "The reaching target pose is: " << m_goalIdxReached << "\n";
			// print the survivability, reachability and success value for the goal
			// std::cout << "survival: " << current->m_survival << ", reachable: " 
			// 		<< current->m_reachability << ", success: " << current->m_successValue << "\n";
			// std::cout << "The cost: " << m_pathCost << "\n";

			return;
		}
		// If it is not the goal, let's keep moving
		// look at each neighbor of the current node
		std::vector<int> neighbors = g.getNodeNeighbors(current->m_id);
		for (auto const &neighbor : neighbors)
		{
			// check neighbor's labels and the survivability
			neighborLabels = 
				label_union(current->m_labels, g.getEdgeLabels(current->m_id, neighbor));
			neighborSurvival = computeSurvival(neighborLabels);	
			neighbor_goalIdxes = update_goalIdxes(current->m_goalIndexes, 
											g.getEdgeLabels(current->m_id, neighbor));
			/// treat it as a normal node first ///
			// check if the neighbor has been expanded before
			if (!m_expanded[neighbor])
			{
				// get the reachability and the success value
				neighbor_reachability = computeReach(neighbor_goalIdxes);
				neighbor_successValue = neighborSurvival * neighbor_reachability;

				// only add the node if either (1) it has a better successValue or
				// (2) it has a tied best successValue but with a smaller cost
				if (neighbor_successValue > m_highestSuccess[neighbor])
				{
					// std::cout << "current neighbor: " << neighbor << "\n";
					m_highestSuccess[neighbor] = neighbor_successValue;
					neighbor_g = current->m_g + g.getEdgeCost(current->m_id, neighbor);
					m_G[neighbor] = neighbor_g;
					// other things to compute before being pushed to open list
					// neighbor_h = computeH(g.getState(neighbor), neighbor_goalIdxes);
					// m_F[neighbor] = neighbor_g + neighbor_h;
					// Now ready to add to open list
					// std::cout << "start to add the current neighbor into open list\n";
					m_open.push( new MaxSuccGreedyNode_t(neighbor, neighbor_g, neighborLabels, neighborSurvival, 
						neighbor_goalIdxes, neighbor_reachability, false, current) );
					// std::cout << "higher success\n";
					// std::cout << "add neighbor: " << neighbor << "  " << neighbor_g + neighbor_h << "\n";
					// std::cout << "[ ";
					// for (auto const &l: neighborLabels)
					// {
					// 	std::cout << l << " ";
					// }
					// std::cout << "]  ,";
					// std::cout << "[ ";
					// for (auto const &gi : neighbor_goalIdxes)
					// {
					// 	std::cout << gi << " ";
					// }
					// std::cout << "]\n";
					// std::cout << neighborSurvival << ", " << neighbor_reachability << ", " 
					// 				<< neighbor_successValue << ", " << "false" 
					// 											<< ", " << current->m_id << "\n\n";
				}
				else if (neighbor_successValue == m_highestSuccess[neighbor])
				{
					// std::cout << "current neighbor: " << neighbor << "\n";
					neighbor_g = current->m_g + g.getEdgeCost(current->m_id, neighbor);
					// neighbor_h = computeH(g.getState(neighbor), neighbor_goalIdxes);
					if ( neighbor_g < m_G[neighbor] )
					{
						m_G[neighbor] = neighbor_g;
						m_open.push( new MaxSuccGreedyNode_t(neighbor, neighbor_g, neighborLabels, neighborSurvival, 
							neighbor_goalIdxes, neighbor_reachability, false, current) );
						// std::cout << "tie success\n";
						// std::cout << "add neighbor: " << neighbor << "  " << neighbor_g + neighbor_h << "\n\n";
						// std::cout << "[ ";
						// for (auto const &l: neighborLabels)
						// {
						// 	std::cout << l << " ";
						// }
						// std::cout << "]  ,";
						// std::cout << "[ ";
						// for (auto const &gi : neighbor_goalIdxes)
						// {
						// 	std::cout << gi << " ";
						// }
						// std::cout << "]\n";
						// std::cout << neighborSurvival << ", " << neighbor_reachability << ", " 
						// 				<< neighbor_successValue << ", " << "false" 
						// 											<< ", " << current->m_id << "\n\n";
					}
				}
				

			}

			/// check if it is a goal node ///
			if ( std::find(m_goalSet.begin(), m_goalSet.end(), neighbor) != m_goalSet.end() )
			{
				// check the remaining valid poses of that goal node
				std::vector<int> remainingPoses;
				remainingPoses = cal_remainingPoses(neighbor_goalIdxes, m_goalmap[neighbor]);
				if ( remainingPoses.size() == 0)
				{
					// it is a goal node, but it does not mean that it is a goal if the 
					// neighbor_goalIdxes does not contain the goal the goal node is associated
					// with, it is NOT a goal
					continue;
				}

				// Now it's time to treat it as a goal node
				// check if the neighbor has been expanded before
				if (!m_expandedForGoals[neighbor])
				{
					neighbor_reachability = computeReach(remainingPoses);
					neighbor_successValue = neighborSurvival * neighbor_reachability;
					// only add the node if either (1) it has a better successValue or
					// (2) it has a tied best successValue but with a smaller cost
					if (neighbor_successValue > m_highestSuccessForGoals[neighbor])
					{
						m_highestSuccessForGoals[neighbor] = neighbor_successValue;

						neighbor_g = current->m_g + g.getEdgeCost(current->m_id, neighbor);
						m_GForGoals[neighbor] = neighbor_g;
						// Now ready to add to open list
						m_open.push( new MaxSuccGreedyNode_t(neighbor, neighbor_g, neighborLabels, neighborSurvival, 
										remainingPoses, neighbor_reachability, true, current) );
						// std::cout << "higher success\n";
						// std::cout << "add a goal to the open list: " << neighbor << " for pose " 
						// 			<< m_goalmap[neighbor] << ": " << neighborSurvival << ", " 
						// 				<< neighbor_reachability << ", " 
						// 					<< neighborSurvival * neighbor_reachability << "\n";
						// std::cout << "add neighbor: " << neighbor << "  " << neighbor_g + neighbor_h << "\n";
						// std::cout << "[ ";
						// for (auto const &l: neighborLabels)
						// {
						// 	std::cout << l << " ";
						// }
						// std::cout << "]  ,";
						// std::cout << "[ ";
						// for (auto const &gi : std::vector<int>(1, m_goalmap[neighbor]))
						// {
						// 	std::cout << gi << " ";
						// }
						// std::cout << "]\n";
						// std::cout << neighborSurvival << ", " << neighbor_reachability << ", " 
						// 				<< neighbor_successValue << ", " << "true" 
						// 											<< ", " << current->m_id << "\n\n";
					}
					else if (neighbor_successValue == m_highestSuccessForGoals[neighbor])
					{
						neighbor_g = current->m_g + g.getEdgeCost(current->m_id, neighbor);
						if ( neighbor_g < m_GForGoals[neighbor] )
						{
							m_GForGoals[neighbor] = neighbor_g;
							m_open.push( new MaxSuccGreedyNode_t(neighbor, neighbor_g, neighborLabels, neighborSurvival, 
											remainingPoses, neighbor_reachability, true, current) );
							// std::cout << "tied success\n";
							// std::cout << "add a goal to the open list: " << neighbor << " for pose " 
							// 		<< m_goalmap[neighbor] << ": " << neighborSurvival << ", " 
							// 			<< neighbor_reachability << ", " 
							// 				<< neighborSurvival * neighbor_reachability << "\n";
							// std::cout << "add neighbor: " << neighbor << "  " << neighbor_g + neighbor_h << "\n";
							// std::cout << "[ ";
							// for (auto const &l: neighborLabels)
							// {
							// 	std::cout << l << " ";
							// }
							// std::cout << "]  ,";
							// std::cout << "[ ";
							// for (auto const &gi : std::vector<int>(1, m_goalmap[neighbor]))
							// {
							// 	std::cout << gi << " ";
							// }
							// std::cout << "]\n";
							// std::cout << neighborSurvival << ", " << neighbor_reachability << ", " 
							// 				<< neighbor_successValue << ", " << "true" 
							// 											<< ", " << current->m_id << "\n\n";
						}
					}
			

				}

			}
			
		}
		// std::cout << "press any key to continue...\n";
		// std::cin.get(); 

	}
	// You are reaching here since the open list is empty and the goal is not found
	std::cout << "The problem is not solvable. Search failed...\n";
	m_isFailure = true;
}

std::vector<int> MaxSuccessGreedySolver_t::cal_remainingPoses(std::vector<int> neighbor_goalIdxes, std::vector<int> goalposeSet)
{
	std::vector<int> remainingPoses;
	for (auto const &p : goalposeSet)
	{
		if ( std::find(neighbor_goalIdxes.begin(), neighbor_goalIdxes.end(), p) != neighbor_goalIdxes.end() )
		{
			remainingPoses.push_back(p);
		}
	}
	return remainingPoses;
}

// void MaxSuccessGreedySolver_t::checkPathSuccess(int nhypo)
// {
// 	m_obstaclesCollided = 0;
// 	m_isPathSuccess = true;
// 	// compute the obstacles collided
// 	// loop through the m_goalLabels
// 	for (auto const &l : m_goalLabels)
// 	{
// 		if (l % nhypo == 0)
// 		{
// 			m_obstaclesCollided += 1;
// 		}
// 	}
// 	if (m_obstaclesCollided != 0 or m_goalIdxReached != 0)
// 	{
// 		m_isPathSuccess = false;
// 	}

// }

std::vector<int> MaxSuccessGreedySolver_t::update_goalIdxes(std::vector<int> currGoalIndexes, 
																		std::vector<int> edgelabels)
{
	std::vector<int> newGoalIndexes;
	// loop through all current goal indexes
	for (auto const &gi : currGoalIndexes)
	{
		if ( std::find(edgelabels.begin(), edgelabels.end(), gi) == edgelabels.end() )
		{
			newGoalIndexes.push_back(gi);
		}
	}

	return newGoalIndexes;
}

void MaxSuccessGreedySolver_t::countObsAndGoalHypos()
{
	m_nobstacles = m_labelWeights[m_labelWeights.size()-1].first + 1;
	for (int ii=0; ii < m_labelWeights.size(); ii++)
	{
		if (m_labelWeights[ii].first == 0)
		{
			m_goalhypos.push_back(ii);
		}
		else
		{
			break;
		}
	}
	// print for testing purposes
	// std::cout << "m_nobstacles: " << m_nobstacles << "\n";
	// std::cout << "m_goalhypos: \n";
	// for (auto const &gh : m_goalhypos)
	// {
	// 	std::cout << gh << " ";
	// }


}

// void MaxSuccessGreedySolver_t::computeGoalMean(Graph_t &g)
// {
// 	// first figure out m_goalhypos (how many unique goal hypos)
// 	std::vector<int>::iterator ip;
// 	m_goalhypos = m_targetPoses;
// 	sort(m_goalhypos.begin(), m_goalhypos.end());
// 	m_goalhypos.resize(std::distance(m_goalhypos.begin(), 
// 											std::unique(m_goalhypos.begin(), m_goalhypos.end())));
// 	// initialization for m_goalCounts and m_goalMean
// 	for (int ii=0; ii < m_goalhypos.size(); ii++)
// 	{
// 		m_goalCounts[m_goalhypos[ii]] = 0;
// 		m_goalMean[m_goalhypos[ii]] = std::vector<float>(g.getState(0).size(), 0.0);
// 	}

// 	// loop through all goals and their correpsonding state values into the right category
// 	for (int ii=0; ii < m_goalSet.size(); ii++)
// 	{
// 		std::vector<float> temp_state = g.getState(m_goalSet[ii]);
// 		m_goalCounts[m_targetPoses[ii]] += 1;
// 		for (int jj=0; jj < temp_state.size(); jj++)
// 		{
// 			m_goalMean[m_targetPoses[ii]][jj] += temp_state[jj];
// 		}
// 	}

// 	for (int ii=0; ii < m_goalhypos.size(); ii++)
// 	{
// 		for (int jj=0; jj < m_goalMean[ii].size(); jj++)
// 		{
// 			m_goalMean[ii][jj] /= m_goalCounts[ii];
// 		}
// 	}

// }

// float MaxSuccessGreedySolver_t::computeH(std::vector<float> temp_state, std::vector<int> goalIdxes)
// {
// 	// Initialize temp_hs
// 	std::map<int, float> temp_hs;
// 	for (auto const &gi : goalIdxes)
// 	{
// 		temp_hs[gi] = 0.0;
// 	}
// 	// fill in temp_hs
// 	for (auto const &gi : goalIdxes)
// 	{
// 		for (int qq=0; qq < temp_state.size(); qq++)
// 		{
// 			temp_hs[gi] += pow(temp_state[qq] - m_goalMean[gi][qq], 2);
// 		}

// 	}
// 	// compute the weighted heurisitic
// 	float temp_h = 0.0;
// 	for (auto const &th : temp_hs)
// 	{
// 		temp_h += sqrt(th.second) * m_labelWeights[th.first].second;
// 	}

// 	return temp_h;
// }

float MaxSuccessGreedySolver_t::computeSurvival(std::vector<int> labels)
{
	float survival = 1.0;
	std::vector<float> CollisionPerObs(m_nobstacles, 0.0);
	for (auto const &label : labels)
	{
		CollisionPerObs[m_labelWeights[label].first] += m_labelWeights[label].second;
	}
	// compute survival based on CollisionPerObs
	for (auto const &collision_prob : CollisionPerObs)
	{
		survival *= (1 - collision_prob); // obstacles independent assumption
	}

	return survival;
}

float MaxSuccessGreedySolver_t::computeReach(std::vector<int> goalIdxes)
{
	float MaxReach = 0.0;
	for (auto const &gi : goalIdxes)
	{
		MaxReach += m_labelWeights[gi].second;
	}

	return MaxReach;

	// float MaxReach = 0.0;
	// float temp_reach;
	// for (auto const &gi : goalIdxes)
	// {
	// 	temp_reach = m_labelWeights[gi].second;
	// 	if (MaxReach < temp_reach)
	// 	{
	// 		MaxReach = temp_reach;
	// 	}
	// }

	// return MaxReach;
}

void MaxSuccessGreedySolver_t::print_path()
{
	// print the path for checking purpose
	std::cout << "path: \n";
	for (int i=m_path.size()-1; i >=0; i--)
	{
		std::cout << m_path[i] << " ";
	}
	// for (auto const &waypoint : m_path)
	// {
	// 	std::cout << waypoint << " ";
	// }
	std::cout << "\n";
}

void MaxSuccessGreedySolver_t::printLabels()
{
	std::cout << "labels: " << "< ";
	for (auto const &l : m_goalLabels)
	{
		std::cout << l << " ";
	}
	std::cout << ">\n";	
}

void MaxSuccessGreedySolver_t::print_cost()
{
	std::cout << "cost: " << m_pathCost << "\n";
}
		
void MaxSuccessGreedySolver_t::print_goalIdxReached()
{
	std::cout << "The reaching target poses are: "; 
	for (auto const &p : m_goalIdxReached)
	{
		std::cout << p << " ";
	}
	std::cout << "\n";
}

void MaxSuccessGreedySolver_t::print_survivability()
{
	std::cout << "survivability: " << m_survivability << "\n";
}

void MaxSuccessGreedySolver_t::print_successValue()
{
	std::cout << "successValue: " << m_pathSuccessValue << "\n";
}


void MaxSuccessGreedySolver_t::printAll()
{
	print_path();
	print_cost();
	printLabels();
	print_goalIdxReached();
	print_survivability();
	print_successValue();
}

std::vector<int> MaxSuccessGreedySolver_t::label_union(std::vector<int> s1, std::vector<int> s2)
{
	// sort the sets first before applying union operation
	std::sort(s1.begin(), s1.end());
	std::sort(s2.begin(), s2.end());

	// Declaring resultant vector for union
	std::vector<int> v(s1.size()+s2.size());
	// using function set_union() to compute union of 2
	// containers v1 and v2 and store result in v
	auto it = std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(), v.begin());

	// resizing new container
	v.resize(it - v.begin());
	return v;
}

int MaxSuccessGreedySolver_t::getIndex(int goalIdx)
{
	int temp_idx = -1;
	for (int ii=0; ii < m_goalSet.size(); ii++)
	{
		if (goalIdx == m_goalSet[ii])
		{
			temp_idx = ii;
			break;
		}
	}
	return temp_idx;
}


void MaxSuccessGreedySolver_t::back_track_path()
{
	// start from the goal
	MaxSuccGreedyNode_t *current = m_closed[m_closed.size()-1];
	while (current->m_id != m_start)
	{
		// keep backtracking the path until you reach the start
		m_path.push_back(current->m_id);
		current = current->m_parent;
	}
	// finally put the start into the path
	m_path.push_back(current->m_id);

	// print the path for checking purpose
	// std::cout << "path: \n";
	// for (auto const &waypoint : m_path)
	// {
	// 	std::cout << waypoint << " ";
	// }
	// std::cout << "\n";
}

void MaxSuccessGreedySolver_t::pathToTrajectory(Graph_t &g)
{
	// start from the start
	for (int i=m_path.size()-1; i >=0; i--)
	{
		m_trajectory.push_back(g.getState(m_path[i]));

	}
	// // print the trajectory for checking purpose
	// std::cout << "The trajectory: \n";
	// for (auto const &t : m_trajectory)
	// {
	// 	for (auto const &d : t)
	// 	{
	// 		std::cout << d << "   ";
	// 	}
	// 	std::cout << "\n";
	// }
}

void MaxSuccessGreedySolver_t::writeTrajectory(std::string trajectory_file)
{
	m_outFile_.open(trajectory_file);
	if (m_outFile_.is_open())
	{
		for (auto const &t : m_trajectory)
		{
			for (auto const &d : t)
			{
				m_outFile_ << d << " ";
			}
			m_outFile_ << "\n";
		}
	}
	m_outFile_.close();
}