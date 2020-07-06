/* This hpp file defines MaxSuccess Exact search on a given labeled graph
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
#include "MaxSuccessExactSolver.hpp"
#include "Timer.hpp"

MaxSuccessExactSolver_t::MaxSuccessExactSolver_t(Graph_t &g)
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
	 
	m_open.push( new MaxSuccExactNode_t(m_start, 0.0, {}, computeSurvival({}), 
		temp_goalIdxes, computeReach(temp_goalIdxes), false, nullptr) );

	m_visited = std::vector<bool>(g.getnNodes(), false);
	m_visited[m_start] = true;
	int n_nodes = g.getnNodes();
	for (int hh=0; hh < n_nodes; hh++)
	{
		m_recordSet.push_back(std::vector<std::vector<int>>());
	}

	m_isFailure = false;

}

MaxSuccessExactSolver_t::~MaxSuccessExactSolver_t()
{
	while (!m_open.empty())
	{
		MaxSuccExactNode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }
}


void MaxSuccessExactSolver_t::MSExact_search(Graph_t &g)
{
	float neighbor_g;
	std::vector<int> neighborLabels;
	float neighborSurvival;
	std::vector<int> neighbor_goalIdxes;
	float neighbor_reachability;

	while (!m_open.empty())
	{
		MaxSuccExactNode_t *current = m_open.top();
		m_open.pop();

		m_closed.push_back(current);
		if (current->m_isGoal == true)
		{
			std::cout << "MSE PATH FOUND\n";
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
			// check the path to neighbors: labels, survival and goalIdxes
			neighborLabels = 
				label_union(current->m_labels, g.getEdgeLabels(current->m_id, neighbor));
			neighborSurvival = computeSurvival(neighborLabels);
			neighbor_goalIdxes = update_goalIdxes(current->m_goalIndexes, 
											g.getEdgeLabels(current->m_id, neighbor));

			// check whether we need to put this neighbor into the priority queue (based on labels)
			// Every time we look at a neighbor, check if the labels it carries
			// is a super set of any of the set in the m_recordSet ( except for the first time :) )

			// The first time visited 
			if (m_visited[neighbor] == false)
			{
				// compute several attributes and then add to the open list
				neighbor_g = current->m_g + g.getEdgeCost(current->m_id, neighbor);
				neighbor_reachability = computeReach(neighbor_goalIdxes);

				// now put it to the open list
				m_open.push( new MaxSuccExactNode_t(neighbor, neighbor_g, neighborLabels, neighborSurvival, 
					neighbor_goalIdxes, neighbor_reachability, false, current) );
				m_recordSet[neighbor].push_back(neighborLabels);
				m_visited[neighbor] = true;


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
					// Now it's a goal. You need to make another copy of the node denote as 
					// a goal node, instead of an intermediate node. Add it to the open list as well
					neighbor_reachability = computeReach(remainingPoses);
					m_open.push( new MaxSuccExactNode_t(neighbor, neighbor_g, neighborLabels, neighborSurvival, 
									remainingPoses, neighbor_reachability, true, current) );
				}

			}
			else // not the first time visited
			{
				// check if the label set is a super set of any set that 
				// we have seen before in m_recordSet
				if (!check_superset(neighbor, neighborLabels))
				{

					// You reach the same node again with a different set (not a super set)
					// let's put it in the open list
					// again lots of things to compute before adding to the open list
					neighbor_g = current->m_g + g.getEdgeCost(current->m_id, neighbor);
					neighbor_reachability = computeReach(neighbor_goalIdxes);

					// now put it to the open list
					m_open.push( new MaxSuccExactNode_t(neighbor, neighbor_g, neighborLabels, neighborSurvival, 
						neighbor_goalIdxes, neighbor_reachability, false, current) );
					m_recordSet[neighbor].push_back(neighborLabels);

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
						// Now it's a goal. You need to make another copy of the node denote as 
						// a goal node, instead of an intermediate node. Add it to the open list as well
						neighbor_reachability = computeReach(remainingPoses);
						m_open.push( new MaxSuccExactNode_t(neighbor, neighbor_g, neighborLabels, neighborSurvival, 
										remainingPoses, neighbor_reachability, true, current) );
					}

				}
			}

		}
	}
	// You are reaching here since the open list is empty and the goal is not found
	std::cout << "The problem is not solvable. Search failed...\n";
	m_isFailure = true;
}

std::vector<int> MaxSuccessExactSolver_t::cal_remainingPoses(std::vector<int> neighbor_goalIdxes, std::vector<int> goalposeSet)
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

// void MaxSuccessExactSolver_t::checkPathSuccess(int nhypo)
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

std::vector<int> MaxSuccessExactSolver_t::update_goalIdxes(std::vector<int> currGoalIndexes, 
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


void MaxSuccessExactSolver_t::countObsAndGoalHypos()
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


// void MaxSuccessExactSolver_t::computeGoalMean(Graph_t &g)
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

// float MaxSuccessExactSolver_t::computeH(std::vector<float> temp_state, std::vector<int> goalIdxes)
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

float MaxSuccessExactSolver_t::computeSurvival(std::vector<int> labels)
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

float MaxSuccessExactSolver_t::computeReach(std::vector<int> goalIdxes)
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

void MaxSuccessExactSolver_t::print_path()
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

void MaxSuccessExactSolver_t::printLabels()
{
	std::cout << "labels: " << "< ";
	for (auto const &l : m_goalLabels)
	{
		std::cout << l << " ";
	}
	std::cout << ">\n";	
}

void MaxSuccessExactSolver_t::print_cost()
{
	std::cout << "cost: " << m_pathCost << "\n";
}
		
void MaxSuccessExactSolver_t::print_goalIdxReached()
{
	std::cout << "The reaching target poses are: "; 
	for (auto const &p : m_goalIdxReached)
	{
		std::cout << p << " ";
	}
	std::cout << "\n";
}

void MaxSuccessExactSolver_t::print_survivability()
{
	std::cout << "survivability: " << m_survivability << "\n";
}

void MaxSuccessExactSolver_t::print_successValue()
{
	std::cout << "successValue: " << m_pathSuccessValue << "\n";
}

void MaxSuccessExactSolver_t::printAll()
{
	print_path();
	print_cost();
	printLabels();
	print_goalIdxReached();
	print_survivability();
	print_successValue();
}

std::vector<int> MaxSuccessExactSolver_t::label_union(std::vector<int> s1, std::vector<int> s2)
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

bool MaxSuccessExactSolver_t::check_superset(int neighbor, std::vector<int> neighborLabels)
{
	bool isSuperset = false;
	for (auto const s: m_recordSet[neighbor])
	{
		if ( check_subset(neighborLabels, s) ) { return true; }
	}

	return isSuperset;
}

bool MaxSuccessExactSolver_t::check_subset(std::vector<int> set, std::vector<int> subset)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	std::sort(set.begin(), set.end());
	std::sort(subset.begin(), subset.end());
	return ( std::includes(set.begin(), set.end(), subset.begin(), subset.end()) );	
}


void MaxSuccessExactSolver_t::back_track_path()
{
	// start from the goal
	MaxSuccExactNode_t *current = m_closed[m_closed.size()-1];
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

void MaxSuccessExactSolver_t::pathToTrajectory(Graph_t &g)
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

void MaxSuccessExactSolver_t::writeTrajectory(std::string trajectory_file)
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

// void MaxSuccessExactSolver_t::printToVerify()
// {
// 	// first print the m_goalSet and m_targetPoses
// 	std::cout << "_________m_goalset_______\n";
// 	for (auto const &gs : m_goalSet)
// 	{
// 		std::cout << gs << " ";
// 	}
// 	std::cout << "\n";
// 	std::cout << "_________m_targetPoses________\n";
// 	for (auto const &tp : m_targetPoses)
// 	{
// 		std::cout << tp << " ";
// 	}
// 	std::cout << "\n";

// 	// then print m_goalmap
// 	std::cout << "_________m_goalmap_________\n";
// 	for (auto const &gm : m_goalmap)
// 	{
// 		std::cout << gm.first << ": " << gm.second << "\n";
// 	}

// 	// print out m_goalhypos and m_goalCounts
// 	std::cout << "_________m_goalhypos________\n";
// 	for (auto const &gh : m_goalhypos)
// 	{
// 		std::cout << gh << " ";
// 	}
// 	std::cout << "\n";
// 	std::cout << "_________m_goalCounts________\n";
// 	for (auto const &gc : m_goalCounts)
// 	{
// 		std::cout << gc.first << " " << gc.second << "\n";
// 	}

// 	// finally print m_goalMean
// 	std::cout << "_________m_goalMean________\n";
// 	for (auto const &gmean : m_goalMean)
// 	{
// 		std::cout << gmean.first << ": ";
// 		for (auto const &e: gmean.second)
// 		{
// 			std::cout << e << " ";
// 		}
// 		std::cout << "\n";
// 	}
// }
