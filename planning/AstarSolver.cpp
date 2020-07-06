/* This cpp file defines A* search on a given graph
with specified start and goal */

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "Graph.hpp"
#include "AstarSolver.hpp"
#include "Timer.hpp"

AstarSolver_t::AstarSolver_t(Graph_t &g, int start, std::vector<int> goalSet)
{
	// initialize the start & goal
	m_start = start;
	m_goalSet = goalSet;
	m_targetPoses = g.getTargetPoses();
	for (int i=0; i < m_goalSet.size(); i++)
	{
		m_goalmap[m_goalSet[i]] = m_targetPoses[i];
	}
	// essential elements for Astar search
	computeH(g); // heuristics
	m_G = std::vector<float>(g.getnNodes(), std::numeric_limits<float>::max());
	m_G[m_start] = 0.0;
	m_open.push(new AstarNode_t(m_start, m_G[m_start], nullptr));
	m_expanded = std::vector<bool>(g.getnNodes(), false);

	m_isFailure = false;
	
}

AstarSolver_t::~AstarSolver_t()
{
	while (!m_open.empty())
	{
		AstarNode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }
}

void AstarSolver_t::computeH(Graph_t &g)
{
	std::vector<float> goal_mean = std::vector<float>(g.getState(0).size(), 0.0);
	for (auto const &goal : m_goalSet)
	{
		std::vector<float> v_goal = g.getState(goal);
		for (int j=0; j < v_goal.size(); j++)
		{
			goal_mean[j] = goal_mean[j] + v_goal[j];
		}
	}
	for (int j=0; j < goal_mean.size(); j++)
	{
		goal_mean[j] = goal_mean[j] / m_goalSet.size();
	}

	std::vector<float> temp_v;
	for (int i=0; i < g.getnNodes(); i++)
	{
		if (std::find(m_goalSet.begin(), m_goalSet.end(), i) != m_goalSet.end())
		{
			m_H.push_back(0.0);
			continue;
		}
		// compute euclidean distance
		float temp_h = 0.0;
		temp_v = g.getState(i);
		for (int j=0; j < goal_mean.size(); j++)
		{
			temp_h += pow(goal_mean[j]-temp_v[j], 2);
		}
		temp_h = sqrt(temp_h);
		m_H.push_back(temp_h);
	}

}

void AstarSolver_t::Astar_search(Graph_t &g)
{
	while (!m_open.empty())
	{
		AstarNode_t *current = m_open.top();
		m_open.pop();
		// Now check if the current node has been expanded
		if (m_expanded[current->m_id] == true)
		{
			// This node has been expanded with the lowest f value for its id
			// No need to put it into the closed list
			delete current;
			continue;
		}
		m_closed.push_back(current);
		m_expanded[current->m_id] = true;

		// a goal in the goalSet has been found
		if ( std::find(m_goalSet.begin(), m_goalSet.end(), current->m_id) != m_goalSet.end() ) 
		{
			std::cout << "OSP PATH FOUND\n";
			back_track_path(); // construct your path
			pathToTrajectory(g); // get the trajectory (a sequence of configurations)
			computeLabelsAndSurvival(g); // get the labels the path carries and corresponding survivability
			// printLabels();
			// print the pose the goal indicates
			m_goalIdxReached = m_goalmap[current->m_id];
			m_pathCost = current->m_g;

			return;
		}
		// get neighbors of the current node
		std::vector<int> neighbors = g.getNodeNeighbors(current->m_id);
		for (auto const &neighbor : neighbors)
		{
			// check if the neighbor node has been visited or extended before
			if ( m_expanded[neighbor] ) {continue;}
			if ( m_G[neighbor] > m_G[current->m_id] + g.getEdgeCost(current->m_id, neighbor) )
			{
				m_G[neighbor] = m_G[current->m_id] + g.getEdgeCost(current->m_id, neighbor);
				m_open.push(new AstarNode_t(neighbor, m_G[neighbor], current));
			}
		}
	}
	// You are reaching here since the open list is empty and the goal is not found
	std::cout << "The problem is not solvable. Search failed...\n";
	m_isFailure = true;
}

// void AstarSolver_t::checkPathSuccess(int nhypo)
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

void AstarSolver_t::back_track_path()
{
	// start from the goal
	AstarNode_t *current = m_closed[m_closed.size()-1];
	while (current->m_id != m_start)
	{
		// keep backtracking the path until you reach the start
		m_path.push_back(current->m_id);
		current = current->m_parent;
	}
	// finally put the start into the path
	m_path.push_back(current->m_id);
}

void AstarSolver_t::print_path()
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

void AstarSolver_t::printLabels()
{
	std::cout << "labels: " << "< ";
	for (auto const &l : m_goalLabels)
	{
		std::cout << l << " ";
	}
	std::cout << ">\n";
}

void AstarSolver_t::print_cost()
{
	std::cout << "cost: " << m_pathCost << "\n";
}
		
void AstarSolver_t::print_goalIdxReached()
{
	std::cout << "The reaching target poses are: "; 
	for (auto const &p : m_goalIdxReached)
	{
		std::cout << p << " ";
	}
	std::cout << "\n";
}

void AstarSolver_t::print_survivability()
{
	std::cout << "survivability: " << m_survivability << "\n";
}

void AstarSolver_t::printAll()
{
	print_path();
	print_cost();
	printLabels();
	print_goalIdxReached();
	print_survivability();
}

void AstarSolver_t::pathToTrajectory(Graph_t &g)
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

void AstarSolver_t::writeTrajectory(std::string trajectory_file)
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

void AstarSolver_t::computeLabelsAndSurvival(Graph_t &g)
{
	std::vector<int> temp_edgelabels;
	for (int i = 0; i < m_path.size()-1; i++)
	{
		temp_edgelabels = g.getEdgeLabels(m_path[i], m_path[i+1]);
		m_goalLabels = label_union(m_goalLabels, temp_edgelabels);
	}

	/// Let's compute the survivability
	std::map<int, std::pair<int, float>> label_weights = g.getLabelWeights();
	std::vector<float> temp_probs(label_weights[label_weights.size()-1].first+1, 0.0);
	for (auto const &l : m_goalLabels)
	{
		temp_probs[label_weights[l].first] += label_weights[l].second;
	}
	m_survivability = 1.0;
	for (auto const &temp_prob : temp_probs)
	{
		m_survivability = m_survivability * (1 - temp_prob);
	}

	// print for testing purposes
	// std::cout << "temp_probs: ";
	// for (auto const &tp: temp_probs)
	// {
	// 	std::cout << tp << " ";
	// }
	// std::cout << "\n";
}


std::vector<int> AstarSolver_t::label_union(std::vector<int> s1, std::vector<int> s2)
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