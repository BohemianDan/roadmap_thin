#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include "Graph.hpp"
#include "AstarSolver.hpp"
#include "MCRGreedySolver.hpp"
#include "MCRExactSolver.hpp"
#include "MaxSuccessExactSolver.hpp"
#include "MaxSuccessGreedySolver.hpp"
#include "Timer.hpp"


int main(int argc, char** argv)
{
	Timer t;
	std::string traj;
	std::string parameter;
	std::ofstream outFile;

	if ( std::string(argv[3]) == "1" )
		parameter = "nHypos";
	else
		parameter = "noiseLevel";


	std::string samples_file = "../roadmap_generator/benchmark/" + std::string(argv[1]) + "/scenario" 
		+ std::string(argv[2]) + "/" + parameter + "/roadmaps/samples_" + std::string(argv[4]) + "_" + std::string(argv[5]) + ".txt";
	std::string roadmap_file = "../roadmap_generator/benchmark/" + std::string(argv[1]) + "/scenario" 
		+ std::string(argv[2]) + "/" + parameter + "/roadmaps/roadmap_" + std::string(argv[4]) + "_" + std::string(argv[5]) + ".txt";
	std::string labelWeight_file = "../roadmap_generator/benchmark/" + std::string(argv[1]) + "/scenario" 
	+ std::string(argv[2]) + "/" + parameter + "/roadmaps/labelWeights_" + std::string(argv[4]) + "_" + std::string(argv[5]) + ".txt";

	int nsamples = atoi(argv[6]);

	// import the graph
	Graph_t g(samples_file, roadmap_file, labelWeight_file, mostPromisingLabels_file, nsamples);
	// g.print_graph();
	// std::cout << "Time to import the graph for " 
	// 					<< g.getnNodes() << " nodes: " << t.elapsed() << "\n\n";

	AstarSolver_t astar_solver(g, g.getStart(), g.getGoalSet());
	astar_solver.Astar_search(g);
	MCRGreedySolver_t mcr_gsolver(g, g.getStart(), g.getGoalSet());
	mcr_gsolver.MCRGreedy_search(g);
	MCRExactSolver_t mcr_esolver(g, g.getStart(), g.getGoalSet());
	mcr_esolver.MCRExact_search(g);
	MaxSuccessGreedySolver_t maxsuccess_gsolver(g);
	maxsuccess_gsolver.MSGreedy_search(g);
	MaxSuccessExactSolver_t maxsuccess_esolver(g);
	maxsuccess_esolver.MSExact_search(g);

	// check if there is any failure in any of the search methods
	if (astar_solver.getFailureIndicator() == true or mcr_gsolver.getFailureIndicator() == true 
		or mcr_esolver.getFailureIndicator() == true or maxsuccess_gsolver.getFailureIndicator() == true
		or maxsuccess_esolver.getFailureIndicator() == true)
	{
		// will not submit the statistics but just to report failure
		std::string failureIndicator_file = "../roadmap_generator/benchmark/" + std::string(argv[1]) + "/scenario"
			+ std::string(argv[2]) + "/" + parameter + "/trajectories/failureIndicator_" + std::string(argv[4])
			+ "_" + std::string(argv[5]) + ".txt";
		outFile.open(failureIndicator_file);
		if (outFile.is_open()) { outFile << true; }
		outFile.close();

	}
	else
	{
		// Congrats! No failure search. Indicate as well.
		std::string failureIndicator_file = "../roadmap_generator/benchmark/" + std::string(argv[1]) + "/scenario"
			+ std::string(argv[2]) + "/" + parameter + "/trajectories/failureIndicator_" + std::string(argv[4])
			+ "_" + std::string(argv[5]) + ".txt";
		outFile.open(failureIndicator_file);
		if (outFile.is_open()) { outFile << false; }
		outFile.close();

		// compute necessary statitiscs 
		astar_solver.checkPathSuccess(atoi(argv[7]));
		mcr_gsolver.checkPathSuccess(atoi(argv[7]));
		mcr_esolver.checkPathSuccess(atoi(argv[7]));
		maxsuccess_gsolver.checkPathSuccess(atoi(argv[7]));
		maxsuccess_esolver.checkPathSuccess(atoi(argv[7]));

		// write the statitics
		std::string statistics_file = "../roadmap_generator/benchmark/" + std::string(argv[1]) + "/scenario"
			+ std::string(argv[2]) + "/" + parameter + "/trajectories/statistics_" + std::string(argv[4])
			+ "_" + std::string(argv[5]) + ".txt";
		outFile.open(statistics_file);
		if (outFile.is_open())
		{
			// write in #obs, whether success, cost, one line one search method
			outFile << astar_solver.getObstaclesCollided() << " " << astar_solver.getIsPathSuccess() 
					<< " " << astar_solver.getPathCost() << "\n";
			outFile << mcr_gsolver.getObstaclesCollided() << " " << mcr_gsolver.getIsPathSuccess() 
					<< " " << mcr_gsolver.getPathCost() << "\n";
			outFile << mcr_esolver.getObstaclesCollided() << " " << mcr_esolver.getIsPathSuccess() 
					<< " " << mcr_esolver.getPathCost() << "\n";
			outFile << maxsuccess_gsolver.getObstaclesCollided() << " " << maxsuccess_gsolver.getIsPathSuccess() 
					<< " " << maxsuccess_gsolver.getPathCost() << "\n";
			outFile << maxsuccess_esolver.getObstaclesCollided() << " " << maxsuccess_esolver.getIsPathSuccess() 
					<< " " << maxsuccess_esolver.getPathCost() << "\n";								
		}
		outFile.close();

		traj = "../roadmap_generator/benchmark/" + std::string(argv[1]) + "/scenario" 
			+ std::string(argv[2]) + "/" + parameter + "/trajectories/Astartraj_" + std::string(argv[4]) 
				+ "_" + std::string(argv[5]) + ".txt";
		astar_solver.writeTrajectory(traj);
		traj = "../roadmap_generator/benchmark/" + std::string(argv[1]) + "/scenario" 
			+ std::string(argv[2]) + "/" + parameter + "/trajectories/MCRGtraj_" + std::string(argv[4]) 
				+ "_" + std::string(argv[5]) + ".txt";
		mcr_gsolver.writeTrajectory(traj);
		traj = "../roadmap_generator/benchmark/" + std::string(argv[1]) + "/scenario" 
			+ std::string(argv[2]) + "/" + parameter + "/trajectories/MCREtraj_" + std::string(argv[4]) 
				+ "_" + std::string(argv[5]) + ".txt";
		mcr_esolver.writeTrajectory(traj);
		traj = "../roadmap_generator/benchmark/" + std::string(argv[1]) + "/scenario" 
			+ std::string(argv[2]) + "/" + parameter + "/trajectories/MSGtraj_" + std::string(argv[4]) 
				+ "_" + std::string(argv[5]) + ".txt";
		maxsuccess_gsolver.writeTrajectory(traj);
		traj = "../roadmap_generator/benchmark/" + std::string(argv[1]) + "/scenario" 
			+ std::string(argv[2]) + "/" + parameter + "/trajectories/MSEtraj_" + std::string(argv[4]) 
				+ "_" + std::string(argv[5]) + ".txt";
		maxsuccess_esolver.writeTrajectory(traj);
	}

	return 0;
}