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
#include "MCRMostCandidateSolver.hpp"
#include "Timer.hpp"


int main(int argc, char** argv)
{
	Timer t;
	std::string traj;
	std::ofstream outFile;

	std::string samples_file = "../roadmap_generator/example/" + std::string(argv[1]) + "/scenario" 
			+ std::string(argv[2]) + "/samples.txt";
	std::string roadmap_file = "../roadmap_generator/example/" + std::string(argv[1]) + "/scenario" 
			+ std::string(argv[2]) + "/roadmap.txt";
	std::string labelWeight_file = "../roadmap_generator/example/" + std::string(argv[1]) + "/scenario" 
			+ std::string(argv[2]) + "/labelWeights.txt";
	std::string mostPromisingLabels_file = "../roadmap_generator/example/" + std::string(argv[1]) + "/scenario" 
			+ std::string(argv[2]) + "/mostPromisingLabels.txt";

	int nsamples = atoi(argv[3]);

	t.reset();
	// import the graph
	Graph_t g(samples_file, roadmap_file, labelWeight_file, mostPromisingLabels_file, nsamples);
	std::cout << "Time to import the graph for " 
						<< g.getnNodes() << " nodes: " << t.elapsed() << "\n\n";
	std::vector<float> planningTime;

	t.reset();
	AstarSolver_t astar_solver(g, g.getStart(), g.getGoalSet());
	astar_solver.Astar_search(g);
	astar_solver.printAll();
	std::cout << "Astar time: " << t.elapsed() << "\n";
	planningTime.push_back(t.elapsed());

	t.reset();
	MCRGreedySolver_t mcr_gsolver(g, g.getStart(), g.getGoalSet());
	mcr_gsolver.MCRGreedy_search(g);
	mcr_gsolver.printAll();
	std::cout << "MCRG time: " << t.elapsed() << "\n";
	planningTime.push_back(t.elapsed());

	t.reset();
	MCRExactSolver_t mcr_esolver(g, g.getStart(), g.getGoalSet());
	mcr_esolver.MCRExact_search(g);
	mcr_esolver.printAll();
	std::cout << "MCRE time: " << t.elapsed() << "\n";
	planningTime.push_back(t.elapsed());

	t.reset();
	MaxSuccessGreedySolver_t maxsuccess_gsolver(g);
	maxsuccess_gsolver.MSGreedy_search(g);
	maxsuccess_gsolver.printAll();
	std::cout << "MSG time: " << t.elapsed() << "\n";
	planningTime.push_back(t.elapsed());

	t.reset();
	MaxSuccessExactSolver_t maxsuccess_esolver(g);
	maxsuccess_esolver.MSExact_search(g);
	maxsuccess_esolver.printAll();
	std::cout << "MSE time: " << t.elapsed() << "\n";
	planningTime.push_back(t.elapsed());

	t.reset();
	MCRMostCandidateSolver_t mcrmc_gsolver(g, g.getStart(), g.getGoalSet());
	mcrmc_gsolver.MCRMCGreedy_search(g);
	mcrmc_gsolver.printAll();
	std::cout << "MCR-MLC time: " << t.elapsed() << "\n";
	planningTime.push_back(t.elapsed());

	// write in the trajectories
	traj = "../roadmap_generator/example/" + std::string(argv[1]) + "/scenario" 
		+ std::string(argv[2]) + "/Astartraj.txt";
	astar_solver.writeTrajectory(traj);
	traj = "../roadmap_generator/example/" + std::string(argv[1]) + "/scenario" 
		+ std::string(argv[2]) + "/MCRGtraj.txt";
	mcr_gsolver.writeTrajectory(traj);
	traj = "../roadmap_generator/example/" + std::string(argv[1]) + "/scenario" 
		+ std::string(argv[2]) + "/MCREtraj.txt";
	mcr_esolver.writeTrajectory(traj);
	traj = "../roadmap_generator/example/" + std::string(argv[1]) + "/scenario" 
		+ std::string(argv[2]) + "/MSGtraj.txt";
	maxsuccess_gsolver.writeTrajectory(traj);
	traj = "../roadmap_generator/example/" + std::string(argv[1]) + "/scenario" 
		+ std::string(argv[2]) + "/MSEtraj.txt";
	maxsuccess_esolver.writeTrajectory(traj);
	traj = "../roadmap_generator/example/" + std::string(argv[1]) + "/scenario" 
		+ std::string(argv[2]) + "/MCRMCGtraj.txt";
	mcrmc_gsolver.writeTrajectory(traj);

	// write in time
	std::string time_file = "../roadmap_generator/example/" + std::string(argv[1]) + "/scenario" 
		+ std::string(argv[2]) + "/times.txt";
			+ "_" + std::string(argv[5]) + ".txt";
	outFile.open(time_file);
	if (outFile.is_open())
	{
		for (auto const &t : planningTime)
		{
			outFile << t << "\n";
		}
	}
	outFile.close();	

	return 0;

}

