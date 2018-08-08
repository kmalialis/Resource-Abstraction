/*
DESCRIPTION
C++ implementation of the Beach Problem Domain (BPD) and Resource abstraction. Resource abstraction aims at solving multiagent congestion / resource management problems by allocating the available resources into abstract groups. This abstraction creates new reward functions that provide a more informative signal to the reinforcement learning (RL) agents and aid the coordination amongst them. The RL implementation is found in "qlearning.h".

PAPER
Please cite our work as follows:
K. Malialis, S. Devlin and D. Kudenko. Resource Abstraction for Reinforcement Learning in Multiagent Congestion Problems. In Proceedings of the 15th International Conference on Autonomous Agents and Multiagent Systems (AAMAS), 2016.

HOW TO RUN EXPERIMENTS
Configure the following 3 things:
1. The parameters STATS_RUNS, EPISODES, ..., NUM_SUPERLANES (lines 33-40)
2. The abstraction configuration in superLane::setMembers() (line 166)
3. The reward signal to be used (lines 402-405)

For instance, if you run the code with the current settings you will generate the results of experiment Figure 1/A-2+1+3 in "global.txt".
*/

#include <cstdlib>
#include <math.h>
#include <time.h>

#include <stdio.h>
#include <iostream>
using namespace std;

#ifndef VECTOR_INCLUDE
#define VECTOR_INCLUDE
#include <vector>
#endif

#include <algorithm>

#define STAT_RUNS 30
#define EPISODES 10000
#define STEPS 5
#define NUM_AGENTS 100
#define LANES 6
#define CAPACITY 6
#define ACTIONS 3
#define NUM_SUPERLANES	3


#define LYRAND (double)rand()/RAND_MAX
#define SMALL 0.0001


#include "qlearning.h"

bool pretty_print = true;

bool command_global = false;
bool command_difference=false;
bool command_local = false;
bool command_coordinated = false;

/******************************
 * Environment:               *
 * Beach Problem Domain (BPD) *
 ******************************/

class highway {
public:
	void start();
	void make_lookup();
	void get_attendance(vector<QLearner>*);
	void evaluate();
	void console_attendance();

	vector<double> highway_lookup;
	vector<int> attendance;
	vector<double> lane_local;
	vector<double> lane_difference;

	double global;
};

void highway::console_attendance() /// Put attendance values out to console.
{
	for (int i = 0; i < attendance.size(); i++) {
		cout << attendance.at(i);
		cout << "\t";
	}
	cout << endl;
}

void highway::make_lookup() /// Creates "L" Lookup table, so that we can skip exponential calculations.
{
	highway_lookup.clear();
	//cout << "BAR LOOKUP CHART: " << endl;
	for (int x = 0; x < NUM_AGENTS + 1; x++) {
		double xx = x;
		highway_lookup.push_back((double) xx * exp(-xx / CAPACITY));
		 }
	}

	void highway::start() { /// Initializes values.
	lane_local.clear();
	lane_difference.clear();
	attendance.clear();
	global = 0;
	}

	void highway::get_attendance(vector<QLearner>* pA) { /// Determines agent attendance per lane
	attendance.clear();
	attendance.resize(LANES, 0);
	for (int i = 0; i < NUM_AGENTS; i++) {
		attendance.at(pA->at(i).state)++;
	}
	}

	void highway::evaluate() { /// Evaluates local, global, difference rewards
	for (int lane = 0; lane < attendance.size(); lane++) {
		lane_local.push_back(highway_lookup.at(attendance.at(lane)));
		global += lane_local.back();
		if (attendance.at(lane) == 0) {
			lane_difference.push_back(0);
		} else {
			lane_difference.push_back(/**/ highway_lookup.at(attendance.at(lane)) - highway_lookup.at(attendance.at(lane) - 1) /**/);
		}
	}
	}

	/************************
	 * Resource abstraction *
	 ************************/

	struct superLane{
		int id;
		int capacity;
		int attendance;
		vector<int> membersID;
		double reward;

		void reset();
		void setMembers();
		void calcAttendReward(highway* pE);
	};

	void superLane::reset() {
		id = 0;
		capacity = 0;
		this->attendance = 0;
		membersID.clear();
		reward = 0.0;
	}

	void superLane::calcAttendReward(highway* pE) {
		// reset first
		this->attendance = 0;
		reward = 0.0;

		// calculate attendance
		for (vector<int>::const_iterator it = membersID.begin(); it != membersID.end(); ++it) {
			this->attendance += pE->attendance[(*it)];
		}

		// calculate reward
		reward = (double) - this->attendance * exp(- this->attendance / capacity);
	}

	/*struct Node {
		int val;
		Node* next[5];
	};*/

	void superLane::setMembers() {
		// reset first
		capacity = 0;
		membersID.clear();

		/*
		if (choice == 1) {
			// 6 lanes - 2 superlanes - 3+3
			if (id == 0) {
				membersID.push_back(0);
				membersID.push_back(1);
				membersID.push_back(2);
			} else if (id == 1) {
				membersID.push_back(3);
				membersID.push_back(4);
				membersID.push_back(5);
			}
		} else if (choice == 2) {
			// 6 lanes - 2 superlanes 4+2
			if (id == 0) {
				membersID.push_back(0);
				membersID.push_back(1);
				membersID.push_back(2);
				membersID.push_back(3);
			} else if (id == 1) {
				membersID.push_back(4);
				membersID.push_back(5);
			}
		} else if (choice == 3) {
			// 6 lanes - 2 superlanes 5+1
			if (id == 0) {
				membersID.push_back(0);
				membersID.push_back(1);
				membersID.push_back(2);
				membersID.push_back(3);
				membersID.push_back(4);
			} else if (id == 1) {
				membersID.push_back(5);
			}
		}
		*/

		/*
		if (choice == 4) {
			// 6 lanes - 3 superlanes 2+2+2
			if (id == 0) {
				membersID.push_back(0);
				membersID.push_back(1);
			} else if (id == 1) {
				membersID.push_back(2);
				membersID.push_back(3);
			} else if (id == 2) {
				membersID.push_back(4);
				membersID.push_back(5);
			}
		} else if (choice == 5) {
			// 6 lanes - 3 superlanes 3+2+1 (or 1+2+3)
			if (id == 0) {
				membersID.push_back(0);
				membersID.push_back(1);
				membersID.push_back(2);
			} else if (id == 1) {
				membersID.push_back(3);
				membersID.push_back(4);
			} else if (id == 2) {
				membersID.push_back(5);
			}
		} else if (choice == 6) {
			// 6 lanes - 3 superlanes 1+3+2 (or 2+3+1)
			if (id == 0) {
				membersID.push_back(0);
			} else if (id == 1) {
				membersID.push_back(1);
				membersID.push_back(2);
				membersID.push_back(3);
			} else if (id == 2) {
				membersID.push_back(4);
				membersID.push_back(5);
			}
		} else if (choice == 7) {
			// 6 lanes - 3 superlanes 2+1+3 (or 3+1+2)
			if (id == 0) {
				membersID.push_back(0);
				membersID.push_back(1);
			} else if (id == 1) {
				membersID.push_back(2);
			} else if (id == 2) {
				membersID.push_back(3);
				membersID.push_back(4);
				membersID.push_back(5);
			}
		}
		*/


			// 6 lanes - 3 superlanes 2+1+3 (or 3+1+2)
			if (id == 0) {
				membersID.push_back(0);
				membersID.push_back(1);
			} else if (id == 1) {
				membersID.push_back(2);
			} else if (id == 2) {
				membersID.push_back(3);
				membersID.push_back(4);
				membersID.push_back(5);
			}

			// 20 lanes - 3 superlanes - 8+1+11 (or 11+1+8)
			/*
			if (id == 0) {
				membersID.push_back(0);
				membersID.push_back(1);
				membersID.push_back(2);
				membersID.push_back(3);
				membersID.push_back(4);
				membersID.push_back(5);
				membersID.push_back(6);
				membersID.push_back(7);
			} else if (id == 1) {
				membersID.push_back(8);
			} else if (id == 2) {
				membersID.push_back(9);
				membersID.push_back(10);
				membersID.push_back(11);
				membersID.push_back(12);
				membersID.push_back(13);
				membersID.push_back(14);
				membersID.push_back(15);
				membersID.push_back(16);
				membersID.push_back(17);
				membersID.push_back(18);
				membersID.push_back(19);
			}
			*/


		// set capacity
		capacity = CAPACITY * membersID.size();
	}

	int findSuperLane(vector<superLane>* pS, int lane) {
		for (vector<superLane>::const_iterator it = pS->begin(); it != pS->end(); ++it) {
			vector<int> members = (*it).membersID;
			vector<int>::const_iterator p = find(members.begin(),members.end(),lane);
			if (*p == lane) {
				return (*it).id;
			}
		}
	}

	/********************************
	 * Interaction with environment *
	 ********************************/

	void sense(vector<QLearner>* pA) {
		for (int agent = 0; agent < NUM_AGENTS; agent++) {
			pA->at(agent).previousState = pA->at(agent).state;
		}
	}

	void decide(vector<QLearner>* pA) { ///agents decide which day they are attending.
	for (int agent = 0; agent < NUM_AGENTS; agent++) {
		pA->at(agent).decay_alpha();
		pA->at(agent).decay_epsilon();
		pA->at(agent).choose_egreedy_action();
	}
	}

	void act(vector<QLearner>* pA, highway* pE) { /// agents attend, highway grabs attendance
	for (int agent = 0; agent < NUM_AGENTS; agent++) {
		//Calculate next state for all agents
		pA->at(agent).state += pA->at(agent).action - 1;
		if (pA->at(agent).state < 0) {
			pA->at(agent).state = 0;
		} else if (pA->at(agent).state >= LANES) {
			pA->at(agent).state = LANES-1;
		}
	}

	pE->start();
	pE->get_attendance(pA);
	}

	void react(vector<QLearner>* pA, highway* pE, vector<superLane>* pS) { /// reward calculations, Q updates.
	pE->evaluate();

	for (vector<superLane>::iterator it = pS->begin(); it != pS->end() ; it++) {
		it->calcAttendReward(pE);
	}

	for (int agent = 0; agent < NUM_AGENTS; agent++) {
		double L,G,D;

		L = pE->lane_local.at(pA->at(agent).state);
		G = pE->global;
		D = pE->lane_difference.at(pA->at(agent).state);


		// Resource abstraction
		double C;
		int agentLane = (*pA)[agent].state;
		int attLane = pE->attendance[agentLane];
		int superLane = findSuperLane(pS,agentLane);

		//cout << agentLane << " " << superLane << endl;

		// Coordinated rewards
		if (attLane > CAPACITY) {
			C = (*pS)[superLane].reward;
		} else {
			C = L;
		}

		pA->at(agent).set_local(L);
		pA->at(agent).set_global(G);
		pA->at(agent).set_difference(D);
		pA->at(agent).set_coordinated(C);	// Resource abstraction

		if(command_local){pA->at(agent).learn_with_local();}
		if(command_global){pA->at(agent).learn_with_global();}
		if(command_difference){pA->at(agent).learn_with_difference();}
		if(command_coordinated){pA->at(agent).learn_with_coordinated();}	// Resource abstraction

		pA->at(agent).Qupdate();
	}
	}

	void report(FILE* pFILE, double global) { /// report to text file
	fprintf(pFILE, "%.5f\t", global);
	}

	int main() {
		srand(time(NULL));

		FILE* pFILE;
		pFILE = fopen("global.txt", "w");
		command_global = 0;
		command_difference=0;
		command_local = 0;
		command_coordinated = 1;	// Resource abstraction

		for(int stat_run=0; stat_run < STAT_RUNS; stat_run++) {
			highway RouteSixtySix;
			RouteSixtySix.make_lookup();
			highway* pE = &RouteSixtySix;

			// Resource abstraction
			vector<superLane> superLanes;
			for (int i = 0; i < NUM_SUPERLANES; i++) {
				superLane S;
				S.reset();
				S.id = i;
				S.setMembers();
				superLanes.push_back(S);
			}
			vector<superLane>* pS = &superLanes;


			vector<QLearner> Agents;
			vector<QLearner>* pA = &Agents;

			for (int i = 0; i < NUM_AGENTS; i++) {
				QLearner Q;
				Q.id = i;
				Q.start();
				pA->push_back(Q);
			}

			for (int episode = 0; episode < EPISODES; episode++){
				if (episode % (EPISODES / 10) == 0) {
					cout << "Run No." << stat_run << " is " << (double) episode / EPISODES * 100 << " % Complete!" << endl;
				}

				for (int time = 0; time < STEPS; time++) {
					//cout << "begin time " << time << endl;
					sense(pA);
					//cout << "sensed time " << time << endl;
					decide(pA);
					//cout << "decided time " << time << endl;
					act(pA, pE);
					//pE->console_attendance();         //Print behaviour to console
					//cout << "acted time " << time << endl;
					react(pA, pE, pS);
					//cout << "reacted time " << time << endl;
				}

				//For beautiful graphs
				if (pretty_print) {
					report(pFILE, pE->global); // Report every result
				} else {
					//For Coarse Results
					if (episode % (EPISODES / 1000) == 0) {
						report(pFILE, pE->global);      //Report only occasionally
					}
				}

				// Pick an action from the final state
				// This action will cause a transition to the absorbing state
				sense(pA);
				decide(pA);
				for (int i = 0; i < NUM_AGENTS; i++) {
					pA->at(i).final_Qupdate();       //Updates the final state->absorbing state transition
					pA->at(i).restart();
				}
			}

			//Start a new line in output file for next run
			fprintf(pFILE,"\n");

			cout << endl << "Lane attendance:" << endl;
			pE->console_attendance();       //Print final behaviour
			cout << endl << "Final performance = " << pE->global << endl << endl;     //Final global reward
		}
		fclose(pFILE);

		return 0;
	}
