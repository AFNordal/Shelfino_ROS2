

#include "taskplanning.hpp"
using namespace naxos;
using namespace std;

int main(int argc, char* argv[])
{
        try {
                int N = 5
                const std::vector<std::vector<int>> travel_time = {
                    {0, 2, 4, 6, 8},
                    {2, 0, 2, 4, 6},
                    {4, 2, 0, 2, 4},
                    {6, 4, 2, 0, 2},
                    {8, 6, 4, 2, 0}}; 
                const int Tmax = 10; // Maximum time budget
                const std::vector<int> profits = {0, 10, 20, 30, 0}; // Profit for each node

                NsProblemManager pm;
                NsDeque<NsIntVarArray> grid(N);
                switch(N) {
                case 5:
                    NsIntVar X_1_1
                    NsIntVar X_1_2
                    NsIntVar X_1_3
                    NsIntVar X_1_4
                    NsIntVar X_1_5
                    NsIntVar X_2_1
                    NsIntVar X_2_2
                    NsIntVar X_2_3
                    NsIntVar X_2_4
                    NsIntVar X_2_5
                    NsIntVar X_3_1
                    NsIntVar X_3_2
                    NsIntVar X_3_3
                    NsIntVar X_3_3
                    NsIntVar X_3_4
                    NsIntVar X_3_5
                    NsIntVar X_4_1
                    NsIntVar X_4_2
                    NsIntVar X_4_3
                    NsIntVar X_4_4
                    NsIntVar X_4_5
                    NsIntVar X_5_1
                    NsIntVar X_5_2
                    NsIntVar X_5_3
                    NsIntVar X_5_4
                    NsIntVar X_5_5

                    grid[1].push_back(NsIntVar X_1_1)
                    grid[1].push_back(NsIntVar X_1_2)
                    grid[1].push_back(NsIntVar X_1_3)
                    grid[1].push_back(NsIntVar X_1_4)
                    grid[1].push_back(NsIntVar X_1_5)
                    grid[2].push_back(NsIntVar X_2_1)
                    grid[2].push_back(NsIntVar X_2_2)
                    grid[2].push_back(NsIntVar X_2_3)
                    grid[2].push_back(NsIntVar X_2_4)
                    grid[2].push_back(NsIntVar X_2_5)
                    grid[3].push_back(NsIntVar X_3_1)
                    grid[3].push_back(NsIntVar X_3_2)
                    grid[3].push_back(NsIntVar X_3_3)
                    grid[3].push_back(NsIntVar X_3_4)
                    grid[3].push_back(NsIntVar X_3_5)

                }

                for (int i = 0; i < N; ++i) {
                    for (int j = 0; j < N; ++j) {
                        NsIntVar "X_"+std::to_string(i)+std::to_string(j)
                        grid[i].push_back("X_"+std::to_string(i)+std::to_string(j));
                    }
                }
                //start in start node and end in end node
                pm.add(NsSum(grid[1], 0, N) = 1)
                pm.add(NsSum(grid[N-1], 0, N) = 1)

                

                //Define the objective               
                for (int i = 0; i < N-1; ++i) {
                    for (int j = 0; j < N-1 ++j) {
                        if j>0{
                             NsIntVar obj_+std::to_string(i)+std::to_string(j) == NsSum(obj_+std::to_string(i)+std::to_string(j-1), "X_"+std::to_string(i)+std::to_string(j) * profit(j))
                        }else{
                             NsIntVar obj_+std::to_string(i)+std::to_string(j) == NsSum(obj_+std::to_string(i-1)+std::to_string(j), "X_"+std::to_string(i)+std::to_string(j) * profit(j))
                        }
                    }
                }
                pm.minimize(obj_+std::to_string(N-1)+std::to_string(N-1));
                pm.addGoal(new NsgLabeling(obj_+std::to_string(N-1)+std::to_string(N-1)));
                pm.addGoal(new NsgLabeling(grid));
                while (pm.nextSolution() != false)
                        cout << "ns" << "\n";
        } catch (exception& exc) {
                cerr << exc.what() << "\n";
                return 1;
        } catch (...) {
                cerr << "Unknown exception\n";
                return 1;
        }
}

