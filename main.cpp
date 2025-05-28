#include <iostream>
#include "map.h"
#include "task.h"
#include "lowLevel.h"
#include "heuristic.h"
#include "highLevel.h"
#include "windows.h"
#include <psapi.h>
#include <iomanip>

void print_memory_usage();
int main() {
    Map map;
    Task task;
    map.getMap("../MapData/Task1.xml");
    task.getTask("../AgentData/Task1_9Agent.xml");
    task.getXY(map);

    CBS cbs;
    Solution res=cbs.findSolution(map,task);


    std::cout<<"Solution:"<<std::boolalpha<<"\n  Found:"<<res.found<< "\n  Runtime: "
              << res.time.count()  << "\n  Cost: "<<res.Cost << std::endl;


    print_memory_usage();
    return 0;
}

void print_memory_usage() {
    PROCESS_MEMORY_COUNTERS_EX pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
        SIZE_T physMemUsedByMe = pmc.WorkingSetSize;
        SIZE_T physMemPeakUsedByMe = pmc.PeakWorkingSetSize;
        SIZE_T virtualMemUsedByMe = pmc.PrivateUsage;

        std::cout << "Memory Usage:" << std::endl;
        std::cout << "  Physical Memory Used: " << physMemUsedByMe / 1024 << " KB" << std::endl;
        std::cout << "  Peak Physical Memory Used: " << physMemPeakUsedByMe / 1024 << " KB" << std::endl;
        std::cout << "  Virtual Memory Used: " << virtualMemUsedByMe / 1024 << " KB" << std::endl;
    } else {
        std::cerr << "  Failed to retrieve memory usage information" << std::endl;
    }
}
