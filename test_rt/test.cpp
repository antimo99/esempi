#include <sched.h>

#include <iostream>
#include <cstring>
#include <fstream>


bool has_realtime_kernel()
{
  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime = false;
  if (realtime_file.is_open()) {
    realtime_file >> has_realtime;
  }
  return has_realtime;
}

bool configure_sched_fifo(int priority)
{
  struct sched_param schedp;
  memset(&schedp, 0, sizeof(schedp));
  schedp.sched_priority = priority;
  return !sched_setscheduler(0, SCHED_FIFO, &schedp);
}


// Dichiarazione della funzione
void saluta();

int main() 
{
    bool var1=has_realtime_kernel();
    bool var2= configure_sched_fifo(98);
    std::cout << "Has real time: " <<var1<< std::endl;
    std::cout << "Configure sched: " <<var2<< std::endl;
    int var3;
    std::cin>>var3;
    return 0;
}

