#include "task_manager.hpp"
#include "config/config_params.hpp"
#include "config/config_file_reader.hpp"

bool terminate = false;

void checkTermination(int signal)
{
    terminate = true;
}

int main()
{
    ConfigParams config_params = ConfigFileReader::load("../config/ccu_config.yaml");
    task::TaskManager task_manager(config_params);

    signal(SIGINT, checkTermination);
    while (!terminate)
    {
        sleep(0.5);
    }

    return 0;
}