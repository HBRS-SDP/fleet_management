#include <string>
#include <vector>

#include "config/config_params.hpp"
#include "config/config_file_reader.hpp"
#include "ccu_manager.hpp"

int main(int argc, char** argv)
{
    ConfigParams config_params = ConfigFileReader::load("../config/ropods.yaml");
    CCUManager ccu_manager(config_params);
}
