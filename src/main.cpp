
#include <libwebsockets.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <assert.h>

#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"  
#include "spdlog/fmt/ostr.h" 
#include <memory>
#include "agent.h"
#include <chrono>
#include "../lib/mqtt/wrapper.h"

using namespace spdlog;

void loadConfig(const std::string& fileName, std::string& mode);
int main(int argc, const char **argv)
{
    spdlog::warn("Easy padding in numbers like {:08d}", 12);
    spdlog::info("Welcome to spdlog version {}.{}.{}  !", SPDLOG_VER_MAJOR, SPDLOG_VER_MINOR,SPDLOG_VER_PATCH);
    std::string mode;
    //std::string configFile = "../../config/config.txt";
    std::string configFile = "./config.txt";
    loadConfig(configFile,mode);

    spdlog::info("run as robot.");
    client_create();
    g_mqttClient_ptr->loadConfig(configFile);
    g_mqttClient_ptr->Start();

//    std::shared_ptr<Console> console = std::make_shared<Console>();
//    console->setClient(g_mqttClient_ptr);
//    console->Start();
    while(1) {
         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void loadConfig(const std::string& fileName, std::string& mode)
{
     std::string line;
     std::size_t pos;
     std::string result;

     std::ifstream ifs(fileName);

     while (std::getline(ifs, line)) {
         line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
         pos = line.find("=");
         if (pos != std::string::npos) {
             std::string key = line.substr(0, pos);
             std::string value = line.substr(pos + 1);
             if (key == "mode") {
                 mode = value;
             } 
         }
     }
}

