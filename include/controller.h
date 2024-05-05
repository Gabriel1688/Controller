#ifndef _RA_MANAGER_
#define _RA_MANAGER_
#include <deque>
#include <mutex>
#include <set>
#include <map>
#include <string>
#include <memory>
#include "fifo.h"
#include "message.h"
#include "spdlog/spdlog.h"

using TCallback= std::function<void (std::string& result)>;

class Controller;

class Controller
{
 public:
   Controller();
   ~Controller();

   int  Start();
   void Shutdown();
   void Message(const std::string& topic,std::shared_ptr<MESSAGE> message, TCallback callback);

// private:
    enum class OamAmoCommandType: int
    {
       EN_AMO_CMD_GET_AO_REQ = 0,
       EN_AMO_CMD_CREATE_AO_REQ ,
       EN_AMO_CMD_DELETE_AO_REQ ,
       EN_AMO_CMD_UPDATE_AO_REQ ,
       EN_AMO_CMD_GET_CFM    ,
       EN_AMO_CMD_ADD_CFM    ,
       EN_AMO_CMD_ADD_NRCELLCU_CFM,
       EN_AMO_CMD_SETUP_NRCELLCU_CFM,
       EN_AMO_CMD_ACTIVE_NRCELLCU_CFM,
       EN_AMO_CMD_DEACTIVE_NRCELLCU_CFM,
       EN_AMO_CMD_RELEASE_NRCELLCU_CFM,
       EN_AMO_CMD_FAULT_INDICATION,
       EN_AMO_CMD_FAULTCEASE_INDICATION,
       EN_AMO_CMD_DELETE_CFM ,
       EN_AMO_CMD_UPDATE_CFM ,
       EN_AMO_CMD_UPDATE_IND ,
       EN_AMO_CMD_UNDEFINE
    };

     void OnMessage(const std::string& topic,std::shared_ptr<MESSAGE> message, TCallback callback);
     // Common Infrastructure of Active Object.
     static void* EntryOfThread(void* arg);
     void Run();
     void OnProxyRequest();
     pthread_t thr_id_;
     bool shutdown_;
     Fifo<std::function<void()>>send_queue_;
     int GetFd() const
     {
         return send_queue_.GetFd();
     }
};
#endif
