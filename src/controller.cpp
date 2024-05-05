#include <unistd.h>
#include <functional>
#include <algorithm>
#include  <iostream>
#include <sys/poll.h>
#include "controller.h"

Controller::Controller() 
{
    shutdown_ = false;
}

int Controller::Start()
{
	if(pthread_create(&thr_id_, nullptr, EntryOfThread,this) != 0)
	{
            return -1;
	}
	return 0;
}

void Controller::Shutdown()
{
	shutdown_ = true;
	if(pthread_cancel(thr_id_) != 0)
	{
	}
	void *res;
	int s=1;
	s = pthread_join(thr_id_,&res);
}

Controller::~Controller()
{
    std::cout << "Destroyed controller" << std::endl;
    std::cout.flush();
}
/*static*/
void* Controller::EntryOfThread(void* arg)
{
    Controller* pMgr = static_cast<Controller*>(arg);
    pMgr->Run();
}

void Controller::Run()
{
    std::vector<pollfd> poll_items;

    struct pollfd item;
    item.fd = GetFd();
    item.events = POLLIN;
    item.revents = 0;
	
    poll_items.push_back(item);

	while(!shutdown_)
	{
		int rc= poll(&poll_items[0], poll_items.size(), 0);
		if(rc <= 0)
		{
		}
		else
		{
			std::vector<pollfd>::const_iterator i;
			for(i = poll_items.begin(); i != poll_items.end(); ++i)
			{
				if((*i).revents != 0)
				{
					OnProxyRequest();
				}
			}
		}
		pthread_testcancel();
	}
}

void Controller::OnMessage(const std::string& p_topic, std::shared_ptr<MESSAGE> message, TCallback callback) 
{
    std::cout << "Controller::OnMessage hit"<<std::endl;
    int ret = 0;
    std::string result ="OK";
     
    spdlog::info("Controller::OnMessage sid [{}],did[{}],type[{}],content[{}]", 
                 message->sid,message->did,message->type,message->Union.smm_OutGoingRequest.PhoneNumber[1]);
    callback(result);
   
}

void Controller::Message(const std::string& topic,std::shared_ptr<MESSAGE> message, TCallback callback)
{
	std::function<void()>* admin = new std::function<void()> (std::bind(&Controller::OnMessage, this, topic, message, callback));
	send_queue_.push(admin);
}

void Controller::OnProxyRequest()
{
	std::function<void()>* f = send_queue_.Pop();
	while(f)
	{
		(*f)();
		delete f;
		f = send_queue_.Pop();
	}
}
     
Controller::OamAmoCommandType Controller::MapAmoCommandTypeToEnum(const std::string& p_topic)
{
    if( p_topic =="GET_REQ" )
        return OamAmoCommandType::EN_AMO_CMD_GET_AO_REQ;
    else if(p_topic =="CREATE_AO_REQ") 
        return OamAmoCommandType::EN_AMO_CMD_CREATE_AO_REQ;
    else if(p_topic =="DELETE_AO_REQ")
        return OamAmoCommandType::EN_AMO_CMD_DELETE_AO_REQ;
    else if(p_topic =="UPDATE_AO_REQ")
        return OamAmoCommandType::EN_AMO_CMD_UPDATE_AO_REQ;
    else if( p_topic =="GET_CFM" )
        return OamAmoCommandType::EN_AMO_CMD_GET_CFM;
    else if(p_topic =="ADD_CFM")
        return OamAmoCommandType::EN_AMO_CMD_ADD_CFM;
    else if(p_topic =="ADD_NRCELLCU_CFM")
        return OamAmoCommandType::EN_AMO_CMD_ADD_NRCELLCU_CFM;
    else if(p_topic =="SETUP_NRCELLCU_CFM")
        return OamAmoCommandType::EN_AMO_CMD_SETUP_NRCELLCU_CFM;
    else if(p_topic =="ACTIVE_NRCELLCU_CFM")
        return OamAmoCommandType::EN_AMO_CMD_ACTIVE_NRCELLCU_CFM;
    else if(p_topic =="DEACTIVE_NRCELLCU_CFM")
        return OamAmoCommandType::EN_AMO_CMD_DEACTIVE_NRCELLCU_CFM;
    else if(p_topic =="RELEASE_NRCELLCU_CFM")
        return OamAmoCommandType::EN_AMO_CMD_RELEASE_NRCELLCU_CFM;
    else if(p_topic =="DELETE_CFM")
        return OamAmoCommandType::EN_AMO_CMD_DELETE_CFM;
    else if(p_topic =="UPDATE_CFM")
        return OamAmoCommandType::EN_AMO_CMD_UPDATE_CFM;
    else if(p_topic =="NRCELLCU_STATUS_UPDATE")
        return OamAmoCommandType::EN_AMO_CMD_UPDATE_IND;
    else if(p_topic =="FAULT_INDICATION")
        return OamAmoCommandType::EN_AMO_CMD_FAULT_INDICATION;
    else if(p_topic =="FAULTCEASE_INDICATION")
        return OamAmoCommandType::EN_AMO_CMD_FAULTCEASE_INDICATION;
    else 
        return OamAmoCommandType::EN_AMO_CMD_UNDEFINE;
}

