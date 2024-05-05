#ifndef _FD_EVENT_H
#define _FD_EVENT_H
 
#include <sys/eventfd.h>

class FdEvent
{
    public:
        FdEvent();
        ~FdEvent();
        int fd() const
        {
            return fd_;
        }
        bool wait();
        bool notify();
    private:
        int fd_;
};
#endif 
