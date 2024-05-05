
#ifndef RA_MESSAGE_QUEUE_H
#define RA_MESSAGE_QUEUE_H

#include "RAMessage.h"

/**
 * A safe message queue to be used for inter-thread communication.
 */
class MessageQueue {
public:
    MessageQueue();

    ~MessageQueue();

    void push(Message &&message);

    std::unique_ptr<Message> get();

private:
    class Impl;

    std::unique_ptr<Impl> impl;

};

#endif //RA_MESSAGE_QUEUE_H
