#include <sys/time.h>
#include <ctime>

#include "TimerManager.h"

/*----------------------------------------------------------------------------*/

/*
 * Timer granularity.
 */
static const long TICK_MILLISECONDS = 100; /* 1/10th second */

/*
 * ctor
 */
TimerManager::TimerManager()
    : next_timer_id_(1) {
    timer_queue_ = new TimerQueue<Timer>;
}

/*
 * dtor, flush the timer queue and delete any pending event objects.
 */
TimerManager::~TimerManager() {
    Timer *timer = timer_queue_->pop();

    while (timer) {
        delete timer->event;
        delete timer;
        timer = timer_queue_->pop();
    }

    delete timer_queue_;
}

/*
 * Post an event to the timer queue.
 */
TimerManager::timer_id_t TimerManager::startTimer(TimerEvent *event,
                                                  long milliseconds) {
    unsigned int timer_id = getNextTimerId();
    struct timeval now;
    gettimeofday(&now, NULL);
    long tick = (now.tv_sec * (1000 / TICK_MILLISECONDS)) + (now.tv_usec / (TICK_MILLISECONDS * 1000));
    long interval = milliseconds / TICK_MILLISECONDS;
    Timer *timer = new Timer(timer_id, event);
    timer_queue_->push(tick, interval, timer);
    return timer_id;
}

/*
 * If there is an expired timer pending, remove it from the queue and
 * return its details.
 */
bool TimerManager::getExpiredTimer(TimerEvent *&event, timer_id_t &id) {
    struct timeval now;
    gettimeofday(&now, NULL);
    long tick = (now.tv_sec * (1000 / TICK_MILLISECONDS)) + (now.tv_usec / (TICK_MILLISECONDS * 1000));
    Timer *timer = timer_queue_->pop(tick);

    if (timer) {
        event = timer->event;
        id = timer->timer_id;
        delete timer;
        return true;
    }

    return false;
}

long TimerManager::getTopExpires() {
    long top_expires = -1;

    if (curl_timer_queue_->empty() && timer_queue_->empty())
        return top_expires;

    struct timespec now;
    clock_gettime(CLOCK_REALTIME_COARSE, &now);

    long tick = (now.tv_sec * (1000 / TICK_MILLISECONDS)) + (now.tv_nsec / (TICK_MILLISECONDS * 1000000));
    long interval;

    if (!curl_timer_queue_->empty()) {
        interval = (curl_timer_queue_->getTopExpires() - tick) * TICK_MILLISECONDS;
        if (interval > 0)
            top_expires = interval;
        else
            top_expires = 0;
    }

    if (!timer_queue_->empty()) {
        interval = (timer_queue_->getTopExpires() - tick) * TICK_MILLISECONDS;
        if (top_expires == -1) {
            if (interval > 0)
                top_expires = interval;
            else
                top_expires = 0;
        } else {
            if (interval > 0)
                top_expires = top_expires < interval ? top_expires : interval;
            else
                top_expires = 0;
        }
    }
    return top_expires;
}