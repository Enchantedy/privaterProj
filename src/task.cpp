#include <core/log.h>
#include <task.h>

using namespace core;

#define TAG ("task")

Task::Task()
{
    state_ = kTaskStateCreated;
}

Task::~Task()
{
    if (thread_ == nullptr) {
        return;
    }

    if (thread_->joinable()) {
        thread_->join();
        thread_ = nullptr;
    }
}

void Task::start()
{
    state_ = kTaskStateStart;
    thread_ = std::make_shared<std::thread>(&Task::threadFunction, this);
    log_info(TAG, "start task thread id 0x%08x\n", thread_->get_id());
}

void Task::stop()
{
    state_ = kTaskStateStop;
    log_info(TAG, "stop task exit thread id 0x%08x\n", thread_->get_id());
    thread_->join();
    thread_ = nullptr;
}

void Task::pause()
{
    state_ = kTaskStatePause;
    log_info(TAG, "pause task exit thread id 0x%08x\n", thread_->get_id());
    thread_->join();
    thread_ = nullptr;
}

void Task::resume()
{
    state_ = kTaskStateResume;
    thread_ = std::make_shared<std::thread>(&Task::threadFunction, this);
    log_info(TAG, "resume task thread id 0x%08x\n", thread_->get_id());
}

bool Task::isCreated()
{
    return state_ == kTaskStateCreated;
}

bool Task::isPaused()
{
    return state_ == kTaskStatePause;
}

bool Task::isRunning()
{
    return state_ == kTaskStateRunning;
}

bool Task::isStopped()
{
    return state_ == kTaskStateStop;
}

Task::TaskState Task::getState()
{
    return state_;
}

bool Task::waitComplete(int timeout_ms)
{
    int complete;

    std::unique_lock<std::mutex> lock(complete_mutex_);
    complete = complete_condition_.wait_for(lock, std::chrono::milliseconds(timeout_ms),  [&] {
        return state_ == kTaskStatePause || state_ == kTaskStateStop;
    });

    return complete;
}

void Task::threadFunction()
{
    if (state_ == kTaskStateStart) {
        onStart();
    } else if (state_ == kTaskStateResume) {
        onResume();
    } else {
        log_error(TAG, "bad task state at starup");
    }

    state_ = kTaskStateRunning;

    while (state_ == kTaskStateRunning)
    {
        taskLoop();
    }

    if (state_ == kTaskStatePause) {
        onPause();
    } else if (state_ == kTaskStateStop) {
        onStop();
    } else {
        log_error(TAG, "bad task state at endup");
    }
}

void Task::onStart()
{

}

void Task::onPause()
{

}

void Task::onResume()
{

}

void Task::onStop()
{

}

void Task::taskLoop()
{

}

bool Task::needPause()
{
    return false;
}

void Task::exit()
{
    state_ = kTaskStateStop;
    log_info(TAG, "task exit");
}
