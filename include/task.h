#ifndef TASK_H
#define TASK_H

#include <thread>
#include <memory>
#include <condition_variable>
#include <mutex>

class Task
{
public:
    enum TaskState {
        kTaskStateCreated = 1,      // 创建状态
        kTaskStateStart,            // 启动状态
        kTaskStateRunning,          // 运行状态
        kTaskStateResume,           // 恢复状态
        kTaskStatePause,            // 暂停状态
        kTaskStateStop              // 停止状态
    };

    enum TaskType {
        kTaskTypeAutoClean = 1,             // 自动清扫
        kTaskTypeFixedClean,                // 定点清扫
        kTaskTypeZoneClean,                 // 划区清扫
        kTaskTypeDivideZoneClean,           // 分房清扫
        kTaskTypeBreakpointResumeClean,     // 断点续扫
        kTaskTypeGotoDock,                  // 回充
    };

public:
    Task();

    ~Task();

    void start();

    void stop();

    void pause();

    void resume();

    bool isCreated();

    bool isPaused();

    bool isRunning();

    bool isStopped();

    TaskState getState();

    bool waitComplete(int timeout_ms = 100);

protected:
    // add return value for get create result?
    virtual void onStart();
    virtual void onPause();
    virtual void onResume();
    virtual void onStop();
    virtual void taskLoop();

    virtual bool needPause();

    void exit();

private:
    void threadFunction();

protected:
    std::shared_ptr<std::thread> thread_;
    std::condition_variable complete_condition_;
    std::mutex complete_mutex_;

private:
    volatile TaskState state_;
};

#endif