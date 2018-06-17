#ifndef WORKER_THREAD_H
#define WORKER_THREAD_H

#include <atomic>
#include <string>
#include <thread>

class WorkerThread
{
public:
	WorkerThread(const std::string thread_name);

	inline bool hasThreadStarted() const
	{
		return m_threadStarted;
	}

    void startThread();
    void stopThread();

protected:
	virtual void onThreadStarted() { };
	virtual void onThreadStopped() { };
	virtual bool doWork() = 0;

private:
	void threadFunc();

private:
    // Multithreaded state
	const std::string m_threadName;
    std::atomic_bool m_exitSignaled;

	// Main Thread State
    bool m_threadStarted;
    std::thread m_workerThread;
};

#endif // WORKER_THREAD_H