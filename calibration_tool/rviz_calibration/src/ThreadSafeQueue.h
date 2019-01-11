#ifndef THREADSAFEQUEUE
#define THREADSAFEQUEUE

#include <vector>
#include <mutex>
#include <memory>
#include <condition_variable>

namespace rviz_calibration
{
    template<typename T>
    class ThreadSafeQueue
    {
    public:
        ThreadSafeQueue() {}
        ~ThreadSafeQueue() {}

        void push(T new_data)
        {
            std::lock_guard<std::mutex> lk(mut);            // 1.全局加锁
            //data_queue.push(std::move(new_data));           // 2.push时独占锁
            data_queue.push_back(std::move(new_data));
            cond.notify_one();
        }
        void wait_and_pop(T& val)
        {
            std::unique_lock<std::mutex> ulk(mut);                    // 3.全局加锁
            cond.wait(ulk,[this]() { return !data_queue.empty(); });  // 4.front 和 pop_front时独占锁
            val=std::move(data_queue.front());
            //data_queue.pop();
            data_queue.pop_back();
        }
        std::shared_ptr<T> wait_and_pop()
        {
            std::unique_lock<std::mutex> ulk(mut);
            cond.wait(ulk,[this]() { return !data_queue.empty(); });
            std::shared_ptr<T> val(std::make_shared<T>(std::move(data_queue.front())));
            //data_queue.pop();
            data_queue.pop_back();
            return val;
        }
        bool try_pop(T& val)
        {
            std::lock_guard<std::mutex> lk(mut);
            if(data_queue.empty())
                return false;
            val=std::move(data_queue.front());
            //data_queue.pop();
            data_queue.pop_back();
            return true;
        }
        std::shared_ptr<T> try_pop()
        {
            std::shared_ptr<T> val;
            std::lock_guard<std::mutex> lk(mut);
            if(data_queue.empty())
                return val;
            val=std::make_shared<T>(std::move(data_queue.front()));
            //data_queue.pop();
            data_queue.pop_back();
            return val;
        }
        bool empty()
        {
            std::lock_guard<std::mutex> lk(mut);
            return data_queue.empty();
        }
    public:
    //private:
        //std::queue<T> data_queue;
        std::vector<T> data_queue;
        std::mutex mut;
        std::condition_variable cond;
    };
}
#endif // THREADSAFEQUEUE

