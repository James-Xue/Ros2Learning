#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

static void worker(int id, int loops, int &shared_counter, std::mutex &mu)
{
    for (int i = 0; i < loops; ++i)
    {
        {
            std::lock_guard<std::mutex> lock(mu);
            shared_counter += 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (i % 10 == 0)
        {
            std::lock_guard<std::mutex> lock(mu);
            std::cout << "worker=" << id << " i=" << i << " counter=" << shared_counter << "\n";
        }
    }
}

int main()
{
    int counter = 0;
    std::mutex mu;

    std::thread t1(worker, 1, 50, std::ref(counter), std::ref(mu));
    std::thread t2(worker, 2, 50, std::ref(counter), std::ref(mu));

    t1.join();
    t2.join();

    std::cout << "final counter=" << counter << "\n";

    // 练习：
    // 1) 删掉互斥锁，看看会不会出现数据竞争（注意：这属于未定义行为）
    // 2) 用 VS Code 的线程视图观察两个线程
    return 0;
}
