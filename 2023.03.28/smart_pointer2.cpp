#include <memory>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

std::shared_ptr<std::string> data;
std::mutex data_mutex;

void callback(const std::shared_ptr<std::string> msg)
{
    data_mutex.lock();
    data = msg;
    data_mutex.unlock();
}

int main()
{
    while(1){ 
        data_mutex.lock();
        auto data_copy = data;
        data_mutex.unlock();
    }
}