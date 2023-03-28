#include <iostream>
#include <memory>
#include <thread>

std::shared_ptr<double> data1_ptr;
std::unique_ptr<double> data2_ptr;
std::unique_ptr<double> data3_ptr;

void thread1()
{
    //std::cout << *data1_ptr << std::endl;
    std::cout << data1_ptr << std::endl;
}

void thread2()
{
    //std::cout << *data2_ptr << std::endl;
    std::cout << data2_ptr.get() << std::endl;
}

int main()
{
    data1_ptr = std::make_shared<double>();
    data2_ptr = std::make_unique<double>();
    data3_ptr = std::move(data2_ptr);

    std::thread thread_1(thread1);
    std::thread thread_2(thread2);

    thread_1.join();
    thread_2.join();
}