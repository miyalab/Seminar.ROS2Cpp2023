#include <iostream>
#include <thread>
#include <string>

void threadPrint(const std::string &msg)
{ 
    std::cout << std::this_thread::get_id() << ": " << msg << std::endl;
}

int main(){
    std::thread thread1(threadPrint, "hello");
    std::thread thread2(threadPrint, "world");
    
    thread1.join();
    thread2.join();
}
