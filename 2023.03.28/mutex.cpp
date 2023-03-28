#include <iostream>
#include <thread>
#include <mutex>

#include <chrono>

int cnt = 0;
int cnt2 = 0;
std::mutex cnt_mutex;

void adder()
{ 
    for(long long i=0; i<100000000; i++){
        cnt_mutex.lock();
        cnt++;
        cnt_mutex.unlock();
    }
}

int main(){
    auto start = std::chrono::high_resolution_clock::now();
    std::thread thread1(adder);
    std::thread thread2(adder);
    thread1.join();
    thread2.join();
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count() << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for(int i=0; i<200000000; i++) cnt2++;
    end = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count() << std::endl;
    
    std::cout << cnt << std::endl;
    std::cout << cnt2 << std::endl;
}
