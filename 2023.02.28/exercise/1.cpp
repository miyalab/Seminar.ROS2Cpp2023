// 九九表
#include <iostream>
#include <iomanip>
int main()
{
    std::cout << std::setw(5) << "|";
    for(int i=1; i<10; i++) std::cout << std::setw(4) << i << "|";
    std::cout << std::endl;

    for(int i=1; i<10; i++){
        std::cout << std::setw(4) << i << "|";
        for(int j=1; j<10; j++) std::cout << std::setw(4) << j*i << "|";
        std::cout << std::endl;
    }
}