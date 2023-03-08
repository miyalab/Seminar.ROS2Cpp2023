// -1が入力されるまで入力を求めるプログラム
#include <iostream>

int main()
{
    int input;
    do{
        std::cin >> input;
    }while(input != -1);
}