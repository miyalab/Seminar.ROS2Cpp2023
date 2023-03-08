// 素因数分解
#include <iostream>
#include <map>

std::map<long long, int> primeFactor(long long n)
{
    std::map<long long, int> ret;
    while(n%2==0){
        ret[2]++;
        n/=2;
    }
    for(long long i=3; i*i<=n; i+=2){
        while(n%i==0){
            ret[i]++;
            n/=i;
        }
    }
    if(n!=1) ret[n] = 1;
    return ret;
}

int main()
{
    long long num;
    std::cin >> num;

    for(const auto p: primeFactor(num)) std::cout << p.first << "^" << p.second << std::endl;
}