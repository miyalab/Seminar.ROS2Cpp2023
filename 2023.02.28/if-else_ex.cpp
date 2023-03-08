#include <iostream>
#include <cmath>

int main()
{
    double a,b,c;
    std::cin >> a >> b >> c;
    double d = b*b - 4*a*c;
    double rel = -b / (2*a);
    if(d > 0){
        double im = std::sqrt(d) / (2*a);
        std::cout << "x1: " << rel + im << std::endl;
        std::cout << "x2: " << rel - im << std::endl;
    }
    else if(d < 0){
        std::cout << "x1,2: " << rel << "±j" << std::sqrt(-d)/(2*a) << std::endl;
    }
    else{
        std::cout << "x: " << rel << std::endl;
    }
}