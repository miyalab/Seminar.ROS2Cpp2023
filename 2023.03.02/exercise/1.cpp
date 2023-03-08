// クエリ
#include <iostream>
#include <string>
#include <map>

struct Date{
    int year = 2023;
    int month = 1;
    int day = 1;

    bool operator>(const Date &date) const {
        return this->year > date.year ||
                (this->year == date.year && this->month > date.month) ||
                (this->year == date.year && this->month == date.month && this->day > date.day);
    }
    bool operator<(const Date &date) const {
        return this->year < date.year ||
                (this->year == date.year && this->month < date.month) ||
                (this->year == date.year && this->month == date.month && this->day < date.day);
    }
    bool operator<=(const Date &date) const {return !(*this>date);}
    bool operator>=(const Date &date) const {return !(*this<date);}
    bool operator==(const Date &date) const {return (this->year == date.year && this->month == date.month && this->day == date.day);}
    bool operator!=(const Date &date) const {return !(*this==date);}
};

struct DataBaseStructure{
    int profit;
    int loss;
    int customers;
};

int main()
{
    int query;
    std::map<Date, DataBaseStructure> master_data;
    do{
        std::cin >> query;

        if(query==1){

        }
        else if(query == 12){
            Date date;
            std::cin >> date.year >> date.month >> date.day;
            master_data.erase(date);
        }
        else if(query == 13){
            Date date;
            DataBaseStructure input;
            std::cin >> date.year >> date.month >> date.day >> input.profit >> input.loss >> input.customers;
            master_data[date] = input;
        }
        else if(query == 14){
            for(const auto &d: master_data){
                std::cout << d.first.year << "/" << d.first.month << "/" << d.first.day << ": "
                            << d.second.profit << ", " << d.second.loss << ", " << d.second.customers << std::endl;
            }
        }
        else{
            std::cout << "not definition query number." << std::endl;
        }

    }while(query);
}