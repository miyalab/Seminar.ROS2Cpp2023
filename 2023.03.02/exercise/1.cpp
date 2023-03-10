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

    std::string toString() const {return std::to_string(this->year) + "/" + std::to_string(this->month) + "/" + std::to_string(this->day);}
};

struct DataBaseStructure{
    int profit;
    int loss;
    int customers;

    DataBaseStructure operator+=(const DataBaseStructure& data){this->profit+=data.profit; this->loss+=data.loss; this->customers+=data.customers; return *this;}
    DataBaseStructure operator-=(const DataBaseStructure& data){this->profit-=data.profit; this->loss-=data.loss; this->customers-=data.customers; return *this;}
};

int main()
{
    int query;
    std::map<Date, DataBaseStructure> master_data;
    std::map<int, std::map<Date, int>> profit_data;
    std::map<int, std::map<Date, int>> loss_data;
    std::map<int, std::map<Date, int>> customers_data;
    DataBaseStructure sum_data;

    do{
        std::cin >> query;

        // 最大売上
        if(query==1){
            const int &profit = profit_data.rbegin()->first;
            const Date &date = profit_data.rbegin()->second.begin()->first;
            std::cout << date.toString() << ": " << profit << std::endl;

        }
        // 最低売上
        else if(query==2){
            const int &profit = profit_data.begin()->first;
            const Date &date = profit_data.begin()->second.begin()->first;
            std::cout << date.toString() << ": " << profit << std::endl;
        }
        // 売上合計
        else if(query==3){
            std::cout << sum_data.profit << std::endl;
        }
        // データの削除
        else if(query == 12){
            Date date;
            std::cin >> date.year >> date.month >> date.day;
            sum_data -= master_data[date];
            profit_data[master_data[date].profit].erase(date); 
            if(profit_data[master_data[date].profit].empty()) profit_data.erase(master_data[date].profit);
            
            loss_data[master_data[date].loss].erase(date);
            if(loss_data[master_data[date].loss].empty()) loss_data.erase(master_data[date].loss);

            customers_data[master_data[date].customers].erase(date);
            if(customers_data[master_data[date].customers].empty()) customers_data.erase(master_data[date].customers);
            master_data.erase(date);
        }
        // データの追加
        else if(query == 13){
            Date date;
            DataBaseStructure input;
            std::cin >> date.year >> date.month >> date.day >> input.profit >> input.loss >> input.customers;
            sum_data += input;
            master_data[date] = input;
            profit_data[input.profit][date] = loss_data[input.loss][date] = customers_data[input.customers][date] = 1;
        }
        // デバッグ用（データの表示）
        else if(query == 14){
            for(const auto &d: master_data){
                std::cout << d.first.toString() << ": " << d.second.profit << ", " << d.second.loss << ", " << d.second.customers << std::endl;
            }
        }
        else if(query != 0){
            std::cout << "not definition query number." << std::endl;
        }

    }while(query);
}