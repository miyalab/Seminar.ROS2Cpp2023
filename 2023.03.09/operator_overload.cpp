#include <iostream>

struct Date{
    // フィールド
    int year;
    int month;
    int day;

    // 演算子オーバーロード
    bool operator>(const Date &date) const;
    bool operator<=(const Date &date) const;
    bool operator<(const Date &date) const;
    bool operator>=(const Date &date) const;
    bool operator==(const Date &date) const;
    bool operator!=(const Date &date) const;
};

int main()
{
    Date date1, date2;
    if(date1 > date2){
        std::cout << "date1 large" << std::endl;
    }
}

bool Date::operator>(const Date &date) const
{
    return (this->year > date.year) ||
            (this->year == date.year && this->month > date.month) ||
            (this->year == date.year && this->month == date.month && this->day > date.day); 
}

bool Date::operator<=(const Date &date) const
{
    return !(*this > date);
}