#include <iostream>

class Human{
public:
    Human();                // コンストラクタ
    Human(const int &_age); // コンストラクタのオーバーロード
    virtual ~Human();       // デストラクタ
    
    void setAge(const int &_age);
    const int &getAge() const;

    void say();

private:
    int age;
protected:

};

int main()
{
    Human human1;
    Human human2(23);

    human1.setAge(11);
    std::cout << human1.getAge() << std::endl;
    std::cout << human2.getAge() << std::endl;
}

Human::Human()
{

}

Human::Human(const int &_age)
{
    this->setAge(_age);
}

Human::~Human()
{

}

void Human::setAge(const int &_age)
{
    this->age = _age;
}

const int &Human::getAge() const
{
    return age;
}