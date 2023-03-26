#include <iostream>
#include <vector>
#include <string>

#include <iomanip> 
#include <chrono>
#include <random>

template <typename NumericValueType>
struct MatrixStructDefine{
public:
    // コンストラクタ
    MatrixStructDefine();
    MatrixStructDefine(const int &rows, const int &cols);
    
    // デストラクタ
    virtual ~MatrixStructDefine();

    // 演算子オーバーロード
    MatrixStructDefine operator +=(const MatrixStructDefine matrix);
    MatrixStructDefine operator -=(const MatrixStructDefine matrix);
    MatrixStructDefine operator *=(const MatrixStructDefine matrix);
    MatrixStructDefine operator *=(const NumericValueType &matrix);
    MatrixStructDefine operator /=(const NumericValueType &matrix);
    MatrixStructDefine operator +(const MatrixStructDefine &matrix);
    MatrixStructDefine operator -(const MatrixStructDefine &matrix);
    MatrixStructDefine operator *(const MatrixStructDefine &matrix);
    MatrixStructDefine operator *(const NumericValueType &matrix);
    MatrixStructDefine operator /(const NumericValueType &matrix);
    bool operator ==(const MatrixStructDefine &matrix);
    bool operator !=(const MatrixStructDefine &matrix);
    
    // メソッド
    size_t rows();                              // 行数
    size_t cols();                              // 列数
    const NumericValueType &at(const int &row, const int &col) const;   // 要素へのアクセスメソッド
    NumericValueType &at(const int &row, const int &col);
    MatrixStructDefine transpose();
    MatrixStructDefine determinate();
    MatrixStructDefine inverse();
    std::string toString();
private:
    // 以下，データ構造（好きなのを選ぶ）
    //double *data;
    //double **data;
    //std::vector<double> data;
    //std::vector<std::vector<double>> data;
};

using Matrix32 = MatrixStructDefine<float>;
using Matrix = MatrixStructDefine<double>;
using Matrix128 = MatrixStructDefine<long double>;

int main(int argc, char **argv)
{
    if(argc < 2){
        std::cout << "input ./a.out [matrix_size]" << std::endl;
        return -1;
    }

    std::cout << std::fixed << std::setprecision(10);
    std::srand(std::time(nullptr));

    int size = std::atoi(argv[1]);
    Matrix matrix1(size, size);
    Matrix matrix2(size, size);

    std::chrono::high_resolution_clock::time_point start, end;
    start = std::chrono::high_resolution_clock::now();
    for(int i=0; i<size; i++){
        for(int j=0; j<size; j++){
            matrix1.at(i,j) = std::rand() % 10;
            matrix2.at(i,j) = std::rand() % 10;
        }
    }

    std::cout << (matrix1 + matrix2).toString() << std::endl;
    std::cout << (matrix1 - matrix2).toString() << std::endl;
    std::cout << (matrix1 * matrix2).toString() << std::endl;
    std::cout << (matrix1 * 100).toString() << std::endl;
    std::cout << (matrix1 / 100).toString() << std::endl;
    std::cout << matrix1.transpose().toString() << std::endl;
    std::cout << matrix1.determinate().toString() << std::endl;
    std::cout << matrix1.inverse().toString() << std::endl;
    end = std::chrono::high_resolution_clock::now();
    std::cout << "calc time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << "[ns]" << std::endl;
}

template<typename NumericValueType> std::string MatrixStructDefine<NumericValueType>::toString()
{
    std::string ret = "[";
    for(int i=0, rows=this->rows(); i<rows; i++){
        ret += "[";
        for(int j=0, cols=this->cols(); j<cols; j++){
            ret += std::to_string(this->at(i,j)) + ", ";
        }
        ret[ret.size() - 2] = ']';
    }
    ret += "]";
    return ret;
}