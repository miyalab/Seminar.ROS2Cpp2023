#include <iostream>
#include <vector>
#include <string>

#include <iomanip> 
#include <chrono>
#include <random>

struct Matrix{
public:
    // コンストラクタ
    Matrix();
    Matrix(int rows, int cols);
    
    // デストラクタ
    virtual ~Matrix();

    // 演算子オーバーロード
    Matrix operator +=(Matrix matrix);
    Matrix operator -=(Matrix matrix);
    Matrix operator *=(Matrix matrix);
    Matrix operator *=(double matrix);
    Matrix operator /=(double matrix);
    Matrix operator +(Matrix matrix);
    Matrix operator -(Matrix matrix);
    Matrix operator *(Matrix matrix);
    Matrix operator *(double matrix);
    Matrix operator /(double matrix);
    bool operator ==(Matrix matrix);
    bool operator !=(Matrix matrix);
    
    // メソッド
    size_t rows();                              // 行数
    size_t cols();                              // 列数
    const double &at(int row, int col) const;   // 要素へのアクセスメソッド
    double &at(int row, int col);
    Matrix transpose();
    Matrix determinate();
    Matrix inverse();
    std::string toString();
private:
    // 以下，データ構造（好きなのを選ぶ）
    //double *data;
    //double **data;
    //std::vector<double> data;
    //std::vector<std::vector<double>> data;
};

int main(int argc, char **argv)
{
    if(argc < 2){
        std::cout << "input ./a.out [matrix_size]" << std::endl;
    }

    std::cout << std::fixed << std::setprecision(10);
    std::srand(std::time(nullptr));

    const size_t &size = std::atoi(argv[1]);
    Matrix matrix1(std::atoi(argv[1]), size);
    Matrix matrix2(std::atoi(argv[1]), size);

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

std::string Matrix::toString()
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