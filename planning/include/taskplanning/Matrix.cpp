#include "Matrix.hpp"
    // Constructor that initializes the matrix with the given size
Matrix::Matrix(int n) : size(n) {
    if (n <= 0) {
        throw std::invalid_argument("Size must be greater than zero.");
    }
    
    // Dynamically allocate memory for the matrix
    data = new int*[size];
    for (int i = 0; i < size; ++i) {
        data[i] = new int[size]();  // Initialize all elements to 0
    }
}

    // Destructor to free the dynamically allocated memory
Matrix::~Matrix() {
    for (int i = 0; i < size; ++i) {
        delete[] data[i];
    }
    delete[] data;
}

    // Function to set a value at a specific position
void Matrix::set(int row, int col, int value) {
    if (row >= size || col >= size || row < 0 || col < 0) {
        throw std::out_of_range("Index out of range.");
    }
    data[row][col] = value;
}

    // Function to get a value at a specific position
int Matrix::get(int row, int col) const {
    if (row >= size || col >= size || row < 0 || col < 0) {
        throw std::out_of_range("Index out of range.");
    }
    return data[row][col];
}

    // Function to print the matrix
void Matrix::print() const {
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            std::cout << data[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

    // Function to get the size of the matrix
int Matrix::getSize() const {
    return size;
}

/*
int main() {
    try {
        // Create a 3x3 matrix
        Matrix mat(3);
        
        // Set some values
        mat.set(0, 0, 1);
        mat.set(0, 1, 2);
        mat.set(0, 2, 3);
        mat.set(1, 0, 4);
        mat.set(1, 1, 5);
        mat.set(1, 2, 6);
        mat.set(2, 0, 7);
        mat.set(2, 1, 8);
        mat.set(2, 2, 9);

        // Print the matrix
        mat.print();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}
*/