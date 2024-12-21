#ifndef MATRIX_HPP
#define MATRIX_HPP
#pragma once
#include <iostream>
#include <stdexcept>

class Matrix {
private:
    int size;        // Size of the matrix (for a square matrix, it's size x size)
    int** data;      // Pointer to 2D array of integers

public:
    // Constructor
    Matrix(int n);

    // Destructor
    ~Matrix();

    // Set value at a specific position
    void set(int row, int col, int value);

    // Get value at a specific position
    int get(int row, int col) const;

    // Print the matrix
    void print() const;

    // Get the size of the matrix
    int getSize() const;
};

// Function to copy a matrix into another matrix
void copyMatrix(const Matrix& source, Matrix& destination);

#endif // MATRIX_HPP
