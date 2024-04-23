#ifndef UTILS_MATRIX_UTILS_H_
#define UTILS_MATRIX_UTILS_H_
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <complex.h>
#include <malloc.h>
#include <float.h>

#define MATRIX_ORDER 2

uint8_t inverse(float inverse[MATRIX_ORDER][MATRIX_ORDER],float input[MATRIX_ORDER][MATRIX_ORDER]);
void multiply(float m1[MATRIX_ORDER][MATRIX_ORDER], float m2[MATRIX_ORDER][1], float res[MATRIX_ORDER][1]);

/*---------------------------------------------------------------------------------*/
//http://www.mymathlib.com/matrices/datamovement/transposing.html
void Transpose_Square_Matrix(float*, int);

//http://www.mymathlib.com/matrices/datamovement/copy_matrix.html
void Copy_Vector(float*, float*, int);
//http://www.mymathlib.com/matrices/datamovement/get_set_submatrix.html
void Get_Submatrix(float*, int, int, float*, int, int, int);
//http://www.mymathlib.com/matrices/arithmetic/AtB_ABt_AtA_AAt.html
void Matrix_x_Its_Transpose(float *C, float *A, int nrows, int ncols);
//http://www.mymathlib.com/matrices/arithmetic/mul_mat.html
void Multiply_Matrices(float *C, float *A, int rows, int cols, float *B, int cols1);
void Multiply_Matrix_by_Scalar(float *C , float *A, float x, int nrows, int ncols);
void Divide_Matrix_by_Scalar(float *C,float *A, float x, int nrows, int ncols);
void Transpose_Matrix(float *At, float *A, int nrows, int ncols);
void Add_Matrices(float *C ,float *A, float *B, int nrows, int ncols);
void Subtract_Matrices(float *C, float *A, float *B, int nrows,int ncols);
void Copy_Matrix(float *A, float *B, int nrows, int ncols);
//http://www.mymathlib.com/matrices/arithmetic/constants.html
void Identity_Matrix(float*, int);
//http://www.mymathlib.com/matrices/linearsystems/choleski.html
int Choleski_LU_Decomposition(float*, int);
int Choleski_LU_Inverse(float*, int);
int Choleski_LU_Solve(float *LU, float B[], float x[], int n);
//http://www.mymathlib.com/matrices/eigen/hessenberg.html
int Hessenberg_Form_Elementary(float*, float*, int);
void Hessenberg_Elementary_Transform(float*, float*, int[], int);

//http://www.mymathlib.com/c_source/matrices/eigen/qr_hessenberg_matrix.c
int QR_Hessenberg_Matrix(float*, float*, float[], float[], int, int);
void float_QR_Iteration(float *H, float *S, int min_row, int max_row, int n, float *shift, int iteration);

void One_Real_Eigenvalue(float[], float[], float[], int, float);
void Two_Eigenvalues(float*, float*, float[], float[], int, int, float);
void Update_Row(float*, float, float, int, int);
void Update_Column(float*, float, float, int, int);
void Update_Transformation(float*, float, float, int, int);
void Product_and_Sum_of_Shifts(float*, int, int, float*, float*, float*, int);
int Two_Consecutive_Small_Subdiagonal(float*, int, int, int, float, float);
void float_QR_Step(float*, int, int, int, float, float, float*, int);
void BackSubstitution(float*, float[], float[], int);
void BackSubstitute_Real_Vector(float*, float[], float[], int, float, int);
void BackSubstitute_Complex_Vector(float*, float[], float[], int, float, int);
void Calculate_Eigenvectors(float*, float*, float[], float[], int);
void Complex_Division(float, float, float, float, float*, float*);

//http://www.mymathlib.com/matrices/linearsystems/triangular.html
int Lower_Triangular_Inverse(float *L, int n);
int Lower_Triangular_Solve(float *L, float B[], float x[], int n);
int Upper_Triangular_Solve(float *U, float B[], float x[], int n);
int Upper_Triangular_Inverse(float *U, int n);

//http://www.mymathlib.com/matrices/datamovement/interchange_rows_or_columns.html
void Interchange_Rows(float *A, int row1, int row2, int ncols);
void Interchange_Columns(float *A, int col1, int col2, int nrows, int ncols);
#endif
