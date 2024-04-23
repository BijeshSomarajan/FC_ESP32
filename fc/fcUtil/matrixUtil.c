#include "matrixUtil.h"
#include <float.h>

// Function to get cofactor of A[p][q] in temp[][]. n is current dimension of A[][]
void getCofactor(float input[MATRIX_ORDER][MATRIX_ORDER], float output[MATRIX_ORDER][MATRIX_ORDER], int p, int q, int n) {
	int i = 0, j = 0;
	// Looping for each element of the matrix
	for (int row = 0; row < n; row++) {
		for (int col = 0; col < n; col++) {
			// Copying into temporary matrix only those element
			// which are not in given row and column
			if (row != p && col != q) {
				output[i][j++] = input[row][col];
				// Row is filled, so increase row index and
				// reset col index
				if (j == n - 1) {
					j = 0;
					i++;
				}
			}
		}

	}
}

/* Recursive function for finding determinant of matrix.
 n is current dimension of A[][]. */
float determinant(float input[MATRIX_ORDER][MATRIX_ORDER], int n) {
	float D = 0; // Initialize result
	// Base case : if matrix contains single element
	if (n == 1) {
		return input[0][0];
	} else {
		float temp[MATRIX_ORDER][MATRIX_ORDER]; // To store cofactors
		int sign = 1; // To store sign multiplier
		// Iterate for each element of first row
		for (int f = 0; f < n; f++) {
			// Getting Cofactor of A[0][f]
			getCofactor(input, temp, 0, f, n);
			D += sign * input[0][f] * determinant(temp, n - 1);
			// terms are to be added with alternate sign
			sign = -sign;
		}
		return D;
	}
}

// Function to get adjoint of A[MATRIX_ORDER][MATRIX_ORDER] in adj[MATRIX_ORDER][MATRIX_ORDER].
void adjoint(float input[MATRIX_ORDER][MATRIX_ORDER], float adj[MATRIX_ORDER][MATRIX_ORDER]) {
	if (MATRIX_ORDER == 1) {
		adj[0][0] = 1;
		return;
	} else {
		float temp[MATRIX_ORDER][MATRIX_ORDER];
		// temp is used to store cofactors of A[][]
		int sign = 1;
		for (int i = 0; i < MATRIX_ORDER; i++) {
			for (int j = 0; j < MATRIX_ORDER; j++) {
				// Get cofactor of A[i][j]
				getCofactor(input, temp, i, j, MATRIX_ORDER);
				// sign of adj[j][i] positive if sum of row
				// and column indexes is even.
				sign = ((i + j) % 2 == 0) ? 1 : -1;
				// Interchanging rows and columns to get the
				// transpose of the cofactor matrix
				adj[j][i] = (sign) * (determinant(temp, MATRIX_ORDER - 1));
			}
		}
	}
}

/**
 * Multiplies a 3X3 with 3X1 matrix
 */
void multiply(float m1[MATRIX_ORDER][MATRIX_ORDER], float m2[MATRIX_ORDER][1], float res[MATRIX_ORDER][1]) {
	int i, j, k;
	for (i = 0; i < MATRIX_ORDER; i++) {
		for (j = 0; j < 1; j++) {
			res[i][j] = 0;
			for (k = 0; k < MATRIX_ORDER; k++) {
				res[i][j] += (m1[i][k] * m2[k][j]);
			}
		}
	}
}

// Function to calculate and store inverse, returns false if
// matrix is singular
uint8_t inverse(float inverse[MATRIX_ORDER][MATRIX_ORDER],float input[MATRIX_ORDER][MATRIX_ORDER]) {
	// Find determinant of A[][]
	float det = determinant(input, MATRIX_ORDER);
	if (det == 0) {
		return 0;
	} else {
		// Find adjoint
		float adj[MATRIX_ORDER][MATRIX_ORDER];
		adjoint(input, adj);
		// Find Inverse using formula "inverse(A) = adj(A)/det(A)"
		for (int i = 0; i < MATRIX_ORDER; i++) {
			for (int j = 0; j < MATRIX_ORDER; j++) {
				inverse[i][j] = adj[i][j] / det;
			}
		}
		return 1;
	}
}

/*----------------------------------- New Matrix -----------------------------------*/
////////////////////////////////////////////////////////////////////////////////
//  void Transpose_Square_Matrix( float *A, int n )                          //
//                                                                            //
//  Description:                                                              //
//     Take the transpose of A and store in place.                            //
//                                                                            //
//  Arguments:                                                                //
//     float *A    Pointer to the first element of the matrix A.             //
//     int    n     The number of rows and columns of the matrix A.           //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     float A[N][N];                                                        //
//                                                                            //
//     (your code to initialize the matrix A)                                 //
//                                                                            //
//     Transpose_Square_Matrix( &A[0][0], N);                                 //
//     printf("The transpose of A is \n"); ...                                //
void Transpose_Square_Matrix(float *A, int n) {
	float *pA, *pAt;
	float temp;
	int i, j;

	for (i = 0; i < n; A += n + 1, i++) {
		pA = A + 1;
		pAt = A + n;
		for (j = i + 1; j < n; pA++, pAt += n, j++) {
			temp = *pAt;
			*pAt = *pA;
			*pA = temp;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//  void Copy_Matrix(float *A, float *B, int nrows, int ncols)              //
//                                                                            //
//  Description:                                                              //
//     Copy the nrows x ncols matrix B to the nrows x ncols matrix A.         //
//     i.e.    A = B.                                                         //
//     The memory locations of the source matrix, B, and the destination      //
//     matrix, A, must not overlap, otherwise the results are installation    //
//     dependent.                                                             //
//                                                                            //
//  Arguments:                                                                //
//     float *A    Pointer to the first element of the destination matrix    //
//                  A[nrows][ncols].                                          //
//     float *B    Pointer to the first element of the source matrix         //
//                  B[nrows][ncols].                                          //
//     int    nrows The number of rows matrices A and B.                      //
//     int    ncols The number of columns of the matrices A and B.            //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     float A[N][M],  B[N][M];                                              //
//                                                                            //
//     (your code to initialize the matrix B)                                 //
//                                                                            //
//     Copy_Matrix(&A[0][0], &B[0][0], N, M);                                 //
//     printf(" Matrix A is \n");                                             //
////////////////////////////////////////////////////////////////////////////////
void Copy_Matrix(float *A, float *B, int nrows, int ncols) {
	memcpy(A, B, sizeof(float) * nrows * ncols);
}

////////////////////////////////////////////////////////////////////////////////
//  void Copy_Vector(float *d, float *s, int n)                             //
//                                                                            //
//  Description:                                                              //
//     Copy the n dimensional vector s(source) to the n dimensional           //
//     vector d(destination).  The memory locations of the source and         //
//     destination vectors must not overlap, otherwise the results            //
//     are installation dependent.                                            //
//                                                                            //
//  Arguments:                                                                //
//      float *d  Pointer to the first element of the destination vector d.  //
//      float *s  Pointer to the first element of the source vector s.       //
//      int    n   The number of elements of the source / destination vectors.//
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     float v[N],  vd[N];                                                   //
//                                                                            //
//     (your code to initialize the vector v)                                 //
//                                                                            //
//     Copy_Vector(vd, v, N);                                                 //
//     printf(" Vector vd is \n");                                            //
////////////////////////////////////////////////////////////////////////////////
void Copy_Vector(float *d, float *s, int n) {
	memcpy(d, s, sizeof(float) * n);
}

////////////////////////////////////////////////////////////////////////////////
//  void Get_Submatrix(float *S, int mrows, int mcols,                       //
//                                   float *A, int ncols, int row, int col)  //
//                                                                            //
//  Description:                                                              //
//     Copy the mrows and mcols of the nrows x ncols matrix A starting with   //
//     A[row][col] to the submatrix S.                                        //
//     Note that S should be declared float S[mrows][mcols] in the calling   //
//     routine.                                                               //
//                                                                            //
//  Arguments:                                                                //
//     float *S    Destination address of the submatrix.                     //
//     int    mrows The number of rows of the matrix S.                       //
//     int    mcols The number of columns of the matrix S.                    //
//     float *A    Pointer to the first element of the matrix A[nrows][ncols]//
//     int    ncols The number of columns of the matrix A.                    //
//     int    row   The row of A corresponding to the first row of S.         //
//     int    col   The column of A corresponding to the first column of S.   //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     #define NB                                                             //
//     #define MB                                                             //
//     float A[M][N],  B[MB][NB];                                            //
//     int row, col;                                                          //
//                                                                            //
//     (your code to set the matrix A, the row number row and column number   //
//      col)                                                                  //
//                                                                            //
//     if ( (row >= 0) && (col >= 0) && ((row + MB) < M) && ((col + NB) < N) )//
//        Get_Submatrix(&B[0][0], MB, NB, &A[0][0], N, row, col);             //
//     printf("The submatrix B is \n"); ... }                                 //
////////////////////////////////////////////////////////////////////////////////
void Get_Submatrix(float *S, int mrows, int mcols, float *A, int ncols, int row, int col) {
	int number_of_bytes = sizeof(float) * mcols;
	for (A += row * ncols + col; mrows > 0; A += ncols, S += mcols, mrows--)
		memcpy(S, A, number_of_bytes);
}
////////////////////////////////////////////////////////////////////////////////
//  void Transpose_Matrix(float *At, float *A, int nrows, int ncols)        //
//                                                                            //
//  Description:                                                              //
//     Take the transpose of A and store in At, i.e. At = A'.                 //
//     The matrix At should be declared as float At[ncols][nrows] in the     //
//     calling routine, and the matrix A declared as float A[nrows[ncols].   //
//     In general, At and A should be disjoint i.e. their memory locations    //
//     should be distinct.                                                    //
//                                                                            //
//  Arguments:                                                                //
//     float *At   Pointer to the first element of the matrix At.            //
//     float *A    Pointer to the first element of the matrix A.             //
//     int    nrows The number of rows of the matrix A and number of columns  //
//                  of the matrix At.                                         //
//     int    ncols The number of columns of the matrix A and the number of   //
//                  rows of the matrix At.                                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     float A[M][N],  At[N][M];                                             //
//                                                                            //
//     (your code to initialize the matrix A)                                 //
//                                                                            //
//     Transpose_Matrix(&At[0][0], &A[0][0], M, N);                           //
//     printf("The transpose of A is the matrix At \n"); ...                  //
////////////////////////////////////////////////////////////////////////////////
void Transpose_Matrix(float *At, float *A, int nrows, int ncols) {
	float *pA;
	float *pAt;
	int i, j;

	for (i = 0; i < nrows; At += 1, A += ncols, i++) {
		pAt = At;
		pA = A;
		for (j = 0; j < ncols; pAt += nrows, j++)
			*pAt = pA[j];
	}
}
////////////////////////////////////////////////////////////////////////////////
//  void Matrix_x_Its_Transpose(float *C, float *A, int nrows, int ncols )  //
//                                                                            //
//  Description:                                                              //
//     Post multiply an nrows x ncols matrix A by its transpose.   The result //
//     is an  nrows x nrows square symmetric matrix C, i.e. C = A A', where ' //
//     denotes the transpose.                                                 //
//     I.e. C = (Cij), where Cij = Sum (Aik Ajk) where the sum extends from   //
//     k = 0 to ncols - 1.                                                    //
//                                                                            //
//     The matrix C should be declared as float C[nrows][nrows] in the       //
//     calling routine.  The memory allocated to C should not include any     //
//     memory allocated to A.                                                 //
//                                                                            //
//  Arguments:                                                                //
//     float *C    Pointer to the first element of the matrix C.             //
//     float *A    Pointer to the first element of the matrix A.             //
//     int    nrows The number of rows of matrix A.                           //
//     int    ncols The number of columns of the matrices A.                  //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     float A[M][N], C[M][M];                                               //
//                                                                            //
//     (your code to initialize the matrix A)                                 //
//                                                                            //
//     Matrix_x_Its_Transpose(&C[0][0], &A[0][0], M, N);                      //
//     printf("The matrix C = AA ' is \n"); ...                               //
////////////////////////////////////////////////////////////////////////////////
void Matrix_x_Its_Transpose(float *C, float *A, int nrows, int ncols) {
	int i, j, k;
	float *pAi0 = A;
	float *pAj0;
	float *pCi0 = C;
	float *pCji;

	for (i = 0; i < nrows; pCi0 += nrows, pAi0 += ncols, i++) {
		pCji = pCi0 + i;
		pAj0 = pAi0;
		for (j = i; j < nrows; pCji += nrows, j++) {
			*(pCi0 + j) = 0.0;
			for (k = 0; k < ncols; k++)
				*(pCi0 + j) += *(pAi0 + k) * *pAj0++;
			*pCji = *(pCi0 + j);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//  void Multiply_Matrices(float *C, float *A, int nrows, int ncols,        //
//                                                    float *B, int mcols)   //
//                                                                            //
//  Description:                                                              //
//     Post multiply the nrows x ncols matrix A by the ncols x mcols matrix   //
//     B to form the nrows x mcols matrix C, i.e. C = A B.                    //
//     The matrix C should be declared as float C[nrows][mcols] in the       //
//     calling routine.  The memory allocated to C should not include any     //
//     memory allocated to A or B.                                            //
//                                                                            //
//  Arguments:                                                                //
//     float *C    Pointer to the first element of the matrix C.             //
//     float *A    Pointer to the first element of the matrix A.             //
//     int    nrows The number of rows of the matrices A and C.               //
//     int    ncols The number of columns of the matrices A and the           //
//                   number of rows of the matrix B.                          //
//     float *B    Pointer to the first element of the matrix B.             //
//     int    mcols The number of columns of the matrices B and C.            //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     #define NB                                                             //
//     float A[M][N],  B[N][NB], C[M][NB];                                   //
//                                                                            //
//     (your code to initialize the matrices A and B)                         //
//                                                                            //
//     Multiply_Matrices(&C[0][0], &A[0][0], M, N, &B[0][0], NB);             //
//     printf("The matrix C is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Multiply_Matrices(float *C, float *A, int nrows, int ncols, float *B, int mcols) {
	float *pB;
	float *p_B;
	int i, j, k;
	for (i = 0; i < nrows; A += ncols, i++)
		for (p_B = B, j = 0; j < mcols; C++, p_B++, j++) {
			pB = p_B;
			*C = 0.0;
			for (k = 0; k < ncols; pB += mcols, k++)
				*C += *(A + k) * *pB;
		}
}

////////////////////////////////////////////////////////////////////////////////
// File: add_matrices.c                                                       //
// Routine(s):                                                                //
//    Add_Matrices                                                            //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Add_Matrices(float *C, float *A, float *B, int nrows, int ncols)  //
//                                                                            //
//  Description:                                                              //
//     This routine computes C = A + B where C is an nrows x ncols real matrix//
//     and A and B are given nrows x ncols real matrices.                     //
//                                                                            //
//     All matrices should be declared as " float X[nrows][ncols] " in the   //
//     calling routine, where X = A, B, C.                                    //
//                                                                            //
//  Arguments:                                                                //
//     float *C    Address of the first element of the matrix C.             //
//     float *A    Address of the first element of the matrix A.             //
//     float *B    Address of the first element of the matrix B.             //
//     int    nrows The number of rows of matrices A, B, and C.               //
//     int    ncols The number of columns of the matrices A, B, and C.        //
//                                                                            //
//  Return Values:                                                            //
//      void                                                                  //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     float A[M][N],  B[M][N], C[M][N];                                     //
//                                                                            //
//     (your code to initialize the matrices A and B)                         //
//                                                                            //
//     Add_Matrices((float *) C, &A[0][0], &B[0][0], M, N);                  //
//     printf("The matrix C = A + B is \n"); ...                              //
////////////////////////////////////////////////////////////////////////////////
void Add_Matrices(float *C, float *A, float *B, int nrows, int ncols) {
	int i;
	int n = nrows * ncols;
	for (i = 0; i < n; i++)
		C[i] = A[i] + B[i];
}

////////////////////////////////////////////////////////////////////////////////
//  void Subtract_Matrices(float *C, float *A, float *B, int nrows,        //
//                                                               int ncols)   //
//                                                                            //
//  Description:                                                              //
//     This routine computes C = A - B where C is an nrows x ncols real matrix//
//     and A and B are given nrows x ncols real matrices.                     //
//                                                                            //
//     All matrices should be declared as " float X[nrows][ncols] " in the   //
//     calling routine, where X = A, B, C.                                    //
//                                                                            //
//  Arguments:                                                                //
//     float *C    Pointer to the first element of the matrix C.             //
//     float *A    Pointer to the first element of the matrix A.             //
//     float *B    Pointer to the first element of the matrix B.             //
//     int    nrows The number of rows of matrices A and B.                   //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     float A[M][N],  B[M][N], C[M][N];                                     //
//                                                                            //
//     (your code to initialize the matrices A and B)                         //
//                                                                            //
//     Subtract_Matrices(&C[0][0], &A[0][0], &B[0][0], M, N);                 //
//     printf("The matrix C = A - B  is \n"); ...                             //
////////////////////////////////////////////////////////////////////////////////
void Subtract_Matrices(float *C, float *A, float *B, int nrows, int ncols) {
	int i;
	int n = nrows * ncols;
	for (i = 0; i < n; i++)
		C[i] = A[i] - B[i];
}

////////////////////////////////////////////////////////////////////////////////
//  void Multiply_Matrix_by_Scalar(float *A, float x, int nrows, int ncols) //
//                                                                            //
//  Description:                                                              //
//     Multiply each element of the matrix A by the scalar x.                 //
//                                                                            //
//  Arguments:                                                                //
//     float *A    Pointer to the first element of the matrix A.             //
//     float x     Scalar to multipy each element of the matrix A.           //
//     int    nrows The number of rows of matrix A.                           //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     float A[M][N],  x;                                                    //
//                                                                            //
//     (your code to initialize the matrix A and scalar x)                    //
//                                                                            //
//     Multiply_Matrix_by_Scalar(&A[0][0], x, M, N);                          //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Multiply_Matrix_by_Scalar(float *C, float *A, float x, int nrows, int ncols) {
	int n = nrows * ncols - 1;
	for (; n >= 0; n--)
		C[n] = A[n] * x;
}

////////////////////////////////////////////////////////////////////////////////
//  void Divide_Matrix_by_Scalar(float *A, float x, int nrows, int ncols)   //
//                                                                            //
//  Description:                                                              //
//     Divide each element of the matrix A by the scalar x.                   //
//     i.e.       A[i][j] <- A[i][j] / x for all i,j.                         //
//                                                                            //
//  Arguments:                                                                //
//     float *A    Pointer to the first element of the matrix A.             //
//     float x     The non-zero scalar used to divide each element of the    //
//                  matrix A.                                                 //
//     int    nrows The number of rows of the matrix A.                       //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     float A[M][N],  x;                                                    //
//                                                                            //
//     (your code to initialize the matrix A and scalar x)                    //
//                                                                            //
//     if ( x != 0.0)  Divide_Matrix_by_Scalar(&A[0][0], x, M, N);            //
//      printf("The matrix A is \n"); ...                                     //
////////////////////////////////////////////////////////////////////////////////
void Divide_Matrix_by_Scalar(float *C, float *A, float x, int nrows, int ncols) {
	int i;
	int n = ncols * nrows;
	for (i = 0; i < n; i++)
		C[i] = A[i] / x;
}

////////////////////////////////////////////////////////////////////////////////
//  void Identity_Matrix(float *A, int n)                                    //
//                                                                            //
//  Description:                                                              //
//     Set the square n x n matrix A equal to the identity matrix, i.e.       //
//     A[i][j] = 0 if i != j and A[i][i] = 1.                                 //
//                                                                            //
//  Arguments:                                                                //
//     float *A    Pointer to the first element of the matrix A.             //
//     int    n     The number of rows and columns of the matrix A.           //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     float A[N][N];                                                        //
//                                                                            //
//     Identity_Matrix(&A[0][0], N);                                          //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Identity_Matrix(float *A, int n) {
	int i, j;
	for (i = 0; i < n - 1; i++) {
		*A++ = 1.0;
		for (j = 0; j < n; j++)
			*A++ = 0.0;
	}
	*A = 1.0;
}

////////////////////////////////////////////////////////////////////////////////
//  int Choleski_LU_Decomposition(float *A, int n)                           //
//                                                                            //
//  Description:                                                              //
//     This routine uses Choleski's method to decompose the n x n positive    //
//     definite symmetric matrix A into the product of a lower triangular     //
//     matrix L and an upper triangular matrix U equal to the transpose of L. //
//     The original matrix A is replaced by L and U with L stored in the      //
//     lower triangular part of A and the transpose U in the upper triangular //
//     part of A. The original matrix A is therefore destroyed.               //
//                                                                            //
//     Choleski's decomposition is performed by evaluating, in order, the     //
//     following pair of expressions for k = 0, ... ,n-1 :                    //
//       L[k][k] = sqrt( A[k][k] - ( L[k][0] ^ 2 + ... + L[k][k-1] ^ 2 ) )    //
//       L[i][k] = (A[i][k] - (L[i][0]*L[k][0] + ... + L[i][k-1]*L[k][k-1]))  //
//                          / L[k][k]                                         //
//     and subsequently setting                                               //
//       U[k][i] = L[i][k], for i = k+1, ... , n-1.                           //
//                                                                            //
//     After performing the LU decomposition for A, call Choleski_LU_Solve    //
//     to solve the equation Ax = B or call Choleski_LU_Inverse to calculate  //
//     the inverse of A.                                                      //
//                                                                            //
//  Arguments:                                                                //
//     float *A   On input, the pointer to the first element of the matrix   //
//                 A[n][n].  On output, the matrix A is replaced by the lower //
//                 and upper triangular Choleski factorizations of A.         //
//     int     n   The number of rows and/or columns of the matrix A.         //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix A is not positive definite symmetric (within   //
//                  working accuracy).                                        //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     float A[N][N];                                                        //
//                                                                            //
//     (your code to initialize the matrix A)                                 //
//     err = Choleski_LU_Decomposition((float *) A, N);                      //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else { printf(" The LLt decomposition of A is \n");                    //
//           ...                                                              //
////////////////////////////////////////////////////////////////////////////////
int Choleski_LU_Decomposition(float *A, int n) {
	int i, k, p;
	float *p_Lk0;                   // pointer to L[k][0]
	float *p_Lkp;                   // pointer to L[k][p]
	float *p_Lkk;                   // pointer to diagonal element on row k.
	float *p_Li0;                   // pointer to L[i][0]
	float reciprocal;
	for (k = 0, p_Lk0 = A; k < n; p_Lk0 += n, k++) {
//            Update pointer to row k diagonal element.
		p_Lkk = p_Lk0 + k;
//            Calculate the difference of the diagonal element in row k
//            from the sum of squares of elements row k from column 0 to
//            column k-1.
		for (p = 0, p_Lkp = p_Lk0; p < k; p_Lkp += 1, p++) {
			*p_Lkk -= *p_Lkp * *p_Lkp;
		}
//            If diagonal element is not positive, return the error code,
//            the matrix is not positive definite symmetric.
		if (*p_Lkk <= 0.0) {
			return -1;
		}
//            Otherwise take the square root of the diagonal element.
		*p_Lkk = sqrt(*p_Lkk);
		reciprocal = 1.0 / *p_Lkk;
//            For rows i = k+1 to n-1, column k, calculate the difference
//            between the i,k th element and the inner product of the first
//            k-1 columns of row i and row k, then divide the difference by
//            the diagonal element in row k.
//            Store the transposed element in the upper triangular matrix.
		p_Li0 = p_Lk0 + n;
		for (i = k + 1; i < n; p_Li0 += n, i++) {
			for (p = 0; p < k; p++) {
				*(p_Li0 + k) -= *(p_Li0 + p) * *(p_Lk0 + p);
			}
			*(p_Li0 + k) *= reciprocal;
			*(p_Lk0 + i) = *(p_Li0 + k);
		}
	}
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//  int Choleski_LU_Inverse(float *LU,  int n)                               //
//                                                                            //
//  Description:                                                              //
//     This routine uses Choleski's method to find the inverse of the matrix  //
//     A.  This routine is called after the matrix A has been decomposed      //
//     into a product of a lower triangular matrix L and an upper triangular  //
//     matrix U which is the transpose of L. The matrix A is the product of   //
//     the L and U.  Upon completion, the inverse of A is stored in LU so     //
//     that the matrix LU is destroyed.                                       //
//                                                                            //
//  Arguments:                                                                //
//     float *LU  On input, the pointer to the first element of the matrix   //
//                 whose elements form the lower and upper triangular matrix  //
//                 factors of A.  On output, the matrix LU is replaced by the //
//                 inverse of the matrix A equal to the product of L and U.   //
//     int     n   The number of rows and/or columns of the matrix LU.        //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix L is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     float A[N][N], B[N], x[N];                                            //
//                                                                            //
//     (your code to create matrix A and column vector B)                     //
//     err = Choleski_LU_Decomposition(&A[0][0], N);                          //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else {                                                                 //
//        err = Choleski_LU_Inverse(&A[0][0], n);                             //
//        if (err < 0) printf(" Matrix A is singular\n");                     //
//        else printf(" The inverse is \n");                                  //
//           ...                                                              //
//     }                                                                      //
////////////////////////////////////////////////////////////////////////////////
int Choleski_LU_Inverse(float *LU, int n) {
	int i, j, k;
	float *p_i, *p_j, *p_k;
	float sum;
	if (Lower_Triangular_Inverse(LU, n) < 0)
		return -1;
//         Premultiply L inverse by the transpose of L inverse.
	for (i = 0, p_i = LU; i < n; i++, p_i += n) {
		for (j = 0, p_j = LU; j <= i; j++, p_j += n) {
			sum = 0.0;
			for (k = i, p_k = p_i; k < n; k++, p_k += n) {
				sum += *(p_k + i) * *(p_k + j);
			}
			*(p_i + j) = sum;
			*(p_j + i) = sum;
		}
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//  int Choleski_LU_Solve(float *LU, float *B, float *x,  int n)           //
//                                                                            //
//  Description:                                                              //
//     This routine uses Choleski's method to solve the linear equation       //
//     Ax = B.  This routine is called after the matrix A has been decomposed //
//     into a product of a lower triangular matrix L and an upper triangular  //
//     matrix U which is the transpose of L. The matrix A is the product LU.  //
//     The solution proceeds by solving the linear equation Ly = B for y and  //
//     subsequently solving the linear equation Ux = y for x.                 //
//                                                                            //
//  Arguments:                                                                //
//     float *LU  Pointer to the first element of the matrix whose elements  //
//                 form the lower and upper triangular matrix factors of A.   //
//     float *B   Pointer to the column vector, (n x 1) matrix, B            //
//     float *x   Solution to the equation Ax = B.                           //
//     int     n   The number of rows and/or columns of the matrix LU.        //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix L is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     float A[N][N], B[N], x[N];                                            //
//                                                                            //
//     (your code to create matrix A and column vector B)                     //
//     err = Choleski_LU_Decomposition(&A[0][0], N);                          //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else {                                                                 //
//        err = Choleski_LU_Solve(&A[0][0], B, x, n);                         //
//        if (err < 0) printf(" Matrix A is singular\n");                     //
//        else printf(" The solution is \n");                                 //
//           ...                                                              //
//     }                                                                      //
////////////////////////////////////////////////////////////////////////////////
int Choleski_LU_Solve(float *LU, float B[], float x[], int n) {
//         Solve the linear equation Ly = B for y, where L is a lower
//         triangular matrix.
	if (Lower_Triangular_Solve(LU, B, x, n) < 0)
		return -1;
//         Solve the linear equation Ux = y, where y is the solution
//         obtained above of Ly = B and U is an upper triangular matrix.
	return Upper_Triangular_Solve(LU, x, x, n);
}

////////////////////////////////////////////////////////////////////////////////
//  int Hessenberg_Form_Elementary(float *A, float *S, int n)               //
//                                                                            //
//  Description:                                                              //
//     This program transforms the square matrix A to a similar matrix in     //
//     Hessenberg form by a multiplying A on the right by a sequence of       //
//     elementary transformations and on the left by the sequence of inverse  //
//     transformations.                                                       //
//     Def:  Two matrices A and B are said to be similar if there exists a    //
//           nonsingular matrix S such that A S = S B.                        //
//     Def   A Hessenberg matrix is the sum of an upper triangular matrix and //
//           a matrix all of whose components are 0 except possibly on its    //
//           subdiagonal.  A Hessenberg matrix is sometimes said to be almost //
//           upper triangular.                                                //
//     The algorithm proceeds by successivly selecting columns j = 0,...,n-3  //
//     and then assuming that columns 0, ..., j-1 have been reduced to Hessen-//
//     berg form, for rows j+1 to n-1, select that row j' for which |a[j'][j]|//
//     is a maximum and interchange rows j+1 and j' and columns j+1 and j'.   //
//     Then for each i = j+2 to n-1, let x = a[i][j] / a[j+1][j] subtract     //
//     x * row j+1 from row i and add x * column i to column j+1.             //
//                                                                            //
//  Arguments:                                                                //
//     float *A     On input a pointer to the first element of the matrix    //
//                   A[n][n].  The matrix A is replaced with the matrix H,    //
//                   a matrix in Hessenberg form similar to A.                //
//     float *S     On output the transform such that A S = S H.             //
//                   The matrix S should be dimensioned at least n x n in the //
//                   calling routine.                                         //
//     int    n      The number of rows or columns of the matrix A.           //
//                                                                            //
//  Return Values:                                                            //
//      0  Success                                                            //
//     -1  Failure - Not enough memory                                        //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     float A[N][N], S[N][N];                                               //
//                                                                            //
//     (your code to create the matrix A)                                     //
//     if (Hessenberg_Form_Elementary(&A[0][0], (float*)S, N ) < 0) {        //
//        printf("Not enough memory\n"); exit(0);                             //
//     }                                                                      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
int Hessenberg_Form_Elementary(float *A, float *S, int n) {
	int i, j, col, row;
	int *perm;
	float *p_row, *pS_row;
	float max;
	float s;
	float *pA, *pB, *pC, *pS;

	// n x n matrices for which n <= 2 are already in Hessenberg form

	if (n <= 1) {
		*S = 1.0;
		return 0;
	}
	if (n == 2) {
		*S++ = 1.0;
		*S++ = 0.0;
		*S++ = 1.0;
		*S = 0.0;
		return 0;
	}

	// Allocate working memory

	perm = (int*) malloc(n * sizeof(int));
	if (perm == 0)
		return -1;             // not enough memory

	// For each column use Elementary transformations
	//   to zero the entries below the subdiagonal.

	p_row = A + n;
	pS_row = S + n;
	for (col = 0; col < (n - 2); p_row += n, pS_row += n, col++) {

		// Find the row in column "col" with maximum magnitude where
		// row >= col + 1.

		row = col + 1;
		perm[row] = row;
		for (pA = p_row + col, max = 0.0, i = row; i < n; pA += n, i++)
			if (fabs(*pA) > max) {
				perm[row] = i;
				max = fabs(*pA);
			}

		// If perm[row] != row, then interchange row "row" and row
		// perm[row] and interchange column "row" and column perm[row].

		if (perm[row] != row) {
			Interchange_Rows(A, row, perm[row], n);
			Interchange_Columns(A, row, perm[row], n, n);
		}

		// Zero out the components lying below the subdiagonal.

		pA = p_row + n;
		pS = pS_row + n;
		for (i = col + 2; i < n; pA += n, pS += n, i++) {
			s = *(pA + col) / *(p_row + col);
			for (j = 0; j < n; j++)
				*(pA + j) -= *(p_row + j) * s;
			*(pS + col) = s;
			for (j = 0, pB = A + col + 1, pC = A + i; j < n; pB += n, pC += n, j++)
				*pB += s * *pC;
		}
	}
	pA = A + n + n;
	pS = S + n + n;
	for (i = 2; i < n; pA += n, pS += n, i++)
		Copy_Vector(pA, pS, i - 1);

	Hessenberg_Elementary_Transform(A, S, perm, n);

	free(perm);
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//  static void Hessenberg_Elementary_Transform(float* H, float *S,         //
//                                                        int perm[], int n)  //
//                                                                            //
//  Description:                                                              //
//     Given a n x n matrix A, let H be the matrix in Hessenberg form similar //
//     to A, i.e. A S = S H.  If v is an eigenvector of H with eigenvalue z,  //
//     i.e. Hv = zv, then ASv = SHv = z Sv, i.e. Sv is the eigenvector of A   //
//     with corresponding eigenvalue z.                                       //
//     This routine returns S where S is the similarity transformation such   //
//     that A S = S H.                                                        //
//                                                                            //
//  Arguments:                                                                //
//     float* H     On input a matrix in Hessenberg form with transformation //
//                   elements stored below the subdiagonal part.              //
//                   On output the matrix in Hessenberg form with elements    //
//                   below the subdiagonal zeroed out.                        //
//     float* S     On output, the transformations matrix such that          //
//                   A S = S H.                                               //
//     int    perm[] Array of row/column interchanges.                        //
//     int    n      The order of the matrices H and S.                       //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void Hessenberg_Elementary_Transform(float *H, float *S, int perm[], int n) {
	int i, j;
	float *pS, *pH;

	Identity_Matrix(S, n);
	for (i = n - 2; i >= 1; i--) {
		pH = H + n * (i + 1);
		pS = S + n * (i + 1);
		for (j = i + 1; j < n; pH += n, pS += n, j++) {
			*(pS + i) = *(pH + i - 1);
			*(pH + i - 1) = 0.0;
		}
		if (perm[i] != i) {
			pS = S + n * i;
			pH = S + n * perm[i];
			for (j = i; j < n; j++) {
				*(pS + j) = *(pH + j);
				*(pH + j) = 0.0;
			}
			*(pH + i) = 1.0;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//  int QR_Hessenberg_Matrix( float *H, float *S, float eigen_real[],      //
//                     float eigen_imag[], int n, int max_iteration_count)   //
//                                                                            //
//  Description:                                                              //
//     This program calculates the eigenvalues and eigenvectors of a matrix   //
//     in Hessenberg form. This routine is adapted from the routine 'hql2'    //
//     appearing in 'Handbook for Automatic Computation, vol 2: Linear        //
//     Algebra' published by Springer Verlag (1971) edited by Wilkinson and   //
//     Reinsch, Contribution II/15 Eigenvectors of Real and Complex Matrices  //
//     by LR and QR Triangularizations by Peters and Wilkinson.               //
//                                                                            //
//  Arguments:                                                                //
//     float *H                                                              //
//            Pointer to the first element of the real n x n matrix H in upper//
//            Hessenberg form.                                                //
//     float *S                                                              //
//            If H is the primary data matrix, the matrix S should be set     //
//            to the identity n x n identity matrix on input.  If H is        //
//            derived from an n x n matrix A, then S should be the            //
//            transformation matrix such that AS = SH.  On output, the i-th   //
//            column of S corresponds to the i-th eigenvalue if that eigen-   //
//            value is real and the i-th and (i+1)-st columns of S correspond //
//            to the i-th eigenvector with the real part in the i-th column   //
//            and positive imaginary part in the (i+1)-st column if that      //
//            eigenvalue is complex with positive imaginary part.  The        //
//            eigenvector corresponding to the eigenvalue with negative       //
//            imaginary part is the complex conjugate of the eigenvector      //
//            corresponding to the complex conjugate of the eigenvalue.       //
//            If on input, S was the identity matrix, then the columns are    //
//            the eigenvectors of H as described, if S was a transformation   //
//            matrix so that AS = SH, then the columns of S are the           //
//            eigenvectors of A as described.                                 //
//     float eigen_real[]                                                    //
//            Upon return, eigen_real[i] will contain the real part of the    //
//            i-th eigenvalue.                                                //
//     float eigen_imag[]                                                    //
//            Upon return, eigen_ima[i] will contain the imaginary part of    //
//            the i-th eigenvalue.                                            //
//     int    n                                                               //
//            The number of rows or columns of the upper Hessenberg matrix A. //
//     int    max_iteration_count                                             //
//            The maximum number of iterations to try to find an eigenvalue   //
//            before quitting.                                                //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - Unable to find an eigenvalue within 'max_iteration_count' //
//                  iterations.                                               //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define MAX_ITERATION_COUNT                                            //
//     float H[N][N], S[N][N], eigen_real[N], eigen_imag[N];                 //
//     int k;                                                                 //
//                                                                            //
//     (code to initialize H[N][N] and S[N][N])                               //
//     k = QR_Hessenberg_Matrix( (float*)H, (float*)S, eigen_real,          //
//                                      eigen_imag, N, MAX_ITERATION_COUNT);  //
//     if (k < 0) {printf("Failed"); exit(1);}                                //
////////////////////////////////////////////////////////////////////////////////
int QR_Hessenberg_Matrix(float *H, float *S, float eigen_real[], float eigen_imag[], int n, int max_iteration_count) {
	int i;
	int row;
	int iteration;
	int found_eigenvalue;
	float shift = 0.0;
	float *pH;

	for (row = n - 1; row >= 0; row--) {
		found_eigenvalue = 0;
		for (iteration = 1; iteration <= max_iteration_count; iteration++) {
			// Search for small subdiagonal element
			for (i = row, pH = H + row * n; i > 0; i--, pH -= n) {
				if (fabs(*(pH + i - 1)) <= DBL_EPSILON * (fabs(*(pH - n + i - 1)) + fabs(*(pH + i)))) {
					break;
				}
			}
			// If the subdiagonal element on row "row" is small, then
			// that row element is an eigenvalue.  If the subdiagonal
			// element on row "row-1" is small, then the eigenvalues
			// of the 2x2 diagonal block consisting rows "row-1" and
			// "row" are eigenvalues.  Otherwise perform a float QR
			// iteration.
			switch (row - i) {
			case 0: // One real eigenvalue
				One_Real_Eigenvalue(pH, eigen_real, eigen_imag, i, shift);
				found_eigenvalue = 1;
				break;
			case 1: // Either two real eigenvalues or a complex pair
				row--;
				Two_Eigenvalues(H, S, eigen_real, eigen_imag, n, row, shift);
				found_eigenvalue = 1;
				break;
			default:
				float_QR_Iteration(H, S, i, row, n, &shift, iteration);
			}
			if (found_eigenvalue)
				break;
		}
		if (iteration > max_iteration_count)
			return -1;
	}

	BackSubstitution(H, eigen_real, eigen_imag, n);
	Calculate_Eigenvectors(H, S, eigen_real, eigen_imag, n);

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//  static void float_QR_Iteration(float *H, float *S, int min_row,        //
//                         int max_row, int n, float* shift, int iteration)  //
//                                                                            //
//  Description:                                                              //
//     Calculate the trace and determinant of the 2x2 matrix:                 //
//                        | H[k-1][k-1]  H[k-1][k] |                          //
//                        |  H[k][k-1]    H[k][k]  |                          //
//     unless iteration = 0 (mod 10) in which case increment the shift and    //
//     decrement the first k elements of the matrix H, then fudge the trace   //
//     and determinant by  trace = 3/2( |H[k][k-1]| + |H[k-1][k-2]| and       //
//     det = 4/9 trace^2.                                                     //
//                                                                            //
//  Arguments:                                                                //
//     float *H                                                              //
//            Pointer to the matrix H in Hessenberg form.                     //
//     float *S                                                              //
//            Pointer to the transformation matrix S.                         //
//     int    min_row                                                         //
//            The top-most row in which the off-diagonal element of H is      //
//            negligible.  If no such row exists, then min_row = 0.           //
//     int    max_row                                                         //
//            The maximum row of the block 2 x 2 diagonal matrix used to      //
//            estimate the two eigenvalues for the two implicit shifts.       //
//     int    n                                                               //
//            The dimensions of the matrix H and S.                           //
//     float *shift                                                          //
//            The cumulative exceptional shift of the diagonal elements of    //
//            the matrix H.                                                   //
//     int    iteration                                                       //
//            Current iteration count.                                        //
////////////////////////////////////////////////////////////////////////////////
void float_QR_Iteration(float *H, float *S, int min_row, int max_row, int n, float *shift, int iteration) {
	int k;
	float trace, det;

	Product_and_Sum_of_Shifts(H, n, max_row, shift, &trace, &det, iteration);
	k = Two_Consecutive_Small_Subdiagonal(H, min_row, max_row, n, trace, det);
	float_QR_Step(H, min_row, max_row, k, trace, det, S, n);
}

////////////////////////////////////////////////////////////////////////////////
//  static void One_Real_Eigenvalue( float Hrow[], float eigen_real[],      //
//                                float eigen_imag[], int row, float shift) //
//                                                                            //
//  Arguments:                                                                //
//     float Hrow[]                                                          //
//            Pointer to the row "row" of the matrix in Hessenberg form.      //
//     float eigen_real[]                                                    //
//            Array of the real parts of the eigenvalues.                     //
//     float eigen_imag[]                                                    //
//            Array of the imaginary parts of the eigenvalues.                //
//     int    row                                                             //
//            The row to which the pointer Hrow[] points of the matrix H.     //
//     float shift                                                           //
//            The cumulative exceptional shift of the diagonal elements of    //
//            the matrix H.                                                   //
////////////////////////////////////////////////////////////////////////////////
void One_Real_Eigenvalue(float Hrow[], float eigen_real[], float eigen_imag[], int row, float shift) {
	Hrow[row] += shift;
	eigen_real[row] = Hrow[row];
	eigen_imag[row] = 0.0;
}

////////////////////////////////////////////////////////////////////////////////
//  static void Two_Eigenvalues( float *H, float *S, float eigen_real[],   //
//                         float eigen_imag[], int n, int row, float shift) //
//                                                                            //
//  Description:                                                              //
//     Given the 2x2 matrix A = (a[i][j]), the characteristic equation is:    //
//     x^2 - Tr(A) x + Det(A) = 0, where Tr(A) = a[0][0] + a[1][1] and        //
//     Det(A) = a[0][0] * a[1][1] - a[0][1] * a[1][0].                        //
//     The solution for the eigenvalues x are:                                //
//         x1 = (Tr(A) + sqrt( (Tr(A))^2 + 4 * a[0][1] * a[1][0] ) / 2 and    //
//         x2 = (Tr(A) - sqrt( (Tr(A))^2 + 4 * a[0][1] * a[1][0] ) / 2.       //
//     Let p = (a[0][0] - a[1][1]) / 2 and q = p^2 - a[0][1] * a[1][0], then  //
//         x1 = a[1][1] + p [+|-] sqrt(q) and x2 = a[0][0] + a[1][1] - x1.    //
//     Choose the sign [+|-] to be the sign of p.                             //
//     If q > 0.0, then both roots are real and the transformation            //
//                 | cos sin |    | a[0][0]  a[0][1] |    | cos -sin |        //
//                 |-sin cos |    | a[1][0]  a[1][1] |    | sin  cos |        //
//      where sin = a[1][0] / r, cos = ( p + sqrt(q) ) / r, where r > 0 is    //
//      determined sin^2 + cos^2 = 1 transforms the matrix A to an upper      //
//      triangular matrix with x1 the upper diagonal element and x2 the lower.//
//      If q < 0.0, then both roots form a complex conjugate pair.            //
//                                                                            //
//  Arguments:                                                                //
//     float *H                                                              //
//            Pointer to the first element of the matrix in Hessenberg form.  //
//     float *S                                                              //
//            Pointer to the first element of the transformation matrix.      //
//     float eigen_real[]                                                    //
//            Array of the real parts of the eigenvalues.                     //
//     float eigen_imag[]                                                    //
//            Array of the imaginary parts of the eigenvalues.                //
//     int    n                                                               //
//            The dimensions of the matrix H and S.                           //
//     int    row                                                             //
//            The upper most row of the block diagonal 2 x 2 submatrix of H.  //
//     float shift                                                           //
//            The cumulative exceptional shift of the diagonal elements of    //
//            the matrix H.                                                   //
////////////////////////////////////////////////////////////////////////////////
void Two_Eigenvalues(float *H, float *S, float eigen_real[], float eigen_imag[], int n, int row, float shift) {
	float p, q, x, discriminant, r;
	float cos, sin;
	float *Hrow = H + n * row;
	float *Hnextrow = Hrow + n;
	int nextrow = row + 1;

	p = 0.5 * (Hrow[row] - Hnextrow[nextrow]);
	x = Hrow[nextrow] * Hnextrow[row];
	discriminant = p * p + x;
	Hrow[row] += shift;
	Hnextrow[nextrow] += shift;
	if (discriminant > 0.0) {                 // pair of real roots
		q = sqrt(discriminant);
		if (p < 0.0)
			q = p - q;
		else
			q += p;
		eigen_real[row] = Hnextrow[nextrow] + q;
		eigen_real[nextrow] = Hnextrow[nextrow] - x / q;
		eigen_imag[row] = 0.0;
		eigen_imag[nextrow] = 0.0;
		r = sqrt(Hnextrow[row] * Hnextrow[row] + q * q);
		sin = Hnextrow[row] / r;
		cos = q / r;
		Update_Row(Hrow, cos, sin, n, row);
		Update_Column(H, cos, sin, n, row);
		Update_Transformation(S, cos, sin, n, row);
	} else {                             // pair of complex roots
		eigen_real[nextrow] = eigen_real[row] = Hnextrow[nextrow] + p;
		eigen_imag[row] = sqrt(fabs(discriminant));
		eigen_imag[nextrow] = -eigen_imag[row];
	}
}

////////////////////////////////////////////////////////////////////////////////
//  static void Update_Row(float *Hrow, float cos, float sin, int n,       //
//                                                                   int row) //
//                                                                            //
//  Description:                                                              //
//     Update rows 'row' and 'row + 1' using the rotation matrix:             //
//                                | cos sin |                                 //
//                                |-sin cos |.                                //
//     I.e. multiply the matrix H on the left by the identity matrix with     //
//     the 2x2 diagonal block starting at row 'row' replaced by the above     //
//     2x2 rotation matrix.                                                   //
//                                                                            //
//  Arguments:                                                                //
//     float Hrow[]                                                          //
//            Pointer to the row "row" of the matrix in Hessenberg form.      //
//     float cos                                                             //
//            Cosine of the rotation angle.                                   //
//     float sin                                                             //
//            Sine of the rotation angle.                                     //
//     int    n                                                               //
//            The dimension of the matrix H.                                  //
//     int    row                                                             //
//            The row to which the pointer Hrow[] points of the matrix H      //
//            in Hessenberg form.                                             //
////////////////////////////////////////////////////////////////////////////////
void Update_Row(float *Hrow, float cos, float sin, int n, int row) {
	float x;
	float *Hnextrow = Hrow + n;
	int i;

	for (i = row; i < n; i++) {
		x = Hrow[i];
		Hrow[i] = cos * x + sin * Hnextrow[i];
		Hnextrow[i] = cos * Hnextrow[i] - sin * x;
	}
}

////////////////////////////////////////////////////////////////////////////////
//  static void Update_Column(float* H, float cos, float sin, int n,       //
//                                                                   int col) //
//                                                                            //
//  Description:                                                              //
//     Update columns 'col' and 'col + 1' using the rotation matrix:          //
//                               | cos -sin |                                 //
//                               | sin  cos |.                                //
//     I.e. multiply the matrix H on the right by the identity matrix with    //
//     the 2x2 diagonal block starting at row 'col' replaced by the above     //
//     2x2 rotation matrix.                                                   //
//                                                                            //
//  Arguments:                                                                //
//     float *H                                                              //
//            Pointer to the matrix in Hessenberg form.                       //
//     float cos                                                             //
//            Cosine of the rotation angle.                                   //
//     float sin                                                             //
//            Sine of the rotation angle.                                     //
//     int    n                                                               //
//            The dimension of the matrix H.                                  //
//     int    col                                                             //
//            The left-most column of the matrix H to update.                 //
////////////////////////////////////////////////////////////////////////////////
void Update_Column(float *H, float cos, float sin, int n, int col) {
	float x;
	int i;
	int next_col = col + 1;

	for (i = 0; i <= next_col; i++, H += n) {
		x = H[col];
		H[col] = cos * x + sin * H[next_col];
		H[next_col] = cos * H[next_col] - sin * x;
	}
}

////////////////////////////////////////////////////////////////////////////////
//  static void Update_Transformation(float *S, float cos, float sin,      //
//                                                             int n, int k)  //
//                                                                            //
//  Description:                                                              //
//     Update columns 'k' and 'k + 1' using the rotation matrix:              //
//                               | cos -sin |                                 //
//                               | sin  cos |.                                //
//     I.e. multiply the matrix S on the right by the identity matrix with    //
//     the 2x2 diagonal block starting at row 'k' replaced by the above       //
//     2x2 rotation matrix.                                                   //
//                                                                            //
//  Arguments:                                                                //
//     float *S                                                              //
//            Pointer to the row "row" of the matrix in Hessenberg form.      //
//     float cos                                                             //
//            Pointer to the first element of the matrix in Hessenberg form.  //
//     float sin                                                             //
//            Pointer to the first element of the transformation matrix.      //
//     int    n                                                               //
//            The dimensions of the matrix H and S.                           //
//     int    k                                                               //
//            The row to which the pointer Hrow[] points of the matrix H.     //
////////////////////////////////////////////////////////////////////////////////
void Update_Transformation(float *S, float cos, float sin, int n, int k) {
	float x;
	int i;
	int k1 = k + 1;

	for (i = 0; i < n; i++, S += n) {
		x = S[k];
		S[k] = cos * x + sin * S[k1];
		S[k1] = cos * S[k1] - sin * x;
	}
}

////////////////////////////////////////////////////////////////////////////////
//  static void Product_and_Sum_of_Shifts(float *H, int n, int max_row,      //
//                 float* shift, float *trace, float *det, int iteration)  //
//                                                                            //
//  Description:                                                              //
//     Calculate the trace and determinant of the 2x2 matrix:                 //
//                        | H[k-1][k-1]  H[k-1][k] |                          //
//                        |  H[k][k-1]    H[k][k]  |                          //
//     unless iteration = 0 (mod 10) in which case increment the shift and    //
//     decrement the first k elements of the matrix H, then fudge the trace   //
//     and determinant by  trace = 3/2( |H[k][k-1]| + |H[k-1][k-2]| and       //
//     det = 4/9 trace^2.                                                     //
//                                                                            //
//  Arguments:                                                                //
//     float *H                                                              //
//            Pointer to the matrix H in Hessenberg form.                     //
//     int    n                                                               //
//            The dimension of the matrix H.                                  //
//     int    max_row                                                         //
//            The maximum row of the block 2 x 2 diagonal matrix used to      //
//            estimate the two eigenvalues for the two implicit shifts.       //
//     float *shift                                                          //
//            The cumulative exceptional shift of the diagonal elements of    //
//            the matrix H.  Modified if an exceptional shift occurs.         //
//     float *trace                                                          //
//            Returns the trace of the 2 x 2 block diagonal matrix starting   //
//            at the row/column max_row-1.  For an exceptional shift, the     //
//            trace is set as described above.                                //
//     float *det                                                            //
//            Returns the determinant of the 2 x 2 block diagonal matrix      //
//            starting at the row/column max_row-1.  For an exceptional shift,//
//            the determinant is set as described above.                      //
//     int    iteration                                                       //
//            Current iteration count.                                        //
////////////////////////////////////////////////////////////////////////////////
void Product_and_Sum_of_Shifts(float *H, int n, int max_row, float *shift, float *trace, float *det, int iteration) {
	float *pH = H + max_row * n;
	float *p_aux;
	int i;
	int min_col = max_row - 1;

	if ((iteration % 10) == 0) {
		*shift += pH[max_row];
		for (i = 0, p_aux = H; i <= max_row; p_aux += n, i++)
			p_aux[i] -= pH[max_row];
		p_aux = pH - n;
		*trace = fabs(pH[min_col]) + fabs(p_aux[min_col - 1]);
		*det = *trace * *trace;
		*trace *= 1.5;
	} else {
		p_aux = pH - n;
		*trace = p_aux[min_col] + pH[max_row];
		*det = p_aux[min_col] * pH[max_row] - p_aux[max_row] * pH[min_col];
	}
}

////////////////////////////////////////////////////////////////////////////////
//  static int Two_Consecutive_Small_Subdiagonal(float* H, int min_row,      //
//                              int max_row, int n, float trace, float det) //
//                                                                            //
//  Description:                                                              //
//     To reduce the amount of computation in Francis' float QR step search  //
//     for two consecutive small subdiagonal elements from row nn to row m,   //
//     where m < nn.                                                          //
//                                                                            //
//  Arguments:                                                                //
//     float *H                                                              //
//            Pointer to the first element of the matrix in Hessenberg form.  //
//     int    min_row                                                         //
//            The row in which to end the search (search is from upwards).    //
//     int    max_row                                                         //
//            The row in which to begin the search.                           //
//     int    n                                                               //
//            The dimension of H.                                             //
//     float trace                                                           //
//            The trace of the lower 2 x 2 block diagonal matrix.             //
//     float det                                                             //
//            The determinant of the lower 2 x 2 block diagonal matrix.       //
//                                                                            //
//  Return Value:                                                             //
//     Row with negligible subdiagonal element or min_row if none found.      //
////////////////////////////////////////////////////////////////////////////////
int Two_Consecutive_Small_Subdiagonal(float *H, int min_row, int max_row, int n, float trace, float det) {
	float x, y, z, s;
	float *pH;
	int i, k;

	for (k = max_row - 2, pH = H + k * n; k >= min_row; pH -= n, k--) {
		x = (pH[k] * (pH[k] - trace) + det) / pH[n + k] + pH[k + 1];
		y = pH[k] + pH[n + k + 1] - trace;
		z = pH[n + n + k + 1];
		s = fabs(x) + fabs(y) + fabs(z);
		x /= s;
		y /= s;
		z /= s;
		if (k == min_row)
			break;
		if ((fabs(pH[k - 1]) * (fabs(y) + fabs(z))) <= DBL_EPSILON * fabs(x) * (fabs(pH[k - 1 - n]) + fabs(pH[k]) + fabs(pH[n + k + 1])))
			break;
	}
	for (i = k + 2, pH = H + i * n; i <= max_row; pH += n, i++)
		pH[i - 2] = 0.0;
	for (i = k + 3, pH = H + i * n; i <= max_row; pH += n, i++)
		pH[i - 3] = 0.0;
	return k;
}

////////////////////////////////////////////////////////////////////////////////
//  static void float_QR_Step(float *H, int min_row, int max_row,           //
//                                            int min_col, float *S, int n)  //
//                                                                            //
//  Description:                                                              //
//     Perform Francis' float QR step from rows 'min_row' to 'max_row'       //
//     and columns 'min_col' to 'max_row'.                                    //
//                                                                            //
//  Arguments:                                                                //
//     float *H                                                              //
//            Pointer to the first element of the matrix in Hessenberg form.  //
//     int    min_row                                                         //
//            The row in which to begin.                                      //
//     int    max_row                                                         //
//            The row in which to end.                                        //
//     int    min_col                                                         //
//            The column in which to begin.                                   //
//     float trace                                                           //
//            The trace of the lower 2 x 2 block diagonal matrix.             //
//     float det                                                             //
//            The determinant of the lower 2 x 2 block diagonal matrix.       //
//     float *S                                                              //
//            Pointer to the first element of the transformation matrix.      //
//     int    n                                                               //
//            The dimensions of H and S.                                      //
////////////////////////////////////////////////////////////////////////////////
void float_QR_Step(float *H, int min_row, int max_row, int min_col, float trace, float det, float *S, int n) {
	float s = 0, x = 0, y = 0, z = 0;
	float a, b, c;
	float *pH;
	float *tH;
	float *pS;
	int i, j, k;
	int last_test_row_col = max_row - 1;

	k = min_col;
	pH = H + min_col * n;
	a = (pH[k] * (pH[k] - trace) + det) / pH[n + k] + pH[k + 1];
	b = pH[k] + pH[n + k + 1] - trace;
	c = pH[n + n + k + 1];
	s = fabs(a) + fabs(b) + fabs(c);
	a /= s;
	b /= s;
	c /= s;

	for (; k <= last_test_row_col; k++, pH += n) {
		if (k > min_col) {
			c = (k == last_test_row_col) ? 0.0 : pH[n + n + k - 1];
			x = fabs(pH[k - 1]) + fabs(pH[n + k - 1]) + fabs(c);
			if (x == 0.0)
				continue;
			a = pH[k - 1] / x;
			b = pH[n + k - 1] / x;
			c /= x;
		}
		s = sqrt(a * a + b * b + c * c);
		if (a < 0.0)
			s = -s;
		if (k > min_col)
			pH[k - 1] = -s * x;
		else if (min_row != min_col)
			pH[k - 1] = -pH[k - 1];
		a += s;
		x = a / s;
		y = b / s;
		z = c / s;
		b /= a;
		c /= a;

		// Update rows k, k+1, k+2
		for (j = k; j < n; j++) {
			a = pH[j] + b * pH[n + j];
			if (k != last_test_row_col) {
				a += c * pH[n + n + j];
				pH[n + n + j] -= a * z;
			}
			pH[n + j] -= a * y;
			pH[j] -= a * x;
		}

		// Update column k+1

		j = k + 3;
		if (j > max_row)
			j = max_row;
		for (i = 0, tH = H; i <= j; i++, tH += n) {
			a = x * tH[k] + y * tH[k + 1];
			if (k != last_test_row_col) {
				a += z * tH[k + 2];
				tH[k + 2] -= a * c;
			}
			tH[k + 1] -= a * b;
			tH[k] -= a;
		}

		// Update transformation matrix

		for (i = 0, pS = S; i < n; pS += n, i++) {
			a = x * pS[k] + y * pS[k + 1];
			if (k != last_test_row_col) {
				a += z * pS[k + 2];
				pS[k + 2] -= a * c;
			}
			pS[k + 1] -= a * b;
			pS[k] -= a;
		}
	};
}

////////////////////////////////////////////////////////////////////////////////
//  static void BackSubstitution(float *H, float eigen_real[],              //
//                                               float eigen_imag[], int n)  //
//                                                                            //
//  Description:                                                              //
//                                                                            //
//  Arguments:                                                                //
//     float *H                                                              //
//            Pointer to the first element of the matrix in Hessenberg form.  //
//     float eigen_real[]                                                    //
//            The real part of an eigenvalue.                                 //
//     float eigen_imag[]                                                    //
//            The imaginary part of an eigenvalue.                            //
//     int    n                                                               //
//            The dimension of H, eigen_real, and eigen_imag.                 //
////////////////////////////////////////////////////////////////////////////////
void BackSubstitution(float *H, float eigen_real[], float eigen_imag[], int n) {
	float zero_tolerance;
	float *pH;
	int i, j, row;

	// Calculate the zero tolerance

	pH = H;
	zero_tolerance = fabs(pH[0]);
	for (pH += n, i = 1; i < n; pH += n, i++)
		for (j = i - 1; j < n; j++)
			zero_tolerance += fabs(pH[j]);
	zero_tolerance *= DBL_EPSILON;

	// Start Backsubstitution

	for (row = n - 1; row >= 0; row--) {
		if (eigen_imag[row] == 0.0)
			BackSubstitute_Real_Vector(H, eigen_real, eigen_imag, row, zero_tolerance, n);
		else if (eigen_imag[row] < 0.0)
			BackSubstitute_Complex_Vector(H, eigen_real, eigen_imag, row, zero_tolerance, n);
	}
}

////////////////////////////////////////////////////////////////////////////////
//  static void BackSubstitute_Real_Vector(float *H, float eigen_real[],    //
//             float eigen_imag[], int row,  float zero_tolerance, int n)   //
//                                                                            //
//  Description:                                                              //
//                                                                            //
//  Arguments:                                                                //
//     float *H                                                              //
//            Pointer to the first element of the matrix in Hessenberg form.  //
//     float eigen_real[]                                                    //
//            The real part of an eigenvalue.                                 //
//     float eigen_imag[]                                                    //
//            The imaginary part of an eigenvalue.                            //
//     int    row                                                             //
//     float zero_tolerance                                                  //
//            Zero substitute. To avoid dividing by zero.                     //
//     int    n                                                               //
//            The dimension of H, eigen_real, and eigen_imag.                 //
////////////////////////////////////////////////////////////////////////////////
void BackSubstitute_Real_Vector(float *H, float eigen_real[], float eigen_imag[], int row, float zero_tolerance, int n) {
	float *pH;
	float *pV;
	float x;
	float u[4] = { 0, 0, 0, 0 };
	float v[2] = { 0, 0 };
	int i, j, k;

	k = row;
	pH = H + row * n;
	pH[row] = 1.0;
	for (i = row - 1, pH -= n; i >= 0; i--, pH -= n) {
		u[0] = pH[i] - eigen_real[row];
		v[0] = pH[row];
		pV = H + n * k;
		for (j = k; j < row; j++, pV += n)
			v[0] += pH[j] * pV[row];
		if (eigen_imag[i] < 0.0) {
			u[3] = u[0];
			v[1] = v[0];
		} else {
			k = i;
			if (eigen_imag[i] == 0.0) {
				if (u[0] != 0.0)
					pH[row] = -v[0] / u[0];
				else
					pH[row] = -v[0] / zero_tolerance;
			} else {
				u[1] = pH[i + 1];
				u[2] = pH[n + i];
				x = (eigen_real[i] - eigen_real[row]);
				x *= x;
				x += eigen_imag[i] * eigen_imag[i];

				pH[row] = (u[1] * v[1] - u[3] * v[0]) / x;

				if (fabs(u[1]) > fabs(u[3]))
					pH[n + row] = -(v[0] + u[0] * pH[row]) / u[1];
				else
					pH[n + row] = -(v[1] + u[2] * pH[row]) / u[3];
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//  static void BackSubstitute_Complex_Vector(float *H, float eigen_real[], //
//              float eigen_imag[], int row,  float zero_tolerance, int n)  //
//                                                                            //
//  Description:                                                              //
//                                                                            //
//  Arguments:                                                                //
//     float *H                                                              //
//            Pointer to the first element of the matrix in Hessenberg form.  //
//     float eigen_real[]                                                    //
//            The real part of an eigenvalue.                                 //
//     float eigen_imag[]                                                    //
//            The imaginary part of an eigenvalue.                            //
//     int    row                                                             //
//     float zero_tolerance                                                  //
//            Zero substitute. To avoid dividing by zero.                     //
//     int    n                                                               //
//            The dimension of H, eigen_real, and eigen_imag.                 //
////////////////////////////////////////////////////////////////////////////////
void BackSubstitute_Complex_Vector(float *H, float eigen_real[], float eigen_imag[], int row, float zero_tolerance, int n) {
	float *pH;
	float *pV;
	float x, y;
	float u[4] = { 0, 0, 0, 0 };
	float v[2] = { 0, 0 };
	float w[2] = { 0, 0 };
	int i, j, k;

	k = row - 1;
	pH = H + n * row;
	if (fabs(pH[k]) > fabs(pH[row - n])) {
		pH[k - n] = -(pH[row] - eigen_real[row]) / pH[k];
		pH[row - n] = -eigen_imag[row] / pH[k];
	} else
		Complex_Division(-pH[row - n], 0.0, pH[k - n] - eigen_real[row], eigen_imag[row], &pH[k - n], &pH[row - n]);
	pH[k] = 1.0;
	pH[row] = 0.0;
	for (i = row - 2, pH = H + n * i; i >= 0; pH -= n, i--) {
		u[0] = pH[i] - eigen_real[row];
		w[0] = pH[row];
		w[1] = 0.0;
		pV = H + k * n;
		for (j = k; j < row; j++, pV += n) {
			w[0] += pH[j] * pV[row - 1];
			w[1] += pH[j] * pV[row];
		}
		if (eigen_imag[i] < 0.0) {
			u[3] = u[0];
			v[0] = w[0];
			v[1] = w[1];
		} else {
			k = i;
			if (eigen_imag[i] == 0.0) {
				Complex_Division(-w[0], -w[1], u[0], eigen_imag[row], &pH[row - 1], &pH[row]);
			} else {
				u[1] = pH[i + 1];
				u[2] = pH[n + i];
				x = eigen_real[i] - eigen_real[row];
				y = 2.0 * x * eigen_imag[row];
				x = x * x + eigen_imag[i] * eigen_imag[i] - eigen_imag[row] * eigen_imag[row];
				if (x == 0.0 && y == 0.0)
					x = zero_tolerance * (fabs(u[0]) + fabs(u[1]) + fabs(u[2]) + fabs(u[3]) + fabs(eigen_imag[row]));
				Complex_Division(u[1] * v[0] - u[3] * w[0] + w[1] * eigen_imag[row], u[1] * v[1] - u[3] * w[1] - w[0] * eigen_imag[row], x, y, &pH[row - 1], &pH[row]);
				if (fabs(u[1]) > (fabs(u[3]) + fabs(eigen_imag[row]))) {
					pH[n + row - 1] = -w[0] - u[0] * pH[row - 1] + eigen_imag[row] * pH[row] / u[1];
					pH[n + row] = -w[1] - u[0] * pH[row] - eigen_imag[row] * pH[row - 1] / u[1];
				} else {
					Complex_Division(-v[0] - u[2] * pH[row - 1], -v[1] - u[2] * pH[row], u[3], eigen_imag[row], &pH[n + row - 1], &pH[n + row]);
				}
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//  static void Calculate_Eigenvectors(float *H, float *S,                  //
//                          float eigen_real[], float eigen_imag[], int n)  //
//                                                                            //
//  Description:                                                              //
//     Multiply by transformation matrix.                                     //
//                                                                            //
//  Arguments:                                                                //
//     float *H                                                              //
//            Pointer to the first element of the matrix in Hessenberg form.  //
//     float *S                                                              //
//            Pointer to the first element of the transformation matrix.      //
//     float eigen_real[]                                                    //
//            The real part of an eigenvalue.                                 //
//     float eigen_imag[]                                                    //
//            The imaginary part of an eigenvalue.                            //
//     int    n                                                               //
//            The dimension of H, S, eigen_real, and eigen_imag.              //
////////////////////////////////////////////////////////////////////////////////
void Calculate_Eigenvectors(float *H, float *S, float eigen_real[], float eigen_imag[], int n) {
	float *pH;
	float *pS;
	float x, y;
	int i, j, k;

	for (k = n - 1; k >= 0; k--) {
		if (eigen_imag[k] < 0.0) {
			for (i = 0, pS = S; i < n; pS += n, i++) {
				x = 0.0;
				y = 0.0;
				for (j = 0, pH = H; j <= k; pH += n, j++) {
					x += pS[j] * pH[k - 1];
					y += pS[j] * pH[k];
				}
				pS[k - 1] = x;
				pS[k] = y;
			}
		} else if (eigen_imag[k] == 0.0) {
			for (i = 0, pS = S; i < n; i++, pS += n) {
				x = 0.0;
				for (j = 0, pH = H; j <= k; j++, pH += n)
					x += pS[j] * pH[k];
				pS[k] = x;
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//  static void Complex_Division(float x, float y, float u, float v,      //
//                                                    float* a, float* b)   //
//                                                                            //
//  Description:                                                              //
//    a + i b = (x + iy) / (u + iv)                                           //
//            = (x * u + y * v) / r^2 + i (y * u - x * v) / r^2,              //
//    where r^2 = u^2 + v^2.                                                  //
//                                                                            //
//  Arguments:                                                                //
//     float x                                                               //
//            Real part of the numerator.                                     //
//     float y                                                               //
//            Imaginary part of the numerator.                                //
//     float u                                                               //
//            Real part of the denominator.                                   //
//     float v                                                               //
//            Imaginary part of the denominator.                              //
//     float *a                                                              //
//            Real part of the quotient.                                      //
//     float *b                                                              //
//            Imaginary part of the quotient.                                 //
////////////////////////////////////////////////////////////////////////////////
void Complex_Division(float x, float y, float u, float v, float *a, float *b) {
	float q = u * u + v * v;
	*a = (x * u + y * v) / q;
	*b = (y * u - x * v) / q;
}

////////////////////////////////////////////////////////////////////////////////
//  int Lower_Triangular_Inverse(float *L,  int n)                           //
//                                                                            //
//  Description:                                                              //
//     This routine calculates the inverse of the lower triangular matrix L.  //
//     The superdiagonal part of the matrix is not addressed.                 //
//     The algorithm follows:                                                 //
//        Let M be the inverse of L, then L M = I,                            //
//     M[i][i] = 1.0 / L[i][i] for i = 0, ..., n-1, and                       //
//     M[i][j] = -[(L[i][j] M[j][j] + ... + L[i][i-1] M[i-1][j])] / L[i][i],  //
//     for i = 1, ..., n-1, j = 0, ..., i - 1.                                //
//                                                                            //
//                                                                            //
//  Arguments:                                                                //
//     float *L   On input, the pointer to the first element of the matrix   //
//                 whose lower triangular elements form the matrix which is   //
//                 to be inverted. On output, the lower triangular part is    //
//                 replaced by the inverse.  The superdiagonal elements are   //
//                 not modified.                                              //
//                 its inverse.                                               //
//     int     n   The number of rows and/or columns of the matrix L.         //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix L is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     float L[N][N];                                                        //
//                                                                            //
//     (your code to create the matrix L)                                     //
//     err = Lower_Triangular_Inverse(&L[0][0], N);                           //
//     if (err < 0) printf(" Matrix L is singular\n");                        //
//     else {                                                                 //
//        printf(" The inverse is \n");                                       //
//           ...                                                              //
//     }                                                                      //
////////////////////////////////////////////////////////////////////////////////
int Lower_Triangular_Inverse(float *L, int n) {
	int i, j, k;
	float *p_i, *p_j, *p_k;
	float sum;
//         Invert the diagonal elements of the lower triangular matrix L.
	for (k = 0, p_k = L; k < n; p_k += (n + 1), k++) {
		if (*p_k == 0.0)
			return -1;
		else
			*p_k = 1.0 / *p_k;
	}
//         Invert the remaining lower triangular matrix L row by row.
	for (i = 1, p_i = L + n; i < n; i++, p_i += n) {
		for (j = 0, p_j = L; j < i; p_j += n, j++) {
			sum = 0.0;
			for (k = j, p_k = p_j; k < i; k++, p_k += n)
				sum += *(p_i + k) * *(p_k + j);
			*(p_i + j) = -*(p_i + i) * sum;
		}
	}
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//  int Lower_Triangular_Solve(float *L, float *B, float x[], int n)       //
//                                                                            //
//  Description:                                                              //
//     This routine solves the linear equation Lx = B, where L is an n x n    //
//     lower triangular matrix.  (The superdiagonal part of the matrix is     //
//     not addressed.)                                                        //
//     The algorithm follows:                                                 //
//                      x[0] = B[0]/L[0][0], and                              //
//     x[i] = [B[i] - (L[i][0] * x[0]  + ... + L[i][i-1] * x[i-1])] / L[i][i],//
//     for i = 1, ..., n-1.                                                   //
//                                                                            //
//  Arguments:                                                                //
//     float *L   Pointer to the first element of the lower triangular       //
//                 matrix.                                                    //
//     float *B   Pointer to the column vector, (n x 1) matrix, B.           //
//     float *x   Pointer to the column vector, (n x 1) matrix, x.           //
//     int     n   The number of rows or columns of the matrix L.             //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix L is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     float A[N][N], B[N], x[N];                                            //
//                                                                            //
//     (your code to create matrix A and column vector B)                     //
//     err = Lower_Triangular_Solve(&A[0][0], B, x, n);                       //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else printf(" The solution is \n");                                    //
//           ...                                                              //
////////////////////////////////////////////////////////////////////////////////
int Lower_Triangular_Solve(float *L, float B[], float x[], int n) {
	int i, k;
//         Solve the linear equation Lx = B for x, where L is a lower
//         triangular matrix.
	for (k = 0; k < n; L += n, k++) {
		if (*(L + k) == 0.0)
			return -1;           // The matrix L is singular
		x[k] = B[k];
		for (i = 0; i < k; i++)
			x[k] -= x[i] * *(L + i);
		x[k] /= *(L + k);
	}
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//  int Upper_Triangular_Solve(float *U, float *B, float x[], int n)       //
//                                                                            //
//  Description:                                                              //
//     This routine solves the linear equation Ux = B, where U is an n x n    //
//     upper triangular matrix.  (The subdiagonal part of the matrix is       //
//     not addressed.)                                                        //
//     The algorithm follows:                                                 //
//                  x[n-1] = B[n-1]/U[n-1][n-1], and                          //
//     x[i] = [B[i] - (U[i][i+1] * x[i+1]  + ... + U[i][n-1] * x[n-1])]       //
//                                                                 / U[i][i], //
//     for i = n-2, ..., 0.                                                   //
//                                                                            //
//  Arguments:                                                                //
//     float *U   Pointer to the first element of the upper triangular       //
//                 matrix.                                                    //
//     float *B   Pointer to the column vector, (n x 1) matrix, B.           //
//     float *x   Pointer to the column vector, (n x 1) matrix, x.           //
//     int     n   The number of rows or columns of the matrix U.             //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix U is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     float A[N][N], B[N], x[N];                                            //
//                                                                            //
//     (your code to create matrix A and column vector B)                     //
//     err = Upper_Triangular_Solve(&A[0][0], B, x, n);                       //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else printf(" The solution is \n");                                    //
//           ...                                                              //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Upper_Triangular_Solve(float *U, float B[], float x[], int n) {
	int i, k;

//         Solve the linear equation Ux = B for x, where U is an upper
//         triangular matrix.

	for (k = n - 1, U += n * (n - 1); k >= 0; U -= n, k--) {
		if (*(U + k) == 0.0)
			return -1;           // The matrix U is singular
		x[k] = B[k];
		for (i = k + 1; i < n; i++)
			x[k] -= x[i] * *(U + i);
		x[k] /= *(U + k);
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//  int Upper_Triangular_Inverse(float *U,  int n)                           //
//                                                                            //
//  Description:                                                              //
//     This routine calculates the inverse of the upper triangular matrix U.  //
//     The subdiagonal part of the matrix is not addressed.                   //
//     The algorithm follows:                                                 //
//        Let M be the inverse of U, then U M = I,                            //
//     M[n-1][n-1] = 1.0 / U[n-1][n-1] and                                    //
//     M[i][j] = -( U[i][i+1] M[i+1][j] + ... + U[i][j] M[j][j] ) / U[i][i],  //
//     for i = n-2, ... , 0,  j = n-1, ..., i+1.                              //
//                                                                            //
//                                                                            //
//  Arguments:                                                                //
//     float *U   On input, the pointer to the first element of the matrix   //
//                 whose upper triangular elements form the matrix which is   //
//                 to be inverted. On output, the upper triangular part is    //
//                 replaced by the inverse.  The subdiagonal elements are     //
//                 not modified.                                              //
//     int     n   The number of rows and/or columns of the matrix U.         //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix U is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     float U[N][N];                                                        //
//                                                                            //
//     (your code to create the matrix U)                                     //
//     err = Upper_Triangular_Inverse(&U[0][0], N);                           //
//     if (err < 0) printf(" Matrix U is singular\n");                        //
//     else {                                                                 //
//        printf(" The inverse is \n");                                       //
//           ...                                                              //
//     }                                                                      //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Upper_Triangular_Inverse(float *U, int n) {
	int i, j, k;
	float *p_i, *p_k;
	float sum;

//         Invert the diagonal elements of the upper triangular matrix U.

	for (k = 0, p_k = U; k < n; p_k += (n + 1), k++) {
		if (*p_k == 0.0)
			return -1;
		else
			*p_k = 1.0 / *p_k;
	}

//         Invert the remaining upper triangular matrix U.

	for (i = n - 2, p_i = U + n * (n - 2); i >= 0; p_i -= n, i--) {
		for (j = n - 1; j > i; j--) {
			sum = 0.0;
			for (k = i + 1, p_k = p_i + n; k <= j; p_k += n, k++) {
				sum += *(p_i + k) * *(p_k + j);
			}
			*(p_i + j) = -*(p_i + i) * sum;
		}
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
//  void Interchange_Rows(float *A, int row1, int row2, int ncols)           //
//                                                                            //
//  Description:                                                              //
//     Interchange the rows 'row1' and 'row2' of the  nrows x ncols matrix A. //
//                                                                            //
//  Arguments:                                                                //
//     float *A    Pointer to the first element of the matrix A.             //
//     int    row1  The row of A which is to be interchanged with row row2.   //
//     int    row2  The row of A which is to be interchanged with row row1.   //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     float A[M][N];                                                        //
//     int i, j;                                                              //
//                                                                            //
//  (your code to initialize the matrix A, the row number i and row number j) //
//                                                                            //
//     if ( (i >= 0) && ( i < M ) && (j > 0) && ( j < M ) )                   //
//        Interchange_Rows(&A[0][0], i, j, N);                                //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Interchange_Rows(float *A, int row1, int row2, int ncols) {
	int i;
	float *pA1, *pA2;
	float temp;

	pA1 = A + row1 * ncols;
	pA2 = A + row2 * ncols;
	for (i = 0; i < ncols; i++) {
		temp = *pA1;
		*pA1++ = *pA2;
		*pA2++ = temp;
	}
}

////////////////////////////////////////////////////////////////////////////////
//  void Interchange_Columns(float *A, int col1, int col2, int nrows,        //
//                                                                 int ncols) //
//                                                                            //
//  Description:                                                              //
//     Interchange the columns 'col1' and 'col2' of the  nrows x ncols        //
//     matrix A.                                                              //
//                                                                            //
//  Arguments:                                                                //
//     float *A    Pointer to the first element of the matrix A.             //
//     int    col1  The column of A which is to be interchanged with col2.    //
//     int    col2  The column of A which is to be interchanged with col1.    //
//     int    nrows The number of rows matrix A.                              //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     float A[M][N];                                                        //
//     int i,j;                                                               //
//                                                                            //
//     (your code to initialize the matrix A, the column number i and column  //
//       number j)                                                            //
//                                                                            //
//     if ( (i >= 0) && ( i < N ) && ( j >= 0 ) && (j < N) )                  //
//        Interchange_Columns(&A[0][0], i, j, M, N);                          //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Interchange_Columns(float *A, int col1, int col2, int nrows, int ncols) {
	int i;
	float *pA1, *pA2;
	float temp;

	pA1 = A + col1;
	pA2 = A + col2;
	for (i = 0; i < nrows; pA1 += ncols, pA2 += ncols, i++) {
		temp = *pA1;
		*pA1 = *pA2;
		*pA2 = temp;
	}
}
