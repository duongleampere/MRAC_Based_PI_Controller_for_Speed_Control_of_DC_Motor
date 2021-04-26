#ifndef __STR_H
#define __STR_H

#include "stdio.h"
#include "string.h"






/*-----Functions to work with Matrix-----*/
void Mat_Add(double x[6][6], unsigned char m, unsigned char n, double y[6][6], unsigned char p, unsigned char q, double z[6][6])
	//(ten matrix 1, so hang, so cot, ten matrix 2, so hang so cot, ten matrix ket qua)
{
	int i,j;
	for( i = 0 ; i < m ; i++ )
	{
		for( j = 0 ; j < n ; j++ )
		{
			z[i][j] = x[i][j] + y[i][j];
		}
	}
}

void Mat_Sub(double x[6][6], unsigned char m, unsigned char n, double y[6][6], unsigned char p, unsigned char q, double z[6][6])
	//(ten matrix 1, so hang, so cot, ten matrix 2, so hang so cot, ten matrix ket qua)
{
	int i,j;
	for( i = 0 ; i < m ; i++ )
	{
		for( j = 0 ; j < n ; j++ )
		{
			z[i][j] = x[i][j] - y[i][j];
		}
	}
}

void Mat_Mul(double x[6][6], unsigned char m, unsigned char n, double y[6][6], unsigned char p, unsigned char q, double z[6][6])
	//(ten matrix 1, so hang, so cot, ten matrix 2, so hang so cot, ten matrix ket qua)
{
	int i,j,k;
	double tp = 0;
	for( i = 0 ; i < m ; i++ )
	{
		for( j = 0 ; j < q ; j++ )
		{
			for( k = 0 ; k < p ; k++ )
			{
				tp = tp + x[i][k] * y[k][j];
			}
			z[i][j] = tp;
			tp = 0;
		}
	}
}

void Mat_Tranpose(double x[6][6], unsigned char m, unsigned char n, double y[6][6])
	//(ten matrix, so hang, so cot, ten matrix chuyenvi)
{
	int i,j;
	for( i = 0 ; i < m ; i++ )
	{
		for( j = 0 ; j < n ; j++ )
		{
			y[i][j] = x[j][i];
		}
	}
}
void numDivMat(double num [6][6], unsigned char m, unsigned char n, double x, double y[6][6])
{
	for(int i = 0 ; i < m ; i++)
	{
		for(int j = 0 ; j < n ; j++ )
		{
			y[i][j] = num[i][j]/x;
		}
	}
}
void numMulMat(double num [6][6], unsigned char m, unsigned char n, double x, double y[6][6])
{
	for(int i = 0 ; i < m ; i++)
	{
		for(int j = 0 ; j < n ; j++ )
		{
			y[i][j] = num[i][j]*x;
		}
	}
}

#endif
