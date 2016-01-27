/****************************************************************
 *文 件 名: Matrix.h
 *描    述:
 *          矩阵运算函数集
 *
 *作    者：申文斌
 *当前版本: 1.0
 *完成日期: 2011-3-22
 *
 *取代版本：
 *原 作 者：
 *完成日期：
 *
 *版权所有：
 ****************************************************************/

#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef int size;            // 矩阵的行列大小

extern char* errmsg[];       // 声明出错信息

#define    OK               0 // 函数正常退出
#define    LEN_INV_GAUSE   20 // 高斯求逆临时矩阵的长度

// 出错代码标识
#define    ERR_NO_ERROR     0  // 无错误
#define    ERR_MEM_NONE     1  // 访问了未初始化的内存
#define    ERR_CANNOT_INV   2  // 矩阵不可以求逆

// 计算矩阵mIna加上mInb，结果存入矩阵mRslt中
// row: ina的行数， col:ina的列数
// 返回值: 成功: OK， 输入或输出矩阵指针为空：ERR_MEM_NONE
int AddMatrix(const void * mIna, const void * mInb, void * mRslt, size row, size col);

// 计算矩阵mIna减去mInb，结果存入矩阵mRslt中
// row: ina的行数， col:ina的列数
// 返回值: 成功: OK， 输入或输出矩阵指针为空：ERR_MEM_NONE
int SubMatrix(const void * mIna, const void * mInb, void * mRslt, size row, size col);

// 矩阵mIna乘 mInb， 结果存入矩阵mRslt中
// rowa: mIna的行数， cola:mIna的列数， colb:mInb的列数
// 返回值: 成功: OK， 输入或输出矩阵指针为空：ERR_MEM_NONE
int MulMatrix(const void * mIna, const void * mInb, void * mRslt, size rowa, size cola, size colb);

// 矩阵mIna乘 mInb，结果存入矩阵mRslt中，并且结果为对称矩阵
// rowa: mIna的行数
// 返回值: 成功: OK， 输入或输出矩阵指针为空：ERR_MEM_NONE
int SymMulMatrix(const void * mIna, const void * mInb, void * mRslt, size rowa, size cola);

// 对矩阵mIn进行转置，结果存入矩阵mTran中
// row: mIn的行数， col:mIn的列数
// 返回值: 成功: OK， 输入或输出矩阵指针为空：ERR_MEM_NONE
int TranMatrix(const void * mIn, void * mTran, size row, size col);

// 对矩阵mIn进行求逆，结果存入矩阵mInv中
// len: mIn的行列数
// 求逆算法为全选主元的高斯算法
// 返回值: 成功: OK，  输入或输出矩阵指针为空：ERR_MEM_NONE
//         矩阵不可以求逆：ERR_CANNOT_INV
int InvMtrx(const void * mIn, void * mInv, size len);

// 对对称矩阵mIn进行求逆，结果存入矩阵mInv中
// len: mIn的行列数
// 求逆算法：首先用不选主元的高斯算法求逆，如果不选主元的
// 过程出现主元为零或输入矩阵mIn不对称，则调用全选主元的高斯算法
// 返回值: 成功: OK，  输入或输出矩阵指针为空：ERR_MEM_NONE
//         矩阵不可以求逆：ERR_CANNOT_INV
int InvSymMtrx(const void * mIn, void * mInv, size len);


#endif
