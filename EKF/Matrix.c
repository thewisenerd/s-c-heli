/****************************************************************
 *文 件 名: Matrix.c
 *描    述:
 *          矩阵相加减、相乘、转置、求逆的实现
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
#include <includs.h>
#include "Matrix.h"

// 声明出错信息 
char* errmsg[] =
{
    "No error.",
    "Memeroy is not enough.",
    "Matrix can't be inversed."
};

/****************************************************************
 *函 数 名: AddMatrix()
 *参    数:
 *         const void * mIna: 输入矩阵
 *         const void * mInb: 输入矩阵
 *               void *mRslt: 输出矩阵
 *               size    row: 输入矩阵行数
 *               size    col: 输入矩阵列数
 *
 *功    能：计算矩阵mIna加上mInb，结果存入矩阵mRslt中
 *
 *返 回 值:
 *          成功: OK
 *          输入或输出矩阵指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int AddMatrix(const void * mIna, const void * mInb, void * mRslt, size row, size col)
{
    double *pmIna = (double *)mIna;
    double *pmInb = (double *)mInb;
    double *pmRslt = (double *)mRslt;
    int     i, j;        // 循环变量

#ifdef __DEBUG__
    if ( (pmIna == NULL) || (pmInb == NULL) || (pmRslt == NULL) )
    {
        return ERR_MEM_NONE;        // 输入或输出矩阵指针为空
    }
#endif

    for (i=0; i<row; i++)
    {
        for (j=0; j<col; j++)
        {
            pmRslt[i*col+j] = pmIna[i*col+j] + pmInb[i*col+j];
        }//for
    }//for

    return OK;
}

/****************************************************************
 *函 数 名: SubMatrix()
 *参    数:
 *         const void * mIna: 输入矩阵
 *         const void * mInb: 输入矩阵
 *               void *mRslt: 输出矩阵
 *               size    row: 输入矩阵行数
 *               size    col: 输入矩阵列数
 *
 *功    能：计算矩阵mIna减去mInb，结果存入矩阵mRslt中
 *
 *返 回 值:
 *          成功: OK
 *          输入或输出矩阵指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int SubMatrix(const void * mIna, const void * mInb, void * mRslt, size row, size col)
{
    double *pmIna = (double *)mIna;
    double *pmInb = (double *)mInb;
    double *pmRslt = (double *)mRslt;
    int     i, j;        // 循环变量

#ifdef __DEBUG__
    if ( (pmIna == NULL) || (pmInb == NULL) || (pmRslt == NULL) )
    {
        return ERR_MEM_NONE;        // 输入或输出矩阵指针为空
    }
#endif

    for (i=0; i<row; i++)
    {
        for (j=0; j<col; j++)
        {
            pmRslt[i*col+j] = pmIna[i*col+j] - pmInb[i*col+j];
        }//for
    }//for

    return OK;
}

/****************************************************************
 *函 数 名: MulMatrix()
 *参    数:
 *         const void * mIna: 输入矩阵
 *         const void * mInb: 输入矩阵
 *               void *mRslt: 输出矩阵
 *               size   rowa: 输入矩阵mIna行数
 *               size   cola: 输入矩阵mIna列数
 *               size   colb: 输入矩阵mInb列数
 *
 *功    能：计算矩阵mIna 乘 mInb，结果存入矩阵mRslt中
 *
 *返 回 值:
 *          成功: OK
 *          输入或输出矩阵指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int MulMatrix(const void * mIna, const void * mInb, void * mRslt, size rowa, size cola, size colb)
{
    double *pmIna = (double *)mIna;
    double *pmInb = (double *)mInb;
    double *pmRslt = (double *)mRslt;
    int     i, j, k; // 循环变量 

#ifdef __DEBUG__
    if ( (pmIna == NULL) || (pmInb == NULL) || (pmRslt == NULL) )
    {
        return ERR_MEM_NONE;        // 输入或输出矩阵指针为空
    }
#endif

    for (i=0; i<rowa; i++)
    {
        for (j=0; j<colb; j++)
        {                           // 输出矩阵的大小应该为rowa * colb
            pmRslt[i*colb+j] = 0;    // 将输出矩阵初始化为0
        }
    }// for

    // 两个矩阵相乘
    for (i=0; i<rowa; i++)
    {
        for (j=0; j<colb; j++)
        {
            for (k=0; k<cola; k++)
            {
                pmRslt[i*colb+j] += pmIna[i*cola+k] * pmInb[k*colb+j];
            }
        }
    }//for

    return OK;
}

/****************************************************************
 *函 数 名: SymMulMatrix()
 *参    数:
 *         const void * mIna: 输入矩阵
 *         const void * mInb: 输入矩阵
 *               void *mRslt: 输出矩阵
 *               size   rowa: 输入矩阵mIna行数
 *               size   cola: 输入矩阵mIna列数
 *
 *功    能：计算矩阵mIna 乘 mInb，结果存入矩阵mRslt中，并且结果为对称矩阵
 *          因为结果为对称矩阵，所以必然为方阵，而且只需要求下三角元素就行了
 *          因此只要这里只需要输入矩阵mIna的行列数就够了
 *
 *返 回 值:
 *          成功: OK
 *          输入或输出矩阵指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int SymMulMatrix(const void * mIna, const void * mInb, void * mRslt, size rowa, size cola)
{
    double *pmIna = (double *)mIna;
    double *pmInb = (double *)mInb;
    double *pmRslt = (double *)mRslt;
    int     i, j, k; // 循环变量 

#ifdef __DEBUG__
    if ( (pmIna == NULL) || (pmInb == NULL) || (pmRslt == NULL) )
    {
        return ERR_MEM_NONE;        // 输入或输出矩阵指针为空
    }
#endif

    for (i=0; i<rowa; i++)
    {
        for (j=0; j<=i; j++)
        {
            pmRslt[i*rowa+j] = 0.0; // 将输出矩阵下三角初始化为0
        }
    }// for

    // 两个矩阵相乘
    for (i=0; i<rowa; i++)
    {
        for (j=0; j<=i; j++)
        {
            for (k=0; k<cola; k++)
            {
                // mInb的列数等于mIna的行数，所以mInb的(k, j)元素为[k*rowa+j]
                pmRslt[i*rowa+j] += pmIna[i*cola+k] * pmInb[k*rowa+j];
            }
            pmRslt[j*rowa+i] = pmRslt[i*rowa+j];        // 上三角元素
        }
    }//for

    return OK;
}

/****************************************************************
 *函 数 名: MulMatrix()
 *参    数:
 *         const void *  mIn: 输入矩阵
 *               void *mTran: 输出矩阵
 *               size    row: 输入矩阵行数
 *               size    col: 输入矩阵列数
 *
 *功    能：对矩阵mIn进行转置，结果存入矩阵mTran中
 *
 *返 回 值:
 *          成功: OK
 *          输入或输出矩阵指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int TranMatrix(const void * mIn, void * mTran, size row, size col)
{
    double *pmIn = (double *)mIn;
    double *pmTran = (double *)mTran;
    int     i, j;      // 循环变量 

#ifdef __DEBUG__
    if ( (pmIn == NULL) || (pmTran == NULL) )
    {
        return ERR_MEM_NONE;        // 输入或输出矩阵指针为空
    }
#endif

    // 矩阵转置
    for (i=0; i<row; i++)
    {
        for (j=0; j<col; j++)
        {
            pmTran[j*row+i] = pmIn[i*col+j];
        }//for
    }//for

    return OK;
}

// 交换x,y的值
#define swap(x, y) { double t = (x); (x) = (y); (y) = t; }

/****************************************************************
 *函 数 名: InvMtrx()
 *参    数:
 *         const void *  mIn: 输入矩阵
 *               void * mInv: 输出矩阵
 *               size    len: 输入矩阵行列数
 *
 *功    能：用Gause全选主元法对矩阵mIn进行求逆，结果存入矩阵mInv中
 *          这个函数只可以求行列数为LEN_INV_GAUSE的矩阵的逆
 *
 *返 回 值:
 *          成功: OK
 *          输入或输出矩阵指针为空：ERR_MEM_NONE
 *          矩阵不可以求逆：ERR_CANNOT_INV
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int InvMtrx(const void * mIn, void * mInv, size len)
{ 
    int i, j, k;                      // 循环变量
    int rowMax[LEN_INV_GAUSE];        // 最大元素行下标
    int colMax[LEN_INV_GAUSE];        // 最大元素列下标

    double maxElem;                   // 最大元素
    double absElem;                   // 元素绝对值

    double *pmIn = (double *)mIn;
    double *pmInv = (double *)mInv;
    double INF = 1e-6;                // 一个很小的数

#ifdef __DEBUG__
    if ( (pmIn == NULL) || (pmInv == NULL) )
    {
        return ERR_MEM_NONE;        // 输入或输出矩阵指针为空
    }
#endif

    // 将输入矩阵的值拷贝到输出矩阵中，
    // 这样使求逆结果直接保存在输出矩阵中。
    for (i=0; i<len; i++)
    {
        for (j=0; j<len; j++)
        {
            pmInv[i*len+j] = pmIn[i*len+j];
        }//for
    }//for

    // 从这里开始到最后应用Gause全选主元求逆
    for (k=0; k<len; k++)
    {
        // 第一步：选主元
        // 从第 k 行、第 k 列开始的右下角子阵中选取绝对值最大的元素，
        // 并记住此元素所在的行号和列号，再通过行交换和列交换将它交换到主元素位置上。
        maxElem = 0.0;
        for (i=k; i<len; i++) 
        { 
            for (j=k; j<len; j++)
            {
                absElem = (pmInv[i*len+j] > 0 ? pmInv[i*len+j] : -pmInv[i*len+j]); //求绝对值
                if (absElem > maxElem) 
                { 
                    maxElem = absElem; 
                    rowMax[k] = i;        // 第k次主循环最大元素的行号
                    colMax[k] = j;        // 第k次主循环最大元素的列号
                } 
            }//for 
        }//for

        if ( (maxElem < INF) && (maxElem > -INF) )        //max 等于零
        {            
            return ERR_CANNOT_INV;        // 不可以求逆了！
        }

        if ( rowMax[k] != k )            // 最大元素不在第k行
        {
            for (j=0; j<len; j++)
            {
                // 将k行和最大元素所在行的元素进行行交换
                swap(pmInv[k*len+j], pmInv[rowMax[k]*len+j]);
            }
        }

        if ( colMax[k] != k )             // 最大元素不在第k列
        {
            for (j=0; j<len; j++)
            {
                // 将k列和最大元素所在列的元素进行列交换
                swap(pmInv[j*len+k], pmInv[j*len+colMax[k]]);
            }
        }

        // 第二步：元素m(k, k)求倒
        pmInv[k*len+k] = 1.0 / pmInv[k*len+k];

        // 第三步 
        for (j=0; j<len; j++) 
        { 
            if (j != k)
            {
                // m(k, j) = m(k, j) * m(k, k)，j = 0, 1, ..., n-1；j != k 
                pmInv[k*len+j] *= pmInv[k*len+k];
            }
        }

        // 第四步 
        for (i=0; i<len; i++) 
        { 
            if (i != k) 
            { 
                for (j=0; j<len; j++) 
                { 
                    if (j != k)
                    {
                        // m(i, j) = m(i, j) - m(i, k) * m(k, j)，i, j = 0, 1, ..., n-1；i, j != k
                        pmInv[i*len+j] -= pmInv[i*len+k] * pmInv[k*len+j];
                    }
                } 
            }//if
        }//for

        // 第五步
        for (i=0; i<len; i++) 
        { 
            if (i != k)
            {
                // m(i, k) = -m(i, k) * m(k, k)，i = 0, 1, ..., n-1；i != k 
                pmInv[i*len+k] *= -pmInv[k*len+k];
            }
        }//for
    }//for

    // 最后，根据在全选主元过程中所记录的行、列交换的信息进行恢复，恢复的原则如下：
    // 在全选主元过程中，先交换的行（列）后进行恢复；
    // 原来的行（列）交换用列（行）交换来恢复。
    for (k=len-1; k>=0; k--) 
    {
        if ( colMax[k] != k )
        {
            for (j=0; j<len; j++)
            {
                swap(pmInv[k*len+j], pmInv[colMax[k]*len+j]);
            }
        }//if

        if ( rowMax[k] != k )
        {
            for (j=0; j<len; j++)
            {
                swap(pmInv[j*len+k], pmInv[j*len+rowMax[k]]);
            }
        }
    }//for

    return OK;
}

/****************************************************************
 *函 数 名: InvSymMtrx()
 *参    数:
 *         const void *  mIn: 输入矩阵，应该为对称矩阵
 *               void * mInv: 输出矩阵
 *               size    len: 输入矩阵行列数
 *
 *功    能：用不选主元Gause法对对称矩阵mIn进行求逆，如果不选主元的
 *          过程出现主元为零，则调用全选主元的高斯算法
 *          结果存入矩阵mInv中
 *注   意： 若输入矩阵mIn不对称，则用全选主元高斯求逆
 *
 *返 回 值:
 *          成功: OK
 *          输入或输出矩阵指针为空：ERR_MEM_NONE
 *          矩阵不可以求逆：ERR_CANNOT_INV
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int InvSymMtrx(const void * mIn, void * mInv, size len)
{
    int    i, j, k;                // 循环变量
    double *pmIn = (double *)mIn;
    double *pmInv = (double *)mInv;
    double INF = 1e-6;                // 一个很小的数

#ifdef __DEBUG__
    if ( (pmIn == NULL) || (pmInv == NULL) )
    {
        return ERR_MEM_NONE;        // 输入或输出矩阵指针为空
    }
#endif

    // 将输入矩阵的值拷贝到输出矩阵中，
    // 这样既可以保存输入矩阵，又可以使求逆结果
    // 直接保存在输出矩阵中。
    for (i=0; i<len; i++)
    {
        for (j=0; j<=i; j++)
        {
            // 判断输入矩阵是否对称，若不对称，则用全选主元高斯求逆
            double tmp = pmIn[i*len+j] - pmIn[j*len+i];
            if ( (tmp < -INF) || (tmp > INF) )
            {
                return InvMtrx(pmIn, pmInv, len);  // 调用全选主元高斯求逆
            }

            pmInv[i*len+j] = pmIn[i*len+j];
            pmInv[j*len+i] = pmIn[i*len+j];
        }//for
    }//for

    for (i=0; i<len; i++)
    {
        if ( (pmInv[i*len+i] < INF) && (pmInv[i*len+i] > -INF) )  // 主元为零
        {
            return InvMtrx(pmIn, pmInv, len);        // 调用全选主元高斯求逆
        }

        // 计算第(i, i)元素
        pmInv[i*len+i] = 1.0 / pmInv[i*len+i];

        for (j=0; j<i; j++)
        {
            // 计算元素(0,0)到(i-1, i-1)
            for (k=0; k<=j; k++)
            {
                pmInv[j*len+k] -= pmInv[j*len+i] * pmInv[i*len+k] * pmInv[i*len+i];
                pmInv[k*len+j]  = pmInv[j*len+k];        // 两元素对称
            }

            // 计算元素(0,i+1)到(i-1, len)及这个区域的对称元素
            for (k=i+1; k<len; k++)
            {
                pmInv[j*len+k] -=  pmInv[j*len+i] * pmInv[i*len+k] * pmInv[i*len+i];
                pmInv[k*len+j]  = -pmInv[j*len+k];        // 两元素反对称
            }
        }//for

        // 计算元素(i+1,i+1)到(len, len)这个区域元素
         for ( j=i+1; j<len; j++)
        {
            for (k=j; k<len; k++)
            {
                pmInv[j*len+k] -= pmInv[j*len+i] * pmInv[i*len+k] * pmInv[i*len+i];
                pmInv[k*len+j]  = pmInv[j*len+k];
            }
        }

        // 计算除(i, i)外第i行k列元素及k行i列元素
        for (k=0; k<i; k++)
        {
            pmInv[i*len+k] *= pmInv[i*len+i];
            pmInv[k*len+i] =  pmInv[i*len+k];
        }
        for (k=i+1; k<len; k++)
        {
            pmInv[i*len+k] *=  pmInv[i*len+i];
            pmInv[k*len+i] =  -pmInv[i*len+k];
        }

    }//for

    return OK;
}

