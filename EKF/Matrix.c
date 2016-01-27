/****************************************************************
 *�� �� ��: Matrix.c
 *��    ��:
 *          ������Ӽ�����ˡ�ת�á������ʵ��
 *
 *��    �ߣ����ı�
 *��ǰ�汾: 1.0
 *�������: 2011-3-22
 *
 *ȡ���汾��
 *ԭ �� �ߣ�
 *������ڣ�
 *
 *��Ȩ���У�
 ****************************************************************/
#include <includs.h>
#include "Matrix.h"

// ����������Ϣ 
char* errmsg[] =
{
    "No error.",
    "Memeroy is not enough.",
    "Matrix can't be inversed."
};

/****************************************************************
 *�� �� ��: AddMatrix()
 *��    ��:
 *         const void * mIna: �������
 *         const void * mInb: �������
 *               void *mRslt: �������
 *               size    row: �����������
 *               size    col: �����������
 *
 *��    �ܣ��������mIna����mInb������������mRslt��
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          ������������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int AddMatrix(const void * mIna, const void * mInb, void * mRslt, size row, size col)
{
    double *pmIna = (double *)mIna;
    double *pmInb = (double *)mInb;
    double *pmRslt = (double *)mRslt;
    int     i, j;        // ѭ������

#ifdef __DEBUG__
    if ( (pmIna == NULL) || (pmInb == NULL) || (pmRslt == NULL) )
    {
        return ERR_MEM_NONE;        // ������������ָ��Ϊ��
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
 *�� �� ��: SubMatrix()
 *��    ��:
 *         const void * mIna: �������
 *         const void * mInb: �������
 *               void *mRslt: �������
 *               size    row: �����������
 *               size    col: �����������
 *
 *��    �ܣ��������mIna��ȥmInb������������mRslt��
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          ������������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int SubMatrix(const void * mIna, const void * mInb, void * mRslt, size row, size col)
{
    double *pmIna = (double *)mIna;
    double *pmInb = (double *)mInb;
    double *pmRslt = (double *)mRslt;
    int     i, j;        // ѭ������

#ifdef __DEBUG__
    if ( (pmIna == NULL) || (pmInb == NULL) || (pmRslt == NULL) )
    {
        return ERR_MEM_NONE;        // ������������ָ��Ϊ��
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
 *�� �� ��: MulMatrix()
 *��    ��:
 *         const void * mIna: �������
 *         const void * mInb: �������
 *               void *mRslt: �������
 *               size   rowa: �������mIna����
 *               size   cola: �������mIna����
 *               size   colb: �������mInb����
 *
 *��    �ܣ��������mIna �� mInb������������mRslt��
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          ������������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int MulMatrix(const void * mIna, const void * mInb, void * mRslt, size rowa, size cola, size colb)
{
    double *pmIna = (double *)mIna;
    double *pmInb = (double *)mInb;
    double *pmRslt = (double *)mRslt;
    int     i, j, k; // ѭ������ 

#ifdef __DEBUG__
    if ( (pmIna == NULL) || (pmInb == NULL) || (pmRslt == NULL) )
    {
        return ERR_MEM_NONE;        // ������������ָ��Ϊ��
    }
#endif

    for (i=0; i<rowa; i++)
    {
        for (j=0; j<colb; j++)
        {                           // �������Ĵ�СӦ��Ϊrowa * colb
            pmRslt[i*colb+j] = 0;    // ����������ʼ��Ϊ0
        }
    }// for

    // �����������
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
 *�� �� ��: SymMulMatrix()
 *��    ��:
 *         const void * mIna: �������
 *         const void * mInb: �������
 *               void *mRslt: �������
 *               size   rowa: �������mIna����
 *               size   cola: �������mIna����
 *
 *��    �ܣ��������mIna �� mInb������������mRslt�У����ҽ��Ϊ�Գƾ���
 *          ��Ϊ���Ϊ�Գƾ������Ա�ȻΪ���󣬶���ֻ��Ҫ��������Ԫ�ؾ�����
 *          ���ֻҪ����ֻ��Ҫ�������mIna���������͹���
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          ������������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int SymMulMatrix(const void * mIna, const void * mInb, void * mRslt, size rowa, size cola)
{
    double *pmIna = (double *)mIna;
    double *pmInb = (double *)mInb;
    double *pmRslt = (double *)mRslt;
    int     i, j, k; // ѭ������ 

#ifdef __DEBUG__
    if ( (pmIna == NULL) || (pmInb == NULL) || (pmRslt == NULL) )
    {
        return ERR_MEM_NONE;        // ������������ָ��Ϊ��
    }
#endif

    for (i=0; i<rowa; i++)
    {
        for (j=0; j<=i; j++)
        {
            pmRslt[i*rowa+j] = 0.0; // ��������������ǳ�ʼ��Ϊ0
        }
    }// for

    // �����������
    for (i=0; i<rowa; i++)
    {
        for (j=0; j<=i; j++)
        {
            for (k=0; k<cola; k++)
            {
                // mInb����������mIna������������mInb��(k, j)Ԫ��Ϊ[k*rowa+j]
                pmRslt[i*rowa+j] += pmIna[i*cola+k] * pmInb[k*rowa+j];
            }
            pmRslt[j*rowa+i] = pmRslt[i*rowa+j];        // ������Ԫ��
        }
    }//for

    return OK;
}

/****************************************************************
 *�� �� ��: MulMatrix()
 *��    ��:
 *         const void *  mIn: �������
 *               void *mTran: �������
 *               size    row: �����������
 *               size    col: �����������
 *
 *��    �ܣ��Ծ���mIn����ת�ã�����������mTran��
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          ������������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int TranMatrix(const void * mIn, void * mTran, size row, size col)
{
    double *pmIn = (double *)mIn;
    double *pmTran = (double *)mTran;
    int     i, j;      // ѭ������ 

#ifdef __DEBUG__
    if ( (pmIn == NULL) || (pmTran == NULL) )
    {
        return ERR_MEM_NONE;        // ������������ָ��Ϊ��
    }
#endif

    // ����ת��
    for (i=0; i<row; i++)
    {
        for (j=0; j<col; j++)
        {
            pmTran[j*row+i] = pmIn[i*col+j];
        }//for
    }//for

    return OK;
}

// ����x,y��ֵ
#define swap(x, y) { double t = (x); (x) = (y); (y) = t; }

/****************************************************************
 *�� �� ��: InvMtrx()
 *��    ��:
 *         const void *  mIn: �������
 *               void * mInv: �������
 *               size    len: �������������
 *
 *��    �ܣ���Gauseȫѡ��Ԫ���Ծ���mIn�������棬����������mInv��
 *          �������ֻ������������ΪLEN_INV_GAUSE�ľ������
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          ������������ָ��Ϊ�գ�ERR_MEM_NONE
 *          ���󲻿������棺ERR_CANNOT_INV
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int InvMtrx(const void * mIn, void * mInv, size len)
{ 
    int i, j, k;                      // ѭ������
    int rowMax[LEN_INV_GAUSE];        // ���Ԫ�����±�
    int colMax[LEN_INV_GAUSE];        // ���Ԫ�����±�

    double maxElem;                   // ���Ԫ��
    double absElem;                   // Ԫ�ؾ���ֵ

    double *pmIn = (double *)mIn;
    double *pmInv = (double *)mInv;
    double INF = 1e-6;                // һ����С����

#ifdef __DEBUG__
    if ( (pmIn == NULL) || (pmInv == NULL) )
    {
        return ERR_MEM_NONE;        // ������������ָ��Ϊ��
    }
#endif

    // ����������ֵ��������������У�
    // ����ʹ������ֱ�ӱ�������������С�
    for (i=0; i<len; i++)
    {
        for (j=0; j<len; j++)
        {
            pmInv[i*len+j] = pmIn[i*len+j];
        }//for
    }//for

    // �����￪ʼ�����Ӧ��Gauseȫѡ��Ԫ����
    for (k=0; k<len; k++)
    {
        // ��һ����ѡ��Ԫ
        // �ӵ� k �С��� k �п�ʼ�����½�������ѡȡ����ֵ����Ԫ�أ�
        // ����ס��Ԫ�����ڵ��кź��кţ���ͨ���н������н���������������Ԫ��λ���ϡ�
        maxElem = 0.0;
        for (i=k; i<len; i++) 
        { 
            for (j=k; j<len; j++)
            {
                absElem = (pmInv[i*len+j] > 0 ? pmInv[i*len+j] : -pmInv[i*len+j]); //�����ֵ
                if (absElem > maxElem) 
                { 
                    maxElem = absElem; 
                    rowMax[k] = i;        // ��k����ѭ�����Ԫ�ص��к�
                    colMax[k] = j;        // ��k����ѭ�����Ԫ�ص��к�
                } 
            }//for 
        }//for

        if ( (maxElem < INF) && (maxElem > -INF) )        //max ������
        {            
            return ERR_CANNOT_INV;        // �����������ˣ�
        }

        if ( rowMax[k] != k )            // ���Ԫ�ز��ڵ�k��
        {
            for (j=0; j<len; j++)
            {
                // ��k�к����Ԫ�������е�Ԫ�ؽ����н���
                swap(pmInv[k*len+j], pmInv[rowMax[k]*len+j]);
            }
        }

        if ( colMax[k] != k )             // ���Ԫ�ز��ڵ�k��
        {
            for (j=0; j<len; j++)
            {
                // ��k�к����Ԫ�������е�Ԫ�ؽ����н���
                swap(pmInv[j*len+k], pmInv[j*len+colMax[k]]);
            }
        }

        // �ڶ�����Ԫ��m(k, k)��
        pmInv[k*len+k] = 1.0 / pmInv[k*len+k];

        // ������ 
        for (j=0; j<len; j++) 
        { 
            if (j != k)
            {
                // m(k, j) = m(k, j) * m(k, k)��j = 0, 1, ..., n-1��j != k 
                pmInv[k*len+j] *= pmInv[k*len+k];
            }
        }

        // ���Ĳ� 
        for (i=0; i<len; i++) 
        { 
            if (i != k) 
            { 
                for (j=0; j<len; j++) 
                { 
                    if (j != k)
                    {
                        // m(i, j) = m(i, j) - m(i, k) * m(k, j)��i, j = 0, 1, ..., n-1��i, j != k
                        pmInv[i*len+j] -= pmInv[i*len+k] * pmInv[k*len+j];
                    }
                } 
            }//if
        }//for

        // ���岽
        for (i=0; i<len; i++) 
        { 
            if (i != k)
            {
                // m(i, k) = -m(i, k) * m(k, k)��i = 0, 1, ..., n-1��i != k 
                pmInv[i*len+k] *= -pmInv[k*len+k];
            }
        }//for
    }//for

    // ��󣬸�����ȫѡ��Ԫ����������¼���С��н�������Ϣ���лָ����ָ���ԭ�����£�
    // ��ȫѡ��Ԫ�����У��Ƚ������У��У�����лָ���
    // ԭ�����У��У��������У��У��������ָ���
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
 *�� �� ��: InvSymMtrx()
 *��    ��:
 *         const void *  mIn: �������Ӧ��Ϊ�Գƾ���
 *               void * mInv: �������
 *               size    len: �������������
 *
 *��    �ܣ��ò�ѡ��ԪGause���ԶԳƾ���mIn�������棬�����ѡ��Ԫ��
 *          ���̳�����ԪΪ�㣬�����ȫѡ��Ԫ�ĸ�˹�㷨
 *          ����������mInv��
 *ע   �⣺ ���������mIn���Գƣ�����ȫѡ��Ԫ��˹����
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          ������������ָ��Ϊ�գ�ERR_MEM_NONE
 *          ���󲻿������棺ERR_CANNOT_INV
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int InvSymMtrx(const void * mIn, void * mInv, size len)
{
    int    i, j, k;                // ѭ������
    double *pmIn = (double *)mIn;
    double *pmInv = (double *)mInv;
    double INF = 1e-6;                // һ����С����

#ifdef __DEBUG__
    if ( (pmIn == NULL) || (pmInv == NULL) )
    {
        return ERR_MEM_NONE;        // ������������ָ��Ϊ��
    }
#endif

    // ����������ֵ��������������У�
    // �����ȿ��Ա�����������ֿ���ʹ������
    // ֱ�ӱ�������������С�
    for (i=0; i<len; i++)
    {
        for (j=0; j<=i; j++)
        {
            // �ж���������Ƿ�Գƣ������Գƣ�����ȫѡ��Ԫ��˹����
            double tmp = pmIn[i*len+j] - pmIn[j*len+i];
            if ( (tmp < -INF) || (tmp > INF) )
            {
                return InvMtrx(pmIn, pmInv, len);  // ����ȫѡ��Ԫ��˹����
            }

            pmInv[i*len+j] = pmIn[i*len+j];
            pmInv[j*len+i] = pmIn[i*len+j];
        }//for
    }//for

    for (i=0; i<len; i++)
    {
        if ( (pmInv[i*len+i] < INF) && (pmInv[i*len+i] > -INF) )  // ��ԪΪ��
        {
            return InvMtrx(pmIn, pmInv, len);        // ����ȫѡ��Ԫ��˹����
        }

        // �����(i, i)Ԫ��
        pmInv[i*len+i] = 1.0 / pmInv[i*len+i];

        for (j=0; j<i; j++)
        {
            // ����Ԫ��(0,0)��(i-1, i-1)
            for (k=0; k<=j; k++)
            {
                pmInv[j*len+k] -= pmInv[j*len+i] * pmInv[i*len+k] * pmInv[i*len+i];
                pmInv[k*len+j]  = pmInv[j*len+k];        // ��Ԫ�ضԳ�
            }

            // ����Ԫ��(0,i+1)��(i-1, len)���������ĶԳ�Ԫ��
            for (k=i+1; k<len; k++)
            {
                pmInv[j*len+k] -=  pmInv[j*len+i] * pmInv[i*len+k] * pmInv[i*len+i];
                pmInv[k*len+j]  = -pmInv[j*len+k];        // ��Ԫ�ط��Գ�
            }
        }//for

        // ����Ԫ��(i+1,i+1)��(len, len)�������Ԫ��
         for ( j=i+1; j<len; j++)
        {
            for (k=j; k<len; k++)
            {
                pmInv[j*len+k] -= pmInv[j*len+i] * pmInv[i*len+k] * pmInv[i*len+i];
                pmInv[k*len+j]  = pmInv[j*len+k];
            }
        }

        // �����(i, i)���i��k��Ԫ�ؼ�k��i��Ԫ��
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

