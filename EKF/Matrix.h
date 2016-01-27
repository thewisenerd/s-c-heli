/****************************************************************
 *�� �� ��: Matrix.h
 *��    ��:
 *          �������㺯����
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

#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef int size;            // ��������д�С

extern char* errmsg[];       // ����������Ϣ

#define    OK               0 // ���������˳�
#define    LEN_INV_GAUSE   20 // ��˹������ʱ����ĳ���

// ��������ʶ
#define    ERR_NO_ERROR     0  // �޴���
#define    ERR_MEM_NONE     1  // ������δ��ʼ�����ڴ�
#define    ERR_CANNOT_INV   2  // ���󲻿�������

// �������mIna����mInb������������mRslt��
// row: ina�������� col:ina������
// ����ֵ: �ɹ�: OK�� ������������ָ��Ϊ�գ�ERR_MEM_NONE
int AddMatrix(const void * mIna, const void * mInb, void * mRslt, size row, size col);

// �������mIna��ȥmInb������������mRslt��
// row: ina�������� col:ina������
// ����ֵ: �ɹ�: OK�� ������������ָ��Ϊ�գ�ERR_MEM_NONE
int SubMatrix(const void * mIna, const void * mInb, void * mRslt, size row, size col);

// ����mIna�� mInb�� ����������mRslt��
// rowa: mIna�������� cola:mIna�������� colb:mInb������
// ����ֵ: �ɹ�: OK�� ������������ָ��Ϊ�գ�ERR_MEM_NONE
int MulMatrix(const void * mIna, const void * mInb, void * mRslt, size rowa, size cola, size colb);

// ����mIna�� mInb������������mRslt�У����ҽ��Ϊ�Գƾ���
// rowa: mIna������
// ����ֵ: �ɹ�: OK�� ������������ָ��Ϊ�գ�ERR_MEM_NONE
int SymMulMatrix(const void * mIna, const void * mInb, void * mRslt, size rowa, size cola);

// �Ծ���mIn����ת�ã�����������mTran��
// row: mIn�������� col:mIn������
// ����ֵ: �ɹ�: OK�� ������������ָ��Ϊ�գ�ERR_MEM_NONE
int TranMatrix(const void * mIn, void * mTran, size row, size col);

// �Ծ���mIn�������棬����������mInv��
// len: mIn��������
// �����㷨Ϊȫѡ��Ԫ�ĸ�˹�㷨
// ����ֵ: �ɹ�: OK��  ������������ָ��Ϊ�գ�ERR_MEM_NONE
//         ���󲻿������棺ERR_CANNOT_INV
int InvMtrx(const void * mIn, void * mInv, size len);

// �ԶԳƾ���mIn�������棬����������mInv��
// len: mIn��������
// �����㷨�������ò�ѡ��Ԫ�ĸ�˹�㷨���棬�����ѡ��Ԫ��
// ���̳�����ԪΪ����������mIn���Գƣ������ȫѡ��Ԫ�ĸ�˹�㷨
// ����ֵ: �ɹ�: OK��  ������������ָ��Ϊ�գ�ERR_MEM_NONE
//         ���󲻿������棺ERR_CANNOT_INV
int InvSymMtrx(const void * mIn, void * mInv, size len);


#endif
