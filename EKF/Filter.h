/****************************************************************
 *�� �� ��: Filter.h
 *��    ��:
 *          ��չKalman�˲�������
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

#ifndef _FILTER_H_
#define _FILTER_H_

#include "Matrix.h"

#define LEN_PV_STATE        6    // λ���ٶ�ģ��״̬�����ĳ���
#define LEN_PV_OBS          6    // λ���ٶ�ģ��״�۲����ĳ���
#define LEN_QG_STATE        5    // ��Ԫ�����ٶ�ģ��״̬�����ĳ���
#define LEN_QG_OBS          3    // ��Ԫ�����ٶ�ģ��״�۲����ĳ���

// ����ϵͳ״̬����󳤶�
#define MAX_STATE  ((LEN_PV_STATE)>(LEN_QG_STATE) ? (LEN_PV_STATE) : (LEN_QG_STATE))
// ����ϵͳ�۲�����󳤶�
#define MAX_OBS    (( LEN_PV_OBS)>(LEN_QG_OBS) ? ( LEN_PV_OBS) : (LEN_QG_OBS))

// ָ��״̬���̵ĺ���
typedef int (*Fxu)(const double *state, const double *param, double *f);

// ָ��״̬���̶�״̬�󵼵ĺ���
typedef int (*FxDot)(const double *state, const double *param, void *fD);

// ָ��۲ⷽ�̵ĺ���
typedef int (*Hx)(const double *state, double *h);

// ָ��۲ⷽ�̶�״̬�󵼵ĺ���
typedef int (*HxDot)(const double *state, void *hxDot);

typedef struct Model           // ��Ҫ����EKF�˲���ģ��
{
    Fxu      fxu;              // ״̬����
    FxDot    fxDot;            // ״̬������
    Hx       hx;               // �۲ⷽ��
    HxDot    hxDot;            // �۲ⷽ����
}*pModel;

// EKF�˲�����Ҫ�Ĳ���
typedef struct EKFParam 
{
    const double *Q;           // Q��
    const double *R;           // R��
    double     *Pk;            // Pk����
    double     *preEst;        // ��һ�εĹ���ֵ
    int        *isFirst;       // �ǵ�һ��ִ���˲���
}*pEKFParam;

typedef struct ModelParam      // ״̬�����۲���������ֵ�����ַ
{
    const double *state;       // �����˲���״ֵ̬
    const double *obs;         // �۲���
    double    *est;            // ����ֵ�����ַ
    const double *othParam;    // �˲���Ҫ����������

    size    lenState;          // ״̬������
    size    lenObs;            // �۲�������
}*pModelParam;

// state:�˲�ǰ״̬ x, y, z, u, v, w, q0, q1, q2, q3, g
// obs :�۲���x, y, z, u, v, w, phi, theta, psi
// param:��Ҫ�Ĳ���  p, q, r, ax, ay, az
// est: �˲����״̬ x, y, z, u, v, w, q0, q1, q2, q3, g
// ��state��ʾ״̬����EKF�˲�
void EKF(const double *state, const double *obs, const double *param, double *est);

// pModel model: ��Ҫ����EKF�˲���ģ��
// pModelParam mParam: ״̬�����۲���������ֵ�����ַ
// pEKFParam ekfParam: KF�˲�����Ҫ�Ĳ���
// ��model��ʾ��ģ�ͽ���EKF�˲�
// �� �� ֵ: �ɹ�: OK, ����ָ��Ϊ�գ�ERR_MEM_NONE
int DoEKF(pModel model, pModelParam mParam, pEKFParam ekfParam);

// ��q���й�һ����
// �� �� ֵ: �ɹ�: OK, ����ָ��Ϊ�գ�ERR_MEM_NONE
int Norm(double *q);

// state: x, y, z, u, v, w
// param:  p, q, r, ax, ay, az, q0, q1, q2, q3
// fxu: fxu ������6��״̬x, y, z, u, v, w
// �� �� ֵ: �ɹ�: OK, ����ָ��Ϊ�գ�ERR_MEM_NONE
int PosVelFxu(const double *state, const double *param, double *fxu);

// state: x, y, z, u, v, w
// param:  p, q, r, ax, ay, az, q0, q1, q2, q3
// fxDot: ��fx�󵼼�����6 * 6����
// �� �� ֵ: �ɹ�: OK
int PosVelFxDot(const double *state, const double *param, void *fxDot);

// state: x, y, z, u, v, w
// hx: hx������6��״̬
//        x, y, z, u, v, w
// �� �� ֵ: �ɹ�: OK, ����ָ��Ϊ�գ�ERR_MEM_NONE
int PosVelHx(const double *state, double *hx);

// state: x, y, z, u, v
// hxDot����ֵ��6 * 6�׵�λ����
// �� �� ֵ: �ɹ�: OK, ����ָ��Ϊ�գ�ERR_MEM_NONE
int PosVelHxDot(const double *state, void *hxDot);

// state:�˲�ǰ״̬ x, y, z, u, v, w
// obs :�۲���x, y, z, u, v, w
// param:��Ҫ�Ĳ���  p, q, r, ax, ay, az, q0, q1, q2, q3
// est: �˲����״̬ x, y, z, u, v, w
void PosVelEKF(const double *state, const double *obs, const double *param, double *est);

// state: q0, q1, q2, q3, g
// param:  p, q, r, ax, ay, az
// fxu: QGFxu ������5��״̬
// �� �� ֵ: �ɹ�: OK, ����ָ��Ϊ�գ�ERR_MEM_NONE
int QGFxu(const double *state, const double *param, double *fxu);

// state: q0, q1, q2, q3, g
// param:  p, q, r, ax, ay, az
// fx_dot: QGFx_dot ������5 * 5����
// �� �� ֵ: �ɹ�: OK
int QGFxDot(const double *state, const double *param, void *fxDot);

// state: q0, q1, q2, q3, g
// hx: QGHx������3��״̬: phi, theta, psi
// �� �� ֵ: �ɹ�: OK
int QGHx(const double *state, double *hx);

// state: q0, q1, q2, q3, g
// hxDot������3 * 5�׾���
// �� �� ֵ: �ɹ�: OK
int QGHxDot(const double *state, void *hxDot);

// state:�˲�ǰ״̬ q0, q1, q2, q3, g
// obs :�۲��� phi, theta, psi
// param:��Ҫ�Ĳ��� p, q, r, ax, ay, az
// est: �˲����״̬ q0, q1, q2, q3, g
void QGEKF(const double *state, const double *obs, const double *param, double *est);

#define dt 0.03333

extern double ahrs_theta[3];
extern double accel_g;

extern double raw_input_uvw[3];
extern double Cnb[3][3];
extern double Cbn[3][3];
extern double Znb[3][3];
extern void ahrs_init(void);
extern void state_init(void);
extern void ahrs_update(void );

extern double raw_ahrs_theta[3];
//extern double Zahrs_theta[3];
//extern double raw_current_xyz[3];
//extern double raw_current_uvw[3];
extern double raw_K_Quat[4];
extern double est_Kalman_State[11];
#endif

