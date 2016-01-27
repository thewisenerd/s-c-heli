/****************************************************************
 *文 件 名: Filter.h
 *描    述:
 *          扩展Kalman滤波函数集
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

#ifndef _FILTER_H_
#define _FILTER_H_

#include "Matrix.h"

#define LEN_PV_STATE        6    // 位置速度模型状态向量的长度
#define LEN_PV_OBS          6    // 位置速度模型状观测量的长度
#define LEN_QG_STATE        5    // 四元数加速度模型状态向量的长度
#define LEN_QG_OBS          3    // 四元数加速度模型状观测量的长度

// 所有系统状态量最大长度
#define MAX_STATE  ((LEN_PV_STATE)>(LEN_QG_STATE) ? (LEN_PV_STATE) : (LEN_QG_STATE))
// 所有系统观测量最大长度
#define MAX_OBS    (( LEN_PV_OBS)>(LEN_QG_OBS) ? ( LEN_PV_OBS) : (LEN_QG_OBS))

// 指向状态方程的函数
typedef int (*Fxu)(const double *state, const double *param, double *f);

// 指向状态方程对状态求导的函数
typedef int (*FxDot)(const double *state, const double *param, void *fD);

// 指向观测方程的函数
typedef int (*Hx)(const double *state, double *h);

// 指向观测方程对状态求导的函数
typedef int (*HxDot)(const double *state, void *hxDot);

typedef struct Model           // 需要进行EKF滤波的模型
{
    Fxu      fxu;              // 状态方程
    FxDot    fxDot;            // 状态方程求导
    Hx       hx;               // 观测方程
    HxDot    hxDot;            // 观测方程求导
}*pModel;

// EKF滤波所需要的参数
typedef struct EKFParam 
{
    const double *Q;           // Q阵
    const double *R;           // R阵
    double     *Pk;            // Pk参数
    double     *preEst;        // 上一次的估计值
    int        *isFirst;       // 是第一次执行滤波吗
}*pEKFParam;

typedef struct ModelParam      // 状态量、观测量及估计值保存地址
{
    const double *state;       // 本次滤波的状态值
    const double *obs;         // 观测量
    double    *est;            // 估计值保存地址
    const double *othParam;    // 滤波需要的其它参数

    size    lenState;          // 状态量长度
    size    lenObs;            // 观测量长度
}*pModelParam;

// state:滤波前状态 x, y, z, u, v, w, q0, q1, q2, q3, g
// obs :观测量x, y, z, u, v, w, phi, theta, psi
// param:需要的参数  p, q, r, ax, ay, az
// est: 滤波后的状态 x, y, z, u, v, w, q0, q1, q2, q3, g
// 对state所示状态进行EKF滤波
void EKF(const double *state, const double *obs, const double *param, double *est);

// pModel model: 需要进行EKF滤波的模型
// pModelParam mParam: 状态量、观测量及估计值保存地址
// pEKFParam ekfParam: KF滤波所需要的参数
// 对model所示的模型进行EKF滤波
// 返 回 值: 成功: OK, 输入指针为空：ERR_MEM_NONE
int DoEKF(pModel model, pModelParam mParam, pEKFParam ekfParam);

// 对q进行归一处理
// 返 回 值: 成功: OK, 输入指针为空：ERR_MEM_NONE
int Norm(double *q);

// state: x, y, z, u, v, w
// param:  p, q, r, ax, ay, az, q0, q1, q2, q3
// fxu: fxu 计算后的6个状态x, y, z, u, v, w
// 返 回 值: 成功: OK, 输入指针为空：ERR_MEM_NONE
int PosVelFxu(const double *state, const double *param, double *fxu);

// state: x, y, z, u, v, w
// param:  p, q, r, ax, ay, az, q0, q1, q2, q3
// fxDot: 对fx求导计算后的6 * 6矩阵
// 返 回 值: 成功: OK
int PosVelFxDot(const double *state, const double *param, void *fxDot);

// state: x, y, z, u, v, w
// hx: hx计算后的6个状态
//        x, y, z, u, v, w
// 返 回 值: 成功: OK, 输入指针为空：ERR_MEM_NONE
int PosVelHx(const double *state, double *hx);

// state: x, y, z, u, v
// hxDot返回值：6 * 6阶单位矩阵
// 返 回 值: 成功: OK, 输入指针为空：ERR_MEM_NONE
int PosVelHxDot(const double *state, void *hxDot);

// state:滤波前状态 x, y, z, u, v, w
// obs :观测量x, y, z, u, v, w
// param:需要的参数  p, q, r, ax, ay, az, q0, q1, q2, q3
// est: 滤波后的状态 x, y, z, u, v, w
void PosVelEKF(const double *state, const double *obs, const double *param, double *est);

// state: q0, q1, q2, q3, g
// param:  p, q, r, ax, ay, az
// fxu: QGFxu 计算后的5个状态
// 返 回 值: 成功: OK, 输入指针为空：ERR_MEM_NONE
int QGFxu(const double *state, const double *param, double *fxu);

// state: q0, q1, q2, q3, g
// param:  p, q, r, ax, ay, az
// fx_dot: QGFx_dot 计算后的5 * 5矩阵
// 返 回 值: 成功: OK
int QGFxDot(const double *state, const double *param, void *fxDot);

// state: q0, q1, q2, q3, g
// hx: QGHx计算后的3个状态: phi, theta, psi
// 返 回 值: 成功: OK
int QGHx(const double *state, double *hx);

// state: q0, q1, q2, q3, g
// hxDot：返回3 * 5阶矩阵
// 返 回 值: 成功: OK
int QGHxDot(const double *state, void *hxDot);

// state:滤波前状态 q0, q1, q2, q3, g
// obs :观测量 phi, theta, psi
// param:需要的参数 p, q, r, ax, ay, az
// est: 滤波后的状态 q0, q1, q2, q3, g
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

