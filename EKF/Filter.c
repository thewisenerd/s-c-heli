/****************************************************************
 *文 件 名: Filter.c
 *描    述:
 *          扩展Kalman滤波实现
 *          文件注释中公式标号全部为张谦论文中的公式标号
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
#include<includs.h>
double K_Quat[4];//={0.0,0.0,0.0,0.0};
double ahrs_theta[3];
double raw_K_Quat[4];
double raw_ahrs_theta[3];
double accel_g = 0.0;
double est_Kalman_State[11];
double raw_input_uvw[3];
double Cnb[3][3];
double Cbn[3][3];
#define offset_p 0.24
double offset_x=0.0;
double offset_y=0.0;
double Z_K_Quat[4];

/*--------------------------------------------------------------------------------------*/ 
void ahrs_init(void)
{
	accel2euler( raw_ahrs_theta, imu_accel, ahrs_compass );
        
	euler2quat( raw_K_Quat, raw_ahrs_theta );
	
	norm( raw_K_Quat );
	
	//printf("INIT_q0=%f\tq1=%f\tq2=%f\tq3=%f\n",quat[0],quat[1],quat[2],quat[3] );
	//bias[0] = ahrs_pqr[0];
	//bias[1] = ahrs_pqr[1];
	//bias[2] = ahrs_pqr[2];
}
void state_init(void)
{
      accel_g=sqrt(ahrs_accel[0]*ahrs_accel[0] + ahrs_accel[1]*ahrs_accel[1] + ahrs_accel[2]*ahrs_accel[2]);      
      accel2euler( raw_ahrs_theta, ahrs_accel, ahrs_compass );      
      euler2quat( raw_K_Quat, raw_ahrs_theta );
      norm( raw_K_Quat );
}
/*double uvw2_update()
{
    static double last_Z = 0.0; 
    double vel_z_current =0.0 ; 
    float sum_vel_z =0.0; 
    static float window_vel_z[8] = {0.0} ;  // z向速度平均值滤波
    double uvw;
    static int UVW2_LEN=1;
    
    vel_z_current = (position.raw_current_xyz[2] - last_Z)*30;
    for( char i =UVW2_LEN-1 ; i != 0 ; --i )
    {
        window_vel_z[i] = window_vel_z[i-1];
        sum_vel_z += window_vel_z[i];
    }
    window_vel_z[0] = vel_z_current ;
    sum_vel_z += window_vel_z[0] ;
    uvw = sum_vel_z/UVW2_LEN ;
	
    if(UVW2_LEN >= 8)
         UVW2_LEN = 8;
      else UVW2_LEN++;
 
    last_Z = position.raw_current_xyz[2];
     
    return uvw;   
}*/
double uvw2_update(double raw_uvw)
{
   double vel_z_current =0.0 ; 
   float sum_vel_z =0.0; 
   static float window_vel_z[3] = {0.0} ;  // z向速度平均值滤波
   static int UVW2_LEN=1;

    for( char i =UVW2_LEN-1 ; i != 0 ; --i )
    {
        window_vel_z[i] = window_vel_z[i-1];
        sum_vel_z += window_vel_z[i];
    }
    window_vel_z[0] = raw_uvw;
    sum_vel_z += window_vel_z[0] ;
    vel_z_current = sum_vel_z/UVW2_LEN ;
	
    if(UVW2_LEN >= 3)
         UVW2_LEN = 3;
      else UVW2_LEN++;   
	  
    return vel_z_current;   
}

void Cnb_update(double*q)
{  
	//此处Cnb表示从NED坐标系到机体坐标系的转换矩阵（用Cbn表示更合适）
    double q0=q[0],q1=q[1],q2=q[2],q3=q[3];
    Cbn[0][0] =  1-2*(q2*q2 + q3*q3);
    Cbn[0][1] =  2*(q1*q2 - q0*q3);
    Cbn[0][2] =  2*(q1*q3+q0*q2);
    
    Cbn[1][0] =   2*(q1*q2 + q0*q3);
    Cbn[1][1] =   1-2*(q1*q1+q3*q3);
    Cbn[1][2] =   2*(q2*q3-q0*q1);
    
    Cbn[2][0] =   2*(q1*q3 - q0*q2);
    Cbn[2][1] =   2*(q2*q3+q0*q1);
    Cbn[2][2] =   1-2*(q1*q1+q2*q2); 
    for(int i=0;i<3;i++)
       for(int j=0;j<3;j++)
         {
           Cnb[i][j]=Cbn[j][i];
         }
}
void ahrs_update(void)
{
  state_init();//得到观测值

//  position.raw_current_xyz[2]=bmp_height;
//  
//  static double height_array[6];                //mei2015-4-20
//  static double aux_sum;
//  static int aux_height_num = 3;             //mei2015-4-20 为6/2
//  static int height_num = 0;                    //mei2015-4-20  
//  aux_sum += height_array[height_num];          //mei2015-4-20 加上最旧的数据，相当于最旧数据清零
//  height_array[height_num] = bmp_height;        //mei2015-4-20
//  aux_sum += height_array[height_num];          //mei2015-4-20 加上最新的数据，引入最新数据
//  aux_sum -= height_array[aux_height_num];      
//  aux_sum -= height_array[aux_height_num];      //mei2015-4-20 减去两倍的中间过渡数，相当于由加变成减
//  position.raw_current_uvw[2] = aux_sum*30/9; //mei2015-4-20//9为3^3
//  height_num++;                                     //mei2015-4-20
//  aux_height_num++;                                 //mei2015-4-20
//  if (height_num == 6) height_num = 0;              //mei2015-4-20
//  if (aux_height_num == 6) aux_height_num = 0;      //mei2015-4-20
  
//  static double height_array[4];        //mei2015-4-20
//  static double bmp_height_0;            //mei2015-4-20
//  static int height_num = 0;            //mei2015-4-20  
//  bmp_height_0 = height_array[height_num];          //mei2015-4-20注意bmp_height_0和height_array[4]总共5个数
//  height_array[height_num] = bmp_height;            //mei2015-4-20
//  position.raw_current_uvw[2] = (height_array[height_num] - bmp_height_0)*30/4;//mei2015-4-20
//  height_num++;                                     //mei2015-4-20
//  if (height_num == 4) height_num = 0;              //mei2015-4-20
  
//mei2015-4-20  position.raw_current_uvw[2] = (position.raw_current_xyz[2] - last_bmp_height)*30;
//  last_bmp_height = position.raw_current_xyz[2];
  //position.raw_current_uvw[2]=uvw2_update();
  
  //注意：kalman滤波需要的速度项的输入值为机体坐标系下的值---在EKF里面转换成机体下值//
//  double Kalman_State[11]={position.raw_current_xyz[0],position.raw_current_xyz[1],position.raw_current_xyz[2],
//                           position.raw_current_uvw[0],position.raw_current_uvw[1],position.raw_current_uvw[2],
//                           raw_K_Quat[0],raw_K_Quat[1],raw_K_Quat[2],raw_K_Quat[3],accel_g};
//  double Obs_State[9] =  {position.raw_current_xyz[0],position.raw_current_xyz[1],position.raw_current_xyz[2],
//                           position.raw_current_uvw[0],position.raw_current_uvw[1],position.raw_current_uvw[2],
//                           raw_ahrs_theta[0],raw_ahrs_theta[1],raw_ahrs_theta[2]};
//  double Need_Param[6] = {ahrs_pqr[0],ahrs_pqr[1],ahrs_pqr[2],
//                          ahrs_accel[0],ahrs_accel[1],ahrs_accel[2]};
//  
//  EKF(Kalman_State,Obs_State,Need_Param,est_Kalman_State);
//    
//  for(int i =0 ; i!=3; ++i) 
//  position.current_xyz[i] = est_Kalman_State[i];
//  
//  for(int i =3 ; i!=6; ++i) 
//  input_uvw[i-3] = est_Kalman_State[i];//滤波后得到标准机体坐标系下飞机三轴线速度
//  
//  for(int i =6 ; i!=10; ++i) 
//  K_Quat[i-6] = est_Kalman_State[i];
//  
//  quat2euler( ahrs_theta, K_Quat );
//  accel_g = est_Kalman_State[10];
//
//   //飞机的高度项控制量均采用ned下位置速度  position.current_uvw为滤波后ned下速度值
//  //position.current_uvw[0]=Cbn[0][0]*input_uvw[0]+Cbn[0][1]*input_uvw[1]+Cbn[0][2]*input_uvw[2];
//  //position.current_uvw[1]=Cbn[1][0]*input_uvw[0]+Cbn[1][1]*input_uvw[1]+Cbn[1][2]*input_uvw[2];
//  position.current_uvw[2]=Cbn[2][0]*input_uvw[0]+Cbn[2][1]*input_uvw[1]+Cbn[2][2]*input_uvw[2];
//  //input_uvw[2] = position.current_uvw[2];
//  input_uvw[2] = uvw2_update(position.current_uvw[2]);
//  //---------------gps补偿-----------------//
//  offset_x=cos(ahrs_theta[2])*offset_p;
//  offset_y=sin(ahrs_theta[2])*offset_p;
//  position.current_xyz[0]=position.current_xyz[0]+offset_x;
//  position.current_xyz[1]=position.current_xyz[1]+offset_y;  //航向角不准 补偿会出现问题
}


/****************************************************************
 *函 数 名: EKF()
 *参    数:
 *         state: 滤波前状态x, y, z, u, v, w, q0, q1, q2, q3, g
 *         obs  : 观测量x, y, z, u, v, w, phi, theta, psi
 *         param: 需要的参数 p, q, r, ax, ay, az
 *         est  : 滤波后的状态x, y, z, u, v, w, q0, q1, q2, q3, g
 *
 *功    能：对位置、速度、四元数、重力加速度进行EKF滤波
 *
 *返 回 值:
 *          成功: OK
 *          输入输出指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
void EKF(const double *state, const double *obs, const double *param, double *est)
{
    // 位置速度滤波初始值
    double pvState[LEN_PV_STATE];    // 滤波前状态 x, y, z, u, v, w
    double pvObs[LEN_PV_OBS];        // 观测量x, y, z, u, v, w

    // 四元数、加速度滤波初始值
    double qgState[LEN_QG_STATE];    // 滤波前状态q0, q1, q2, q3, g
    double qgObs[LEN_QG_OBS];        // 观测量phi, theta, psi

    // 滤波需要的辅助参数p, q, r, ax, ay, az, q0, q1, q2, q3共10个
    // 对四元数、加速度滤波时用到前6个
    // 对位置、速度滤波时全部用到
    double othParam[10];
    int    qgLenParam = 6;
    int    pvLenParam = 10;
    int i, j;                    // 循环变量
    // 对四元数、加速度滤波参数初始化
    for (i=0; i<LEN_QG_STATE; i++)
    {
        qgState[i] = state[i+ LEN_PV_STATE];        // 状态量
    }
    for (i=0; i<LEN_QG_OBS; i++)
    {
        qgObs[i] = obs[i+ LEN_PV_OBS];        // 观测量
    }
    for (i=0; i<qgLenParam; i++)
    {
        othParam[i] = param[i];               // 辅助参数
    }
    // 对四元数、加速度滤波，滤波后四元素的存储首地址为est+LEN_PV_STATE
    QGEKF(qgState, qgObs, othParam, est+LEN_PV_STATE);
    
	// 对速度位置滤波参数初始化	
	j = LEN_PV_STATE;
    for (i=qgLenParam; i<pvLenParam; i++)
    {
        othParam[i] = est[j++];       // 将滤波后的四元数给othParam
    }   
    for (i=0; i<LEN_PV_STATE; i++)
    {
        pvState[i] = state[i];        // 状态量
        // 观测量,它和状态量相同，所以可以在同一个循环里面进行
        pvObs[i] = obs[i];
    }	
	//--------------------------------------syc-------------------------------//
	//把NED下的速度转化为机体坐标下的速度  作为kalman的输入值
	Cnb_update(est+LEN_PV_STATE);      
	pvObs[3]=Cnb[0][0]*obs[3]+Cnb[0][1]*obs[4]+Cnb[0][2]*obs[5];
    pvObs[4]=Cnb[1][0]*obs[3]+Cnb[1][1]*obs[4]+Cnb[1][2]*obs[5];
    pvObs[5]=Cnb[2][0]*obs[3]+Cnb[2][1]*obs[4]+Cnb[2][2]*obs[5];
    pvState[3]= pvObs[3];
    pvState[4]= pvObs[4];
    pvState[5]= pvObs[5];
    // 对位置、速度滤波，滤波后的存储首地址为est
    PosVelEKF(pvState, pvObs, othParam, est);
}

/****************************************************************
 *函 数 名: DoEKF()
 *参    数:
 *               pModel model: 需要进行EKF滤波的模型
 *         pModelParam mParam: 状态量、观测量及估计值保存地址
 *         pEKFParam ekfParam: KF滤波所需要的参数
 *
 *功    能：对model所示的模型进行EKF滤波
 *          考虑了Pkk-1和Pk为对称矩阵，当求解这两项时的算法和
 *          普通矩阵的乘法不一样。
 *
 *返 回 值:
 *          成功: OK
 *          输入输出指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int DoEKF(pModel model, pModelParam mParam, pEKFParam ekfParam)
{
    const double T = 0.033;      // 采样周期
    int i, j, k;                // 循环变量

    // 以下为滤波过程中要用的临时变量
    // 这里统一用一维数组表示矩阵,为了复用地址，这里按最大需求开辟
    // 由于在没有操作系统的ARM7上不可以动态开辟空间，所以这里只能用
    // 这种比较笨的方法了。
    double fx[MAX_STATE];    // 保存状态方程计算值
    double Xhatkk_1[MAX_STATE];
    double Hk[MAX_OBS * MAX_STATE];            // 保存观测矩阵Hk
    double Phikk_1[MAX_STATE * MAX_STATE];
    double tmp[MAX_STATE * MAX_STATE];
    double Pkk_1[MAX_STATE * MAX_STATE];
    double Kk[MAX_STATE * MAX_OBS];
    double PxHk[MAX_STATE * MAX_OBS];
    double TranHk[MAX_STATE * MAX_OBS];
    double KkTmp[MAX_OBS * MAX_OBS];
    double h[MAX_OBS];
    double KkxHk[MAX_STATE * MAX_STATE];

    //double TranPhikk_1[MAX_STATE * MAX_STATE];
      
#ifdef __DEBUG__
    if ( (model == NULL) || (mParam == NULL) || (ekfParam == NULL) )
    {
        return ERR_MEM_NONE;        // 输入输出指针为空
    }
#endif

    if (*ekfParam->isFirst == 1)        // 第一次执行滤波
    {
        
        *ekfParam->isFirst = 0;
        // 将传入的状态直接当作估计值
        for (i=0; i<mParam->lenState; i++)
        {
            ekfParam->preEst[i] = mParam->state[i];
            mParam->est[i] = mParam->state[i];
        }
        return OK;
    }//if

// EKF滤波过程
   
    
    // 进一步滤波Xhatkk_1 = Xhatk_1 + T * fxu(Xhatk_1) 式(3-60)
    model->fxu(ekfParam->preEst, mParam->othParam, fx);
    for (i=0; i<mParam->lenState; i++)
    {
        fx[i] *= T;
    }
    AddMatrix(ekfParam->preEst, fx, Xhatkk_1,  mParam->lenState, 1);

    if ( model->fxu == QGFxu)      // 如果状态方程为四元数的则需要归一Q
    {
        Norm(Xhatkk_1);       // 归一Q
    }

    // 求得观测矩阵对状态的求导，Hk被用的大小为mParam->lenObs * mParam->lenState
    model->hxDot(Xhatkk_1, Hk); 

    // 状态转移矩阵,离散化导致矩状态转移阵前面需要乘以T
    // Phikk_1被用的大小为mParam->lenState * mParam->lenState
    model->fxDot(ekfParam->preEst, mParam->othParam, Phikk_1);
    for (i=0; i<mParam->lenState; i++)
    {
        for (j=0; j<mParam->lenState; j++)
        {
            Phikk_1[i * mParam->lenState + j] *= T;
        }
    }

    // 预测误差方差矩阵Pkk_1
    // 计算Pkk_1，这里考虑Pkk_1为对称矩阵
    // 中间变量tmp用到的大小为mParam->lenState * mParam->lenState
    MulMatrix(Phikk_1, ekfParam->Pk, tmp,
              mParam->lenState, mParam->lenState, mParam->lenState);
    // 这里两矩阵相乘为对称矩阵，所以可以简化
    // 初始化Pkk_1,被用到的大小为mParam->lenState * mParam->lenState
    for (i=0; i<mParam->lenState; i++)
    {
        for (j=0; j<=i; j++)
        {
            Pkk_1[i * mParam->lenState + j] = 0.0;    // 将输出矩阵下三角初始化为0
        }
    }
    for (i=0; i<mParam->lenState; i++)        // 矩阵相乘
    {
        for (j=0; j<=i; j++)
        {
            for (k=0; k<mParam->lenState; k++)
            {
                // 这里乘以Phikk_1的转置
                Pkk_1[i * mParam->lenState + j] += 
                    tmp[i * mParam->lenState + k] *  Phikk_1[j * mParam->lenState + k];
            }
            Pkk_1[j * mParam->lenState + i] = Pkk_1[i * mParam->lenState + j];        // 对称的元素
        }
    }

    for (i=0; i<mParam->lenState; i++)
    {
        // 加上Q阵，这里只要加上对角线上的元素即可
        Pkk_1[i * mParam->lenState + i] += ekfParam->Q[i];
    }

    // 求取滤波增益矩阵 式(3-62)
    TranMatrix(Hk, TranHk, mParam->lenObs, mParam->lenState);
    // PxHk被用到的大小为 mParam->lenState * mParam->lenObs
    MulMatrix(Pkk_1, TranHk, PxHk, mParam->lenState, mParam->lenState, mParam->lenObs);
    // KkTmp被用到的大小为mParam->lenObs * mParam->lenObs
    MulMatrix(Hk, PxHk, KkTmp, mParam->lenObs, mParam->lenState, mParam->lenObs);

    // Hk*Pkk_1*Hk' + R，考虑到R为对角矩阵，所以不用按一般相乘计算
    // 只需要加对角线元素即可
    for (i=0; i<mParam->lenObs; i++)
    {
        KkTmp[i * mParam->lenObs + i] += ekfParam->R[i];
    }

    // Hk*Pkk_1*Hk' + R 对称矩阵
    if ( InvSymMtrx(KkTmp, KkTmp, mParam->lenObs) != OK )
    {
        // 如果Hk*Pkk_1*Hk' + R奇异无法求逆
        // 则将传入的状态直接当作估计值
        for (i=0; i<mParam->lenState; i++)
        {
            ekfParam->preEst[i] = mParam->state[i];
            mParam->est[i] = mParam->state[i];
        }
        return OK;
    }

    // Kk被用到的大小为mParam->lenState * mParam->lenObs
    MulMatrix(PxHk, KkTmp, Kk, mParam->lenState, mParam->lenObs, mParam->lenObs);

    // 状态估计
    model->hx(Xhatkk_1, h);
    SubMatrix( mParam->obs, h, h, mParam->lenObs, 1);        // h = obs - hx(Xhatkk_1)
     //----------------------- syc 2011.11.14------------------------------//
   if ( model->fxu == QGFxu)     // 如果状态方程为四元数的则需要调整角度
    {
      for(int i=0;i<3;i++)
      {
        if(h[i]>PI)
          h[i]-=2.0*PI;
        else
          if(h[i]<-PI)
            h[i]+=2.0*PI; 
      }
    }
    //------------------------------------------------------------------//
    MulMatrix(Kk, h, ekfParam->preEst, mParam->lenState, mParam->lenObs, 1);
    // ekfParam->preEst是本次求得的滤波值
    AddMatrix(ekfParam->preEst, Xhatkk_1, ekfParam->preEst, mParam->lenState, 1);

    if ( model->fxu == QGFxu)     // 如果状态方程为四元数的则需要归一Q
    {
        Norm(ekfParam->preEst);   // 归一Q
    }

    // 滤波方差矩阵
    // Kk被用到的大小为mParam->lenState * mParam->lenState
    MulMatrix(Kk, Hk, KkxHk, mParam->lenState, mParam->lenObs, mParam->lenState);
    for (i=0; i<mParam->lenState; i++)
    {
        for (j=0; j<mParam->lenState; j++)
        {
            KkxHk[i * mParam->lenState + j] = (i == j) - KkxHk[i * mParam->lenState + j];
        }
    }
   // MulMatrix(KkxHk, Pkk_1, ekfParam->Pk, mParam->lenState, mParam->lenState, mParam->lenState);
    for (i=0; i<mParam->lenState; i++)
    {
        for (j=0; j<=i; j++)
        {
            ekfParam->Pk[i*mParam->lenState + j] = 0.0;    // Pk下三角矩阵为零
        }
    }

    for (i=0; i<mParam->lenState; i++)        // 矩阵相乘
    {
        for (j=0; j<=i; j++)
        {
            for (k=0; k<mParam->lenState; k++)
            {
                ekfParam->Pk[i*mParam->lenState + j] +=
                    KkxHk[i * mParam->lenState + k] *  Pkk_1[k * mParam->lenState + j];
            }
            // 对称的元素
            ekfParam->Pk[j*mParam->lenState + i] = ekfParam->Pk[i*mParam->lenState + j];
        }
    }

    // 将结果复制到目的地址
    for (i=0; i<mParam->lenState; i++)
    {
        mParam->est[i] = ekfParam->preEst[i];
    }

    return OK;
}

/****************************************************************
 *函 数 名: Norm()
 *参    数:
 *         double * q: 需要归一的元素的首地址
 *
 *功    能：对q进行归一处理
 *
 *返 回 值:
 *          成功: OK
 *          输入指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int Norm(double *q)
{
    const int N = 4;            // 元素个数
    double sum = 0.0;           // 保存欲归一的元素的平方和，然后求和的平方根
    int i;

#ifdef __DEBUG__
    if (q == NULL)
    {
        return ERR_MEM_NONE;  // 输入指针为空
    }
#endif

    for (i=0; i<N; i++)
    {
        sum += q[i] * q[i];
    }
    sum = sqrt(sum);           // sum = sqrt(...)

    for (i=0; i<N; i++)
    {
         q[i] /= sum ;         // 归一后的元素
    }

    return OK;
}

/****************************************************************
 *函 数 名: PosVelFxu()
 *参    数:
 *         const double *state: 输入状态x, y, z, u, v, w
 *         const double *param: 辅助参数p, q, r, ax, ay, az, q0, q1, q2, q3
 *               double *fxu  : 状态方程输出x, y, z, u, v, w
 *
 *功    能：计算位置和速度的状态方程
 *
 *返 回 值:
 *          成功: OK
 *          输入输出指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int PosVelFxu(const double *state, const double *param, double *fxu)
{
    double u, v, w;             // 三个方向的速度
    double q0, q1, q2, q3;      // 四元数
    double g;                   // 重力加速度
    double p, q, r;             // 陀螺仪三个方向角加速度

#ifdef __DEBUG__
    if ( (state == NULL) || (param == NULL) || (fxu == NULL) )
    {
        return ERR_MEM_NONE;        // 输入输出指针为空
    }
#endif

    u  = state[3];
    v  = state[4];
    w  = state[5];

    p  = param[0];
    q  = param[1];
    r  = param[2];

    q0 = param[6];
    q1 = param[7];
    q2 = param[8];
    q3 = param[9];

    // param[3~5]分别表示: ax, ay, az
    //g = sqrt(param[3] * param[3] + param[4] * param[4] + param[5] * param[5]);//4.16
    g=accel_g; 
    // 式(3-67)的第一二部分
    fxu[0] = Cbn[0][0] * u + Cbn[0][1] * v + Cbn[0][2] * w;
    fxu[1] = Cbn[1][0] * u + Cbn[1][1] * v + Cbn[1][2] * w;
    fxu[2] = Cbn[2][0] * u + Cbn[2][1] * v + Cbn[2][2] * w;
    //fxu[0] = (1-2*(q2*q2 + q3*q3)) * u + 2*(q1*q2 - q0*q3) * v + 2*(q1*q3+q0*q2) * w;
    //fxu[1] = 2*(q1*q2 + q0*q3) * u + (1-2*(q1*q1+q3*q3)) * v + 2*(q2*q3-q0*q1) * w;
    //fxu[2] = 2*(q1*q3 - q0*q2) * u + 2*(q2*q3+q0*q1) * v + (1-2*(q1*q1+q2*q2)) * w;
    fxu[3] = param[3] - 2 * (q1 * q3 - q0 * q2) * g - ( -r * v + q * w );
    fxu[4] = param[4] - 2 * (q2 * q3 + q0 * q1) * g - ( r * u - p * w );
    fxu[5] = param[5] - (1 - 2 * (q1 * q1 + q2 * q2)) * g - ( -q * u + p * v);

    return OK;
}

/****************************************************************
 *函 数 名: PosVelFxDot()
 *参    数:
 *         const double *state: 输入状态x, y, z, u, v, w
 *         const double *param: 辅助参数p, q, r, ax, ay, az, q0, q1, q2, q3
 *               void   *fxDot: 对fx求导计算后的6 * 6矩阵
 *
 *功    能：对位置和速度的状态方程求导
 *
 *返 回 值:
 *          成功: OK
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int PosVelFxDot(const double *state, const double *param, void *fxDot)
{
    // 这里原本应该对输入参数的有效性进行检查的
    // 但是在C语言中所有的变量声明必须放在执行语句前面
    // 另外规定对数组的整体赋值只能在声明的时候才可以
    // 所以这里如果先声明变量，再检查输入参数的有效性，
    // 就不可以对数组整体赋值了，从而破坏了公式的整体性
    // 虽然有些编译器版本支持用的时候声明定义变量，但是为了
    // 移植性，这里统一将声明放到最前面。
    double p  = param[0];
    double q  = param[1];
    double r  = param[2];

    //double q0 = param[6];
    //double q1 = param[7];
    //double q2 = param[8];
    //double q3 = param[9];

    double *pfxDot = (double *)fxDot;
    int i, j;

    // 位置速度状态方程对速度求导，式(3-72)第一二部分
   /* double fuvw[][3] =  { 1-2*(q2*q2+q3*q3), 2 * (q1*q2-q0*q3), 2 * (q1*q3+q0*q2),
                          2 * (q1*q2+q0*q3), 1-2*(q1*q1+q3*q3), 2 * (q2*q3-q0*q1),
                          2 * (q1*q3-q0*q2), 2 * (q2*q3+q0*q1), 1-2*(q1*q1+q2*q2),
                                    0,                 r,                -q,
                                   -r,                 0,                 p,
                                    q,                -p,                 0 };*/
    double fuvw[][3] =  { Cbn[0][0], Cbn[0][1], Cbn[0][2],
                          Cbn[1][0], Cbn[1][1], Cbn[1][2],
                          Cbn[2][0], Cbn[2][1], Cbn[2][2],
                             0,          r,            -q,
                            -r,          0,             p,
                             q,         -p,             0 };
    double *pfuvw = fuvw[0];    // 取得fuvw的首地址
    
    // 位置速度状态方程对位置求导全为零，式3-71
    // 并将这个结果放到输出矩阵中
    // 求导后对位置求导结果在第0～2列
    for (i=0; i<LEN_PV_STATE; i++)
    {
        for (j=0; j<3; j++)        // 共有3个方向的位置
        {
            pfxDot[i*LEN_PV_STATE+j] = 0.0;
        }
    }

    // 共有3个方向的速度，求导后对速度求导结果在第3～5列
    for (j=3; j<LEN_PV_STATE; j++)
    {
        for (i=0; i<LEN_PV_STATE; i++)
        {
            pfxDot[i*LEN_PV_STATE+j] = pfuvw[i*3+(j-3)];
        }
    }//for

    return OK;

}

/****************************************************************
 *函 数 名: PosVelHx()
 *参    数:
 *         const double *state: 输入状态x, y, z, u, v, w
 *               double *fxu  : 观测方程输出x, y, z, u, v, w
 *
 *功    能：计算位置和速度的观测方程
 *
 *返 回 值:
 *          成功: OK
 *          输入输出指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int PosVelHx(const double *state, double *hx)
{
    int i;        // 循环变量
     
     
#ifdef __DEBUG__
    if ( (state == NULL) || (hx == NULL) )
    {
        return ERR_MEM_NONE;        // 输入输出指针为空
    }
#endif

    // 式(3-78)第一二部分，观测量等于状态量
    for (i=0; i<LEN_PV_OBS; i++)
    {
        *hx++ = *state++;  
    }
 
  return OK;
}

/****************************************************************
 *函 数 名: PosVelHxDot()
 *参    数:
 *         const double *state: 输入状态x, y, z, u, v, w
 *               void   *hxDot: 观测方程对位置速度变量的求导，为6 * 6阶单位矩阵
 *
 *功    能：位置速度观测方程对位置速度变量的求导
 *
 *返 回 值:
 *          成功: OK
 *          输入输出指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int PosVelHxDot(const double *state, void *hxDot)
{
    double *phxDot = (double *)hxDot;
    int i, j;                // 循环变量

#ifdef __DEBUG__
    if ( (state == NULL) || (hxDot == NULL) )
    {
        return ERR_MEM_NONE;        // 输入输出指针为空
    }
#endif

    for (i=0; i<LEN_PV_STATE; i++)        // 式3-79 3-80
    {
        for (j=0; j<LEN_PV_OBS; j++)
        {
            // 对角线元素为1，其它为零
            phxDot[i*LEN_PV_STATE+j] = (i == j);
        }
    }//for

    return OK;
}

/****************************************************************
 *函 数 名: PosVelEKF()
 *参    数:
 *         state: 滤波前状态x, y, z, u, v, w
 *         obs  : 观测量x, y, z, u, v, w
 *         param: 需要的参数p, q, r, ax, ay, az, q0, q1, q2, q3
 *         est  : 滤波后的状态x, y, z, u, v, w
 *
 *功    能：位置速度变量的EKF滤波
 *
 *返 回 值:
 *          成功: OK
 *          输入输出指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
void PosVelEKF(const double *state, const double *obs, const double *param, double *est)
{
    //const double Q[LEN_PV_STATE] = {0.08,0.08,0.08,0.5,0.5,0.05};    // Q阵
    //const double R[LEN_PV_OBS] =   {0.3,0.3,0.3,0.3,0.3,0.3};        // R阵
    
    //const double Q[LEN_PV_STATE] = {0.08,0.08,0.08,0.08,0.08,0.05};    // Q阵
    //const double R[LEN_PV_OBS] =   {0.3,0.3,0.3,0.3,0.3,0.3};        // R阵
	
	const double Q[LEN_PV_STATE] = {0.1,0.1,0.15,0.1,0.1,0.03};
    const double R[LEN_PV_OBS] =   {0.3,0.3,0.3,0.3,0.3,0.3};
    
    //const double Q[LEN_PV_STATE] = {0.07,0.007,0.07,0.007,0.005,0.2};    // Q阵
    //const double R[LEN_PV_OBS] =   {0.2,0.02,0.12,0.02,0.01,1.8};    // 悬停稳 
    //const double Q[LEN_PV_STATE] = { 0.01, 0.03, 0.03, 0.03, 0.03, 0.03 };    // Q阵
    //const double R[LEN_PV_OBS] =   { 0.01, 0.08, 0.08, 0.3, 0.3, 0.3 };  //激光数据  
    //Pk参数初始化
    static double s_Pk[LEN_PV_STATE][LEN_PV_STATE] = {{0.01}, {0.0, 0.01}, {0.0, 0.0, 0.01}, {0.0, 0.0, 0.0, 0.01},
                       {0.0, 0.0, 0.0, 0.0, 0.01}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.01} };
    static double s_preEst[LEN_PV_STATE];    // 上次滤波值
    static int    s_isFirst = 1;             // 第一次滤波标志

    struct Model model;                      // 系统模型
    struct ModelParam mParam;                // 状态量、观测量及估计值保存地址
    struct EKFParam ekfParam;                // EKF滤波所需要的参数

    // 设置系统模型
    model.fxu = PosVelFxu;              // 状态方程
    model.fxDot = PosVelFxDot;          // 状态方程求导
    model.hx = PosVelHx;                // 观测方程
    model.hxDot = PosVelHxDot;          // 观测方程求导

    // 设置状态量、观测量及估计值保存地址
    mParam.state = state;        // 本次滤波的状态值
    mParam.obs = obs;            // 观测量
    mParam.est = est;            // 估计值保存地址
    mParam.othParam = param;     // 滤波需要的其它参数
    mParam.lenState = LEN_PV_STATE;    // 状态量长度
    mParam.lenObs = LEN_PV_OBS;        // 观测量长度

    // 设置EKF滤波所需要的参数
    ekfParam.Q = Q;                    // Q阵
    ekfParam.R = R;                    // R阵
    ekfParam.Pk = (double*)s_Pk;       // Pk参数
    ekfParam.preEst = s_preEst;        // 上一次的估计值
    ekfParam.isFirst = &s_isFirst;     // 是第一次执行滤波吗

    DoEKF(&model, &mParam, &ekfParam);    // EKF滤波

}

/****************************************************************
 *函 数 名: QGFxu()
 *参    数:
 *         const double *state: 输入状态q0, q1, q2, q3, g
 *         const double *param: 辅助参数p, q, r, ax, ay, az
 *               double *fxu  : 状态方程输出q0, q1, q2, q3, g
 *
 *功    能：计算位置和速度的状态方程
 *
 *返 回 值:
 *          成功: OK
 *          输入输出指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int QGFxu(const double *state, const double *param, double *fxu)
{    
   double q0, q1, q2, q3;        // 四元数

#ifdef __DEBUG__
    if ( (state == NULL) || (param == NULL) || (fxu == NULL) )
    {
        return ERR_MEM_NONE;        // 输入输出指针为空
    }
#endif

    
    
    q0 = state[0];
    q1 = state[1];
    q2 = state[2];
    q3 = state[3];
    // 式(3-67)第三部分
    fxu[0] = 0.5 * ( -param[0] * q1 - param[1] * q2 - param[2] * q3 );
    fxu[1] = 0.5 * (  param[0] * q0 + param[2] * q2 - param[1] * q3 );
    fxu[2] = 0.5 * (  param[1] * q0 - param[2] * q1 + param[0] * q3 );
    fxu[3] = 0.5 * (  param[2] * q0 + param[1] * q1 - param[0] * q2 );

    // 如果fx[0]~fx[3]全部为零
    if ( (fxu[0] < 1e-6 && fxu[0] > -1e-6) && (fxu[1] < 1e-6 && fxu[1] > -1e-6)
      && (fxu[2] < 1e-6 && fxu[2] > -1e-6) && (fxu[3] < 1e-6 && fxu[3] > -1e-6) )
    {
       fxu[0] = state[0];
       fxu[1] = state[1];
       fxu[2] = state[2];
       fxu[3] = state[3];
    }

    //Norm(fxu);        // 不对 去掉 by syc 2011.11.14

    fxu[4] = 0.0;    // 式(3-67)第四部分

    return OK;
}

/****************************************************************
 *函 数 名: QGFxDot()
 *参    数:
 *         const double *state: 输入状态q0, q1, q2, q3, g
 *         const double *param: 辅助参数p, q, r, ax, ay, az
 *               void   *fxDot: 对fx求导计算后的5 * 5矩阵
 *
 *功    能：四元数和重力加速度模型的状态方程求导
 *
 *返 回 值:
 *          成功: OK
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int QGFxDot(const double *state, const double *param, void *fxDot)
{
    // 同PosVelFxDot一样，这里不进行参数有效性检查
    double p  = param[0];
    double q  = param[1];
    double r  = param[2];

    double *pfxDot = (double*)fxDot;
    int i, j;

    // 式(3-73)中对q0~q3, g求导部分
    double fq0[LEN_QG_STATE] = {      0,
                                  0.5 * p,
                                  0.5 * q,
                                  0.5 * r,
                                      0     };

    double fq1[LEN_QG_STATE] = { -0.5 * p,
                                      0,
                                 -0.5 * r,
                                  0.5 * q,
                                      0     };

    double fq2[LEN_QG_STATE] = { -0.5 * q,
                                  0.5 * r,
                                      0,
                                 -0.5 * p,
                                      0     }; 

    double fq3[LEN_QG_STATE] = { -0.5 * r,
                                 -0.5 * q,
                                  0.5 * p,
                                      0,
                                      0     };

    int lenFq = 4;        // fq0~fq3一共有4个
    // pfq为 [fq0, fq1, fq2, fq3]'组成的矩阵，
    // 注意这里的fq0~fq3为行向量，所以这里若将pfq转置
    // 就和式(3-73)中的表达式一致了
    double *pfq[4] = { fq0, fq1, fq2, fq3 };

    // 式(3-73)
    for (j=0; j<lenFq; j++)
    {
        for (i=0; i<LEN_QG_STATE; i++)
        {
            pfxDot[i*LEN_QG_STATE+j] = pfq[j][i];
        }
    }//for

    // 式(3-74)
    for (i=0; i<LEN_QG_STATE; i++)
    {
        pfxDot[i*LEN_QG_STATE+lenFq] = 0.0;
    }

    return OK;
}

/****************************************************************
 *函 数 名: QGHx()
 *参    数:
 *         const double *state: 输入状态q0, q1, q2, q3, g
 *               void      *hx: 对hx计算后的3个状态: phi, theta, psi
 *
 *功    能：求四元数和重力加速度模型的观测方程
 *
 *返 回 值:
 *          成功: OK
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int QGHx(const double *state, double *hx)
{
    // 同PosVelFxDot一样，这里不进行参数有效性检查
    double q0 = state[0];
    double q1 = state[1];
    double q2 = state[2];
    double q3 = state[3];

    double angular[] = { atan2( 2 * (q2 * q3 + q0 * q1) , (1 - 2 * (q1 * q1 + q2 * q2)) ),   // phi
                        -asin( 2 * (q1 * q3 - q0 * q2) ),                                   // theta
                         atan2( 2 * (q1 * q2 + q0 * q3) , (1 - 2 * (q2 * q2 + q3 * q3)) ) }; // psi
    
    int lenAng = 3;        // 角度个数有3个
    int i;
    for (i=0; i<lenAng; i++)
    {
        hx[i] = angular[i];    // 三个角度
    }

    return OK;
}

/****************************************************************
 *函 数 名: QGHxDot()
 *参    数:
 *         const double *state: 输入状态q0, q1, q2, q3, g
 *               void   *hxDot: 观测方程对位置速度变量的求导，为3 * 5阶单位矩阵
 *
 *功    能：四元数、重力加速度观测方程对位置速度变量的求导
 *
 *返 回 值:
 *          成功: OK
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
int QGHxDot(const double *state, void *hxDot)
{
    // 同PosVelFxDot一样，这里不进行参数有效性检查

    double q0 = state[0];
    double q1 = state[1];
    double q2 = state[2];
    double q3 = state[3];
    double DCM[3][3];
    double C[3][4];
    double err[3];
    
    DCM[0][0] = 1 - 2 * ( q2*q2 + q3*q3 );
    DCM[0][1] = 2 * ( q1*q2 + q0*q3 );
    DCM[0][2] = 2 * ( q1*q3 - q0*q2 );
                                            
    //DCM[1][0] = 2 * ( q1*q2 - q0*q3 );
    //DCM[1][1] = 1 - 2 * ( q1*q1 + q3*q3 );
    DCM[1][2] = 2 * ( q2*q3 + q0*q1 );
                                                                    
    //DCM[2][0] = 2 * ( q1*q3 + q0*q2 );
    //DCM[2][1] = 2 * ( q2*q3- q0*q1 );
    DCM[2][2] = 1 - 2 * ( q1*q1 + q2*q2 );
            
    err[0] = 2.0 / ( (DCM[2][2]*DCM[2][2]) + (DCM[1][2]*DCM[1][2]) );
    err[1] = 1.0 / sqrt(1.0 - (DCM[0][2]*DCM[0][2]) );
    err[2] = 2.0 / ((DCM[0][0]*DCM[0][0]) + (DCM[0][1]*DCM[0][1]));
    
    C[0][0] = err[0]*( q1 * DCM[2][2] );
    C[1][0] = err[1]*2.0 * q2;
    C[2][0] = err[2]*( q3 * DCM[0][0] );
    
    C[0][1] = err[0]*( q0 * DCM[2][2] + 2.0 * q1 * DCM[1][2] );
    C[1][1] = err[1]*(-2.0)*q3;
    C[2][1] = err[2]*( q2 * DCM[0][0] );
    
    C[0][2] = err[0]*( q3 * DCM[2][2] + 2.0 * q2 * DCM[1][2] );
    C[1][2] = err[1]*2.0 * q0;
    C[2][2] = err[2]*( q1 * DCM[0][0] + 2.0 * q2 * DCM[0][1] );
     
    C[0][3] = err[0]*( q2 * DCM[2][2]);
    C[1][3] = err[1]*(-2.0) * q1; 
    C[2][3] = err[2]*( q0 * DCM[0][0] + 2.0 * q3 * DCM[0][1] );
        
    // dh/dq0式(3-83)
    double  HQ0[LEN_QG_OBS] = {C[0][0],C[1][0],C[2][0]};
    // dh/dq1式(3-84)
    double  HQ1[LEN_QG_OBS] = {C[0][1],C[1][1],C[2][1]};
    // dh/dq2式(3-85)
    double  HQ2[LEN_QG_OBS] = {C[0][2],C[1][2],C[2][2]};  
    // dh/dq3式(3-86)
    double  HQ3[LEN_QG_OBS] = {C[0][3],C[1][3],C[2][3]};  

    int i, j;
    double *phxDot = (double*)hxDot;
    int lenHQ = 4;        // HQ0~HQ3个数有4个
    // pHQ为 [HQ0, HQ1, HQ2, HQ3]'组成的矩阵，
    // 注意这里的HQ0~HQ3为行向量，所以这里若将pHQ转置
    // 就和式(3-83)~(3-86)中的表达式一致了
    double *pHQ[4] = { HQ0, HQ1, HQ2, HQ3 };

    for (j=0; j<lenHQ; j++)        // 式3-83到3-86
    {
        for (i=0; i<LEN_QG_OBS; i++)        // 按列赋值
        {
            phxDot[i*LEN_QG_STATE+j] = pHQ[j][i];
        }
    }//for

    // hx对g求导
    for (i=0; i<LEN_QG_OBS; i++)
    {
        phxDot[i*LEN_QG_STATE+lenHQ] = 0.0;
    }

    return OK;
}

/****************************************************************
 *函 数 名: QGEKF()
 *参    数:
 *         state: 滤波前状态q0, q1, q2, q3, g
 *         obs  : 观测量 phi, theta, psi
 *         param: 需要的参数 p, q, r, ax, ay, az
 *         est  : 滤波后的状态 q0, q1, q2, q3, g
 *
 *功    能：对四元数、重力加速度进行EKF滤波
 *
 *返 回 值:
 *          成功: OK
 *          输入输出指针为空：ERR_MEM_NONE
 *
 *作    者：申文斌
 *完成日期：2011-3-22
 ****************************************************************/
void QGEKF(const double *state, const double *obs, const double *param, double *est)
{   
    //double Q[LEN_QG_STATE] = { 0.005,0.01,0.01,0.001,0.03};    // Q阵
    //double R[LEN_QG_OBS] = { 1,1,2}; //2011 12 14
   
    //double Q[LEN_QG_STATE] = { 0.0008,0.0008,0.0008,0.0008,0.03};    // Q阵 12.9last
    double Q[LEN_QG_STATE] = { 0.001,0.001,0.001,0.001,0.03};
    double R[LEN_QG_OBS] = {1,1,0.1};
    // Pk参数初始化
    static double s_Pk[LEN_QG_STATE][LEN_QG_STATE] = { {0.01}, {0.0, 0.01}, {0.0, 0.0, 0.01},{0.0, 0.0, 0.0, 0.01},{0.0, 0.0, 0.0,0.0,0.01} };
    static double s_preEst[LEN_QG_STATE];    // 上次滤波值
    static int s_isFirst = 1;                // 第一次滤波

    struct Model model;                      // 系统模型
    struct ModelParam mParam;                // 状态量、观测量及估计值保存地址
    struct EKFParam ekfParam;                // EKF滤波所需要的参数

    //int i = 0;                               // 循环变量
    //double err;                              // 滤波值与原值的偏差
    //double INF = 2e-3;                       // 一个很小的值

    // 设置系统模型
    model.fxu = QGFxu;                // 状态方程
    model.fxDot = QGFxDot;            // 状态方程求导
    model.hx = QGHx;                  // 观测方程
    model.hxDot = QGHxDot;            // 观测方程求导

    // 设置状态量、观测量及估计值保存地址
    mParam.state = state;        // 本次滤波的状态值
    mParam.obs = obs;            // 观测量
    mParam.est = est;            // 估计值保存地址
    mParam.othParam = param;     // 滤波需要的其它参数
    mParam.lenState = LEN_QG_STATE;    // 状态量长度
    mParam.lenObs = LEN_QG_OBS;        // 观测量长度

    // 设置EKF滤波所需要的参数
    ekfParam.Q = Q;                    // Q阵
    ekfParam.R = R;                    // R阵
    ekfParam.Pk = (double*)s_Pk;       // Pk参数
    ekfParam.preEst = s_preEst;        // 上一次的估计值
    ekfParam.isFirst = &s_isFirst;     // 是第一次执行滤波吗

    DoEKF(&model, &mParam, &ekfParam);    // EKF滤波

    // 如果q0~q3的滤波值与原值差的太多就用原值
    /*for ( i=0; i<4; i++)
    {
        err = est[i] - state[i];
        if ( (err > INF) || (err < -INF) )
        {
            est[i] = state[i];
            s_preEst[i] = state[i];
        }
    
    }*/  //by  syc去掉
   
}
