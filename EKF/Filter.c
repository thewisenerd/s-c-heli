/****************************************************************
 *�� �� ��: Filter.c
 *��    ��:
 *          ��չKalman�˲�ʵ��
 *          �ļ�ע���й�ʽ���ȫ��Ϊ��ǫ�����еĹ�ʽ���
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
    static float window_vel_z[8] = {0.0} ;  // z���ٶ�ƽ��ֵ�˲�
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
   static float window_vel_z[3] = {0.0} ;  // z���ٶ�ƽ��ֵ�˲�
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
	//�˴�Cnb��ʾ��NED����ϵ����������ϵ��ת��������Cbn��ʾ�����ʣ�
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
  state_init();//�õ��۲�ֵ

//  position.raw_current_xyz[2]=bmp_height;
//  
//  static double height_array[6];                //mei2015-4-20
//  static double aux_sum;
//  static int aux_height_num = 3;             //mei2015-4-20 Ϊ6/2
//  static int height_num = 0;                    //mei2015-4-20  
//  aux_sum += height_array[height_num];          //mei2015-4-20 ������ɵ����ݣ��൱�������������
//  height_array[height_num] = bmp_height;        //mei2015-4-20
//  aux_sum += height_array[height_num];          //mei2015-4-20 �������µ����ݣ�������������
//  aux_sum -= height_array[aux_height_num];      
//  aux_sum -= height_array[aux_height_num];      //mei2015-4-20 ��ȥ�������м���������൱���ɼӱ�ɼ�
//  position.raw_current_uvw[2] = aux_sum*30/9; //mei2015-4-20//9Ϊ3^3
//  height_num++;                                     //mei2015-4-20
//  aux_height_num++;                                 //mei2015-4-20
//  if (height_num == 6) height_num = 0;              //mei2015-4-20
//  if (aux_height_num == 6) aux_height_num = 0;      //mei2015-4-20
  
//  static double height_array[4];        //mei2015-4-20
//  static double bmp_height_0;            //mei2015-4-20
//  static int height_num = 0;            //mei2015-4-20  
//  bmp_height_0 = height_array[height_num];          //mei2015-4-20ע��bmp_height_0��height_array[4]�ܹ�5����
//  height_array[height_num] = bmp_height;            //mei2015-4-20
//  position.raw_current_uvw[2] = (height_array[height_num] - bmp_height_0)*30/4;//mei2015-4-20
//  height_num++;                                     //mei2015-4-20
//  if (height_num == 4) height_num = 0;              //mei2015-4-20
  
//mei2015-4-20  position.raw_current_uvw[2] = (position.raw_current_xyz[2] - last_bmp_height)*30;
//  last_bmp_height = position.raw_current_xyz[2];
  //position.raw_current_uvw[2]=uvw2_update();
  
  //ע�⣺kalman�˲���Ҫ���ٶ��������ֵΪ��������ϵ�µ�ֵ---��EKF����ת���ɻ�����ֵ//
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
//  input_uvw[i-3] = est_Kalman_State[i];//�˲���õ���׼��������ϵ�·ɻ��������ٶ�
//  
//  for(int i =6 ; i!=10; ++i) 
//  K_Quat[i-6] = est_Kalman_State[i];
//  
//  quat2euler( ahrs_theta, K_Quat );
//  accel_g = est_Kalman_State[10];
//
//   //�ɻ��ĸ߶��������������ned��λ���ٶ�  position.current_uvwΪ�˲���ned���ٶ�ֵ
//  //position.current_uvw[0]=Cbn[0][0]*input_uvw[0]+Cbn[0][1]*input_uvw[1]+Cbn[0][2]*input_uvw[2];
//  //position.current_uvw[1]=Cbn[1][0]*input_uvw[0]+Cbn[1][1]*input_uvw[1]+Cbn[1][2]*input_uvw[2];
//  position.current_uvw[2]=Cbn[2][0]*input_uvw[0]+Cbn[2][1]*input_uvw[1]+Cbn[2][2]*input_uvw[2];
//  //input_uvw[2] = position.current_uvw[2];
//  input_uvw[2] = uvw2_update(position.current_uvw[2]);
//  //---------------gps����-----------------//
//  offset_x=cos(ahrs_theta[2])*offset_p;
//  offset_y=sin(ahrs_theta[2])*offset_p;
//  position.current_xyz[0]=position.current_xyz[0]+offset_x;
//  position.current_xyz[1]=position.current_xyz[1]+offset_y;  //����ǲ�׼ �������������
}


/****************************************************************
 *�� �� ��: EKF()
 *��    ��:
 *         state: �˲�ǰ״̬x, y, z, u, v, w, q0, q1, q2, q3, g
 *         obs  : �۲���x, y, z, u, v, w, phi, theta, psi
 *         param: ��Ҫ�Ĳ��� p, q, r, ax, ay, az
 *         est  : �˲����״̬x, y, z, u, v, w, q0, q1, q2, q3, g
 *
 *��    �ܣ���λ�á��ٶȡ���Ԫ�����������ٶȽ���EKF�˲�
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          �������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
void EKF(const double *state, const double *obs, const double *param, double *est)
{
    // λ���ٶ��˲���ʼֵ
    double pvState[LEN_PV_STATE];    // �˲�ǰ״̬ x, y, z, u, v, w
    double pvObs[LEN_PV_OBS];        // �۲���x, y, z, u, v, w

    // ��Ԫ�������ٶ��˲���ʼֵ
    double qgState[LEN_QG_STATE];    // �˲�ǰ״̬q0, q1, q2, q3, g
    double qgObs[LEN_QG_OBS];        // �۲���phi, theta, psi

    // �˲���Ҫ�ĸ�������p, q, r, ax, ay, az, q0, q1, q2, q3��10��
    // ����Ԫ�������ٶ��˲�ʱ�õ�ǰ6��
    // ��λ�á��ٶ��˲�ʱȫ���õ�
    double othParam[10];
    int    qgLenParam = 6;
    int    pvLenParam = 10;
    int i, j;                    // ѭ������
    // ����Ԫ�������ٶ��˲�������ʼ��
    for (i=0; i<LEN_QG_STATE; i++)
    {
        qgState[i] = state[i+ LEN_PV_STATE];        // ״̬��
    }
    for (i=0; i<LEN_QG_OBS; i++)
    {
        qgObs[i] = obs[i+ LEN_PV_OBS];        // �۲���
    }
    for (i=0; i<qgLenParam; i++)
    {
        othParam[i] = param[i];               // ��������
    }
    // ����Ԫ�������ٶ��˲����˲�����Ԫ�صĴ洢�׵�ַΪest+LEN_PV_STATE
    QGEKF(qgState, qgObs, othParam, est+LEN_PV_STATE);
    
	// ���ٶ�λ���˲�������ʼ��	
	j = LEN_PV_STATE;
    for (i=qgLenParam; i<pvLenParam; i++)
    {
        othParam[i] = est[j++];       // ���˲������Ԫ����othParam
    }   
    for (i=0; i<LEN_PV_STATE; i++)
    {
        pvState[i] = state[i];        // ״̬��
        // �۲���,����״̬����ͬ�����Կ�����ͬһ��ѭ���������
        pvObs[i] = obs[i];
    }	
	//--------------------------------------syc-------------------------------//
	//��NED�µ��ٶ�ת��Ϊ���������µ��ٶ�  ��Ϊkalman������ֵ
	Cnb_update(est+LEN_PV_STATE);      
	pvObs[3]=Cnb[0][0]*obs[3]+Cnb[0][1]*obs[4]+Cnb[0][2]*obs[5];
    pvObs[4]=Cnb[1][0]*obs[3]+Cnb[1][1]*obs[4]+Cnb[1][2]*obs[5];
    pvObs[5]=Cnb[2][0]*obs[3]+Cnb[2][1]*obs[4]+Cnb[2][2]*obs[5];
    pvState[3]= pvObs[3];
    pvState[4]= pvObs[4];
    pvState[5]= pvObs[5];
    // ��λ�á��ٶ��˲����˲���Ĵ洢�׵�ַΪest
    PosVelEKF(pvState, pvObs, othParam, est);
}

/****************************************************************
 *�� �� ��: DoEKF()
 *��    ��:
 *               pModel model: ��Ҫ����EKF�˲���ģ��
 *         pModelParam mParam: ״̬�����۲���������ֵ�����ַ
 *         pEKFParam ekfParam: KF�˲�����Ҫ�Ĳ���
 *
 *��    �ܣ���model��ʾ��ģ�ͽ���EKF�˲�
 *          ������Pkk-1��PkΪ�Գƾ��󣬵����������ʱ���㷨��
 *          ��ͨ����ĳ˷���һ����
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          �������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int DoEKF(pModel model, pModelParam mParam, pEKFParam ekfParam)
{
    const double T = 0.033;      // ��������
    int i, j, k;                // ѭ������

    // ����Ϊ�˲�������Ҫ�õ���ʱ����
    // ����ͳһ��һά�����ʾ����,Ϊ�˸��õ�ַ�����ﰴ������󿪱�
    // ������û�в���ϵͳ��ARM7�ϲ����Զ�̬���ٿռ䣬��������ֻ����
    // ���ֱȽϱ��ķ����ˡ�
    double fx[MAX_STATE];    // ����״̬���̼���ֵ
    double Xhatkk_1[MAX_STATE];
    double Hk[MAX_OBS * MAX_STATE];            // ����۲����Hk
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
        return ERR_MEM_NONE;        // �������ָ��Ϊ��
    }
#endif

    if (*ekfParam->isFirst == 1)        // ��һ��ִ���˲�
    {
        
        *ekfParam->isFirst = 0;
        // �������״ֱ̬�ӵ�������ֵ
        for (i=0; i<mParam->lenState; i++)
        {
            ekfParam->preEst[i] = mParam->state[i];
            mParam->est[i] = mParam->state[i];
        }
        return OK;
    }//if

// EKF�˲�����
   
    
    // ��һ���˲�Xhatkk_1 = Xhatk_1 + T * fxu(Xhatk_1) ʽ(3-60)
    model->fxu(ekfParam->preEst, mParam->othParam, fx);
    for (i=0; i<mParam->lenState; i++)
    {
        fx[i] *= T;
    }
    AddMatrix(ekfParam->preEst, fx, Xhatkk_1,  mParam->lenState, 1);

    if ( model->fxu == QGFxu)      // ���״̬����Ϊ��Ԫ��������Ҫ��һQ
    {
        Norm(Xhatkk_1);       // ��һQ
    }

    // ��ù۲�����״̬���󵼣�Hk���õĴ�СΪmParam->lenObs * mParam->lenState
    model->hxDot(Xhatkk_1, Hk); 

    // ״̬ת�ƾ���,��ɢ�����¾�״̬ת����ǰ����Ҫ����T
    // Phikk_1���õĴ�СΪmParam->lenState * mParam->lenState
    model->fxDot(ekfParam->preEst, mParam->othParam, Phikk_1);
    for (i=0; i<mParam->lenState; i++)
    {
        for (j=0; j<mParam->lenState; j++)
        {
            Phikk_1[i * mParam->lenState + j] *= T;
        }
    }

    // Ԥ���������Pkk_1
    // ����Pkk_1�����￼��Pkk_1Ϊ�Գƾ���
    // �м����tmp�õ��Ĵ�СΪmParam->lenState * mParam->lenState
    MulMatrix(Phikk_1, ekfParam->Pk, tmp,
              mParam->lenState, mParam->lenState, mParam->lenState);
    // �������������Ϊ�Գƾ������Կ��Լ�
    // ��ʼ��Pkk_1,���õ��Ĵ�СΪmParam->lenState * mParam->lenState
    for (i=0; i<mParam->lenState; i++)
    {
        for (j=0; j<=i; j++)
        {
            Pkk_1[i * mParam->lenState + j] = 0.0;    // ��������������ǳ�ʼ��Ϊ0
        }
    }
    for (i=0; i<mParam->lenState; i++)        // �������
    {
        for (j=0; j<=i; j++)
        {
            for (k=0; k<mParam->lenState; k++)
            {
                // �������Phikk_1��ת��
                Pkk_1[i * mParam->lenState + j] += 
                    tmp[i * mParam->lenState + k] *  Phikk_1[j * mParam->lenState + k];
            }
            Pkk_1[j * mParam->lenState + i] = Pkk_1[i * mParam->lenState + j];        // �ԳƵ�Ԫ��
        }
    }

    for (i=0; i<mParam->lenState; i++)
    {
        // ����Q������ֻҪ���϶Խ����ϵ�Ԫ�ؼ���
        Pkk_1[i * mParam->lenState + i] += ekfParam->Q[i];
    }

    // ��ȡ�˲�������� ʽ(3-62)
    TranMatrix(Hk, TranHk, mParam->lenObs, mParam->lenState);
    // PxHk���õ��Ĵ�СΪ mParam->lenState * mParam->lenObs
    MulMatrix(Pkk_1, TranHk, PxHk, mParam->lenState, mParam->lenState, mParam->lenObs);
    // KkTmp���õ��Ĵ�СΪmParam->lenObs * mParam->lenObs
    MulMatrix(Hk, PxHk, KkTmp, mParam->lenObs, mParam->lenState, mParam->lenObs);

    // Hk*Pkk_1*Hk' + R�����ǵ�RΪ�ԽǾ������Բ��ð�һ����˼���
    // ֻ��Ҫ�ӶԽ���Ԫ�ؼ���
    for (i=0; i<mParam->lenObs; i++)
    {
        KkTmp[i * mParam->lenObs + i] += ekfParam->R[i];
    }

    // Hk*Pkk_1*Hk' + R �Գƾ���
    if ( InvSymMtrx(KkTmp, KkTmp, mParam->lenObs) != OK )
    {
        // ���Hk*Pkk_1*Hk' + R�����޷�����
        // �򽫴����״ֱ̬�ӵ�������ֵ
        for (i=0; i<mParam->lenState; i++)
        {
            ekfParam->preEst[i] = mParam->state[i];
            mParam->est[i] = mParam->state[i];
        }
        return OK;
    }

    // Kk���õ��Ĵ�СΪmParam->lenState * mParam->lenObs
    MulMatrix(PxHk, KkTmp, Kk, mParam->lenState, mParam->lenObs, mParam->lenObs);

    // ״̬����
    model->hx(Xhatkk_1, h);
    SubMatrix( mParam->obs, h, h, mParam->lenObs, 1);        // h = obs - hx(Xhatkk_1)
     //----------------------- syc 2011.11.14------------------------------//
   if ( model->fxu == QGFxu)     // ���״̬����Ϊ��Ԫ��������Ҫ�����Ƕ�
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
    // ekfParam->preEst�Ǳ�����õ��˲�ֵ
    AddMatrix(ekfParam->preEst, Xhatkk_1, ekfParam->preEst, mParam->lenState, 1);

    if ( model->fxu == QGFxu)     // ���״̬����Ϊ��Ԫ��������Ҫ��һQ
    {
        Norm(ekfParam->preEst);   // ��һQ
    }

    // �˲��������
    // Kk���õ��Ĵ�СΪmParam->lenState * mParam->lenState
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
            ekfParam->Pk[i*mParam->lenState + j] = 0.0;    // Pk�����Ǿ���Ϊ��
        }
    }

    for (i=0; i<mParam->lenState; i++)        // �������
    {
        for (j=0; j<=i; j++)
        {
            for (k=0; k<mParam->lenState; k++)
            {
                ekfParam->Pk[i*mParam->lenState + j] +=
                    KkxHk[i * mParam->lenState + k] *  Pkk_1[k * mParam->lenState + j];
            }
            // �ԳƵ�Ԫ��
            ekfParam->Pk[j*mParam->lenState + i] = ekfParam->Pk[i*mParam->lenState + j];
        }
    }

    // ��������Ƶ�Ŀ�ĵ�ַ
    for (i=0; i<mParam->lenState; i++)
    {
        mParam->est[i] = ekfParam->preEst[i];
    }

    return OK;
}

/****************************************************************
 *�� �� ��: Norm()
 *��    ��:
 *         double * q: ��Ҫ��һ��Ԫ�ص��׵�ַ
 *
 *��    �ܣ���q���й�һ����
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          ����ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int Norm(double *q)
{
    const int N = 4;            // Ԫ�ظ���
    double sum = 0.0;           // ��������һ��Ԫ�ص�ƽ���ͣ�Ȼ����͵�ƽ����
    int i;

#ifdef __DEBUG__
    if (q == NULL)
    {
        return ERR_MEM_NONE;  // ����ָ��Ϊ��
    }
#endif

    for (i=0; i<N; i++)
    {
        sum += q[i] * q[i];
    }
    sum = sqrt(sum);           // sum = sqrt(...)

    for (i=0; i<N; i++)
    {
         q[i] /= sum ;         // ��һ���Ԫ��
    }

    return OK;
}

/****************************************************************
 *�� �� ��: PosVelFxu()
 *��    ��:
 *         const double *state: ����״̬x, y, z, u, v, w
 *         const double *param: ��������p, q, r, ax, ay, az, q0, q1, q2, q3
 *               double *fxu  : ״̬�������x, y, z, u, v, w
 *
 *��    �ܣ�����λ�ú��ٶȵ�״̬����
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          �������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int PosVelFxu(const double *state, const double *param, double *fxu)
{
    double u, v, w;             // ����������ٶ�
    double q0, q1, q2, q3;      // ��Ԫ��
    double g;                   // �������ٶ�
    double p, q, r;             // ��������������Ǽ��ٶ�

#ifdef __DEBUG__
    if ( (state == NULL) || (param == NULL) || (fxu == NULL) )
    {
        return ERR_MEM_NONE;        // �������ָ��Ϊ��
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

    // param[3~5]�ֱ��ʾ: ax, ay, az
    //g = sqrt(param[3] * param[3] + param[4] * param[4] + param[5] * param[5]);//4.16
    g=accel_g; 
    // ʽ(3-67)�ĵ�һ������
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
 *�� �� ��: PosVelFxDot()
 *��    ��:
 *         const double *state: ����״̬x, y, z, u, v, w
 *         const double *param: ��������p, q, r, ax, ay, az, q0, q1, q2, q3
 *               void   *fxDot: ��fx�󵼼�����6 * 6����
 *
 *��    �ܣ���λ�ú��ٶȵ�״̬������
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int PosVelFxDot(const double *state, const double *param, void *fxDot)
{
    // ����ԭ��Ӧ�ö������������Ч�Խ��м���
    // ������C���������еı��������������ִ�����ǰ��
    // ����涨����������帳ֵֻ����������ʱ��ſ���
    // ������������������������ټ�������������Ч�ԣ�
    // �Ͳ����Զ��������帳ֵ�ˣ��Ӷ��ƻ��˹�ʽ��������
    // ��Ȼ��Щ�������汾֧���õ�ʱ�������������������Ϊ��
    // ��ֲ�ԣ�����ͳһ�������ŵ���ǰ�档
    double p  = param[0];
    double q  = param[1];
    double r  = param[2];

    //double q0 = param[6];
    //double q1 = param[7];
    //double q2 = param[8];
    //double q3 = param[9];

    double *pfxDot = (double *)fxDot;
    int i, j;

    // λ���ٶ�״̬���̶��ٶ��󵼣�ʽ(3-72)��һ������
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
    double *pfuvw = fuvw[0];    // ȡ��fuvw���׵�ַ
    
    // λ���ٶ�״̬���̶�λ����ȫΪ�㣬ʽ3-71
    // �����������ŵ����������
    // �󵼺��λ���󵼽���ڵ�0��2��
    for (i=0; i<LEN_PV_STATE; i++)
    {
        for (j=0; j<3; j++)        // ����3�������λ��
        {
            pfxDot[i*LEN_PV_STATE+j] = 0.0;
        }
    }

    // ����3��������ٶȣ��󵼺���ٶ��󵼽���ڵ�3��5��
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
 *�� �� ��: PosVelHx()
 *��    ��:
 *         const double *state: ����״̬x, y, z, u, v, w
 *               double *fxu  : �۲ⷽ�����x, y, z, u, v, w
 *
 *��    �ܣ�����λ�ú��ٶȵĹ۲ⷽ��
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          �������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int PosVelHx(const double *state, double *hx)
{
    int i;        // ѭ������
     
     
#ifdef __DEBUG__
    if ( (state == NULL) || (hx == NULL) )
    {
        return ERR_MEM_NONE;        // �������ָ��Ϊ��
    }
#endif

    // ʽ(3-78)��һ�����֣��۲�������״̬��
    for (i=0; i<LEN_PV_OBS; i++)
    {
        *hx++ = *state++;  
    }
 
  return OK;
}

/****************************************************************
 *�� �� ��: PosVelHxDot()
 *��    ��:
 *         const double *state: ����״̬x, y, z, u, v, w
 *               void   *hxDot: �۲ⷽ�̶�λ���ٶȱ������󵼣�Ϊ6 * 6�׵�λ����
 *
 *��    �ܣ�λ���ٶȹ۲ⷽ�̶�λ���ٶȱ�������
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          �������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int PosVelHxDot(const double *state, void *hxDot)
{
    double *phxDot = (double *)hxDot;
    int i, j;                // ѭ������

#ifdef __DEBUG__
    if ( (state == NULL) || (hxDot == NULL) )
    {
        return ERR_MEM_NONE;        // �������ָ��Ϊ��
    }
#endif

    for (i=0; i<LEN_PV_STATE; i++)        // ʽ3-79 3-80
    {
        for (j=0; j<LEN_PV_OBS; j++)
        {
            // �Խ���Ԫ��Ϊ1������Ϊ��
            phxDot[i*LEN_PV_STATE+j] = (i == j);
        }
    }//for

    return OK;
}

/****************************************************************
 *�� �� ��: PosVelEKF()
 *��    ��:
 *         state: �˲�ǰ״̬x, y, z, u, v, w
 *         obs  : �۲���x, y, z, u, v, w
 *         param: ��Ҫ�Ĳ���p, q, r, ax, ay, az, q0, q1, q2, q3
 *         est  : �˲����״̬x, y, z, u, v, w
 *
 *��    �ܣ�λ���ٶȱ�����EKF�˲�
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          �������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
void PosVelEKF(const double *state, const double *obs, const double *param, double *est)
{
    //const double Q[LEN_PV_STATE] = {0.08,0.08,0.08,0.5,0.5,0.05};    // Q��
    //const double R[LEN_PV_OBS] =   {0.3,0.3,0.3,0.3,0.3,0.3};        // R��
    
    //const double Q[LEN_PV_STATE] = {0.08,0.08,0.08,0.08,0.08,0.05};    // Q��
    //const double R[LEN_PV_OBS] =   {0.3,0.3,0.3,0.3,0.3,0.3};        // R��
	
	const double Q[LEN_PV_STATE] = {0.1,0.1,0.15,0.1,0.1,0.03};
    const double R[LEN_PV_OBS] =   {0.3,0.3,0.3,0.3,0.3,0.3};
    
    //const double Q[LEN_PV_STATE] = {0.07,0.007,0.07,0.007,0.005,0.2};    // Q��
    //const double R[LEN_PV_OBS] =   {0.2,0.02,0.12,0.02,0.01,1.8};    // ��ͣ�� 
    //const double Q[LEN_PV_STATE] = { 0.01, 0.03, 0.03, 0.03, 0.03, 0.03 };    // Q��
    //const double R[LEN_PV_OBS] =   { 0.01, 0.08, 0.08, 0.3, 0.3, 0.3 };  //��������  
    //Pk������ʼ��
    static double s_Pk[LEN_PV_STATE][LEN_PV_STATE] = {{0.01}, {0.0, 0.01}, {0.0, 0.0, 0.01}, {0.0, 0.0, 0.0, 0.01},
                       {0.0, 0.0, 0.0, 0.0, 0.01}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.01} };
    static double s_preEst[LEN_PV_STATE];    // �ϴ��˲�ֵ
    static int    s_isFirst = 1;             // ��һ���˲���־

    struct Model model;                      // ϵͳģ��
    struct ModelParam mParam;                // ״̬�����۲���������ֵ�����ַ
    struct EKFParam ekfParam;                // EKF�˲�����Ҫ�Ĳ���

    // ����ϵͳģ��
    model.fxu = PosVelFxu;              // ״̬����
    model.fxDot = PosVelFxDot;          // ״̬������
    model.hx = PosVelHx;                // �۲ⷽ��
    model.hxDot = PosVelHxDot;          // �۲ⷽ����

    // ����״̬�����۲���������ֵ�����ַ
    mParam.state = state;        // �����˲���״ֵ̬
    mParam.obs = obs;            // �۲���
    mParam.est = est;            // ����ֵ�����ַ
    mParam.othParam = param;     // �˲���Ҫ����������
    mParam.lenState = LEN_PV_STATE;    // ״̬������
    mParam.lenObs = LEN_PV_OBS;        // �۲�������

    // ����EKF�˲�����Ҫ�Ĳ���
    ekfParam.Q = Q;                    // Q��
    ekfParam.R = R;                    // R��
    ekfParam.Pk = (double*)s_Pk;       // Pk����
    ekfParam.preEst = s_preEst;        // ��һ�εĹ���ֵ
    ekfParam.isFirst = &s_isFirst;     // �ǵ�һ��ִ���˲���

    DoEKF(&model, &mParam, &ekfParam);    // EKF�˲�

}

/****************************************************************
 *�� �� ��: QGFxu()
 *��    ��:
 *         const double *state: ����״̬q0, q1, q2, q3, g
 *         const double *param: ��������p, q, r, ax, ay, az
 *               double *fxu  : ״̬�������q0, q1, q2, q3, g
 *
 *��    �ܣ�����λ�ú��ٶȵ�״̬����
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          �������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int QGFxu(const double *state, const double *param, double *fxu)
{    
   double q0, q1, q2, q3;        // ��Ԫ��

#ifdef __DEBUG__
    if ( (state == NULL) || (param == NULL) || (fxu == NULL) )
    {
        return ERR_MEM_NONE;        // �������ָ��Ϊ��
    }
#endif

    
    
    q0 = state[0];
    q1 = state[1];
    q2 = state[2];
    q3 = state[3];
    // ʽ(3-67)��������
    fxu[0] = 0.5 * ( -param[0] * q1 - param[1] * q2 - param[2] * q3 );
    fxu[1] = 0.5 * (  param[0] * q0 + param[2] * q2 - param[1] * q3 );
    fxu[2] = 0.5 * (  param[1] * q0 - param[2] * q1 + param[0] * q3 );
    fxu[3] = 0.5 * (  param[2] * q0 + param[1] * q1 - param[0] * q2 );

    // ���fx[0]~fx[3]ȫ��Ϊ��
    if ( (fxu[0] < 1e-6 && fxu[0] > -1e-6) && (fxu[1] < 1e-6 && fxu[1] > -1e-6)
      && (fxu[2] < 1e-6 && fxu[2] > -1e-6) && (fxu[3] < 1e-6 && fxu[3] > -1e-6) )
    {
       fxu[0] = state[0];
       fxu[1] = state[1];
       fxu[2] = state[2];
       fxu[3] = state[3];
    }

    //Norm(fxu);        // ���� ȥ�� by syc 2011.11.14

    fxu[4] = 0.0;    // ʽ(3-67)���Ĳ���

    return OK;
}

/****************************************************************
 *�� �� ��: QGFxDot()
 *��    ��:
 *         const double *state: ����״̬q0, q1, q2, q3, g
 *         const double *param: ��������p, q, r, ax, ay, az
 *               void   *fxDot: ��fx�󵼼�����5 * 5����
 *
 *��    �ܣ���Ԫ�����������ٶ�ģ�͵�״̬������
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int QGFxDot(const double *state, const double *param, void *fxDot)
{
    // ͬPosVelFxDotһ�������ﲻ���в�����Ч�Լ��
    double p  = param[0];
    double q  = param[1];
    double r  = param[2];

    double *pfxDot = (double*)fxDot;
    int i, j;

    // ʽ(3-73)�ж�q0~q3, g�󵼲���
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

    int lenFq = 4;        // fq0~fq3һ����4��
    // pfqΪ [fq0, fq1, fq2, fq3]'��ɵľ���
    // ע�������fq0~fq3Ϊ��������������������pfqת��
    // �ͺ�ʽ(3-73)�еı��ʽһ����
    double *pfq[4] = { fq0, fq1, fq2, fq3 };

    // ʽ(3-73)
    for (j=0; j<lenFq; j++)
    {
        for (i=0; i<LEN_QG_STATE; i++)
        {
            pfxDot[i*LEN_QG_STATE+j] = pfq[j][i];
        }
    }//for

    // ʽ(3-74)
    for (i=0; i<LEN_QG_STATE; i++)
    {
        pfxDot[i*LEN_QG_STATE+lenFq] = 0.0;
    }

    return OK;
}

/****************************************************************
 *�� �� ��: QGHx()
 *��    ��:
 *         const double *state: ����״̬q0, q1, q2, q3, g
 *               void      *hx: ��hx������3��״̬: phi, theta, psi
 *
 *��    �ܣ�����Ԫ�����������ٶ�ģ�͵Ĺ۲ⷽ��
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int QGHx(const double *state, double *hx)
{
    // ͬPosVelFxDotһ�������ﲻ���в�����Ч�Լ��
    double q0 = state[0];
    double q1 = state[1];
    double q2 = state[2];
    double q3 = state[3];

    double angular[] = { atan2( 2 * (q2 * q3 + q0 * q1) , (1 - 2 * (q1 * q1 + q2 * q2)) ),   // phi
                        -asin( 2 * (q1 * q3 - q0 * q2) ),                                   // theta
                         atan2( 2 * (q1 * q2 + q0 * q3) , (1 - 2 * (q2 * q2 + q3 * q3)) ) }; // psi
    
    int lenAng = 3;        // �Ƕȸ�����3��
    int i;
    for (i=0; i<lenAng; i++)
    {
        hx[i] = angular[i];    // �����Ƕ�
    }

    return OK;
}

/****************************************************************
 *�� �� ��: QGHxDot()
 *��    ��:
 *         const double *state: ����״̬q0, q1, q2, q3, g
 *               void   *hxDot: �۲ⷽ�̶�λ���ٶȱ������󵼣�Ϊ3 * 5�׵�λ����
 *
 *��    �ܣ���Ԫ�����������ٶȹ۲ⷽ�̶�λ���ٶȱ�������
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
int QGHxDot(const double *state, void *hxDot)
{
    // ͬPosVelFxDotһ�������ﲻ���в�����Ч�Լ��

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
        
    // dh/dq0ʽ(3-83)
    double  HQ0[LEN_QG_OBS] = {C[0][0],C[1][0],C[2][0]};
    // dh/dq1ʽ(3-84)
    double  HQ1[LEN_QG_OBS] = {C[0][1],C[1][1],C[2][1]};
    // dh/dq2ʽ(3-85)
    double  HQ2[LEN_QG_OBS] = {C[0][2],C[1][2],C[2][2]};  
    // dh/dq3ʽ(3-86)
    double  HQ3[LEN_QG_OBS] = {C[0][3],C[1][3],C[2][3]};  

    int i, j;
    double *phxDot = (double*)hxDot;
    int lenHQ = 4;        // HQ0~HQ3������4��
    // pHQΪ [HQ0, HQ1, HQ2, HQ3]'��ɵľ���
    // ע�������HQ0~HQ3Ϊ��������������������pHQת��
    // �ͺ�ʽ(3-83)~(3-86)�еı��ʽһ����
    double *pHQ[4] = { HQ0, HQ1, HQ2, HQ3 };

    for (j=0; j<lenHQ; j++)        // ʽ3-83��3-86
    {
        for (i=0; i<LEN_QG_OBS; i++)        // ���и�ֵ
        {
            phxDot[i*LEN_QG_STATE+j] = pHQ[j][i];
        }
    }//for

    // hx��g��
    for (i=0; i<LEN_QG_OBS; i++)
    {
        phxDot[i*LEN_QG_STATE+lenHQ] = 0.0;
    }

    return OK;
}

/****************************************************************
 *�� �� ��: QGEKF()
 *��    ��:
 *         state: �˲�ǰ״̬q0, q1, q2, q3, g
 *         obs  : �۲��� phi, theta, psi
 *         param: ��Ҫ�Ĳ��� p, q, r, ax, ay, az
 *         est  : �˲����״̬ q0, q1, q2, q3, g
 *
 *��    �ܣ�����Ԫ�����������ٶȽ���EKF�˲�
 *
 *�� �� ֵ:
 *          �ɹ�: OK
 *          �������ָ��Ϊ�գ�ERR_MEM_NONE
 *
 *��    �ߣ����ı�
 *������ڣ�2011-3-22
 ****************************************************************/
void QGEKF(const double *state, const double *obs, const double *param, double *est)
{   
    //double Q[LEN_QG_STATE] = { 0.005,0.01,0.01,0.001,0.03};    // Q��
    //double R[LEN_QG_OBS] = { 1,1,2}; //2011 12 14
   
    //double Q[LEN_QG_STATE] = { 0.0008,0.0008,0.0008,0.0008,0.03};    // Q�� 12.9last
    double Q[LEN_QG_STATE] = { 0.001,0.001,0.001,0.001,0.03};
    double R[LEN_QG_OBS] = {1,1,0.1};
    // Pk������ʼ��
    static double s_Pk[LEN_QG_STATE][LEN_QG_STATE] = { {0.01}, {0.0, 0.01}, {0.0, 0.0, 0.01},{0.0, 0.0, 0.0, 0.01},{0.0, 0.0, 0.0,0.0,0.01} };
    static double s_preEst[LEN_QG_STATE];    // �ϴ��˲�ֵ
    static int s_isFirst = 1;                // ��һ���˲�

    struct Model model;                      // ϵͳģ��
    struct ModelParam mParam;                // ״̬�����۲���������ֵ�����ַ
    struct EKFParam ekfParam;                // EKF�˲�����Ҫ�Ĳ���

    //int i = 0;                               // ѭ������
    //double err;                              // �˲�ֵ��ԭֵ��ƫ��
    //double INF = 2e-3;                       // һ����С��ֵ

    // ����ϵͳģ��
    model.fxu = QGFxu;                // ״̬����
    model.fxDot = QGFxDot;            // ״̬������
    model.hx = QGHx;                  // �۲ⷽ��
    model.hxDot = QGHxDot;            // �۲ⷽ����

    // ����״̬�����۲���������ֵ�����ַ
    mParam.state = state;        // �����˲���״ֵ̬
    mParam.obs = obs;            // �۲���
    mParam.est = est;            // ����ֵ�����ַ
    mParam.othParam = param;     // �˲���Ҫ����������
    mParam.lenState = LEN_QG_STATE;    // ״̬������
    mParam.lenObs = LEN_QG_OBS;        // �۲�������

    // ����EKF�˲�����Ҫ�Ĳ���
    ekfParam.Q = Q;                    // Q��
    ekfParam.R = R;                    // R��
    ekfParam.Pk = (double*)s_Pk;       // Pk����
    ekfParam.preEst = s_preEst;        // ��һ�εĹ���ֵ
    ekfParam.isFirst = &s_isFirst;     // �ǵ�һ��ִ���˲���

    DoEKF(&model, &mParam, &ekfParam);    // EKF�˲�

    // ���q0~q3���˲�ֵ��ԭֵ���̫�����ԭֵ
    /*for ( i=0; i<4; i++)
    {
        err = est[i] - state[i];
        if ( (err > INF) || (err < -INF) )
        {
            est[i] = state[i];
            s_preEst[i] = state[i];
        }
    
    }*/  //by  sycȥ��
   
}
