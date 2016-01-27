/******
2015-6-29 byM , ��βavcs

*/                                          
#include <includs.h>

#define _avcs_tuning_ //ʦ���״ε���ʱ����Ҫ�ö���!!!!!!!!!ע��,�������֮��,Ҫ��ע�͵��ö���,�Ļ�"�̻�����ģʽ"


/*******************************************************
//������:
//��������:2015/7/12
//����޸�����:
//��������:
//ʹ�÷�ʽ:
*******************************************************/
void avcs(int *pwm)
{
#define init_gain_P 64          //��βģʽ�ı�������   >>>>>>����:���������3588(arm7PWMֵ)��Ӧң����MODE��SERVO�е�100(ң������SERVOֵ,����END POINTֵ!) , 2088��Ӧ-100 , ����((3588+2088)/2)��Ӧ0 , ((3588-2088)/2/100)��ӦSERVO��ÿ�仯һ��pwm�仯����
                                  //ʦ�����ò�����,�ò����ļ��㷽��:gain_P=0.3240*(2846-pwm),����pwm=((3588+2088)/2)-((3588-2088)/2)*(x/100),xΪ����β������"һ��ģʽ"��ң����MODE��SERVO��gainͨ������
                                  //����ʽ:gain_P=0.3240*(8+7.5*x)
#define init_gain_I 8.8          //��βģʽ�Ļ�������
                                  //ʦ�����ò�����,�ò����ļ��㷽��:gain_I=0.0324*(pwm-2830),����pwm=((3588+2088)/2)+((3588-2088)/2)*(y/100),yΪ����β������"��βģʽ"��ң����MODE��SERVO��gainͨ������
                                  //����ʽ:gain_I=0.0324*(8+7.5*y)
#define init_gain_D 0.0;//3.86
#define output_limit 430.0        //output_limit��Ϊ�˸�β���޷�,��β�治��ת,��Ӧservo_widths_temp�ı仯�����ֵ,ȡ���������Сֵ
                                  //��򵥵�limitֵ���԰취,�Ȱ�β����ڵ�5ͨ��,gainͨ��,��ң����MODE��END POINT�������л�������gainͨ����ֵ,ʹ�ﵽ�������г̵����ֵ,Ȼ���л���ң����MODE��SERVO��,������ͨ����ֵ������output_limit
                                  //����ʽ:output_limit=7.5*z,zΪ�����г��н�Сֵ
#define init_gain_coll_compensation 0.0//0.65                        //�ܾಹ����
  
/*�̻���β����ģʽ*///�·ɻ���Ҫ����"������β����ģʽ"
#ifndef _avcs_tuning_
  /*�����Ǹж�*/
  #define gain_P   init_gain_P  
  #define gain_I   init_gain_I      
  #define gain_D   init_gain_D//��βģʽ��΢������   
  #define gain_coll_compensation  init_gain_coll_compensation  //�ܾಹ����
#endif 
  
/*������β����ģʽ*/
#ifdef _avcs_tuning_ 
  /*�����Ǹж�*/ 
  static double gain_P = init_gain_P;     //��βģʽ�ı�������
  static double gain_I = init_gain_I;      //��βģʽ�Ļ�������
  static double gain_D = init_gain_D;      //��βģʽ��΢������
  static double gain_coll_compensation = init_gain_coll_compensation;      //�ܾಹ����
#endif
 
  
  
/*ң��������*/
#define gain_remote_to_rate -0.007//ҡ������ɻ�����rad/s�ٶ�(300/180*3.14159)/((3588-2090)/2),��Ӧҡ����ƫ300��/s
/*��β�������������*/
#define _Direction_ -       //������Ϊ"#define _Direction_ -"
/*��βͨ���������λֵ*/
#define output_mid  2838           //��Ӧ��ʼ�����1.52ms���ŷ�����,��Ӧ��servo_widths_temp���е���� 
/*��ͨ�˲�����ֹƵ��*/
#define alpha 1.0 //0.5945  //0.6768//0.8073//rate_error�ĵ�ͨ�˲�ϵ��,���㷽��:Dt = 0.033333;filt_hz=30;alpha = Dt/(Dt+1/(2*pi*filt_hz)),����Dt��0.0333s,filt_hz�ǽ���Ƶ��

  
/*�õ��ľ�̬����*/
static double manual_mid;               //ҡ��ƫ����,���ڱ��ң����λֵ,����ʱ��ʼ��
static double gyro_bias;                //���ٶ�ƫ����,����У�����ٶ�ֵ,����ʱ��ʼ��
static double _rate_error = 0.0;        //����rate_error�ĵ�ͨ�˲�,��¼�ɵ�ֵ
static double i_rate_error = 0.0;       //���ڿ�����,�����������ֵ
static int stage = 0;                   //������β��ʼ���ֽ׶�,�ڳ�ʼ���׶�,ȷ��ң������ҡ�˸�λ,����ֱ����,��Ϊ�ý׶λ��ȡmanual_mid��gyro_bias
static bool flag_breach_limit = 0;      //yaw_out���ޱ�־λ,0:û�г�,1:����
static int coll_zero;           //�ܾಹ����

static double tar_rate_pppppppp = 0.0;
static double tar_rate_ppppppp = 0.0;
static double tar_rate_pppppp = 0.0;
static double tar_rate_ppppp = 0.0;
static double tar_rate_pppp = 0.0;
static double tar_rate_ppp = 0.0;//���ڻ��ֿ�����,pΪprevious,ǰһ������˼,����ͺ�Խ��,ppppԽ��
static double tar_rate_pp = 0.0;
static double tar_rate_p = 0.0;

double target_rate = 0.0;
double d_rate_error ;
double rate_error;
double yaw_out;


/*�������manual_mid��gyro_bias�Ļ�ȡ*/
if (stage < 200)
  {
      stage++; 
      manual_mid += *pwm;
      gyro_bias += ahrs_pqr[2];//������
      *pwm = output_mid;
  }
else if(stage < 201)
  { 
      stage++;  
      manual_mid = manual_mid/200.0;
      gyro_bias = gyro_bias/200.0;
      *pwm = output_mid;
      coll_zero = *(pwm+2);             //�ܾಹ����
  }
/*���涯����ʾ��β���ݳ�ʼ���Ѿ����*/
else if(stage < 216)
  {
      stage++;        
      *pwm = output_mid + output_limit;    
  }
else if(stage < 231)
  {
      stage++; 
      *pwm = output_mid - output_limit;
  }


/*��������β�����ǵĿ����㷨,����βģʽ��һ��ģʽ,һ��ģʽ���ڵ��ɻ�����λֵ*/
else 
  {
    /*�ڴ����ٶ�*/
    int target_pwm;
    target_pwm = *pwm - manual_mid;
     if(*(pwm+1) > 2838)//��βģʽ��ƫ��ң��������΢��ҡ�˿������������������ڽ������ֲ������Ŷ�
    {
    if(target_pwm > 23) target_pwm = target_pwm - 23;
    else if(target_pwm < -23) target_pwm = target_pwm + 23;
    else target_pwm = 0;
    }
    target_rate = target_pwm * gain_remote_to_rate;//ҡ���ź�תΪ���ٶ��ź�,��λrad/s ������
    yaw_commend[1]= target_rate;//��ɾ
     /*���ٶ����*/
    rate_error = (target_rate - (ahrs_pqr[2] - gyro_bias));//ת�����,
    
    
    /*���ĵ�ͨ�˲�*/
    d_rate_error = (rate_error - _rate_error)*alpha;//d_rate_error����,�˴�����仯��(delta)
    rate_error = d_rate_error + _rate_error;
    d_rate_error = d_rate_error/0.03333;            //d_rate_error����,�˴�����仯��(derivative)
    _rate_error = rate_error;                       //_rate_error������һ���˲����˵�����Ҫ�ĸ�Ƶ����Ƶ��ʹ�ɻ�β��
    
    
#ifdef _avcs_tuning_   
//    if(*(pwm+4) <= 2838)
//    {
//      if(*(pwm+4) <= 2830)
//        gain_coll_compensation = 0.003165 * (2830 - *(pwm+4));//�ܾಹ����
//      else
//        gain_coll_compensation = 0.0;
//    }
 
    if(*(pwm+4) <= 2838)
    {
      if(*(pwm+4) <= 2830)
        gain_D = (2830 - *(pwm+4))/50.0;//΢����
      else
        gain_D = 0.0;
    }
#endif
    
    /*һ��ģʽ*/
    if(*(pwm+1) <= 2838)//����βģʽ/��avcs
      {
#ifdef _avcs_tuning_
        gain_P = 0.3240 * (2846 - *(pwm+1));//���100%�жȶ�������,�޸ı���
#endif
        in_loop_yaw_PID[0] = gain_P;//��ɾ
        in_loop_yaw_PID[1] = 0;//��ɾ
        in_loop_yaw_PID[2] = gain_D;//gain_coll_compensation;//��ɾ//�ܾಹ����
        
        i_rate_error = 0.0;
        flag_breach_limit = 0;
        
        /*P������*/
        yaw_out = rate_error * gain_P;
//        yaw_out += gain_coll_compensation * (coll_zero - *(pwm+2));        //�ܾಹ����
        
        if(yaw_out > output_limit) 
        {
          yaw_out = output_limit;
        }
        else if(yaw_out < -output_limit)
        {
          yaw_out = -output_limit;
        } 
        *pwm = output_mid + _Direction_(yaw_out);//_Direction_Ϊ�������Ķ���
      }
    
    /*��βģʽ avcs*/
    else
      {
#ifdef _avcs_tuning_
        gain_I = 0.0324 * (*(pwm+1) - 2830);//���100%�жȶ�������,�޸ı���
#endif
        in_loop_yaw_PID[0] = gain_P;//��ɾ
        in_loop_yaw_PID[1] = gain_I;//��ɾ
        in_loop_yaw_PID[2] = gain_D;//gain_coll_compensation;//��ɾ//�ܾಹ����
        if(((!flag_breach_limit) || (i_rate_error>0&&rate_error<0) || (i_rate_error<0&&rate_error>0)))//û�л��ֱ���,�����˳����ֱ���
        {
//        if (abs(ahrs_pqr[2] - gyro_bias)<0.5)
        i_rate_error += (tar_rate_pppp - (ahrs_pqr[2] - gyro_bias));
        }
        /*PI������,PID����,��Ҫ��PI��������,D����Ϊ0������Ҫ����*/
        if(rate_error <0.0)
          yaw_out = rate_error*gain_P*2.0*(1.0+abs(rate_error/6.0)) + i_rate_error*gain_I + d_rate_error*gain_D;
        else
          yaw_out = rate_error*gain_P*(1.0+abs(rate_error/6.0)) + i_rate_error*gain_I + d_rate_error*gain_D;
//        yaw_out += gain_coll_compensation * (coll_zero - *(pwm+2));        //�ܾಹ����
        
        if(yaw_out > output_limit) 
        {
          flag_breach_limit = 1;
          yaw_out = output_limit;
        }
        else if(yaw_out < -output_limit)
        {
          flag_breach_limit = 1;
          yaw_out = -output_limit;
        } 
        else
        {
          flag_breach_limit = 0;  
        }
        *pwm = output_mid + _Direction_(yaw_out);//_Direction_Ϊ�������Ķ���
      }
      tar_rate_pppppppp =   tar_rate_ppppppp;
      tar_rate_ppppppp =    tar_rate_pppppp;
      tar_rate_pppppp =     tar_rate_ppppp;
      tar_rate_ppppp =      tar_rate_pppp;
      tar_rate_pppp =       tar_rate_ppp;
      tar_rate_ppp =        tar_rate_pp;
      tar_rate_pp =         tar_rate_p;
      tar_rate_p =          target_rate;
  } 
}
