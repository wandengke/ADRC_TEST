#include <math.h>  
#include <iostream>  

using namespace std;
float v[2];
float z[3];
//float u;
float u0;
float sign(float x);
void adrc_td(float v0,float td_h,float td_w,float V1,float V2);
void adrc_eso(float y,float Z1,float Z2,float Z3,float eso_h,float B1,float B2,float B3,float Q,float a1,float a2,float a3,float b0,float u);
void adrc_nlsef(float e1,float e2,float B1_N,float B2_N,float Q_N,float A1,float A2);
float adrc(float set_point,float model_output,float dt);

int main()
{
    static int i=0;
    float a;
    float output_control;
    for(int i;i<1000;i++)
    {
        if(i<50)
        {
            a=1;
        }
        else
        {
            a=1;
        }
        output_control=adrc(a,a,0.01);

        cout <<"output_control "<<output_control<< endl;            
    }
    return 0;
}

float adrc(float set_point,float model_output,float dt)
{   float control_output;

    //adrc_td(float v0,float td_h,float td_w,float V1,float V2)
    adrc_td(set_point,dt,100.0,v[0],v[1]);

    //adrc_eso(float y,float Z1,float Z2,float Z3,float eso_h,float B1,float B2,float B3,float Q,float a1,float a2,float a3,float b0,float u)
    adrc_eso(model_output,z[0],z[1],z[2],dt,35,300,100,0.006,0.75,0.5,0.25,0.9,u0-z[2]/0.9);
   

    //adrc_nlsef(float e1,float e2,float B1_N,float B2_N,float Q_N,float A1,float A2)
    adrc_nlsef(v[0]-z[0],v[1]-z[1],15,9,0.006,0.5,0.05);


    control_output=u0-z[2]/0.9;
    return control_output;

}

void adrc_td(float v0,float td_h,float td_w,float V1,float V2)
{
    float a;
    float fst,d,d0,x1,x2,y,a0;

    d=td_w*td_h;
    d0=td_h*d;
    x1=V1-v0;
    x2=V2;
    y=x1+td_h*x2;
    a0=sqrt(d*d+8*td_w*fabsf(y));

    if(fabsf(y)>d0)
    {
         a=x2+0.5*(a0-d)*sign(y);
    }
    else
    {
         a=x2+y/td_h;
    }

    if(fabsf(a)>d)
    {
         fst=-td_w*sign(a);
    } 
    else
    {
         fst=-td_w*a/d;
    }

    v[0]=V1+td_h*V2;
    v[1]=V2+td_h*fst;
    //cout <<"v[0] " <<v[1]<< endl;
}

void adrc_eso(float y,float Z1,float Z2,float Z3,float eso_h,float B1,float B2,float B3,float Q,float a1,float a2,float a3,float b0,float u)
{
    float e,fal1,fal2,fal3;
    e=Z1-y;

    if (fabsf(e)>Q)
    {
        fal1=sign(e)*powf(fabsf(e),a1);
        fal2=sign(e)*powf(fabsf(e),a2);
        fal3=sign(e)*powf(fabsf(e),a3);
    }
    else
    {
        fal1=e/powf(Q,1-a1);
        fal2=e/powf(Q,1-a2);
        fal3=e/powf(Q,1-a3);
    }

    z[0]=Z1+eso_h*(Z2-B1*fal1);
    z[1]=Z2+eso_h*(Z3-B2*fal2+b0*u);
    z[2]=Z3-B3*eso_h*fal3;
}

void adrc_nlsef(float e1,float e2,float B1_N,float B2_N,float Q_N,float A1,float A2)
{
    float fal1,fal2;
    if(fabsf(e1)>Q_N)
    {
        fal1=sign(e1)*powf(fabsf(e1),A1);
    }
    else
    {
        fal1=e1/powf(Q_N,1-A1);
    }
    if(fabsf(e2)>Q_N)
    {
        fal2=sign(e2)*powf(fabsf(e2),A2);
    }
    else
    {
        fal2=e2/powf(Q_N,1-A2);
    }

    u0=B1_N*fal1+B2_N*fal2;
}

float sign(float x)
{
  if(x>0)
  {
    return(1.0f);
  }
  else if (x<0)
  {
    return(-1.0f);
  }
  else
  {
     return(0.0f);
  }
}
