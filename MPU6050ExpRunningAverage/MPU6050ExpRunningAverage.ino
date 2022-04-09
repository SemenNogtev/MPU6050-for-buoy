#include "I2Cdev.h"
#include "MPU6050.h"
#include "control.h"

//#define TO_DEG 180/M_PI // 180/pi
#define TO_DEG 57.3 // 180/pi = 57.295779513082320876798154814105
#define T_OUT 10 // каждый 10 миллисекунд будем проводить вычисления 
#define KF 0.1 // коэффициент комплементарного фильтра

MPU6050 accelgyro;
expRunningAverage Angle_ax,Angle_ay;
expRunningAverage Ax,Ay,Az;

unsigned long int t;
int16_t ax_raw, ay_raw, az_raw, ax_raw_g, ay_raw_g, az_raw_g, gx_raw, gy_raw, gz_raw;
float ax, ay, az, ax_g, ay_g, az_g, ax_n, ay_n, az_n, gx, gy, gz;
float angle_ax, angle_ay, angle_gx, angle_gy, angle_gz, angle_x, angle_y, angle_z;
float vx_n, vy_n, vz_n;
float x_n, y_n, z_n; 
float prev_ax_n=0, prev_ay_n=0, prev_az_n=0;
int circle=0, circle1=0;
int kx=0, ky=0, kz=0;
float fx=0, fy=0, fz=0;
float arr1_az_n[200];
float az_n_max, az_n_max1;
float sum=0;

float centering_ax = 0, noize_ax = 0;
float centering_ay = 0, noize_ay = 0;
float centering_az = 0, noize_az = 0;
float centering_gx = 0, noize_gx = 0;
float centering_gy = 0, noize_gy = 0;
float centering_gz = 0, noize_gz = 0;

void setup() {
    Serial.begin(9600);
    //Serial.println("f,k,z");
    accelgyro.initialize();
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    
    // Экспоненциальное бегущее среднее
    Angle_ax.set_values(0.3, 0.0);
    Angle_ay.set_values(0.3, 0.0);
    Ax.set_values(0.3, 0.0);
    Ay.set_values(0.3, 0.0);
    Az.set_values(0.3, 0.0);
  
//    for (int i = 0; i < 1000; i++)
//    {
//      accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
//
//      noize_ax = ax_raw;
//      noize_ay = ay_raw;
//      noize_az = az_raw;
//      centering_ax += noize_ax;
//      centering_ay += noize_ay;
//      centering_az += noize_az;
//
//      if (i >= 992)
//      {
//          //F1[i - 992] = noize_ax;
//          //F2[i - 992] = noize_ay;
//          //F3[i - 992] = noize_az;
//      }
//    }
//    
//    centering_ax = centering_ax / 1000;
//    centering_ay = centering_ay / 1000;
//    centering_az = centering_az / 1000;
//
//    for (int i = 0; i < 1000; i++)
//    {
//      accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
//
//      noize_gx = gx_raw;
//      noize_gy = gy_raw;
//      noize_gz = gz_raw;
//      centering_gx += noize_gx;
//      centering_gy += noize_gy;
//      centering_gz += noize_gz;
//
//      if (i >= 992)
//      {
//          //F1[i - 992] = noize_ax;
//          //F2[i - 992] = noize_ay;
//          //F3[i - 992] = noize_az;
//      }
//    }
//    
//    centering_gx = centering_gx / 1000;
//    centering_gy = centering_gy / 1000;
//    centering_gz = centering_gz / 1000;
}

void loop() {
    if( millis()-t > T_OUT ){
        t = millis();
        
        //////////////////////////////////////Считывание показаний с датчика///////////////////////////////////
        accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////Вычисление углов//////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        // ограничиваем +-1g
        ax_raw_g = constrain(ax_raw, -16384, 16384);
        ay_raw_g = constrain(ay_raw, -16384, 16384);
        az_raw_g = constrain(az_raw, -16384, 16384);

        // переводим в +-1.0
        ax_g = ax_raw_g / 16384.0;
        ay_g = ay_raw_g / 16384.0;
        az_g = az_raw_g / 16384.0;
        //Serial.print(ax_g);Serial.print("\t");
        //Serial.print(ay_g);Serial.print("\t");
        //Serial.print(az_g);Serial.print("\t");
           
        // угол наклона по акселерометру
        angle_ax = 90.0 - degrees(acos(ay_g));
        angle_ay = 90.0 - degrees(acos(ax_g));
        //Serial.print(angle_ax);Serial.print("\t");
        //Serial.print(angle_ay);Serial.print("\t");
        
        angle_ax = Angle_ax.expRunningAverage_solver(angle_ax);
        angle_ay = Angle_ay.expRunningAverage_solver(angle_ay);
        //Serial.print(angle_ax);Serial.println("\t");
        //Serial.print(angle_ay);Serial.println("\t");

        // преобразование сырых данных гироскопа в град/сек 250 град/сек = 32768/250
        gx = gx_raw / 32768.0 * 250;
        gy = gy_raw / 32768.0 * 250;
        gz = gz_raw / 32768.0 * 250;
        //Serial.print(gx);Serial.print("\t");
        //Serial.print(gy);Serial.print("\t");
        //Serial.print(gz);Serial.println("\t");
        
        // угол наклона по гироскопу
        angle_gx = angle_gx + gx * T_OUT/1000.0;
        angle_gy = angle_gy + gy * T_OUT/1000.0;
        angle_gz = angle_gz + gz * T_OUT/1000.0;
        //Serial.print(angle_gx);Serial.print("\t");
        //Serial.print(angle_gy);Serial.println("\t");

        // угол наклона по акселерометру и гироскопу
        angle_x = (1-KF)*(angle_x+gx*T_OUT/1000.0) + KF*angle_ax;
        angle_y = (1-KF)*(angle_y+gy*T_OUT/1000.0) + KF*angle_ay;
        angle_z = 0;
        //Serial.print(angle_x);Serial.print("\t");
        //Serial.print(angle_y);Serial.println("\t");
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////Вычисление ускорений////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        // преобразование сырых данных акселерометра в G 2G = 32768/2
        ax = ax_raw / 32768.0 * 2;
        ay = ay_raw / 32768.0 * 2;
        az = az_raw / 32768.0 * 2;
        
        ax = Ax.expRunningAverage_solver(ax);
        ay = Ay.expRunningAverage_solver(ay);
        az = Az.expRunningAverage_solver(az);
        //Serial.print(ax);Serial.print("\t");
        //Serial.print(ay);Serial.print("\t");
        //Serial.print(az);Serial.println("\t");
        
        // ускорения датчика в связанной СК (без учета ускорения свободного падения)
        ax=ax-1*sin((angle_y/TO_DEG));
        ay=ay-1*sin((angle_x/TO_DEG));
        az=az-1*cos(angle_y/TO_DEG)*cos(angle_x/TO_DEG);
        
        // преобразование ускорений из связанной СК в нормальную СК
        ax_n=(cos(angle_y/TO_DEG)*cos(angle_x/TO_DEG)-sin(angle_y/TO_DEG))*ax + (-sin(angle_y/TO_DEG)*cos(angle_x/TO_DEG))*ay + (sin(angle_x/TO_DEG))*az;
        ay_n=(sin(angle_y/TO_DEG))*ax + (cos(angle_y/TO_DEG))*ay + (0)*az;
        az_n=(-cos(angle_y/TO_DEG)*sin(angle_x/TO_DEG))*ax + (sin(angle_x/TO_DEG)*sin(angle_y/TO_DEG))*ay + (cos(angle_x/TO_DEG))*az;
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////Вычисление отклонения///////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        arr1_az_n[circle1]=abs(az_n);
        
        if(prev_az_n*az_n<0){
          az_n_max1=arr1_az_n[circle1/2];
          sum+=az_n_max1;
          circle1=0;
          kz++;
        }

        if(circle>=999){
          fz=kz/20.0;
          az_n_max=sum/kz;
          //Serial.print(fz);Serial.print("\t");
          //Serial.print(kz);Serial.print("\t");
          circle=0;
          circle1=0;
          kz=0;
          sum=0;
          z_n=(9.81*az_n_max)/(2*2*3.14*3.14*fz*fz);
          //Serial.print(z_n);Serial.println("\t");
        }
        
        prev_az_n=az_n;
        
        circle++;
        circle1++;
        
        //x_n=2*2*3.14*3.14*fx*fx*ax;
        //y_n=2*2*3.14*3.14*fy*fy*ay;
        //z_n=2*2*3.14*3.14*fz*fz*az_n_max;

        // скорость в нормальной СК 
        //vx_n=vx_n+ax_n*T_OUT/1000.0;
        //vy_n=vy_n+ay_n*T_OUT/1000.0;
        //vz_n=vz_n+az_n*T_OUT/1000.0;

        // перемещение в нормальной СК
        //x_n=x_n+vx_n*T_OUT/1000.0+0.5*ax_n*T_OUT*T_OUT/(1000000.0);
        //y_n=y_n+vy_n*T_OUT/1000.0+0.5*ay_n*T_OUT*T_OUT/(1000000.0);
        //z_n=z_n+vz_n*T_OUT/1000.0+0.5*az_n*T_OUT*T_OUT/(1000000.0);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////Serial Port///////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        //Serial.print(ax_raw);Serial.print("\t");
        //Serial.print(ay_raw);Serial.print("\t");
        //Serial.print(az_raw);Serial.println("\t");
        
        //Serial.print(ax_n);Serial.print("\t");
        //Serial.print(ay_n);Serial.print("\t");
        //Serial.print(az_n);Serial.println("\t");
        
        //Serial.print(vx_n);Serial.print("\t");
        //Serial.print(vy_n);Serial.print("\t");
        //Serial.print(vz_n);Serial.print("\t");
        
        //Serial.print(x_n);Serial.print("\t");
        //Serial.print(y_n);Serial.print("\t");
        //Serial.print(z_n);Serial.println("\t");
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}
