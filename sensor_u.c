#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <time.h>
#include <linux/types.h> 
#include <inttypes.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include<unistd.h>
#include<signal.h>
#include<time.h>

#define M_PI 3.14159265358979323846

#define RAD_TO_DEG 57.29578
#define GAIN_FACTOR 5880

float x_acc,y_acc,z_acc;
float x_gyro,y_gyro,z_gyro;
float KP = 1.0;
float KI = 0.05;
float KD = 0.3;

void *read_acc();
void *read_gyro();
void control_loop();
void motor_init();
void write_motor(int,int);
void sig_handler(int);

void writeRegister(unsigned int, unsigned int);
void i2cOpen();
void i2cSetAddress(unsigned int);
unsigned int i2cRead(unsigned int reg);
void i2cClose();

int g_i2cFile;
FILE *m1_file,*m2_file,*m3_file,*m4_file;
float pitch=0.0,roll=0.0,yaw=0.0,yaw_acc=0.0,pitch_acc=0.0,roll_acc=0.0;
float pitch_offset=0.0,roll_offset=0.0,yaw_offset=0.0;
float xg,yg,zg;
float x_offset,y_offset,z_offset;

/*
* Start of Main
*
*/

int flag=0;
void main()
{
int i;
int cnt=0,delay=0;
struct timeval start,end;
if (signal(SIGINT, sig_handler) == SIG_ERR)
{  printf("\ncan't catch SIGINT\n");

exit(0);

}
//while(1);

i2cOpen();
motor_init();
read_acc();
for(i=0;i<10;i++)
{
read_acc();
read_gyro();
usleep(10000);
}

while(1)
{
//	gettimeofday(&start,NULL);
	read_gyro();
//	if(cnt==5)
	{	
	read_acc();
	//printf("%f\t%f\t%f\n",pitch,roll,yaw);
	}
	pitch = (0.98 * pitch + 0.02*pitch_acc);//-pitch_offset;
	roll = (0.98 * roll + 0.02*roll_acc);//-roll_offset;
 	yaw =  (0.98*yaw + 0.02*yaw_acc);
	
         if(cnt>150)
	 {
		 control_loop();
		 cnt=200;
	 }
	//printf("%f\t%f\n",pitch,roll);
	//usleep(2000);
	cnt++;
  //      gettimeofday(&end,NULL);
//	printf("Time:%ld\n", (((end.tv_sec * 1000000 + end.tv_usec)
  //                - (start.tv_sec * 1000000 + start.tv_usec))));
//printf("%f\t%f\t%f\n",pitch,roll,yaw);
}

}

float last_x=0.0,last_y=0.0,last_z=0.0;
float GX,GY,GZ;
void motor_init()
{
}


void write_motor(int num,int val)
{
//printf("***\n");
if((m1_file = fopen("/sys/devices/ocp.3/pwm_test_P8_13.12/duty","w"))==NULL)
perror("P8_13_error\n");
if((m2_file = fopen("/sys/devices/ocp.3/pwm_test_P8_19.13/duty", "w"))==NULL)
perror("P8_19_error\n");
if((m3_file = fopen("/sys/devices/ocp.3/pwm_test_P9_14.10/duty", "w"))==NULL)
perror("P9_14_error\n");
if((m4_file = fopen("/sys/devices/ocp.3/pwm_test_P9_16.11/duty", "w"))==NULL)
perror("P9_16_error\n");
switch (num)
{

case 1 : fprintf(m1_file,"%d",val); break;
case 2 : fprintf(m2_file,"%d",val); break;
case 3 : fprintf(m3_file,"%d",val); break;
case 4 : fprintf(m4_file,"%d",val); break;
}

fclose(m1_file);
fclose(m2_file);
fclose(m3_file);
fclose(m4_file);


}

void sig_handler(int signo)
{
FILE *m1,*m2,*m3,*m4;
int val=1000000;
  if (signo == SIGINT)
    printf("received SIGINT\n");

if((m1 = fopen("/sys/devices/ocp.3/pwm_test_P8_13.12/duty","w"))==NULL)
perror("P8_13_error\n");
if((m2 = fopen("/sys/devices/ocp.3/pwm_test_P8_19.13/duty", "w"))==NULL)
perror("P8_19_error\n");
if((m3 = fopen("/sys/devices/ocp.3/pwm_test_P9_14.10/duty", "w"))==NULL)
perror("P9_14_error\n");
if((m4 = fopen("/sys/devices/ocp.3/pwm_test_P9_16.11/duty", "w"))==NULL)
perror("P9_16_error\n");
fprintf(m1,"%d",val); 
fprintf(m2,"%d",val); 
fprintf(m3,"%d",val); 
fprintf(m4,"%d",val); 

fclose(m1);
fclose(m2);
fclose(m3);
fclose(m4);

if(m1_file!=NULL)
fclose(m1_file);
if(m2_file!=NULL)
fclose(m2_file);
if(m3_file!=NULL)
fclose(m3_file);
if(m4_file!=NULL)
fclose(m4_file);
exit(1);
}


void control_loop()
{
int i=0;
float x_err, y_err, z_err;
int PID_ROLL,PID_YAW,PID_PITCH;
int throttle = 1500000;
int m1_val,m2_val,m3_val,m4_val;
x_err = 0.0-roll;
y_err= 0.0-pitch;
z_err= 89.3-yaw;
//printf("Err:%f\t%f\t%f\n",x_err,y_err,z_err);

PID_ROLL = KP * x_err - (KD* (x_err-last_x)/0.013);
PID_PITCH = KP * y_err- (KD* (y_err-last_y)/0.013);
PID_YAW = KP * z_err -(KD* (z_err-last_z)/0.013);
//printf("%d\t%d\t%d\n",PID_PITCH,PID_ROLL,PID_YAW);
last_x = x_err;
last_y = y_err;
last_z = z_err;

if(PID_ROLL < -85)
  PID_ROLL= - 85;
if(PID_PITCH< -85)
PID_PITCH=-85;
if(PID_YAW<-85)
PID_YAW=-85;



if(PID_ROLL > 85)
  PID_ROLL=85;
if(PID_PITCH>85)
PID_PITCH=85;
if(PID_YAW>85)
PID_YAW=85;

m1_val = throttle+(PID_PITCH*GAIN_FACTOR);
m2_val = throttle-(PID_PITCH*GAIN_FACTOR);
m3_val = throttle+(PID_ROLL*GAIN_FACTOR);
m4_val = throttle-(PID_ROLL*GAIN_FACTOR);

printf("%d\t%d\t%d\t%d\n",m1_val,m2_val,m3_val,m4_val);


if(m1_val > 2000000)
m1_val=2000000;

if(m2_val > 2000000)
m2_val=2000000;

if(m3_val > 2000000)
m3_val=2000000;

if(m4_val > 2000000)
m4_val=2000000;

if(m1_val< 1000000)
m1_val = 1000000;

if(m2_val< 1000000)
m2_val = 1000000;

if(m3_val< 1000000)
m3_val = 1000000;
if(m4_val< 1000000)
m4_val = 1000000;



write_motor(1,m1_val);
write_motor(2,m2_val);
write_motor(3,m3_val);
write_motor(4,m4_val);

}




void *read_gyro()
{
volatile unsigned int  gx_l,gx_h,gy_l,gy_h,gz_l,gz_h;
//unsigned int GX,GY,GZ;
int16_t gx,gy,gz;
float xg,yg,zg;
int fp_wd;

i2cSetAddress(0x6b);
writeRegister(0x20,0xff); // 100 hz cutoff 760 Hz odr
writeRegister(0x23,0x30);
        
        gx_h =  i2cRead(0x29);
        gx_l =  i2cRead(0x28);
        gy_h = i2cRead(0x2B);
        gy_l = i2cRead(0x2A);
        gz_h = i2cRead(0x2D);
        gz_l = i2cRead(0x2C);
gx =     ((gx_h <<8) | gx_l);
gy =     ((gy_h <<8) | gy_l);
gz =     ((gz_h <<8) | gz_l);


GX = gx * 0.07;
GY = gy * 0.07;
GZ = gz * 0.07;

pitch += gx*0.07*0.013;
roll += gy*0.07*0.013;
yaw  += gz*0.07*0.013;
}



void *read_acc()
{
volatile unsigned int  ax_l,ax_h,ay_l,ay_h,az_l,az_h;
unsigned int AX,AY,AZ;
int16_t ax,ay,az;
//float xg,yg,zg;

        i2cSetAddress(0x19);
        writeRegister(0x20,0x57); // 100 HZ Update rate
            
        ax_h =  i2cRead(0x29);
        ax_l =  i2cRead(0x28);
        ay_h = i2cRead(0x2B);
        ay_l = i2cRead(0x2A);
        az_h = i2cRead(0x2D);
        az_l = i2cRead(0x2C);
ax =     ((ax_h <<8) | ax_l);
ay =     ((ay_h <<8) | ay_l);
az =     ((az_h <<8) | az_l);

xg = ax * 0.0000610351;
yg = ay * 0.0000610351;
zg = az * 0.0000610351;
pitch_acc = atan(xg/(sqrt(pow(yg,2)+pow(zg,2))));
roll_acc = atan(yg/(sqrt(pow(xg,2)+pow(zg,2))));
yaw_acc = atan(zg/(sqrt(pow(xg,2)+pow(yg,2))));

pitch_acc = pitch_acc * RAD_TO_DEG;
roll_acc = roll_acc * RAD_TO_DEG;
yaw_acc = yaw_acc * RAD_TO_DEG;

}

void writeRegister(unsigned int reg, unsigned int value)
{
	unsigned char data[3];
	data[0]=reg & 0xff;
	data[1]=value & 0xff;
	data[2]=0;

	if (write(g_i2cFile, data, 2) != 2) {
		                perror("WriteRegister");
				        }
}

void i2cSetAddress(unsigned int address)
{
if (ioctl(g_i2cFile, I2C_SLAVE, address) < 0) {
   perror("i2cSetAddress");
   exit(1);
        }
}

unsigned int i2cRead(unsigned int reg)
{
	char buf[2];
	buf[0] = reg & 0xff;

	if(write(g_i2cFile, buf,1) != 1) {
		perror("Write Error\n");
		exit(1);
	}

	if(read(g_i2cFile, buf,1) !=1){
		perror("Read Error\n");
		exit(1);
	}

	//printf("AX:0x%02X\n",buf[0]);
//	printf("AX_H:%d\tAX_L:%d\n",buf[0],buf[1]);
	return (unsigned int)buf[0];

}

void i2cOpen()
{
	        g_i2cFile = open("/dev/i2c-1", O_RDWR);
		        if (g_i2cFile < 0) {
				                perror("i2cOpen");
						                exit(1);
								        }
}

void i2cClose()
{
	        close(g_i2cFile);
}



