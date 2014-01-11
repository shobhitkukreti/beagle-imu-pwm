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
//#include <linux/ktime.h>

#define M_PI 3.14159265358979323846

#define RAD_TO_DEG 57.29578

void writeRegister(unsigned int, unsigned int);
void i2cOpen();
void i2cSetAddress(unsigned int);
unsigned int i2cRead(unsigned int reg);
void i2cClose();

int g_i2cFile;
void main()
{
unsigned int  ax_l,ax_h,ay_l,ay_h,az_l,az_h;
unsigned int AX,AY,AZ;
int16_t ax,ay,az;
float xg,yg,zg;
int fp_wd;
float pitch,roll,yaw;

fp_wd = open("/dev/watchdog",O_RDWR);
if(fp_wd<0)
{
printf("Watchdog File Error\n");
write(fp_wd,"\n",1);
exit(1);
}
	
i2cOpen();
	i2cSetAddress(0x19);
	writeRegister(0x20,0x47); // 10 HZ Update rate
	while(1)
	{
	write(fp_wd,"\n",1);
	ax_h =  i2cRead(0x29);
	ax_l =	i2cRead(0x28);
	ay_h = i2cRead(0x2B);
	ay_l = i2cRead(0x2A);
	az_h = i2cRead(0x2D);
	az_l = i2cRead(0x2C);	
ax = 	 ((ax_h <<8) | ax_l);
ay =     ((ay_h <<8) | ay_l);
az = 	 ((az_h <<8) | az_l);
//ax=AX;
//ay=AY;
//az=AZ;

/*
if( AY & 0x8000)
{	AX = (AX ^ 0xffff) +1;
	AX= AX & 0xfff;
}

if( AY & 0x8000)
{       AY = (AY ^ 0xffff) +1;
        AY= AY & 0xffff;
}

if( AY & 0x8000)
{       AZ = (AZ ^ 0xffff) +1;
        AZ= AZ & 0xfff;
}

*/

xg = ax * 0.0000610351;
yg = ay * 0.0000610351;
zg = az * 0.0000610351;
//printf("AX:%" PRId16 "\tAY:%" PRId16"\t:AZ:%" PRId16 "\t\n",ax,ay,az);	
//printf("X:%f\tY:%f\tZ:%f\t\n",xg,yg,zg);
pitch = atan(xg/(sqrt(pow(yg,2)+pow(zg,2))));

roll = atan(yg/(sqrt(pow(xg,2)+pow(zg,2))));

yaw = atan(zg/(sqrt(pow(yg,2)+pow(xg,2))));
pitch = pitch * RAD_TO_DEG;
roll = roll * RAD_TO_DEG;
yaw = yaw * RAD_TO_DEG;
printf("PITCH:%f\tROLL:%f\tYAW:%f\n",pitch,roll,yaw);

usleep(100000);

	

}
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



