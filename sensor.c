/* Beaglebone UserSpace PID Controller for maintaining orientation of the craft 
 * by Controlling the BLDC Motors through PWM. Sensors is an ST Micro IMU comprising
 * a gyrscope and a accelerometer
*/

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
#define M_PI 3.14159265358979323846

#define RAD_TO_DEG 57.29578

float x_acc,y_acc,z_acc;
float x_gyro,y_gyro,z_gyro;
float KP = 20;
float KI = 0.05;
float KD = 8;

void *read_acc();
void *read_gyro();
void control_loop();
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
float last_x=0.0,last_y=0.0,last_z=0.0;
float GX,GY,GZ;


/*
 * Start of Main
 *
 */

int flag=0;
void main()
{
		int cnt=0,delay=0;

		if (signal(SIGINT, sig_handler) == SIG_ERR)
		{  printf("\ncan't catch SIGINT\n");

				exit(0);

		}

		i2cOpen();
		motor_init();
		read_acc();
		while(1)
		{
				read_gyro();
				if(cnt==5 || cnt == 10)
						read_acc();


				pitch = (0.98 * pitch + 0.02*pitch_acc);/*-pitch_offset;*/
				roll = (0.98 * roll + 0.02*roll_acc);/*-roll_offset */
				yaw =  (0.85*yaw + 0.15*yaw_acc);
				if(flag==0 && delay > 200)	
				{
						flag=1;
						delay=0;
				}
				else 
						delay++;
				if(cnt==10)
				{
						if(flag)
								control_loop();
						cnt=0;
				}  
				usleep(1900);
				cnt++;
		}
}


void write_motor(int num,int val)
{
		if((m1_file = fopen("/sys/devices/ocp.3/pwm_test_P8_13.17/duty","w"))==NULL)
				perror("P8_13_error\n");
		if((m2_file = fopen("/sys/devices/ocp.3/pwm_test_P8_19.18/duty", "w"))==NULL)
				perror("P8_19_error\n");
		if((m3_file = fopen("/sys/devices/ocp.3/pwm_test_P9_14.15/duty", "w"))==NULL)
				perror("P9_14_error\n");
		if((m4_file = fopen("/sys/devices/ocp.3/pwm_test_P9_16.16/duty", "w"))==NULL)
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
		int val=1000000; /*Stops the motors */
		if (signo == SIGINT)
				printf("received SIGINT\n");

		if((m1 = fopen("/sys/devices/ocp.3/pwm_test_P8_13.17/duty","w"))==NULL)
				perror("P8_13_error\n");
		if((m2 = fopen("/sys/devices/ocp.3/pwm_test_P8_19.18/duty", "w"))==NULL)
				perror("P8_19_error\n");
		if((m3 = fopen("/sys/devices/ocp.3/pwm_test_P9_14.15/duty", "w"))==NULL)
				perror("P9_14_error\n");
		if((m4 = fopen("/sys/devices/ocp.3/pwm_test_P9_16.16/duty", "w"))==NULL)
				perror("P9_16_error\n");
		fprintf(m1,"%d",val); 
		fprintf(m2,"%d",val); 
		fprintf(m3,"%d",val); 
		fprintf(m4,"%d",val); 
		fclose(m1);
		fclose(m2);
		fclose(m3);
		fclose(m4);
		exit(1);
}


void control_loop()
{
		float x_err, y_err, z_err;
		int PID_ROLL,PID_YAW,PID_PITCH;
		int throttle = 1500000;
		int m1_val,m2_val,m3_val,m4_val;

		/*offsets for error*/
		x_err = -0.4-roll;
		y_err= -0.3-pitch;
		z_err= 89.5-yaw;

		PID_ROLL = KP * x_err - (KD* (x_err-last_x)/0.02);
		PID_PITCH = KP * y_err- (KD* (y_err-last_y)/0.02);
		PID_YAW = KP * z_err -(KD* (z_err-last_z)/0.02);
		last_x = x_err;
		last_y = y_err;
		last_z = z_err;

		if(PID_ROLL < -1500)
				PID_ROLL= - 1500;
		if(PID_PITCH< -1500)
				PID_PITCH=-1500;
		if(PID_YAW<-1500)
				PID_YAW=-1500;



		if(PID_ROLL > 1500)
				PID_ROLL=1500;
		if(PID_PITCH>1500)
				PID_PITCH=1500;
		if(PID_YAW>1500)
				PID_YAW=1500;

		m1_val = throttle+((PID_PITCH+PID_YAW)*250);
		m2_val = throttle-((PID_PITCH+PID_YAW)*250);
		m3_val = throttle+((PID_ROLL-PID_YAW)*250);
		m4_val = throttle-((PID_ROLL-PID_YAW)*250);

		/* Setting Max and Min value for the motors */
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


/* Read Gyroscope Data Via I2C bus */

void *read_gyro()
{
		volatile unsigned int  gx_l,gx_h,gy_l,gy_h,gz_l,gz_h;
		int16_t gx,gy,gz;
		float xg,yg,zg;
		int fp_wd;

		i2cSetAddress(0x6b);
		writeRegister(0x20,0xff); // 100 hz cutoff 760 Hz odr
		writeRegister(0x23,0x30);
#if 0
		writeRegister(0x24,0x10); // HP Filter enabled
		writeRegister(0x21,0x03);
#endif

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

		pitch += gx*0.07*0.002;
		roll += gy*0.07*0.002;
		yaw  += gz*0.07*0.02;


}


/* Read Accelerometer Data via I2C bus */
void *read_acc()
{
		volatile unsigned int  ax_l,ax_h,ay_l,ay_h,az_l,az_h;
		unsigned int AX,AY,AZ;
		int16_t ax,ay,az;

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



