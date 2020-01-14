#include <unistd.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include <sstream>

#define ROS_ERROR printf
#define ROS_INFO printf
#define ROS_WARN printf


#pragma pack(push)
#pragma pack(1)
typedef struct
{
  int16_t head;
  int16_t gyro[3];
  int16_t acce[3];
  int16_t mag[3];
  int16_t barometer;
  int16_t temperature;
  uint32_t timestamp;
  int16_t checksum;
  int16_t end;
} USB_Frame;
#pragma pack(pop)

USB_Frame usb_frame;

void syncFrame(int fd) {
    unsigned char a = 0;
    while (a != 0x12) {
        long l = read(fd, &a, 1);
        if (l != -1)
            ROS_INFO("sync:read %ld char: %x \n", l, a);
    }
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "adis16448_bridge");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/adis16448/imu", 1000);
	ros::Rate loop_rate(10000);

	const char *sp_path = (argc > 1) ? argv[1] : "/dev/adis16448";
    int fd = open(sp_path, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        ROS_ERROR("open port failed");
        return fd;
    }
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        ROS_ERROR("tcgetattr failed");
    }
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);
    tty.c_cflag     &=  ~PARENB;  // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~CRTSCTS;  // no flow control
    tty.c_lflag     =   0;  // no signaling chars,no echo,nocanonicalprocessing
    tty.c_oflag     =   0;  // no remapping, no delays
    tty.c_cc[VMIN]      =   0;  // read doesn't block
    tty.c_cc[VTIME]     =   5;  // 0.5 seconds read timeout

    tty.c_cflag     |=  CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);  // turn off s/w flow ctrl
    tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG);  // make raw
    tty.c_oflag     &=  ~OPOST;  // make raw
    tty.c_iflag     &=  ~(ICRNL|IGNCR|INLCR|ISTRIP|PARMRK|BRKINT|IGNBRK);  // raw mode, no converting 0x0d to 0x0a

    /* Flush Port, then applies attributes */
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ROS_ERROR("tcsetattr failed.");
    }

    while (ros::ok()) {
        int readCount = 0;
		while (1) {
		    usleep(100);
		    long r = read(fd, ((unsigned char *) &usb_frame) + readCount, sizeof(usb_frame) - readCount);
		    if (r > 0) {
			    readCount += r;
		    }
		    if (readCount > sizeof(usb_frame)) {
			    ROS_ERROR("overflow.\n");
			    return(666);
		    }
		    if (readCount == sizeof(usb_frame)) {
			    if (usb_frame.head == 0x1234 && usb_frame.end == 0x5678) {
				    break;
			    } else {
				    ROS_WARN("read invalid frame");
				    syncFrame(fd);
				    readCount = 0;
			    }
		    }
	    }


    float gx = usb_frame.gyro[0]/25.*M_PI/180;
    float gy = usb_frame.gyro[1]/25.*M_PI/180;
    float gz = usb_frame.gyro[2]/25.*M_PI/180;
    float ax = usb_frame.acce[0]/1200.*9.8065;
    float ay = usb_frame.acce[1]/1200.*9.8065;
    float az = usb_frame.acce[2]/1200.*9.8065;

	//printf("%+9.4f %+9.4f %+9.4f %+9.4f %+9.4f %+9.4f\n",gx,gy,gz,ax,ay,az);

	sensor_msgs::Imu data;
    data.header.frame_id = "imu";
    data.header.stamp = ros::Time::now();

    // Linear acceleration
    data.linear_acceleration.x = ax;
    data.linear_acceleration.y = ay;
    data.linear_acceleration.z = az;

    // Angular velocity
    data.angular_velocity.x = gx;
    data.angular_velocity.y = gy;
    data.angular_velocity.z = gz;

    // Orientation (not provided)
    data.orientation.x = 0;
    data.orientation.y = 0;
    data.orientation.z = 0;
    data.orientation.w = 1;


    imu_pub.publish(data);

    ros::spinOnce();
    loop_rate.sleep();


        //char check_ok=1;
        //for(int k=1;k<sizeof(usb_frame)-2;k+=2){
        //    if( 0xff & ~( ((uint8_t*)&usb_frame)[k] ^ ((uint8_t*)&usb_frame)[k+1] ) ){
        //        check_ok = 0;
        //        printf("\n..%02x %02x %d..\n",((uint8_t*)&usb_frame)[k] ,  ((uint8_t*)&usb_frame)[k+1],k)  ;
        //    }
        //}
        //uint8_t cs = 0;
        //for(int k=1;k<sizeof(usb_frame)-2;k++){
        //    cs += ((uint8_t*)&usb_frame)[k];
        //}
        //if(cs!=usb_frame.check_sum)
        //    check_ok = 0;


        //if(check_ok){
        //    printf(".");
        //    fflush(stdout);
        //} else {
        //   for(int k=0;k<sizeof(usb_frame);k++){
        //        printf("%02X ",((uint8_t*)&usb_frame)[k]);
        //    }
        //   printf("\n");
        //}

    }
    return 0;
}

