#define RIGHT_DRIVE_MOTOR_BIT 0x01
#define LEFT_DRIVE_MOTOR_BIT  0x02
#define REAR_DEPTH_MOTOR_BIT   0x04
#define FRONT_DEPTH_MOTOR_BIT  0x08
#define REAR_TURN_MOTOR_BIT   0x10
#define FRONT_TURN_MOTOR_BIT  0x20
#define ALL_MOTORS_MASK       0x3F

void setMotors(unsigned int mask, 
		int DriveL, int DriveR, 
		int DiveF,  int DiveR, 
		int TurnF,  int TurnR);

//Example usage: 
//To set both drive motors to full power:
//
//  setMotors(RIGHT_DRIVE_MOTOR_BIT | LEFT_DRIVE_MOTOR_BIT, 255, 255, 0 0 0 0);
//
//  Note the 0's above are ignored due to the mask so the other motors will not
//  be adjusted
//

