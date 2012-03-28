
struct Position {
	float depth;
	float x;
	float y;
};

struct Motors {
	float mainR;
	float mainL;
	float depthF;
	float depthR;
	float turnF;
	float turnR;
};

struct Rotation {
	float roll;
	float pitch;
	float yaw;
};

struct Submarine {
	Motors motor;
	Position velocity;
	Position position;
	Rotation rotation;
	Rotation rotationalVelocity;

	const static float motorPowerConst = .4;
	const static float frictionConst = -.5;
	const static float turnRotationalPowerConst = .03;
	const static float mainRotationalPowerConst = .01;
	const static float rotationalFrictionConst = -.5;
};

const double weight = -10;
const double bouyancy = .15;
