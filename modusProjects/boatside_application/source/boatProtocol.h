#ifndef BOATPROTOCOL_H
#define BOATPROTOCOL_H

typedef enum {
	THROTTLE,
	STEERING,
	SERVO,
	KEEPALIVE,
	KILL,
	LEFT,
	RIGHT,
	NONE,
} opcode_t;

#endif
