#ifndef BOATPROTOCOL_H
#define BOATPROTOCOL_H

enum OPCODE {
  THROTTLE,
  STEERING,
  SERVO,
  KEEPALIVE,
  KILL,
};

enum STEERING {
  LEFT,
  RIGHT,
  NONE,
};

#endif
