
/* struct to store system/vehicle status */

struct status_struct {
  int speed;                  // in mph
  int gear;                   // 0 -> park, 1 -> reverse, 2 -> neutral, 3 -> drive
  bool cruise_control_active;
  bool wipers_on;
  bool cars_in_front;
  bool cars_in_back;
  bool cars_on_left;
  bool cars_on_right;
  int lane_warning;           // 0 = warning on left, 1 = on right
  int headlights;             // 0 = off, 1 = on, 2 = highbeams
  bool rear_view;
  int lane;
  int num_lanes;
  bool leftTurn;
  bool rightTurn;
};


