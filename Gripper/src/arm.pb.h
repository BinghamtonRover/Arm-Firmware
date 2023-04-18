/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_ARM_PB_H_INCLUDED
#define PB_ARM_PB_H_INCLUDED
#include "utils/pb.h"
#include "coordinates.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _MotorDirection {
    MotorDirection_MOTOR_DIRECTION_UNDEFINED = 0,
    MotorDirection_UP = 1,
    MotorDirection_DOWN = 2,
    MotorDirection_LEFT = 3,
    MotorDirection_RIGHT = 4,
    MotorDirection_CLOCKWISE = 5,
    MotorDirection_COUNTER_CLOCKWISE = 6,
    MotorDirection_OPENING = 7,
    MotorDirection_CLOSING = 8,
    MotorDirection_NOT_MOVING = 9
} MotorDirection;

/* Struct definitions */
typedef struct _MotorData {
    bool is_moving;
    bool is_limit_switch_pressed;
    MotorDirection direction;
    int32_t current_step;
    int32_t target_step;
    float angle;
} MotorData;

typedef struct _MotorCommand {
    /* Debug control: Move by individual steps */
    int32_t move_steps;
    /* Precise control: Move by radians */
    float move_radians;
} MotorCommand;

/* TODO: Replace with ArmData2 */
typedef struct _ArmData {
    bool shoulder_limit_switch;
    bool elbow_limit_switch;
    float swivel_radians;
    float shoulder_radians;
    float elbow_radians;
    MotorDirection swivel_direction;
    MotorDirection shoulder_direction;
    MotorDirection elbow_direction;
} ArmData;

/* TODO: Replace with ArmCommand2 */
typedef struct _ArmCommand {
    /* General commands */
    bool stop;
    bool calibrate;
    /* Debug control: Move by individual steps */
    int32_t move_swivel_steps;
    int32_t move_shoulder_steps;
    int32_t move_elbow_steps;
    /* Precise control: Move by radians */
    float move_swivel_radians;
    float move_shoulder_radians;
    float move_elbow_radians;
    /* IK control to move motors by coordinates */
    float x;
    float y;
    float z;
    /* Workaround for not being able to (reliably) detect missing values */
    bool has_x;
    bool has_y;
    bool has_z;
    /* Needed for IK: If the wrist-lift moves, we need to re-calculate IK to keep the end-effector
 stationary. See /Arm/src/ik/README.md in the Arm-Firmware repository. */
    float gripper_lift_radians;
} ArmCommand;

typedef struct _ArmData2 {
    bool has_currentPosition;
    Coordinates currentPosition;
    bool has_targetPosition;
    Coordinates targetPosition;
    bool has_base;
    MotorData base;
    bool has_shoulder;
    MotorData shoulder;
    bool has_elbow;
    MotorData elbow;
} ArmData2;

typedef struct _ArmCommand2 {
    /* General commands */
    bool stop;
    bool calibrate;
    /* Move individual motors */
    bool has_swivel;
    MotorCommand swivel;
    bool has_shoulder;
    MotorCommand shoulder;
    bool has_elbow;
    MotorCommand elbow;
    /* Needed for IK: If the wrist-lift moves, we need to re-calculate IK to keep the end-effector
 stationary. See /Arm/src/ik/README.md in the Arm-Firmware repository. */
    bool has_gripper_lift;
    MotorCommand gripper_lift;
    bool has_ik_target;
    Coordinates ik_target;
} ArmCommand2;

typedef struct _GripperData {
    bool lift_limit_switch;
    bool pinch_limit_switch;
    float lift_radians;
    float rotate_radians;
    float pinch_radians;
    MotorDirection lift_direction;
    MotorDirection rotate_direction;
    MotorDirection pinch_direction;
} GripperData;

/* TODO: Precise motion */
typedef struct _GripperCommand {
    /* General commands */
    bool stop;
    bool calibrate;
    /* Debug control: Move by individual steps */
    int32_t move_lift_steps;
    int32_t move_rotate_steps;
    int32_t move_pinch_steps;
    /* Precise control: Move by radians */
    float move_lift_radians;
    float move_rotate_radians;
    float move_pinch_radians;
} GripperCommand;

typedef struct _GripperData2 {
    bool has_lift;
    MotorData lift;
    bool has_rotate;
    MotorData rotate;
    bool has_pinch;
    MotorData pinch;
} GripperData2;

typedef struct _GripperCommand2 {
    /* General commands */
    bool stop;
    bool calibrate;
    /* Move individual motors */
    bool has_lift;
    MotorCommand lift;
    bool has_rotate;
    MotorCommand rotate;
    bool has_pinch;
    MotorCommand pinch;
} GripperCommand2;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _MotorDirection_MIN MotorDirection_MOTOR_DIRECTION_UNDEFINED
#define _MotorDirection_MAX MotorDirection_NOT_MOVING
#define _MotorDirection_ARRAYSIZE ((MotorDirection)(MotorDirection_NOT_MOVING+1))

#define MotorData_direction_ENUMTYPE MotorDirection


#define ArmData_swivel_direction_ENUMTYPE MotorDirection
#define ArmData_shoulder_direction_ENUMTYPE MotorDirection
#define ArmData_elbow_direction_ENUMTYPE MotorDirection




#define GripperData_lift_direction_ENUMTYPE MotorDirection
#define GripperData_rotate_direction_ENUMTYPE MotorDirection
#define GripperData_pinch_direction_ENUMTYPE MotorDirection





/* Initializer values for message structs */
#define MotorData_init_default                   {0, 0, _MotorDirection_MIN, 0, 0, 0}
#define MotorCommand_init_default                {0, 0}
#define ArmData_init_default                     {0, 0, 0, 0, 0, _MotorDirection_MIN, _MotorDirection_MIN, _MotorDirection_MIN}
#define ArmCommand_init_default                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define ArmData2_init_default                    {false, Coordinates_init_default, false, Coordinates_init_default, false, MotorData_init_default, false, MotorData_init_default, false, MotorData_init_default}
#define ArmCommand2_init_default                 {0, 0, false, MotorCommand_init_default, false, MotorCommand_init_default, false, MotorCommand_init_default, false, MotorCommand_init_default, false, Coordinates_init_default}
#define GripperData_init_default                 {0, 0, 0, 0, 0, _MotorDirection_MIN, _MotorDirection_MIN, _MotorDirection_MIN}
#define GripperCommand_init_default              {0, 0, 0, 0, 0, 0, 0, 0}
#define GripperData2_init_default                {false, MotorData_init_default, false, MotorData_init_default, false, MotorData_init_default}
#define GripperCommand2_init_default             {0, 0, false, MotorCommand_init_default, false, MotorCommand_init_default, false, MotorCommand_init_default}
#define MotorData_init_zero                      {0, 0, _MotorDirection_MIN, 0, 0, 0}
#define MotorCommand_init_zero                   {0, 0}
#define ArmData_init_zero                        {0, 0, 0, 0, 0, _MotorDirection_MIN, _MotorDirection_MIN, _MotorDirection_MIN}
#define ArmCommand_init_zero                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define ArmData2_init_zero                       {false, Coordinates_init_zero, false, Coordinates_init_zero, false, MotorData_init_zero, false, MotorData_init_zero, false, MotorData_init_zero}
#define ArmCommand2_init_zero                    {0, 0, false, MotorCommand_init_zero, false, MotorCommand_init_zero, false, MotorCommand_init_zero, false, MotorCommand_init_zero, false, Coordinates_init_zero}
#define GripperData_init_zero                    {0, 0, 0, 0, 0, _MotorDirection_MIN, _MotorDirection_MIN, _MotorDirection_MIN}
#define GripperCommand_init_zero                 {0, 0, 0, 0, 0, 0, 0, 0}
#define GripperData2_init_zero                   {false, MotorData_init_zero, false, MotorData_init_zero, false, MotorData_init_zero}
#define GripperCommand2_init_zero                {0, 0, false, MotorCommand_init_zero, false, MotorCommand_init_zero, false, MotorCommand_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define MotorData_is_moving_tag                  1
#define MotorData_is_limit_switch_pressed_tag    2
#define MotorData_direction_tag                  3
#define MotorData_current_step_tag               4
#define MotorData_target_step_tag                5
#define MotorData_angle_tag                      6
#define MotorCommand_move_steps_tag              1
#define MotorCommand_move_radians_tag            2
#define ArmData_shoulder_limit_switch_tag        1
#define ArmData_elbow_limit_switch_tag           2
#define ArmData_swivel_radians_tag               3
#define ArmData_shoulder_radians_tag             4
#define ArmData_elbow_radians_tag                5
#define ArmData_swivel_direction_tag             6
#define ArmData_shoulder_direction_tag           7
#define ArmData_elbow_direction_tag              8
#define ArmCommand_stop_tag                      1
#define ArmCommand_calibrate_tag                 2
#define ArmCommand_move_swivel_steps_tag         3
#define ArmCommand_move_shoulder_steps_tag       4
#define ArmCommand_move_elbow_steps_tag          5
#define ArmCommand_move_swivel_radians_tag       6
#define ArmCommand_move_shoulder_radians_tag     7
#define ArmCommand_move_elbow_radians_tag        8
#define ArmCommand_x_tag                         9
#define ArmCommand_y_tag                         10
#define ArmCommand_z_tag                         11
#define ArmCommand_has_x_tag                     12
#define ArmCommand_has_y_tag                     13
#define ArmCommand_has_z_tag                     14
#define ArmCommand_gripper_lift_radians_tag      15
#define ArmData2_currentPosition_tag             1
#define ArmData2_targetPosition_tag              2
#define ArmData2_base_tag                        3
#define ArmData2_shoulder_tag                    4
#define ArmData2_elbow_tag                       5
#define ArmCommand2_stop_tag                     1
#define ArmCommand2_calibrate_tag                2
#define ArmCommand2_swivel_tag                   3
#define ArmCommand2_shoulder_tag                 4
#define ArmCommand2_elbow_tag                    5
#define ArmCommand2_gripper_lift_tag             6
#define ArmCommand2_ik_target_tag                7
#define GripperData_lift_limit_switch_tag        1
#define GripperData_pinch_limit_switch_tag       2
#define GripperData_lift_radians_tag             3
#define GripperData_rotate_radians_tag           4
#define GripperData_pinch_radians_tag            5
#define GripperData_lift_direction_tag           6
#define GripperData_rotate_direction_tag         7
#define GripperData_pinch_direction_tag          8
#define GripperCommand_stop_tag                  1
#define GripperCommand_calibrate_tag             2
#define GripperCommand_move_lift_steps_tag       3
#define GripperCommand_move_rotate_steps_tag     4
#define GripperCommand_move_pinch_steps_tag      5
#define GripperCommand_move_lift_radians_tag     6
#define GripperCommand_move_rotate_radians_tag   7
#define GripperCommand_move_pinch_radians_tag    8
#define GripperData2_lift_tag                    1
#define GripperData2_rotate_tag                  2
#define GripperData2_pinch_tag                   3
#define GripperCommand2_stop_tag                 1
#define GripperCommand2_calibrate_tag            2
#define GripperCommand2_lift_tag                 3
#define GripperCommand2_rotate_tag               4
#define GripperCommand2_pinch_tag                5

/* Struct field encoding specification for nanopb */
#define MotorData_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     is_moving,         1) \
X(a, STATIC,   SINGULAR, BOOL,     is_limit_switch_pressed,   2) \
X(a, STATIC,   SINGULAR, UENUM,    direction,         3) \
X(a, STATIC,   SINGULAR, INT32,    current_step,      4) \
X(a, STATIC,   SINGULAR, INT32,    target_step,       5) \
X(a, STATIC,   SINGULAR, FLOAT,    angle,             6)
#define MotorData_CALLBACK NULL
#define MotorData_DEFAULT NULL

#define MotorCommand_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, INT32,    move_steps,        1) \
X(a, STATIC,   SINGULAR, FLOAT,    move_radians,      2)
#define MotorCommand_CALLBACK NULL
#define MotorCommand_DEFAULT NULL

#define ArmData_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     shoulder_limit_switch,   1) \
X(a, STATIC,   SINGULAR, BOOL,     elbow_limit_switch,   2) \
X(a, STATIC,   SINGULAR, FLOAT,    swivel_radians,    3) \
X(a, STATIC,   SINGULAR, FLOAT,    shoulder_radians,   4) \
X(a, STATIC,   SINGULAR, FLOAT,    elbow_radians,     5) \
X(a, STATIC,   SINGULAR, UENUM,    swivel_direction,   6) \
X(a, STATIC,   SINGULAR, UENUM,    shoulder_direction,   7) \
X(a, STATIC,   SINGULAR, UENUM,    elbow_direction,   8)
#define ArmData_CALLBACK NULL
#define ArmData_DEFAULT NULL

#define ArmCommand_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     stop,              1) \
X(a, STATIC,   SINGULAR, BOOL,     calibrate,         2) \
X(a, STATIC,   SINGULAR, INT32,    move_swivel_steps,   3) \
X(a, STATIC,   SINGULAR, INT32,    move_shoulder_steps,   4) \
X(a, STATIC,   SINGULAR, INT32,    move_elbow_steps,   5) \
X(a, STATIC,   SINGULAR, FLOAT,    move_swivel_radians,   6) \
X(a, STATIC,   SINGULAR, FLOAT,    move_shoulder_radians,   7) \
X(a, STATIC,   SINGULAR, FLOAT,    move_elbow_radians,   8) \
X(a, STATIC,   SINGULAR, FLOAT,    x,                 9) \
X(a, STATIC,   SINGULAR, FLOAT,    y,                10) \
X(a, STATIC,   SINGULAR, FLOAT,    z,                11) \
X(a, STATIC,   SINGULAR, BOOL,     has_x,            12) \
X(a, STATIC,   SINGULAR, BOOL,     has_y,            13) \
X(a, STATIC,   SINGULAR, BOOL,     has_z,            14) \
X(a, STATIC,   SINGULAR, FLOAT,    gripper_lift_radians,  15)
#define ArmCommand_CALLBACK NULL
#define ArmCommand_DEFAULT NULL

#define ArmData2_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  currentPosition,   1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  targetPosition,    2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  base,              3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  shoulder,          4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  elbow,             5)
#define ArmData2_CALLBACK NULL
#define ArmData2_DEFAULT NULL
#define ArmData2_currentPosition_MSGTYPE Coordinates
#define ArmData2_targetPosition_MSGTYPE Coordinates
#define ArmData2_base_MSGTYPE MotorData
#define ArmData2_shoulder_MSGTYPE MotorData
#define ArmData2_elbow_MSGTYPE MotorData

#define ArmCommand2_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     stop,              1) \
X(a, STATIC,   SINGULAR, BOOL,     calibrate,         2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  swivel,            3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  shoulder,          4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  elbow,             5) \
X(a, STATIC,   OPTIONAL, MESSAGE,  gripper_lift,      6) \
X(a, STATIC,   OPTIONAL, MESSAGE,  ik_target,         7)
#define ArmCommand2_CALLBACK NULL
#define ArmCommand2_DEFAULT NULL
#define ArmCommand2_swivel_MSGTYPE MotorCommand
#define ArmCommand2_shoulder_MSGTYPE MotorCommand
#define ArmCommand2_elbow_MSGTYPE MotorCommand
#define ArmCommand2_gripper_lift_MSGTYPE MotorCommand
#define ArmCommand2_ik_target_MSGTYPE Coordinates

#define GripperData_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     lift_limit_switch,   1) \
X(a, STATIC,   SINGULAR, BOOL,     pinch_limit_switch,   2) \
X(a, STATIC,   SINGULAR, FLOAT,    lift_radians,      3) \
X(a, STATIC,   SINGULAR, FLOAT,    rotate_radians,    4) \
X(a, STATIC,   SINGULAR, FLOAT,    pinch_radians,     5) \
X(a, STATIC,   SINGULAR, UENUM,    lift_direction,    6) \
X(a, STATIC,   SINGULAR, UENUM,    rotate_direction,   7) \
X(a, STATIC,   SINGULAR, UENUM,    pinch_direction,   8)
#define GripperData_CALLBACK NULL
#define GripperData_DEFAULT NULL

#define GripperCommand_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     stop,              1) \
X(a, STATIC,   SINGULAR, BOOL,     calibrate,         2) \
X(a, STATIC,   SINGULAR, INT32,    move_lift_steps,   3) \
X(a, STATIC,   SINGULAR, INT32,    move_rotate_steps,   4) \
X(a, STATIC,   SINGULAR, INT32,    move_pinch_steps,   5) \
X(a, STATIC,   SINGULAR, FLOAT,    move_lift_radians,   6) \
X(a, STATIC,   SINGULAR, FLOAT,    move_rotate_radians,   7) \
X(a, STATIC,   SINGULAR, FLOAT,    move_pinch_radians,   8)
#define GripperCommand_CALLBACK NULL
#define GripperCommand_DEFAULT NULL

#define GripperData2_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  lift,              1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  rotate,            2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  pinch,             3)
#define GripperData2_CALLBACK NULL
#define GripperData2_DEFAULT NULL
#define GripperData2_lift_MSGTYPE MotorData
#define GripperData2_rotate_MSGTYPE MotorData
#define GripperData2_pinch_MSGTYPE MotorData

#define GripperCommand2_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     stop,              1) \
X(a, STATIC,   SINGULAR, BOOL,     calibrate,         2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  lift,              3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  rotate,            4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  pinch,             5)
#define GripperCommand2_CALLBACK NULL
#define GripperCommand2_DEFAULT NULL
#define GripperCommand2_lift_MSGTYPE MotorCommand
#define GripperCommand2_rotate_MSGTYPE MotorCommand
#define GripperCommand2_pinch_MSGTYPE MotorCommand

extern const pb_msgdesc_t MotorData_msg;
extern const pb_msgdesc_t MotorCommand_msg;
extern const pb_msgdesc_t ArmData_msg;
extern const pb_msgdesc_t ArmCommand_msg;
extern const pb_msgdesc_t ArmData2_msg;
extern const pb_msgdesc_t ArmCommand2_msg;
extern const pb_msgdesc_t GripperData_msg;
extern const pb_msgdesc_t GripperCommand_msg;
extern const pb_msgdesc_t GripperData2_msg;
extern const pb_msgdesc_t GripperCommand2_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define MotorData_fields &MotorData_msg
#define MotorCommand_fields &MotorCommand_msg
#define ArmData_fields &ArmData_msg
#define ArmCommand_fields &ArmCommand_msg
#define ArmData2_fields &ArmData2_msg
#define ArmCommand2_fields &ArmCommand2_msg
#define GripperData_fields &GripperData_msg
#define GripperCommand_fields &GripperCommand_msg
#define GripperData2_fields &GripperData2_msg
#define GripperCommand2_fields &GripperCommand2_msg

/* Maximum encoded size of messages (where known) */
#define ArmCommand2_size                         111
#define ArmCommand_size                          78
#define ArmData2_size                            175
#define ArmData_size                             25
#define GripperCommand2_size                     58
#define GripperCommand_size                      52
#define GripperData2_size                        105
#define GripperData_size                         25
#define MotorCommand_size                        16
#define MotorData_size                           33

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
