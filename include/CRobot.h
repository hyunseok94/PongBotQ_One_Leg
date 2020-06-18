#ifndef CROBOT_H
#define CROBOT_H

#include "rbdl/rbdl.h"
#include <rbdl/addons/urdfreader/urdfreader.h>

//#define PI  3.14159265359
//#define PI2 6.28318530718
#define GRAVITY 9.81

//#define R2D 57.295779513
//#define D2R 0.0174532925

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef enum {
    CTRLMODE_NONE = 0,
    CTRLMODE_INITIALIZE,
    CTRLMODE_WALK_READY_HS,
    CTRLMODE_WALK_HS,
    CTRLMODE_POSTURE_GENERATION_HS,
    CTRLMODE_TEST_HS
} _CONTROL_MODE;

typedef enum {
    NO_ACT,
    EXIT_PROGRAM,
    SET_MOTOR_GAIN,
    SET_CURRENT_GAIN,
    LOAD_PARAMETER,
    SAVE_PARAMETER,
    SET_JOINT_PARAMETER,
    GET_JOINT_PARAMETER,
    SET_BOARD_PARAMETER,
    GET_BOARD_PARAMETER,
    PRINT_JOINT_PARAMETER,
    CHECK_DEVICE,
    GAIN_SETTING,
    ENABLE_FET,
    ENABLE_FET_EACH,
    DISABLE_FET,
    DISABLE_FET_EACH,
    RUN_CMD,
    RUN_CMD_EACH,
    STOP_CMD,
    STOP_CMD_EACH,
    GOTO_LIMIT_POS,
    GOTO_LIMIT_POS_UPPER_ALL,
    GOTO_LIMIT_POS_LOWER_ALL,
    GOTO_LIMIT_POS_ALL,
    ENCODER_ZERO,
    ENCODER_ZERO_EACH,
    SAVE_ZMP_INIT_POS,
    SET_ENCODER_RESOLUTION,
    SET_DEADZONE,
    SET_JAMPWM_FAULT,
    SET_MAX_VEL_ACC,
    SET_CONTROL_MODE,
    SET_HOME_SEARCH_PARAMETER,
    SET_HOME_MAX_VEL_ACC,
    SET_POSITION_LIMIT,
    SET_ERROR_BOUND,
    REQUEST_PARAMETER,
    POSITION_LIMIT_ONOFF,
    BEEP,
    JOINT_REF_SET_RELATIVE,
    JOINT_REF_SET_ABS,
    SET_FT_PARAMETER,
    GET_FT_PARAMETER,
    NULL_FT_SENSOR,
    NULL_WRIST_FT_SENSOR,
    NULL_FOOT_ANGLE_SENSOR,
    NULL_IMU_SENSOR,
    SET_IMU_OFFSET,
    PRINT_FT_PARAMETER,
    SET_IMU_PARAMETER,
    GET_IMU_PARAMETER,
    PRINT_IMU_PARAMETER,
    SET_DAMPING_GAIN,
    SET_DSP_GAIN,
    GOTO_WALK_READY_POS,
    GOTO_HOME_POS,
    START_ZMP_INITIALIZATION,
    STOP_ZMP_INITIALIZATION,
    GOTO_FORWARD,
    STOP_WALKING,
    SET_MOCAP,
    C_CONTROL_MODE,
    P_CONTROL_MODE,
    GRIP_ON,
    GRIP_OFF,
    GRIP_STOP,
    DEMO_FLAG,
    TEST_FUNCTION,
    DEMO_GRASP, // jungho77
    SET_PREDEF_WALK, // jungho77
    INIT_WB_MOCAP, // by Inhyeok
    DEMO_CONTROL_OFF_POS,
    DEMO_CONTROL_ON_POS,
    RBT_ON_MODE, //CDI
    RBT_OFF_MODE, //CDI
    CCTM_ON,
    CCTM_OFF,
    JUMP_ONESTEP, // BKCho
    NOMAL_TROT_WALKING,
    FLYING_TROT_RUNNING,
    TORQUE_OFF,
    NO_ACT_WITH_CTC,
    GOTO_INIT_POS_HS,
    GOTO_WALK_READY_POS_HS,
    GOTO_TROT_POS_HS,
    GOTO_WALK_POS_HS,
    GOTO_POSTURE_GENERATION_HS,
    GOTO_TEST_POS_HS

} _COMMAND_FLAG;

typedef enum {
    TEST_NONE = 0,
    TEST_STATIC_POSE,
    TEST_STATIC_ORI,
    TEST_CYCLE_POSE
} _TEST_MODE;

typedef struct Base //coordinate of Base
{
    //* Current information
    double currentX, currentXvel, currentXacc;
    double currentY, currentYvel, currentYacc;
    double currentZ, currentZvel, currentZacc;
    double currentRoll, currentRollvel, currentRollacc;
    double currentPitch, currentPitchvel, currenPitchacc;
    double currentYaw, currentYawvel, currenYawacc;

    //* Reference information
    double refX, refXvel, refXacc;
    double refY, refYvel, refYacc;
    double refZ, refZvel, refZacc;
    double refRoll, refRollvel, refRollacc;
    double refPitch, refPitchvel, refPitchacc;
    double refYaw, refYawvel, refYawacc;

    VectorNd current; //* Current values
    VectorNd pre; //* Pre values
    VectorNd vel; //* vel values
    VectorNd prevel;
    VectorNd acc;
    VectorNd preacc;

    VectorNd refCoM; //* Current values
    VectorNd prerefCoM; //* Pre values
    VectorNd refvel;
    VectorNd prerefvel; //* Pre values
    VectorNd refacc;

    int ID;

} BASE;

typedef struct Joint {
    //* Current information
    double currentAngle;
    double currentVel;
    double currentAcc;
    double torque;

    //* Reference information
    double refAngle;
    double refVel;
    double refAcc;

    //* Control P I D gain
    double gain_P;
    double gain_I;
    double gain_D;
} JOINT;

typedef struct FTSennsor {
    double Fx, Fy, Fz;
    double Mx, My, Mz;
} FTS;

typedef struct EndPoint {
    //ORI orientation;
    //POS pos;
    FTS ftSensor;

    //RigidBodyDynamics::Math::VectorNd current; //* Current values
    VectorNd current; //* Current values
    VectorNd pre; //* Pre values
    VectorNd vel; //* vel values
    VectorNd prevel;
    VectorNd acc;
    VectorNd preacc;

    VectorNd refpos; //* Current values
    VectorNd prerefpos; //* Pre values
    VectorNd refvel;
    VectorNd prerefvel; //* Pre values
    VectorNd refacc;
    VectorNd Target;
    Matrix3d T_matrix;

    int ID;
} ENDPOINT;

class CRobot {
public:
    //Functions
    CRobot();
    CRobot(const CRobot& orig);
    virtual ~CRobot();
    void setRobotModel(Model* getModel); //* get Robot Model
    void getCurrentJoint(VectorNd Angle, VectorNd Vel); // get Current Joint
    void getRobotState(VectorNd BasePosOri, VectorNd BaseVel, VectorNd jointAngle, VectorNd jointVel);
    void ComputeTorqueControl();
    VectorNd FK1(VectorNd q);
    VectorNd IK1(VectorNd EP);
    
    BASE base; //* coordinate of Body
    JOINT* joint; //* joints of the robot
    ENDPOINT FR, FL, RR, RL, front_body;
    int nDOF; //* number of DOFs of a robot

    int ControlMode;
    int CommandFlag;
    int sub_ctrl_flag;
    int TestMode;

    RigidBodyDynamics::Model* m_pModel; //* URDF Model
    //RigidBodyDynamics::Math::VectorNd RobotState;
    VectorNd RobotState;
    VectorNd RobotStatedot;
    VectorNd RobotState2dot;
    VectorNd BasePosOri;
    VectorNd BaseVel;
    VectorNd JointAngle;
    VectorNd JointVel;
    
    MatrixNd M_term = MatrixNd::Zero(10, 10); //10=3*1+6+1
    //VectorNd hatNonLinearEffects = VectorNd::Zero(10);
    //VectorNd G_term = VectorNd::Zero(10);
    //VectorNd C_term = VectorNd::Zero(10);
    //VectorNd CTC_Torque = VectorNd::Zero(10);

private:
};

#endif /* CROBOT_H */
