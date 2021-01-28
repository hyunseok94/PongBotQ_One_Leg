#ifndef CROBOT_H
#define CROBOT_H

#include "rbdl/rbdl.h"
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "servo_def.h"
#include "main.h"

//#define PI  3.14159265359
//#define PI2 6.28318530718
#define GRAVITY 9.81

//#define R2D 57.295779513
//#define D2R 0.0174532925

#define AXIS_X     0
#define AXIS_Y     1
#define AXIS_Z     2
#define AXIS_Roll  3
#define AXIS_Pitch 4
#define AXIS_Yaw   5

using Eigen::VectorXd;
using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef enum {
    CTRLMODE_NONE = 0,
    CTRLMODE_INITIALIZE, //1
    CTRLMODE_WALK_READY_HS, //2
    CTRLMODE_CYCLE_TEST_HS, //3
    CTRLMODE_JOYSTICK_HS, //4
    //    CTRLMODE_WALK_HS, //4
    //    CTRLMODE_POSTURE_GENERATION_HS, //5
    //    CTRLMODE_TEST_HS //6
} _CONTROL_MODE;

typedef enum {
    JOYMODE_NONE = 0,
    JOYMODE_HOME, //1
    JOYMODE_MOVE, //2
    JOYMODE_WALK //3
} _JOY_MODE;

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
    GOTO_TEST_POS_HS,
    GOTO_CYCLE_POS_HS,
    GOTO_JOYSTICK_POS_HS

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

    void Joint_Controller(void);
    void Cartesian_Controller(void);
    VectorNd Localization_Hip2Base_Pos_HS(VectorNd EP_pos_local_hip);
    void Mode_Change(void);
    void Controller_Change(void);
    void Init_Pos_Traj_HS(void); // Joint target
    void Home_Pos_Traj_HS(void); // End point target
    void Cycle_Test_Pos_Traj_HS(void);
    void Joystick_Pos_Traj_HS(void);
    void ComputeTorqueControl(void);
    void SF_EP_Traj_Gen_HS(double travel_time, VectorNd init_EP_pos, VectorNd goal_EP_pos);
    void coefficient_5thPoly(double *init_x, double *final_x, double tf, double *output);
    

    /*******Motor & Encoder Setting parameters**********/
    double Count2Deg(int Gear_Ratio, INT32 Count);
    double Count2DegDot(int Gear_Ratio, INT32 CountPerSec);
    double Count2Rad(int Gear_Ratio, INT32 Count);
    double Count2RadDot(int Gear_Ratio, INT32 CountPerSec);
    double Count2Rad_ABS(int _Resolution, INT32 Count);
    INT16 Tor2Cur(double OutputTorque, double _Kt, int _Gear, double _ratedCur);
    
    bool Encoder_Reset_Flag = true;
    int Gear[NUM_OF_ELMO] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
    double ratedCur[NUM_OF_ELMO] = {2.85, 2.85, 8.9, 8.9, 2.85, 2.85, 2.85, 2.85, 8.9, 8.9, 2.85, 2.85};
    double Kt[NUM_OF_ELMO] = {0.159, 0.159, 0.156, 0.156, 0.159, 0.159, 0.159, 0.159, 0.156, 0.156, 0.159, 0.159};
    int32_t Resolution[NUM_OF_ELMO] = {262144, 262144, 16384, 16384, 262144, 262144,  262144, 262144, 16384, 16384 , 262144, 262144}; //16384(2^14)

    //**************************** RBDL **************************//
    void setRobotModel(Model* getModel); //* get Robot Model
    void getRobotState(VectorNd BasePosOri, VectorNd BaseVel, VectorNd jointAngle, VectorNd jointVel);
    
    BASE base; //* coordinate of Body
    JOINT* joint; //* joints of the robot
    ENDPOINT FR, FL, RR, RL;
    
    int nDOF; //* number of DOFs of a robot
    RigidBodyDynamics::Model* m_pModel; //* URDF Model
    VectorNd RobotState = VectorNd::Zero(19);
    VectorNd RobotStatedot = VectorNd::Zero(18);
    VectorNd RobotState2dot = VectorNd::Zero(18);
    VectorNd BasePosOri = VectorNd::Zero(6);
    VectorNd BaseVel = VectorNd::Zero(6);

    Math::Quaternion QQ;

    MatrixNd M_term = MatrixNd::Zero(18, 18); //10=3*1+6+1
    VectorNd hatNonLinearEffects = VectorNd::Zero(18);
    VectorNd G_term = VectorNd::Zero(18);
    VectorNd C_term = VectorNd::Zero(18);
    VectorNd CTC_Torque = VectorNd::Zero(18);

    double L3_x = 0; //0.025516;
    double L3_y = 0; //0.0;
    double L3_z = 0.309; //0.304515;

    VectorNd EP_OFFSET_RL = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_RR = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_FL = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_FR = Vector3d(-L3_x, L3_y, -L3_z);
    
    Vector3d Originbase = Vector3d(0, 0, 0);

    MatrixNd J_BASE = MatrixNd::Zero(6, 9);   
    MatrixNd J_RL = MatrixNd::Zero(3, 18);
    MatrixNd J_RR = MatrixNd::Zero(3, 18);
    MatrixNd J_FL = MatrixNd::Zero(3, 18);
    MatrixNd J_FR = MatrixNd::Zero(3, 18);
    MatrixNd J_RL2 = MatrixNd::Zero(3, 3);
    MatrixNd J_RR2 = MatrixNd::Zero(3, 3);
    MatrixNd J_FL2 = MatrixNd::Zero(3, 3);
    MatrixNd J_FR2 = MatrixNd::Zero(3, 3);
    
    MatrixNd J_A = MatrixNd::Zero(12, 12);
    
     /***************Matrix for FK,IK*****************/
    //Variable  of Rear Left
    VectorNd FK(VectorNd q);
    VectorNd IK(VectorNd pos);
    MatrixNd Jac(VectorNd q);
    
    double l1 = 0.105;
    double l2 = 0.305;
    double l3 = 0.309; 
    
    VectorNd B_O = VectorNd::Zero(4);
    MatrixNd RL_I2B = MatrixNd::Zero(4, 4);
    MatrixNd RL_B2HR = MatrixNd::Zero(4, 4);
    MatrixNd RL_HR2HP = MatrixNd::Zero(4, 4);
    MatrixNd RL_HP2KN = MatrixNd::Zero(4, 4);
    MatrixNd RL_KN2T = MatrixNd::Zero(4, 4);

    MatrixNd RR_I2B = MatrixNd::Zero(4, 4);
    MatrixNd RR_B2HR = MatrixNd::Zero(4, 4);
    MatrixNd RR_HR2HP = MatrixNd::Zero(4, 4);
    MatrixNd RR_HP2KN = MatrixNd::Zero(4, 4);
    MatrixNd RR_KN2T = MatrixNd::Zero(4, 4);

    MatrixNd FL_I2B = MatrixNd::Zero(4, 4);
    MatrixNd FL_B2HR = MatrixNd::Zero(4, 4);
    MatrixNd FL_HR2HP = MatrixNd::Zero(4, 4);
    MatrixNd FL_HP2KN = MatrixNd::Zero(4, 4);
    MatrixNd FL_KN2T = MatrixNd::Zero(4, 4);

    MatrixNd FR_I2B = MatrixNd::Zero(4, 4);
    MatrixNd FR_B2HR = MatrixNd::Zero(4, 4);
    MatrixNd FR_HR2HP = MatrixNd::Zero(4, 4);
    MatrixNd FR_HP2KN = MatrixNd::Zero(4, 4);
    MatrixNd FR_KN2T = MatrixNd::Zero(4, 4);
    
    //***************
    VectorNd Raw_ABS_actual_joint_pos = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd Raw_Incre_actual_joint_pos = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd Raw_actual_joint_vel = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd actual_joint_pos = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd ABS_actual_joint_pos = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd online_ABS_actual_joint_pos = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd Incre_actual_joint_pos = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd tmp_Incre_actual_joint_pos = VectorNd::Zero(NUM_OF_ELMO);

    VectorNd actual_joint_vel = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd actual_EP_pos_local = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd actual_base_ori_local = VectorNd::Zero(3);
    VectorNd actual_base_ori_vel_local = VectorNd::Zero(3);
    VectorNd actual_base_acc_local = VectorNd::Zero(3);
    VectorNd tmp_actual_base_ori_vel_local=VectorNd::Zero(3);
    VectorNd pre_actual_base_ori_loacl=VectorNd::Zero(3);
    
//VectorNd tmp_actual_joint_vel_HS = VectorNd::Zero(9);
//    VectorNd init_joint_pos_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd target_joint_pos_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd target_joint_vel_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd goal_joint_pos_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd goal_joint_vel_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd joint_pos_err_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd joint_vel_err_HS = VectorNd::Zero(NUM_OF_ELMO);
    
    
    //VectorNd pre_actual_EP_pos_local_HS = VectorNd::Zero(NUM_OF_ELMO);
    //VectorNd actual_EP_vel_local_HS = VectorNd::Zero(NUM_OF_ELMO);
    //VectorNd tmp_actual_EP_vel_local_HS = VectorNd::Zero(9);
//    VectorNd actual_EP_pos_global_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd pre_actual_EP_pos_global_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd actual_EP_vel_global_HS = VectorNd::Zero(NUM_OF_ELMO);
//
//    VectorNd init_EP_pos_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd target_EP_pos_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd target_EP_vel_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd goal_EP_pos_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd goal_EP_vel_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd EP_pos_err_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd EP_vel_err_HS = VectorNd::Zero(NUM_OF_ELMO);

//    VectorNd actual_Base_pos_global_HS = VectorNd::Zero(NUM_OF_ELMO);
//    VectorNd Cart_Controller_HS = VectorNd::Zero(9);
//    VectorNd Joint_Controller_HS = VectorNd::Zero(9);

//    VectorNd tmp1_target_EP_pos_HS = VectorNd::Zero(3);
//    VectorNd tmp2_target_EP_pos_HS = VectorNd::Zero(3);

    VectorNd init_EP_pos = VectorNd::Zero(12);
    VectorNd target_EP_pos = VectorNd::Zero(12);
    VectorNd target_EP_vel = VectorNd::Zero(12);
    VectorNd actual_EP_pos = VectorNd::Zero(12);
    VectorNd goal_EP_pos = VectorNd::Zero(12);

    VectorNd joint_angle = VectorNd::Zero(12);
    
     unsigned int cnt_Control_change = 0;

    VectorNd abs_kp_joint_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd abs_kd_joint_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd abs_kp_EP_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd abs_kd_EP_HS = VectorNd::Zero(NUM_OF_ELMO);

    VectorNd kp_joint_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd kd_joint_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd kp_EP_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd kd_EP_HS = VectorNd::Zero(NUM_OF_ELMO);

    VectorNd init_kp_joint_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd init_kd_joint_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd init_kp_EP_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd init_kd_EP_HS = VectorNd::Zero(NUM_OF_ELMO);

    VectorNd target_kp_joint_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd target_kd_joint_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd target_kp_EP_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd target_kd_EP_HS = VectorNd::Zero(NUM_OF_ELMO);

    VectorNd goal_kp_joint_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd goal_kd_joint_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd goal_kp_EP_HS = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd goal_kd_EP_HS = VectorNd::Zero(NUM_OF_ELMO);
        
    double alpha = 0.0;
    bool stop_flag = false;

    // JoyStick
    double joy_vel_x = 0.0;
    double joy_vel_y = 0.0;
    double joy_vel_z = 0.0;

    //Walking Traj
    double init_x[3] = {0, 0, 0};
    double final_x[3] = {0, 0, 0};
    double foot_height_HS=0.0;
    double z_up[6];
    double z_down[6];
    VectorNd R = VectorNd::Zero(6);
    VectorNd P = VectorNd::Zero(6);
    MatrixNd A = MatrixNd::Zero(6, 6);
    unsigned int tsp_cnt_HS=250;
    unsigned int fsp_cnt_HS=100;
    double tsp_time_HS=tsp_cnt_HS*dt;
    double fsp_time_HS=fsp_cnt_HS*dt;
    double step_time_HS=tsp_time_HS+fsp_time_HS;
    double walk_time, t1, t2, dsp_t1, dsp_t2;
    bool walk_stop_flag=false;
    
      int ControlMode = 0;
    int CommandFlag = 0;
    int ControlMode_print = 0;
    bool Control_mode_flag = false;
    int JoyMode = 0;

    bool Mode_Change_flag = false;
    bool tmp_Mode_Change_flag = false;
    unsigned int cnt_mode_change = 0;

    double dt = 0.001;
    double cycle_time_HS = 0.0;
    unsigned int cnt_HS = 0;

    
private:
};

#endif /* CROBOT_H */
