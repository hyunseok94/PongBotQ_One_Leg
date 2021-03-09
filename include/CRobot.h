#ifndef CROBOT_H
#define CROBOT_H

#include "rbdl/rbdl.h"
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "servo_def.h"
#include "main.h"

#include "osqp.h"

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

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef enum {
    MODE_SIMULATION, MODE_ACTUAL_ROBOT
} _MODE;

typedef enum {
    CTRLMODE_NONE,
    CTRLMODE_HOME_POS,
    CTRLMODE_WALK_READY,
    CTRLMODE_TROT_WALKING,
    CTRLMODE_FLYING_TROT,
    CTRLMODE_PRONK_JUMP,
    CTRLMODE_TEST,
    CTRLMODE_TORQUE_OFF,
    CTRLMODE_STAIR,
    CTRLMODE_SLOW_WALK_HS
} _CONTROL_MODE;

typedef enum {
    NO_ACT,
    GOTO_HOME_POS,
    GOTO_WALK_READY_POS,
    NOMAL_TROT_WALKING,
    FLYING_TROT_RUNNING,
    TORQUE_OFF,
    PRONK_JUMP,
    TEST_FLAG,
    STAIR_WALKING,
    GOTO_SLOW_WALK_POS_HS
} _COMMAND_FLAG;

typedef enum {
    TW_PHASE_INIT,
    TW_DSP_RLFR_FIRST,
    TW_FSP,
    TW_DSP_RRFL,
    TW_DSP_RLFR,
    TW_FSP_FINAL

} _TW_PHASE;

typedef enum {
    CONTACT_OFF, CONTACT_ON

} _CONTACT_INFO;

typedef enum {
    QP_CON, MPC_CON
} _WB_CON;

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
    FTS ftSensor;
    RigidBodyDynamics::Math::VectorNd current; //* Current values
    RigidBodyDynamics::Math::VectorNd pre; //* Pre values
    RigidBodyDynamics::Math::VectorNd vel; //* vel values
    RigidBodyDynamics::Math::VectorNd prevel;
    RigidBodyDynamics::Math::VectorNd acc;
    RigidBodyDynamics::Math::VectorNd preacc;
    RigidBodyDynamics::Math::VectorNd refpos; //* Current values
    RigidBodyDynamics::Math::VectorNd prerefpos; //* Pre values
    RigidBodyDynamics::Math::VectorNd refvel;
    RigidBodyDynamics::Math::VectorNd prerefvel; //* Pre values
    RigidBodyDynamics::Math::VectorNd refacc;
    RigidBodyDynamics::Math::VectorNd Target;
    RigidBodyDynamics::Math::Matrix3d T_matrix;
    int ID;
} ENDPOINT;

class CRobot {
public:
    /*******Motor & Encoder Setting parameters**********/
    double Count2Deg(int Gear_Ratio, INT32 Count);
    double Count2DegDot(int Gear_Ratio, INT32 CountPerSec);
    double Count2Rad(int Gear_Ratio, INT32 Count);
    double Count2RadDot(int Gear_Ratio, INT32 CountPerSec);
    double Count2Rad_ABS(int _Resolution, INT32 Count);
    INT16 Tor2Cur(double OutputTorque, double _Kt, int _Gear, double _ratedCur);
    
    int Low_Gear[NUM_OF_ELMO] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
    double Low_ratedCur[NUM_OF_ELMO] = {2.85, 2.85, 8.9, 8.9, 2.85, 2.85, 2.85, 2.85, 8.9, 8.9, 2.85, 2.85};
    double Low_Kt[NUM_OF_ELMO] = {0.159, 0.159, 0.156, 0.156, 0.159, 0.159, 0.159, 0.159, 0.156, 0.156, 0.159, 0.159};
    int32_t Low_Resolution[NUM_OF_ELMO] = {262144, 262144, 16384, 16384, 262144, 262144,  262144, 262144, 16384, 16384 , 262144, 262144}; //16384(2^14)

    //**********************Functions***************//
    CRobot();
    CRobot(const CRobot& orig);
    virtual ~CRobot();
    void setRobotModel(Model* getModel); //* get Robot Model
    void getRobotState(VectorNd BasePosOri, VectorNd BaseVel, VectorNd jointAngle, VectorNd jointVel);
    void StateUpdate(void);
    void ComputeTorqueControl(void);
    void Torque_off(void);
    void FK2(void);
    VectorNd IK1(VectorNd EP);
    VectorNd Get_COM(VectorNd base, VectorNd q);
    void coefficient_5thPoly(double *init_x, double *final_x, double tf, double *output);
    void Get_act_com(void);   
    void WalkReady_Pos_Traj(void);
    void set_simul_para(void);
    void set_act_robot_para(void);
    void QP_Con_Init(void);
    void QP_process(void);
    VectorNd FK(VectorNd q);
    VectorNd IK(VectorNd pos);
    void ComputeTorqueControl_HS(void);
    //void Joint_Controller(void);
    //void Cartesian_Controller(void);   
    //void Controller_Change(void);
    //void Init_Pos_Traj_HS(void); // Joint target
   // void Home_Pos_Traj_HS(void); // End point target
    //void SF_EP_Traj_Gen_HS(double travel_time, VectorNd init_EP_pos, VectorNd goal_EP_pos);
    
    //*************************[Constructor]****************************//
    int Mode;
    int WH_Mode;
    int ControlMode = 0;
    int CommandFlag = 0;
    
    //*************************[Time & Counts]****************************//
    double dt = 0.001;
    int move_cnt = 0;
    unsigned int wr_cnt = 0;
    double dsp_time;
    double fsp_time;
    double step_time; // = dsp_time + fsp_time;
    int dsp_cnt;
    int fsp_cnt;
    int step_cnt; // = dsp_cnt + fsp_cnt;
        
     //**************************** [RBDL Paramter] **************************//
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
    VectorNd JointAngle = VectorNd::Zero(12);
    VectorNd JointVel = VectorNd::Zero(12);
    Math::Quaternion QQ;
    
    MatrixNd M_term = MatrixNd::Zero(18, 18); //10=3*1+6+1
    VectorNd hatNonLinearEffects = VectorNd::Zero(18);
    VectorNd G_term = VectorNd::Zero(18);
    VectorNd C_term = VectorNd::Zero(18);
    
    double L3_x = 0; //0.025516;
    double L3_y = 0; //0.0;
    double L3_z = 0.309; //0.304515;

    VectorNd EP_OFFSET_RL = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_RR = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_FL = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_FR = Vector3d(-L3_x, L3_y, -L3_z);
    
    Vector3d Originbase = Vector3d(0, 0, 0);

    MatrixNd J_BASE = MatrixNd::Zero(6, 18);   
    MatrixNd J_RL = MatrixNd::Zero(3, 18);
    MatrixNd J_RR = MatrixNd::Zero(3, 18);
    MatrixNd J_FL = MatrixNd::Zero(3, 18);
    MatrixNd J_FR = MatrixNd::Zero(3, 18);
    MatrixNd J_RL2 = MatrixNd::Zero(3, 3);
    MatrixNd J_RR2 = MatrixNd::Zero(3, 3);
    MatrixNd J_FL2 = MatrixNd::Zero(3, 3);
    MatrixNd J_FR2 = MatrixNd::Zero(3, 3);
    
    MatrixNd J_A = MatrixNd::Zero(18, 18);
    
     /***************Matrix for FK,IK*****************/
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
    
    //***************** IMU Parameters *********************//
    double IMURoll = 0.0;
    double IMUPitch = 0.0;
    double IMUYaw = 0.0;
    double init_IMUYaw = 0.0;
    double IMURoll_dot = 0.0;
    double IMUPitch_dot = 0.0;
    double IMUYaw_dot = 0.0;
    double IMUAccX = 0.0;
    double IMUAccY = 0.0;
    double IMUAccZ = 0.0;
    //***************Joint Parameters **********************//
    VectorNd Low_ABS_actual_pos = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd Low_Incre_actual_pos = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd Low_actual_vel = VectorNd::Zero(NUM_OF_ELMO);
    
    VectorNd ABS_actual_pos = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd Incre_actual_pos = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd tmp_Incre_actual_joint_pos = VectorNd::Zero(NUM_OF_ELMO);
    
    VectorNd actual_pos = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd actual_vel = VectorNd::Zero(NUM_OF_ELMO);
    VectorNd actual_acc = VectorNd::Zero(NUM_OF_ELMO);
    
    //***************Base Parameters **********************//
    VectorNd base_ori_quat = VectorNd::Zero(4); // roll,pitch,yaw

    VectorNd base_vel = VectorNd::Zero(3);
    VectorNd base_ori = VectorNd::Zero(3);
    VectorNd base_ori_dot = VectorNd::Zero(3);
    
    VectorNd actual_base_ori_local = VectorNd::Zero(3);
    VectorNd actual_base_ori_vel_local = VectorNd::Zero(3);
    VectorNd actual_base_acc_local = VectorNd::Zero(3);
    VectorNd tmp_actual_base_ori_vel_local=VectorNd::Zero(3);
    VectorNd pre_actual_base_ori_loacl=VectorNd::Zero(3);
    
    VectorNd actual_quaternion = VectorNd::Zero(4);
    VectorNd actual_base_ori_local2 = VectorNd::Zero(3);
    VectorNd actual_base_ori_vel_local2 = VectorNd::Zero(3);
    VectorNd actual_base_acc_local2 = VectorNd::Zero(3);
        
    //*************************** Flags ************************//
    bool moving_done_flag = true;
    bool move_stop_flag = false;
    bool pre_sub_ctrl_flag_HS = false;
    bool walk_ready_moving_done_flag = false;
    bool CP_con_onoff_flag;
    bool Slope_con_onoff_flag;
    bool gain_scheduling_flag;
    
    //****************** Move Paramter ****************//
    double x_moving_speed = 0.0;
    double y_moving_speed = 0.0;
    double pre_x_moving_speed = 0.0;
    double tmp_x_moving_speed = 0.0;
    double tmp_y_moving_speed = 0.0;
    VectorNd tmp_base_ori = VectorNd::Zero(3);
    double speed_x_HS = 0.0;
    double speed_y_HS = 0.0;
    double speed_yaw_HS = 0.0;
    
    //******************WalkReady Paramters *****************//
    double walk_ready_time = 2;
    unsigned int walk_ready_cnt = 2000;
    
    int contact_num = 4;
    double pos_alpha = 0;
    double vel_alpha = 0;
    VectorNd base_offset = VectorNd::Zero(3, 1);
    
    double foot_height = 0.0;
    double com_height;
    
    double lpf_tar_pitch_ang = 0.0;
    double fc_weight = 0.0;
    
    //*************[Joint Paramters Init] ************//
    VectorNd init_target_pos = VectorNd::Zero(12);
    VectorNd target_pos = VectorNd::Zero(12);
    VectorNd target_vel = VectorNd::Zero(12);
    VectorNd target_acc = VectorNd::Zero(12);
        
    VectorNd tar_RL_q_dot = VectorNd::Zero(3);
    VectorNd tar_RR_q_dot = VectorNd::Zero(3);
    VectorNd tar_FL_q_dot = VectorNd::Zero(3);
    VectorNd tar_FR_q_dot = VectorNd::Zero(3);

    VectorNd act_RL_q_dot = VectorNd::Zero(3);
    VectorNd act_RR_q_dot = VectorNd::Zero(3);
    VectorNd act_FL_q_dot = VectorNd::Zero(3);
    VectorNd act_FR_q_dot = VectorNd::Zero(3);
    //***************End point Parameters **********************//
    
    VectorNd RL_foot_pos_local_offset = VectorNd::Zero(3);
    VectorNd RR_foot_pos_local_offset = VectorNd::Zero(3);
    VectorNd FL_foot_pos_local_offset = VectorNd::Zero(3);
    VectorNd FR_foot_pos_local_offset = VectorNd::Zero(3);
        
    VectorNd base2hip_pos = VectorNd::Zero(12);
    VectorNd RL_base2hip_pos = VectorNd::Zero(3);
    VectorNd RR_base2hip_pos = VectorNd::Zero(3);
    VectorNd FL_base2hip_pos = VectorNd::Zero(3);
    VectorNd FR_base2hip_pos = VectorNd::Zero(3);
    
    VectorNd act_RL_foot_pos_local = VectorNd::Zero(3);
    VectorNd act_RR_foot_pos_local = VectorNd::Zero(3);
    VectorNd act_FL_foot_pos_local = VectorNd::Zero(3);
    VectorNd act_FR_foot_pos_local = VectorNd::Zero(3);

    VectorNd act_RL_foot_pos = VectorNd::Zero(3);
    VectorNd act_RR_foot_pos = VectorNd::Zero(3);
    VectorNd act_FL_foot_pos = VectorNd::Zero(3);
    VectorNd act_FR_foot_pos = VectorNd::Zero(3);
    
    VectorNd act_RL_foot_vel = VectorNd::Zero(3);
    VectorNd act_RR_foot_vel = VectorNd::Zero(3);
    VectorNd act_FL_foot_vel = VectorNd::Zero(3);
    VectorNd act_FR_foot_vel = VectorNd::Zero(3);

    VectorNd actual_EP = VectorNd::Zero(12);
    VectorNd actual_EP_vel = VectorNd::Zero(12);
    VectorNd actual_EP_acc = VectorNd::Zero(12);
        
    VectorNd target_EP = VectorNd::Zero(12);
    VectorNd RL_foot_pos = VectorNd::Zero(3);
    VectorNd RR_foot_pos = VectorNd::Zero(3);
    VectorNd FL_foot_pos = VectorNd::Zero(3);
    VectorNd FR_foot_pos = VectorNd::Zero(3);
    VectorNd init_RL_foot_pos = VectorNd::Zero(3);
    VectorNd init_RR_foot_pos = VectorNd::Zero(3);
    VectorNd init_FL_foot_pos = VectorNd::Zero(3);
    VectorNd init_FR_foot_pos = VectorNd::Zero(3);    
    VectorNd tar_init_RL_foot_pos = VectorNd::Zero(3);
    VectorNd tar_init_RR_foot_pos = VectorNd::Zero(3);
    VectorNd tar_init_FL_foot_pos = VectorNd::Zero(3);
    VectorNd tar_init_FR_foot_pos = VectorNd::Zero(3);
    
    VectorNd tar_RL_foot_pos_local = VectorNd::Zero(3);
    VectorNd tar_RR_foot_pos_local = VectorNd::Zero(3);
    VectorNd tar_FL_foot_pos_local = VectorNd::Zero(3);
    VectorNd tar_FR_foot_pos_local = VectorNd::Zero(3);
    
    VectorNd RL_foot_vel = VectorNd::Zero(3);
    VectorNd RR_foot_vel = VectorNd::Zero(3);
    VectorNd FL_foot_vel = VectorNd::Zero(3);
    VectorNd FR_foot_vel = VectorNd::Zero(3);
    VectorNd tar_init_RL_foot_vel = VectorNd::Zero(3);
    VectorNd tar_init_RR_foot_vel = VectorNd::Zero(3);
    VectorNd tar_init_FL_foot_vel = VectorNd::Zero(3);
    VectorNd tar_init_FR_foot_vel = VectorNd::Zero(3);

    VectorNd tar_RL_foot_vel_local = VectorNd::Zero(3);
    VectorNd tar_RR_foot_vel_local = VectorNd::Zero(3);
    VectorNd tar_FL_foot_vel_local = VectorNd::Zero(3);
    VectorNd tar_FR_foot_vel_local = VectorNd::Zero(3);
    
    //*************Base & COM ****************///
    VectorNd act_base_pos = VectorNd::Zero(3);
    VectorNd act_base_vel = VectorNd::Zero(3);
    VectorNd tmp_act_base_pos = VectorNd::Zero(3);
    VectorNd tmp_act_base_vel = VectorNd::Zero(3);   
    
    VectorNd tmp_com_pos = VectorNd::Zero(3);
    VectorNd act_com_pos = VectorNd::Zero(3);
    VectorNd act_com_vel = VectorNd::Zero(3);
    VectorNd act_com_acc = VectorNd::Zero(3);
    VectorNd pre_act_com_vel = VectorNd::Zero(3);
    VectorNd tmp_act_com_acc = VectorNd::Zero(3);
    
    VectorNd act_base_ori = VectorNd::Zero(3);
    VectorNd act_base_ori_dot = VectorNd::Zero(3);
    VectorNd tmp_act_base_ori = VectorNd::Zero(3);
    VectorNd tmp_act_base_ori_dot = VectorNd::Zero(3);
    
    VectorNd base_pos = VectorNd::Zero(3); // x,y,z
    VectorNd com_pos = VectorNd::Zero(3); // x,y,z
    VectorNd com_vel = VectorNd::Zero(3); // x,y,z
    VectorNd com_acc = VectorNd::Zero(3); // x,y,z
    VectorNd pre_com_pos = VectorNd::Zero(3); // x,y,z
    VectorNd pre_com_vel = VectorNd::Zero(3);
    VectorNd init_base_pos = VectorNd::Zero(3);
    VectorNd init_base_ori = VectorNd::Zero(3);
    
    VectorNd init_com_pos = VectorNd::Zero(3);
    VectorNd tar_init_com_pos = VectorNd::Zero(3);
    VectorNd init_com_vel = VectorNd::Zero(3);
    VectorNd tar_init_com_vel = VectorNd::Zero(3); // x,y,z
    VectorNd tar_init_com_acc = VectorNd::Zero(3); // x,y,z

    VectorNd base_pos_ori = VectorNd::Zero(6);
    
    //**************[CoM Position Parameters Init]******************//
    VectorNd p_base2body_com = VectorNd::Zero(4);
    VectorNd p_RL_hp_com = VectorNd::Zero(4);
    VectorNd p_RL_thigh_com = VectorNd::Zero(4);
    VectorNd p_RL_calf_com = VectorNd::Zero(4);
    VectorNd p_RR_hp_com = VectorNd::Zero(4);
    VectorNd p_RR_thigh_com = VectorNd::Zero(4);
    VectorNd p_RR_calf_com = VectorNd::Zero(4);
    VectorNd p_FL_hp_com = VectorNd::Zero(4);
    VectorNd p_FL_thigh_com = VectorNd::Zero(4);
    VectorNd p_FL_calf_com = VectorNd::Zero(4);
    VectorNd p_FR_hp_com = VectorNd::Zero(4);
    VectorNd p_FR_thigh_com = VectorNd::Zero(4);
    VectorNd p_FR_calf_com = VectorNd::Zero(4);
    
    MatrixNd R_w2base_R = MatrixNd::Zero(4, 4);
    MatrixNd R_w2base_P = MatrixNd::Zero(4, 4);
    MatrixNd R_w2base_Y = MatrixNd::Zero(4, 4);
    MatrixNd R_w2base = MatrixNd::Zero(4, 4);
    MatrixNd T_w2base = MatrixNd::Zero(4, 4);
    MatrixNd TR_RL_base2hp = MatrixNd::Zero(4, 4);
    MatrixNd TR_RL_hp2thigh = MatrixNd::Zero(4, 4);
    MatrixNd TR_RL_thigh2calf = MatrixNd::Zero(4, 4);
    MatrixNd TR_RR_base2hp = MatrixNd::Zero(4, 4);
    MatrixNd TR_RR_hp2thigh = MatrixNd::Zero(4, 4);
    MatrixNd TR_RR_thigh2calf = MatrixNd::Zero(4, 4);
    MatrixNd TR_FL_base2hp = MatrixNd::Zero(4, 4);
    MatrixNd TR_FL_hp2thigh = MatrixNd::Zero(4, 4);
    MatrixNd TR_FL_thigh2calf = MatrixNd::Zero(4, 4);
    MatrixNd TR_FR_base2hp = MatrixNd::Zero(4, 4);
    MatrixNd TR_FR_hp2thigh = MatrixNd::Zero(4, 4);
    MatrixNd TR_FR_thigh2calf = MatrixNd::Zero(4, 4);
    
    VectorNd p_RL_base2hp_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_base2thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_base2calf_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_base2hp_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_base2thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_base2calf_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_base2hp_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_base2thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_base2calf_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_base2hp_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_base2thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_base2calf_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_com = VectorNd::Zero(4, 1);
    
    VectorNd p_robot_com_from_base = VectorNd::Zero(4, 1);
    VectorNd p_robot_com_from_w = VectorNd::Zero(4, 1);
    VectorNd p_robot_com = VectorNd::Zero(3, 1);
    
    //**************[CoM Position Parameters End]******************//
        
    /************************************************/
    MatrixNd ROT_Y = MatrixNd::Zero(3, 3);
    VectorNd tmp_foot_pos = VectorNd::Zero(3);
    VectorNd tmp_foot_pos2 = VectorNd::Zero(3);
        
     //*************Gain***********************//
    VectorNd Kp_q = VectorNd::Zero(12);
    VectorNd Kd_q = VectorNd::Zero(12);
    VectorNd init_Kp_q = VectorNd::Zero(12);
    VectorNd init_Kd_q = VectorNd::Zero(12);
    VectorNd tar_Kp_q = VectorNd::Zero(12);
    VectorNd tar_Kd_q = VectorNd::Zero(12);
    VectorNd tar_Kp_q_low = VectorNd::Zero(12);
    VectorNd tar_Kd_q_low = VectorNd::Zero(12);
    
    VectorNd Kp_t = VectorNd::Zero(12);
    VectorNd Kd_t = VectorNd::Zero(12);
    VectorNd init_Kp_t = VectorNd::Zero(12);
    VectorNd init_Kd_t = VectorNd::Zero(12);
    VectorNd tar_Kp_t = VectorNd::Zero(12);
    VectorNd tar_Kd_t = VectorNd::Zero(12);
    VectorNd tar_Kp_t_low = VectorNd::Zero(12);
    VectorNd tar_Kd_t_low = VectorNd::Zero(12);
    
    MatrixNd Kp_x = MatrixNd::Zero(3, 3);
    MatrixNd Kd_x = MatrixNd::Zero(3, 3);
    MatrixNd init_Kp_x = MatrixNd::Zero(3, 3);
    MatrixNd init_Kd_x = MatrixNd::Zero(3, 3);
    MatrixNd tar_Kp_x = MatrixNd::Zero(3, 3);
    MatrixNd tar_Kd_x = MatrixNd::Zero(3, 3);
    
    MatrixNd Kp_w = MatrixNd::Zero(3, 3);
    MatrixNd Kd_w = MatrixNd::Zero(3, 3);
    MatrixNd init_Kp_w = MatrixNd::Zero(3, 3);
    MatrixNd init_Kd_w = MatrixNd::Zero(3, 3);
    MatrixNd tar_Kp_w = MatrixNd::Zero(3, 3);
    MatrixNd tar_Kd_w = MatrixNd::Zero(3, 3);
    
     //===========Controller==========//
    VectorNd CTC_Torque = VectorNd::Zero(18);
    VectorNd tmp_CTC_Torque = VectorNd::Zero(18);
    VectorNd Fc = VectorNd::Zero(18);
    VectorNd pd_con_joint = VectorNd::Zero(18);
    VectorNd pd_con_task = VectorNd::Zero(18);
    // =============== [flying trot parameters Init] ================= //
    double ts, tf;
    int ts_cnt, tf_cnt;
    double h_0, h_1, h_2, h_3, v_0, v_1, v_2, v_3, a_0, a_1, a_2, a_3;
    double swing_foot_height;
    double c_com_z1[6], c_com_z2[6], c_com_z3[6], c_com_z4[6];
    double c_com_x1[6], c_com_x2[6], c_com_x3[6], c_com_x4[6], c_com_x5[6];
    double c_com_y1[6], c_com_y2[6], c_com_y3[6], c_com_y4[6], c_com_y5[6];
    double c_state_x1[6];
    double c_sf_z1[6], c_sf_z2[6], c_sf_z3[6], c_sf_z4[6];
    double c_sf_x1[6], c_sf_x2[6], c_sf_x3[6], c_sf_x4[6], c_sf_x5[6];
    double c_sf_y1[6], c_sf_y2[6], c_sf_y3[6], c_sf_y4[6], c_sf_y5[6];
    double ft_time, ft_step_time;
    double ft_time2;
    int ft_cnt, ft_step_cnt, ft_ready_cnt, ft_finish_cnt;
    int ft_cnt2;
    // =================[flying trot parameters End]==================//
    

     //===========================[ QP ]=========================//
    // Workspace structures
    OSQPWorkspace *QP_work;
    OSQPSettings *QP_settings = (OSQPSettings *) c_malloc(sizeof (OSQPSettings));
    OSQPData *QP_data = (OSQPData *) c_malloc(sizeof (OSQPData));

    c_int QP_exitflag = 0;

    c_int P_nnz = 78;
    c_float P_x[78];
    c_int P_i[78];
    c_int P_p[13];
    c_float q[12];

    c_float A_x[12]; // = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    c_int A_nnz = 12;
    c_int A_i[12]; // = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    c_int A_p[13]; // = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    c_float l[12]; // = {_d_l(0), _d_l(1), _d_l(2), _d_l(3), _d_l(4), _d_l(5), _d_l(6), _d_l(7), _d_l(8), _d_l(9), _d_l(10), _d_l(11)};
    c_float u[12]; // = {_d_u(0), _d_u(1), _d_u(2), _d_u(3), _d_u(4), _d_u(5), _d_u(6), _d_u(7), _d_u(8), _d_u(9), _d_u(10), _d_u(11)};
    c_int n = 12;
    c_int m = 12;
    
    MatrixNd S_mat = MatrixNd::Zero(18, 18);
    VectorNd des_x_2dot = VectorNd::Zero(3);
    VectorNd des_w_dot = VectorNd::Zero(3);
    double _m;
    MatrixNd _I_g = MatrixNd::Zero(3, 3);
    MatrixNd _A = MatrixNd::Zero(6, 12);
    MatrixNd p_com_oross_pro = MatrixNd::Zero(3, 12);
    VectorNd _g = VectorNd::Zero(3);
    VectorNd _b = VectorNd::Zero(6);
    MatrixNd _C = MatrixNd::Identity(12, 12);
    VectorNd _d_u = VectorNd::Zero(12);
    VectorNd _d_l = VectorNd::Zero(12);
    VectorNd _c = VectorNd::Zero(4);
    VectorNd c_vec = VectorNd::Zero(12);
    
    MatrixNd _S = MatrixNd::Identity(6, 6);
    MatrixNd _W = MatrixNd::Identity(12, 12);
    double _alpha = 0.001; //0.000001; // 0.00001

    MatrixNd _P = MatrixNd::Zero(12, 12);
    VectorNd _q = VectorNd::Zero(12);

    double fz_RL_max = 0;
    double fz_RL_min = 0;
    double fz_RR_max = 0;
    double fz_RR_min = 0;
    double fz_FL_max = 0;
    double fz_FL_min = 0;
    double fz_FR_max = 0;
    double fz_FR_min = 0;

    double mu = 0.7; // static friction coefficient
    double max_Fext_z = 1000;
    double tmp_weight;
    
    //*************** 5th Polynomial Param Init ************//
    double init_x[3] = {0, 0, 0};
    double final_x[3] = {0, 0, 0};
    VectorNd R = VectorNd::Zero(6);
    VectorNd P = VectorNd::Zero(6);
    MatrixNd A = MatrixNd::Zero(6, 6);
    //*************** 5th Polynomial Param End ************//
    
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

    VectorNd actual_EP_local = VectorNd::Zero(NUM_OF_ELMO);
//    double alpha = 0.0;
//    bool stop_flag = false;

    // JoyStick
//    double joy_vel_x = 0.0;
//    double joy_vel_y = 0.0;
//    double joy_vel_z = 0.0;

    //Walking Traj
    
    //double foot_height_HS=0.0;
//    double z_up[6];
//    double z_down[6];
    
//    unsigned int tsp_cnt_HS=250;
//    unsigned int fsp_cnt_HS=100;
//    double tsp_time_HS=tsp_cnt_HS*dt;
//    double fsp_time_HS=fsp_cnt_HS*dt;
//    double step_time_HS=tsp_time_HS+fsp_time_HS;
//    double walk_time, t1, t2, dsp_t1, dsp_t2;
//    bool walk_stop_flag=false;

    
private:
};

#endif /* CROBOT_H */
