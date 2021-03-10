#ifndef CROBOT_H
#define CROBOT_H

#include "rbdl/rbdl.h"
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "osqp.h"

#define PI  3.14159265359
#define PI2 6.28318530718
#define GRAVITY 9.81

//#define onemsec 0.001
//#define onesec  1
//#define tasktime 0.001
//#define onesecSize 1000 // 0.001 x 1000 = 1sec

#define AXIS_X     0
#define AXIS_Y     1
#define AXIS_Z     2
#define AXIS_Roll  3
#define AXIS_Pitch 4
#define AXIS_Yaw   5

#define R2D 57.295779513
#define D2R 0.0174532925
#define JOINT_NUM 12

#define Nx 12  
#define Nu 12  
#define Nh 10   // Horizon number
#define Th 1.0    // horizon time
//#define N 10   // Horizon number
//#define T 1.0    // horizon time
//#define T 0.5    // sec

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using Eigen::MatrixXd;
using Eigen::VectorXd;

typedef enum {
    MODE_SIMULATION,
    MODE_ACTUAL_ROBOT
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
    CONTACT_OFF,
    CONTACT_ON

} _CONTACT_INFO;

typedef enum {
    QP_CON,
    MPC_CON
} _WB_CON;

typedef enum {
    FRONK_NONE,
    FRONK_JUMP_READY,
    FRONK_TAKE_OFF,
    FRONK_FLIGHT,
    FRONK_LAND,
    FRONK_FINAL
} _JUMP_PHASE;

typedef struct Base //coordinate of Base
{
    VectorNd des_pos = VectorNd::Zero(3);
    VectorNd act_pos = VectorNd::Zero(3);
    VectorNd init_pos = VectorNd::Zero(3);
    
    VectorNd des_vel = VectorNd::Zero(3);
    VectorNd act_vel = VectorNd::Zero(3);
    VectorNd init_vel = VectorNd::Zero(3);
    
    VectorNd des_Euler_Ang = VectorNd::Zero(3);
    VectorNd act_Euler_Ang = VectorNd::Zero(3);
    VectorNd init_goal_Euler_Ang = VectorNd::Zero(3);
    VectorNd init_Euler_Ang = VectorNd::Zero(3);
    
    VectorNd des_Ang_Vel = VectorNd::Zero(3);
    VectorNd act_Ang_Vel = VectorNd::Zero(3);
    VectorNd init_goal_Ang_Vel = VectorNd::Zero(3);
    VectorNd init_Ang_Vel = VectorNd::Zero(3);
    
    VectorNd des_Euler_Ang_quad = VectorNd::Zero(4); // roll,pitch,yaw 
    
    VectorNd tmp_des_Euler_Ang = VectorNd::Zero(3);
    VectorNd tmp_des_Ang_Vel = VectorNd::Zero(3);

    int ID;

} BASE;

typedef struct CoM //coordinate of Base
{
    double height;
    
    VectorNd des_pos = VectorNd::Zero(3);
    VectorNd des_pos_now = VectorNd::Zero(3);
    VectorNd act_pos = VectorNd::Zero(3);
    VectorNd init_pos = VectorNd::Zero(3);
    VectorNd init_goal_pos = VectorNd::Zero(3);
    
    VectorNd des_vel = VectorNd::Zero(3);
    VectorNd des_vel_now = VectorNd::Zero(3);
    VectorNd act_vel = VectorNd::Zero(3);
    VectorNd init_vel = VectorNd::Zero(3);
    VectorNd init_goal_vel = VectorNd::Zero(3);
    
    VectorNd des_acc = VectorNd::Zero(3);
    VectorNd act_acc = VectorNd::Zero(3);
    VectorNd init_acc = VectorNd::Zero(3);
    
    VectorNd offset = VectorNd::Zero(3);
    
    VectorNd tmp_des_pos = VectorNd::Zero(3);
    VectorNd tmp_des_vel = VectorNd::Zero(3);

    int ID;

} COM;

typedef struct Joint {

    VectorNd torque = VectorNd::Zero(JOINT_NUM);
    
    VectorNd act_pos = VectorNd::Zero(JOINT_NUM);
    VectorNd act_vel = VectorNd::Zero(JOINT_NUM);
    VectorNd act_acc = VectorNd::Zero(JOINT_NUM);
    
    VectorNd init_pos = VectorNd::Zero(JOINT_NUM);
    VectorNd des_pos = VectorNd::Zero(JOINT_NUM);
    VectorNd des_vel = VectorNd::Zero(JOINT_NUM);
    VectorNd des_acc = VectorNd::Zero(JOINT_NUM);

} JOINT;


typedef struct EndPoint {

    VectorNd act_pos_global = VectorNd::Zero(3);
    VectorNd des_pos_global_now = VectorNd::Zero(3);
    
    VectorNd act_pos_local = VectorNd::Zero(3);
    VectorNd init_pos_global = VectorNd::Zero(3);
    VectorNd init_goal_pos_global = VectorNd::Zero(3);
    VectorNd des_pos_global = VectorNd::Zero(3);
    VectorNd des_pos_local = VectorNd::Zero(3); 
    
    VectorNd act_vel_local = VectorNd::Zero(3);
    VectorNd des_vel_global_now = VectorNd::Zero(3);
    VectorNd des_vel_global = VectorNd::Zero(3);
    VectorNd init_vel_global = VectorNd::Zero(3);
    VectorNd des_vel_local = VectorNd::Zero(3);
    VectorNd init_goal_vel_global = VectorNd::Zero(3);
    
    VectorNd act_q_dot = VectorNd::Zero(3);

    VectorNd tmp_f = VectorNd::Zero(3);
    VectorNd tmp_des_pos_global = VectorNd::Zero(3);
    VectorNd tmp_des_pos_local = VectorNd::Zero(3);
    MatrixNd des_pos_global_tilde = MatrixNd::Zero(3,Nh);
    MatrixNd des_vel_global_tilde = MatrixNd::Zero(3,Nh);
    
    

    int ID;
//    bool CT; // contact : 1: on, 0: off
} ENDPOINT;

typedef struct DDP {

    // Weight
    VectorNd Q_vec = VectorNd::Zero(Nx);
    MatrixNd Q = MatrixNd::Zero(Nx,Nx);
    MatrixNd Q_f = MatrixNd::Zero(Nx,Nx);
    
//    VectorNd R_f = VectorNd::Zero(Nu);
//    VectorNd R_p = VectorNd::Zero(12);
    VectorNd R_vec = VectorNd::Zero(Nu);
    MatrixNd R = MatrixNd::Zero(Nu,Nu);
    
    // parameters
    VectorNd f_init = VectorNd::Zero(12);
//    VectorNd p_foot_init = VectorNd::Zero(12);
    VectorNd u_init = VectorNd::Zero(Nu);
    VectorNd x_init = VectorNd::Zero(Nx);
    VectorNd x_ref = VectorNd::Zero(Nx);
    MatrixNd x_mat = MatrixNd::Zero(Nx,Nh);
    VectorNd x = VectorNd::Zero(Nx);
            
    MatrixNd u_mat_init = MatrixNd::Zero(Nu,Nh-1);
    MatrixNd u = MatrixNd::Zero(Nu,Nh-1);
//    MatrixNd u_new = MatrixNd::Zero(Nu,N-1);
    VectorNd u_ref = VectorNd::Zero(Nu);
    
    MatrixNd out_x_mat = MatrixNd::Zero(Nx,Nh); 
    MatrixNd out_u_mat = MatrixNd::Zero(Nu,Nh-1);
    double out_J = 0;
    
    MatrixNd x_des_tilde = MatrixNd::Zero(Nx,Nh); 
    MatrixNd u_des_tilde = MatrixNd::Zero(Nu,Nh); 

    MatrixNd c_tilde = MatrixNd::Zero(4,Nh);
    VectorNd c_ref = VectorNd::Zero(4);
    
    VectorNd u_min = VectorNd::Zero(Nu);
    VectorNd u_max = VectorNd::Zero(Nu);
   
} DDP;

class CRobot {
public:

    //Variables
    COM com;
    BASE base; //* coordinate of Body
    JOINT* joint; //* joints of the robot
    ENDPOINT FR, FL, RR, RL;
    
    DDP ddp;
    
    //Functions
    CRobot();
    CRobot(const CRobot& orig);
    virtual ~CRobot();
    

    
    //  ============== Function ============== //
    void setRobotModel(Model* getModel); //* get Robot Model
    void Home_Pos_PD_Con(void);
    void Torque_off(void);
    void StateUpdate(void);
    void ComputeTorqueControl(void);
    void WalkReady_Pos_Traj(void);
    void Pronk_Jump(void);
    void FK1(void);
    VectorNd IK1(VectorNd EP,VectorNd base_ori);
    void Get_act_com(void);
    VectorNd Get_COM(VectorNd base_pos, VectorNd base_ori, VectorNd q);
    MatrixNd Kron(MatrixNd AA, MatrixNd BB);
    MatrixNd PB_Dynamics(VectorNd x_vec,MatrixNd u_mat);
    VectorNd PB_Dynamics_once(VectorNd x_vec,VectorNd u_vec, int i);
    VectorNd Cross_prod(VectorNd x,VectorNd y);
    void DDP_Process(MatrixNd x_mat, MatrixNd u_mat);
    void Finite_Diff(MatrixNd x_mat,MatrixNd u_mat, MatrixNd x_des_mat, MatrixNd u_des_mat);
    void Backward_Pass(void);
    void Forward_Pass(MatrixNd x_mat, MatrixNd u_mat);
    void Array2Eigen(int i);
    double cal_J(MatrixNd x_mat, MatrixNd u_mat);
    double PB_cost(VectorNd x_vec, VectorNd u_vec, int i);
    double PB_cost_final(VectorNd x_vec);
    void NMPC_Process(void);
    void RBDL_Init(Model* getModel);
    void PARA_Init(void);
    void NMPC_Init(void);
    void OSQP_Init(void);
    void Get_u_min_max(VectorNd _c, VectorNd _u);
    void Jump_COM_Z_Traj_Gen(double h0, double t[3]);
    void coefficient_5thPoly(double *init_x, double *final_x, double tf, double *output);
    double fifth_order_poly(double c[], double t);
    double fifth_order_poly_dot(double c[], double t);
    double fifth_order_poly_2dot(double c[], double t);
    
//    double PB_cost_final(VectorNd x_vec);
    
    
    // ============= RBDL ============== //
    
    Quaternion QQ;
    RigidBodyDynamics::Model* m_pModel; //* URDF Model
    MatrixNd M_term = MatrixNd::Zero(18, 18);
    VectorNd hatNonLinearEffects = VectorNd::Zero(18);
    VectorNd G_term = VectorNd::Zero(18);
    VectorNd C_term = VectorNd::Zero(18);
    VectorNd CTC_Torque = VectorNd::Zero(18);
    
    // ============= RBDL Done =============== //
    
    int nDOF; //* number of DOFs of a robot
    int ControlMode;
    int CommandFlag;
    double dt = 0.001;

    double l1 = 0.105;
    double l2 = 0.305;
    double l3 = 0.309;

    double L3_x = 0; //0.025516;
    double L3_y = 0; //0.0;
    double L3_z = 0.309; //0.304515;

    VectorNd EP_OFFSET_RL = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_RR = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_FL = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd EP_OFFSET_FR = Vector3d(-L3_x, L3_y, -L3_z);
    VectorNd Originbase = Vector3d(0, 0, 0);    
    
    MatrixNd J_RL = MatrixNd::Zero(3, JOINT_NUM+6);
    MatrixNd J_RR = MatrixNd::Zero(3, JOINT_NUM+6);
    MatrixNd J_FL = MatrixNd::Zero(3, JOINT_NUM+6);
    MatrixNd J_FR = MatrixNd::Zero(3, JOINT_NUM+6);
    
    MatrixNd J_RL2 = MatrixNd::Zero(3, 3);
    MatrixNd J_RR2 = MatrixNd::Zero(3, 3);
    MatrixNd J_FL2 = MatrixNd::Zero(3, 3);
    MatrixNd J_FR2 = MatrixNd::Zero(3, 3);

    MatrixNd J_BASE = MatrixNd::Zero(6, JOINT_NUM+6);
    MatrixNd J_A = MatrixNd::Zero(JOINT_NUM+6, JOINT_NUM+6);    
        
    VectorNd RobotState     = VectorNd::Zero(JOINT_NUM+7);
    VectorNd RobotStatedot  = VectorNd::Zero(JOINT_NUM+6);
    VectorNd RobotState2dot = VectorNd::Zero(JOINT_NUM+6);
    
    
    // ============== Robot Dynamics Parameters ============= //
    double M = 50.1;
    double g = 9.81;
    
    MatrixNd MIT = MatrixNd::Zero(3,3);
    MatrixNd inv_MIT = MatrixNd::Zero(3,3);
    
    // =============== IMU ================ //
    double IMURoll, IMUPitch, IMUYaw;
    double IMURoll_dot, IMUPitch_dot, IMUYaw_dot;
    double init_IMUYaw;
    
    // =============== count parameters ================ //
    unsigned int wr_cnt = 0;
    unsigned int walk_ready_cnt = 2000; 
    double walk_ready_time = 2;
        
    // =================== etc =================== //
//    VectorNd base_offset = VectorNd::Zero(3);
    VectorNd tmp_data1 = VectorNd::Zero(100);
    VectorNd tmp_data2 = VectorNd::Zero(100);
//    double tmp_x_moving_speed, tmp_y_moving_speed;
//    VectorNd tmp_base_ori = VectorNd::Zero(3);
    double speed_x = 0.0;
    double speed_y = 0.0;
    double speed_yaw = 0.0;
    
    VectorNd RL_base2hip_pos = VectorNd::Zero(3);
    VectorNd RR_base2hip_pos = VectorNd::Zero(3);
    VectorNd FL_base2hip_pos = VectorNd::Zero(3);
    VectorNd FR_base2hip_pos = VectorNd::Zero(3);
    
    VectorNd init_RL_hip2EP = VectorNd::Zero(3);
    VectorNd init_RR_hip2EP = VectorNd::Zero(3);
    VectorNd init_FL_hip2EP = VectorNd::Zero(3);
    VectorNd init_FR_hip2EP = VectorNd::Zero(3);
    VectorNd init_EP = VectorNd::Zero(12);
    VectorNd pd_con_joint = VectorNd::Zero(18);
    VectorNd pd_con_task  = VectorNd::Zero(18);
    VectorNd Kp_t = VectorNd::Zero(12);
    VectorNd Kd_t = VectorNd::Zero(12);
    VectorNd Kp_q = VectorNd::Zero(12);
    VectorNd Kd_q = VectorNd::Zero(12);
    VectorNd F_ext = VectorNd::Zero(18);
//    VectorNd F_ext2 = VectorNd::Zero(18);
    
//    double fc_weight = 1;
    int tmp_cnt = 0;
    double tmp_time = 0;
    
    MatrixNd x_next_mat = MatrixNd::Zero(Nx,Nh);
    VectorNd x_next_vec = VectorNd::Zero(Nx);
    MatrixNd inv_E = MatrixNd::Zero(3,3);
    VectorNd p_com_dot = VectorNd::Zero(3);
    VectorNd Euler_Ang_dot = VectorNd::Zero(3);
    VectorNd p_com_2dot = VectorNd::Zero(3);
    VectorNd Ang_Vel_dot = VectorNd::Zero(3);
    VectorNd x_dot = VectorNd::Zero(Nx);
    VectorNd H_dot = VectorNd::Zero(3);
//    Vector3d gravity_vec(0,0,g);
    VectorNd gravity_vec = VectorNd::Zero(3);
    
    VectorNd tmp_cx = VectorNd::Zero(Nx);
    VectorNd tmp_cu = VectorNd::Zero(Nu);
    MatrixNd tmp_cxx = MatrixNd::Zero(Nx,Nx);
    MatrixNd tmp_cuu = MatrixNd::Zero(Nu,Nu);
    MatrixNd tmp_cux = MatrixNd::Zero(Nu,Nx);
    MatrixNd tmp_fx = MatrixNd::Zero(Nx,Nx);
    MatrixNd tmp_fx2 = MatrixNd::Zero(Nx,Nh);
    MatrixNd tmp_fu = MatrixNd::Zero(Nx,Nu);
    
    VectorNd Qx = VectorNd::Zero(Nx);
    VectorNd Qu = VectorNd::Zero(Nu);
    MatrixNd Qxx = MatrixNd::Zero(Nx,Nx);
    MatrixNd Quu = MatrixNd::Zero(Nu,Nu);
    MatrixNd Qux = MatrixNd::Zero(Nu,Nx);
    
    VectorNd Vx = VectorNd::Zero(Nx);
    MatrixNd Vxx = MatrixNd::Zero(Nx,Nx);
    
    VectorNd tmp_k = VectorNd::Zero(Nu);
    MatrixNd tmp_K = MatrixNd::Zero(Nu,Nx);
    
    MatrixNd x_pur_mat1 = MatrixNd::Zero(Nx,Nh);
    MatrixNd x_pur_mat2 = MatrixNd::Zero(Nx,Nh);
    MatrixNd x_pur1 = MatrixNd::Zero(Nx,Nh);
    MatrixNd x_pur2 = MatrixNd::Zero(Nx,Nh);
    MatrixNd u_pur1 = MatrixNd::Zero(Nu,Nh-1);
    MatrixNd u_pur2 = MatrixNd::Zero(Nu,Nh-1);
    
    VectorNd x_err = VectorNd::Zero(Nx);
    VectorNd u_err = VectorNd::Zero(Nu);
    
    double fx[Nx][Nx][Nh];
    double fu[Nx][Nu][Nh];
    double cx[Nx][Nh];
    double cu[Nu][Nh-1];
    double cxx[Nx][Nx][Nh];
    double cuu[Nu][Nu][Nh-1];
    double cux[Nu][Nx][Nh-1];
    
    double k[Nu][Nh-1];
    double K[Nu][Nx][Nh-1];
    
    VectorNd tmp_x_vec = VectorNd::Zero(Nx);
    VectorNd tmp_u_vec = VectorNd::Zero(Nu);
    
    VectorNd dx = VectorNd::Zero(Nx);
    VectorNd du = VectorNd::Zero(Nu);
    
    double Ts = (double)Th/(double)Nh;
    int Ns = (int)(Ts/dt);
    
    int NMPC_thread_cnt = 0;
    
    VectorNd tmp_F_ext = VectorNd::Zero(Nu);
    VectorNd pre_F_ext = VectorNd::Zero(Nu);
    
    VectorNd target_EP = VectorNd::Zero(12);
    VectorNd actual_EP = VectorNd::Zero(12);
    VectorNd target_EP_vel = VectorNd::Zero(12);
    VectorNd actual_EP_vel = VectorNd::Zero(12);
    
    VectorNd max_Fz_vec = VectorNd::Zero(4);
    VectorNd Fz_vec = VectorNd::Zero(4);
    
    VectorNd out = VectorNd::Zero(3);
    
    int jump_num = 0;
    int pr_cnt = 0;
    int jump_phase = 0;
    
    double jump_z1[6], jump_z2[6], jump_z3[6], jump_z4[6];
    
    double init_x[3] = {0, 0, 0};
    double final_x[3] = {0, 0, 0};
    
    VectorNd R = VectorNd::Zero(6);
    VectorNd P = VectorNd::Zero(6);
    MatrixNd A = MatrixNd::Zero(6, 6);
    
    // ==================== flags =================== //
    bool move_done_flag;    
    bool move_stop_flag;
    bool walk_ready_move_done_flag;
    bool nmpc_run_flag;
    
    
    //****************** Move Paramter ****************//
    double x_moving_speed = 0.0;
    double y_moving_speed = 0.0;
    double pre_x_moving_speed = 0.0;
    double tmp_x_moving_speed = 0.0;
    double tmp_y_moving_speed = 0.0;
    VectorNd tmp_base_ori = VectorNd::Zero(3);
//    double speed_x_HS = 0.0;
//    double speed_y_HS = 0.0;
//    double speed_yaw_HS = 0.0;

private:
};

#endif /* CROBOT_H */