#include <stdio.h>
#include <math.h>
#include "CRobot.h"
#include <stdint.h>

CRobot::CRobot() {
}

CRobot::CRobot(const CRobot& orig) {
}

CRobot::~CRobot() {
}

// Elmo transform Functions

double CRobot::Count2Deg(int Gear_Ratio, INT32 Count) {
    double th = (double) Count * 360 / (2048 * Gear_Ratio);
    return th; // 
}

double CRobot::Count2DegDot(int Gear_Ratio, INT32 CountPerSec) {
    double th_dot = (double) CountPerSec * 360 / (2048 * Gear_Ratio);
    return th_dot; // [deg/s]
}

double CRobot::Count2Rad(int Gear_Ratio, INT32 Count) {
    double th = (double) Count * 2 * PI / (2048 * Gear_Ratio);
    //double th = (double) Count * 2 * PI / (36 * Gear_Ratio);
    return th; // [rad]
}

double CRobot::Count2RadDot(int Gear_Ratio, INT32 CountPerSec) {
    double th_dot = (double) CountPerSec * 2 * PI / (2048 * Gear_Ratio);
    //double th_dot = (double) CountPerSec * 2 * PI / (36 * Gear_Ratio);
    return th_dot; // [rad]
}

double CRobot::Count2Rad_ABS(int _Resolution, INT32 Count) {
    double th = (double) Count * 2 * PI / (_Resolution);
    return th;
}

//INT32 CRobot::Count_tf(INT32 _Count_in) {
//    INT32 _Count_out = (INT32) _Count_in;
//    return _Count_out;
//}

INT16 CRobot::Tor2Cur(double OutputTorque, double _Kt, int _Gear, double _ratedCur) {
    INT16 inputCurrent = (OutputTorque / _Gear) / _Kt / _ratedCur * 1000;

    return inputCurrent;
}

void CRobot::setRobotModel(Model* getModel) {
    printf( "Set Robot Model... \n" );
    
     //Mode = MODE_SIMULATION;
     Mode = MODE_ACTUAL_ROBOT;
     WH_Mode = QP_CON; //QP_CON;// MPC_CON
     
    //============= [Controller OnOff Init]================ //
    //    Base_Ori_Con_onoff_flag = false; //false; //true; //true;
    CP_con_onoff_flag = false; //true; //false;//true;
    Slope_con_onoff_flag = true; //true; //false;//true;
    gain_scheduling_flag = true;
    // =========== [Controller OnOff End] ================ //

    RL_base2hip_pos << -0.350, 0.115, -0.053;
    RR_base2hip_pos << -0.350, -0.115, -0.053;
    FL_base2hip_pos << 0.350, 0.115, -0.053;
    FR_base2hip_pos << 0.350, -0.115, -0.053;
        
    base2hip_pos << RL_base2hip_pos, RR_base2hip_pos, FL_base2hip_pos, FR_base2hip_pos;

    // global init foot position
    tar_init_RL_foot_pos << RL_base2hip_pos(0), RL_base2hip_pos(1) + 0.105 - 0.04, 0.0;
    tar_init_RR_foot_pos << RR_base2hip_pos(0), RR_base2hip_pos(1) - 0.105 + 0.04, 0.0;
    tar_init_FL_foot_pos << FL_base2hip_pos(0), FL_base2hip_pos(1) + 0.105 - 0.04, 0.0;
    tar_init_FR_foot_pos << FR_base2hip_pos(0), FR_base2hip_pos(1) - 0.105 + 0.04, 0.0;
    
    com_height = 0.42; //0.42;
//    
    if (Mode == MODE_SIMULATION) {
        set_simul_para();
    } else if (Mode == MODE_ACTUAL_ROBOT) {
        set_act_robot_para();
    }

    RL_foot_pos = tar_init_RL_foot_pos;
    RR_foot_pos = tar_init_RR_foot_pos;
    FL_foot_pos = tar_init_FL_foot_pos;
    FR_foot_pos = tar_init_FR_foot_pos;
    
    init_base_pos << 0, 0, com_height;
    init_base_ori << 0, 0, 0;
    base_pos = init_base_pos;
    base_ori = init_base_ori;
    base_vel << 0, 0, 0;
    base_ori_dot << 0, 0, 0;
    base_pos_ori << init_base_pos, init_base_ori;

    init_Kp_q = Kp_q;
    init_Kd_q = Kd_q;

    x_moving_speed = 0;
    y_moving_speed = 0;

    // Link com position
    p_base2body_com << -0.038, 0, -0.01, 1;
    p_RL_hp_com << 0, -0.0029, 0, 1;
    p_RL_thigh_com << -0.001, -0.006, -0.0227, 1;
    p_RL_calf_com << 0, 0.003, -0.094, 1;
    p_RR_hp_com << 0, 0.0029, 0, 1;
    p_RR_thigh_com << -0.001, 0.006, -0.0227, 1;
    p_RR_calf_com << 0, -0.003, -0.094, 1;
    p_FL_hp_com << 0, -0.0029, 0, 1;
    p_FL_thigh_com << -0.001, -0.006, -0.0227, 1;
    p_FL_calf_com << 0, 0.003, -0.094, 1;
    p_FR_hp_com << 0, 0.0029, 0, 1;
    p_FR_thigh_com << -0.001, 0.006, -0.0227, 1;
    p_FR_calf_com << 0, -0.003, -0.094, 1;

    target_EP << tar_init_RL_foot_pos - init_base_pos, tar_init_RR_foot_pos - init_base_pos, tar_init_FL_foot_pos - init_base_pos, tar_init_FR_foot_pos - init_base_pos;
    //target_pos = IK1(target_EP);
    
    VectorNd com_offset = VectorNd::Zero(3);
    VectorNd tmp_init_com_pos = VectorNd::Zero(3);

    com_offset << 0.0, 0, 0;
    //com_offset << 0.02, 0, 0;
    //com_offset << 0.0, 0, 0;
//
    tmp_init_com_pos = Get_COM(base_pos_ori, target_pos);
    init_com_pos = tmp_init_com_pos + com_offset;
    
    printf("--> tmp_init_com_pos= ( %3f / %3f/ %3f ) \n", tmp_init_com_pos(0), tmp_init_com_pos(1), tmp_init_com_pos(2));
    printf("--> init_com_pos= ( %3f / %3f/ %3f ) \n", init_com_pos(0), init_com_pos(1), init_com_pos(2));

    base_offset = init_base_pos - init_com_pos;
    printf("--> base_offset= ( %3f / %3f/ %3f ) \n", base_offset(0), base_offset(1), base_offset(2));
    tar_init_com_pos << 0.0, 0.0, com_height;
    tar_init_com_vel << 0.0, 0.0, 0.0;    
    contact_num = 4;
    
    // ==========================[RBDL Setting Init]===========================//
    m_pModel = getModel;
    m_pModel->gravity = Vector3d(0., 0., -9.81);
    nDOF = m_pModel->dof_count - 6; //* get Degree of freedom, Except x,y,z,roll,pitch,yaw of the robot
    joint = new JOINT[nDOF]; //* only joint of the robot excepting x,y,z,roll,pitch,yaw of the robot

    RobotState = VectorNd::Zero(19);
    RobotStatedot = VectorNd::Zero(18);
    RobotState2dot = VectorNd::Zero(18);
    BasePosOri = VectorNd::Zero(6);
    BaseVel = VectorNd::Zero(6);
    JointAngle = VectorNd::Zero(nDOF);
    JointVel = VectorNd::Zero(nDOF);

    base.ID = m_pModel->GetBodyId("BODY");
    RL.ID = m_pModel->GetBodyId("RL_CALF");
    RR.ID = m_pModel->GetBodyId("RR_CALF");
    FL.ID = m_pModel->GetBodyId("FL_CALF");
    FR.ID = m_pModel->GetBodyId("FR_CALF");

    QQ << 0, 0, 0, 1;
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);
    // ======================[RBDL Setting End]=========================//
    
    init_target_pos << 0, 45, -90, 0, 45, -90, 0, 45, -90, 0, 45, -90;
    
    for (unsigned int i = 0; i < nDOF; ++i) {
        joint[i].torque = 0;
    }
    
    IMURoll = 0;
    IMUPitch = 0;
    IMUYaw = 0;
    moving_done_flag = true;
    walk_ready_moving_done_flag = false;
    
    for (unsigned int i = 0; i < 6; ++i) {
        pd_con_joint[i] = 0;
        pd_con_task[i] = 0;
    }
    
    tmp_CTC_Torque = CTC_Torque;
    
    if (WH_Mode == QP_CON) {
        QP_Con_Init();
        //set_osqp_HS();
    }
    
    //Get_gain_HS();
}

void CRobot::StateUpdate(void) {
    RobotState(AXIS_X) = base_pos(0);
    RobotState(AXIS_Y) = base_pos(1);
    RobotState(AXIS_Z) = base_pos(2);
    RobotState(AXIS_Roll) = base_ori(0);
    RobotState(AXIS_Pitch) = base_ori(1);
    RobotState(AXIS_Yaw) = base_ori(2);
    RobotStatedot(AXIS_X) = base_vel(0);
    RobotStatedot(AXIS_Y) = base_vel(1);
    RobotStatedot(AXIS_Z) = base_vel(2);
    RobotStatedot(AXIS_Roll) = base_ori_dot(0);
    RobotStatedot(AXIS_Pitch) = base_ori_dot(1);
    RobotStatedot(AXIS_Yaw) = base_ori_dot(2);

    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        RobotState(6 + nJoint) = actual_pos[nJoint];
        RobotStatedot(6 + nJoint) = actual_vel[nJoint];
        RobotState2dot(6 + nJoint) = actual_acc[nJoint];
    }

    base_ori_quat = Math::Quaternion::fromXYZAngles(base_ori);
    Math::Quaternion QQ(base_ori_quat);
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);
    
    FK2();
    
    //Get_act_com();
}

void CRobot::ComputeTorqueControl(void) {
	// ================= Cal Jacobian ================= //
	//	cout << "1" << endl;

    CalcPointJacobian6D(*m_pModel, RobotState, base.ID, Originbase, J_BASE,true);
    CalcPointJacobian(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, J_RL, true);
    CalcPointJacobian(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, J_RR, true);
    CalcPointJacobian(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, J_FL, true);
    CalcPointJacobian(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, J_FR, true);

     //   cout << J_RL << endl;
    J_A.block(0, 0, 6, 18) = J_BASE;
    J_A.block(6, 0, 3, 18) = J_RL;
    J_A.block(9, 0, 3, 18) = J_RR;
    J_A.block(12, 0, 3, 18) = J_FL;
    J_A.block(15, 0, 3, 18) = J_FR;

    //cout << J_RL.block(0,6,3,3) << endl;
    J_RL2 = J_RL.block(0, 6, 3, 3);
    J_RR2 = J_RR.block(0, 9, 3, 3);
    J_FL2 = J_FL.block(0, 12, 3, 3);
    J_FR2 = J_FR.block(0, 15, 3, 3);

    act_RL_q_dot << actual_vel[0], actual_vel[1], actual_vel[2];
    act_RR_q_dot << actual_vel[3], actual_vel[4], actual_vel[5];
    act_FL_q_dot << actual_vel[7], actual_vel[8], actual_vel[9];
    act_FR_q_dot << actual_vel[10], actual_vel[11], actual_vel[12];

    act_RL_foot_vel = J_RL2 * act_RL_q_dot;
    act_RR_foot_vel = J_RR2 * act_RR_q_dot;
    act_FL_foot_vel = J_FL2 * act_FL_q_dot;
    act_FR_foot_vel = J_FR2 * act_FR_q_dot;
    actual_EP_vel << act_RL_foot_vel, act_RR_foot_vel, act_FL_foot_vel, act_FR_foot_vel;

    // ================= Cal Jacobian END ================= //
    tar_RL_foot_pos_local = (RL_foot_pos - base_pos) + RL_foot_pos_local_offset;
    tar_RR_foot_pos_local = (RR_foot_pos - base_pos) + RR_foot_pos_local_offset;
    tar_FL_foot_pos_local = (FL_foot_pos - base_pos) + FL_foot_pos_local_offset;
    tar_FR_foot_pos_local = (FR_foot_pos - base_pos) + FR_foot_pos_local_offset;

    base_vel = com_vel;

    tar_RL_foot_vel_local = RL_foot_vel - base_vel;
    tar_RR_foot_vel_local = RR_foot_vel - base_vel;
    tar_FL_foot_vel_local = FL_foot_vel - base_vel;
    tar_FR_foot_vel_local = FR_foot_vel - base_vel;

    target_EP << tar_RL_foot_pos_local, tar_RR_foot_pos_local, tar_FL_foot_pos_local, tar_FR_foot_pos_local;
    actual_EP << act_RL_foot_pos_local, act_RR_foot_pos_local, act_FL_foot_pos_local, act_FR_foot_pos_local;

    target_EP_vel << tar_RL_foot_vel_local, tar_RR_foot_vel_local, tar_FL_foot_vel_local, tar_FR_foot_vel_local;
    target_pos = IK1(target_EP);

    tar_RL_q_dot = J_RL2.inverse() * tar_RL_foot_vel_local;
    tar_RR_q_dot = J_RR2.inverse() * tar_RR_foot_vel_local;
    tar_FL_q_dot = J_FL2.inverse() * tar_FL_foot_vel_local;
    tar_FR_q_dot = J_FR2.inverse() * tar_FR_foot_vel_local;
    target_vel << tar_RL_q_dot, tar_RR_q_dot, 0, tar_FL_q_dot, tar_FR_q_dot;

    for (unsigned int i = 0; i < nDOF; ++i) {
        //    pd_con_task[i + 7] = Kp_t[i]*(target_EP[i] - actual_EP[i]) + Kd_t[i]*(target_EP_vel[i] - actual_EP_vel[i]); // + target_EP_offset[i];
        pd_con_task[i + 6] = Kp_t[i] * (target_EP[i] - actual_EP[i]) + Kd_t[i] * (0 - actual_EP_vel[i]);
    }

    for (unsigned int i = 0; i < nDOF; ++i) {
        pd_con_joint[i + 6] = Kp_q[i] * (target_pos[i] - actual_pos[i]) + Kd_q[i] * (target_vel[i] - actual_vel[i]);
    }

    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);
    C_term = hatNonLinearEffects - G_term;

    //	Fc << 0,0,0,0,0,0,0, 0,0,110, 0,0,110, 0,0,110, 0,0,110;
    //    Fc << 0,0,0,0,0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0;

    //    MPC_Fc << 0,0,0,0,0,0,0, 0,tar_Fc_y/4,0, 0,tar_Fc_y/4,0, 0,tar_Fc_y/4,0, 0,tar_Fc_y/4,0;
    //    cout << "Torque by MPC_FC = " << - J_A.transpose() * (MPC_Fc) << endl;

    //    CTC_Torque = fc_weight * (C_term + G_term - J_A.transpose() * (Fc - pd_con_task + MPC_Fc*10));
    
    //CTC_Torque = fc_weight * (C_term + G_term - J_A.transpose() * (Fc - pd_con_task)) + pd_con_joint;

    for (int nJoint = 0; nJoint < nDOF; nJoint++) {
        joint[nJoint].torque = CTC_Torque(6 + nJoint);
    }
}

void CRobot::ComputeTorqueControl_HS(void) {
    CalcPointJacobian6D(*m_pModel, RobotState, base.ID, Originbase, J_BASE, true);
    CalcPointJacobian(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, J_RL, true);
    CalcPointJacobian(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, J_RR, true);
    CalcPointJacobian(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, J_FL, true);
    CalcPointJacobian(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, J_FR, true);

    J_A.block(0, 0, 6, 18) = J_BASE;
    J_A.block(6, 0, 3, 18) = J_RL;
    J_A.block(9, 0, 3, 18) = J_RR;
    J_A.block(12, 0, 3, 18) = J_FL;
    J_A.block(15, 0, 3, 18) = J_FR;
    
    J_RL2 = J_RL.block(0, 6, 3, 3);
    J_RR2 = J_RR.block(0, 9, 3, 3);
    J_FL2 = J_FL.block(0, 12, 3, 3);
    J_FR2 = J_FR.block(0, 15, 3, 3);
   
    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);

    C_term = hatNonLinearEffects - G_term;

    //Joint_Controller();
    // Cartesian_Controller();
    //Controller_Change();

    //    CTC_Torque = Joint_Controller_HS + C_term + G_term - J_A.transpose() * (Cart_Controller_HS);
    //CTC_Torque = Joint_Controller_HS - J_A.transpose() * (Cart_Controller_HS);

    //CTC_Torque = Joint_Controller_HS;
    //CTC_Torque = G_term + C_term;
    //CTC_Torque = Joint_Controller_HS + G_term + C_term;

    //CTC_Torque = C_term + G_term - J_A.transpose() * (Cart_Controller_HS);
    //CTC_Torque = G_term;
    //CTC_Torque = C_term + G_term - J_A.transpose() * (Cart_Controller_HS);
    //CTC_Torque = Joint_Controller_HS + G_term + C_term;

    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        //joint[i].torque = CTC_Torque(6 + i);
    }
}

VectorNd CRobot::FK(VectorNd q) {

    VectorNd EP_pos = VectorNd::Zero(12);
    VectorNd RL_End_Point = VectorNd::Zero(3);
    VectorNd RR_End_Point = VectorNd::Zero(3);
    VectorNd FL_End_Point = VectorNd::Zero(3);
    VectorNd FR_End_Point = VectorNd::Zero(3);

    B_O << 0.0, 0.0, 0.0, 1;
    //========================Rear Left========================//

    RL_I2B << 1, 0, 0, 0.0\
, 0, 1, 0, 0.0\
, 0, 0, 1, 0.0\
, 0, 0, 0, 1;

    RL_B2HR << 1, 0, 0, -0.35\
, 0, cos(q[0]), -sin(q[0]), 0.115\
, 0, sin(q[0]), cos(q[0]), 0.0\
, 0, 0, 0, 1;

    RL_HR2HP << cos(q[1]), 0, sin(q[1]), 0.0\
, 0, 1, 0, l1\
, -sin(q[1]), 0, cos(q[1]), 0.0\
, 0, 0, 0, 1;

    RL_HP2KN << cos(q[2]), 0, sin(q[2]), 0.0\
, 0, 1, 0, 0.0\
, -sin(q[2]), 0, cos(q[2]), -l2\
, 0, 0, 0, 1;

    RL_KN2T << 1, 0, 0, 0.0\
, 0, 1, 0, 0.0\
, 0, 0, 1, -l3\
, 0, 0, 0, 1;

    RL_End_Point = RL_I2B * RL_B2HR * RL_HR2HP * RL_HP2KN * RL_KN2T * B_O;

    EP_pos[0] = RL_End_Point(0);
    EP_pos[1] = RL_End_Point(1);
    EP_pos[2] = RL_End_Point(2);

    //========================Rear Right========================//
    RR_I2B << 1, 0, 0, 0.0\
, 0, 1, 0, 0.0\
, 0, 0, 1, 0.0\
, 0, 0, 0, 1;

    RR_B2HR << 1, 0, 0, -0.35\
, 0, cos(q[3]), -sin(q[3]), -0.115\
, 0, sin(q[3]), cos(q[3]), 0.0\
, 0, 0, 0, 1;

    RR_HR2HP << cos(q[4]), 0, sin(q[4]), 0.0\
, 0, 1, 0, -l1\
, -sin(q[4]), 0, cos(q[4]), 0.0\
, 0, 0, 0, 1;

    RR_HP2KN << cos(q[5]), 0, sin(q[5]), 0.0\
, 0, 1, 0, 0.0\
, -sin(q[5]), 0, cos(q[5]), -l2\
, 0, 0, 0, 1;

    RR_KN2T << 1, 0, 0, 0.0\
, 0, 1, 0, 0.0\
, 0, 0, 1, -l3\
, 0, 0, 0, 1;

    RR_End_Point = RR_I2B * RR_B2HR * RR_HR2HP * RR_HP2KN * RR_KN2T * B_O;

    EP_pos[3] = RR_End_Point(0);
    EP_pos[4] = RR_End_Point(1);
    EP_pos[5] = RR_End_Point(2);


    //========================Front Left========================//
    FL_I2B << 1, 0, 0, 0.0\
, 0, 1, 0, 0.0\
, 0, 0, 1, 0\
, 0, 0, 0, 1;

    FL_B2HR << 1, 0, 0, 0.35\
, 0, cos(q[6]), -sin(q[6]), 0.115\
, 0, sin(q[6]), cos(q[6]), 0.0\
, 0, 0, 0, 1;

    FL_HR2HP << cos(q[7]), 0, sin(q[7]), 0.0\
, 0, 1, 0, l1\
, -sin(q[7]), 0, cos(q[7]), 0.0\
, 0, 0, 0, 1;

    FL_HP2KN << cos(q[8]), 0, sin(q[8]), 0.0\
, 0, 1, 0, 0.0\
, -sin(q[8]), 0, cos(q[8]), -l2\
, 0, 0, 0, 1;

    FL_KN2T << 1, 0, 0, 0.0\
, 0, 1, 0, 0.0\
, 0, 0, 1, -l3\
, 0, 0, 0, 1;

    FL_End_Point = FL_I2B * FL_B2HR * FL_HR2HP * FL_HP2KN * FL_KN2T * B_O;


    EP_pos[6] = FL_End_Point(0);
    EP_pos[7] = FL_End_Point(1);
    EP_pos[8] = FL_End_Point(2);


    //========================Front Right========================//
    FR_I2B << 1, 0, 0, 0.0\
, 0, 1, 0, 0.0\
, 0, 0, 1, 0\
, 0, 0, 0, 1;

    FR_B2HR << 1, 0, 0, 0.35\
, 0, cos(q[9]), -sin(q[9]), -0.115\
, 0, sin(q[9]), cos(q[9]), 0.0\
, 0, 0, 0, 1;

    FR_HR2HP << cos(q[10]), 0, sin(q[10]), 0.0\
, 0, 1, 0, -l1\
, -sin(q[10]), 0, cos(q[10]), 0.0\
, 0, 0, 0, 1;

    FR_HP2KN << cos(q[11]), 0, sin(q[11]), 0.0\
, 0, 1, 0, 0.0\
, -sin(q[11]), 0, cos(q[11]), -l2\
, 0, 0, 0, 1;

    FR_KN2T << 1, 0, 0, 0.0\
, 0, 1, 0, 0.0\
, 0, 0, 1, -l3\
, 0, 0, 0, 1;

    FR_End_Point = FR_I2B * FR_B2HR * FR_HR2HP * FR_HP2KN * FR_KN2T * B_O;

    EP_pos[9] = FR_End_Point(0);
    EP_pos[10] = FR_End_Point(1);
    EP_pos[11] = FR_End_Point(2);

    return EP_pos;

}

VectorNd CRobot::IK(VectorNd pos) {


    //===================Rear Left===================//
    VectorNd RL_pos = VectorNd::Zero(3);
    VectorNd RL_q = VectorNd::Zero(3);
    double RL_c, RL_theta, RL_d, RL_gamma;

    VectorNd RR_pos = VectorNd::Zero(3);
    VectorNd RR_q = VectorNd::Zero(3);
    double RR_c, RR_theta, RR_d, RR_gamma;

    VectorNd FL_pos = VectorNd::Zero(3);
    VectorNd FL_q = VectorNd::Zero(3);
    double FL_c, FL_theta, FL_d, FL_gamma;

    VectorNd FR_pos = VectorNd::Zero(3);
    VectorNd FR_q = VectorNd::Zero(3);
    double FR_c, FR_theta, FR_d, FR_gamma;


    RL_pos[0] = pos[0] + 0.35;
    RL_pos[1] = pos[1] - 0.115;
    RL_pos[2] = pos[2];

    double RL_a = sqrt(pow(RL_pos[1], 2) + pow(RL_pos[2], 2));

    if (RL_pos[1] >= 0.0)
        RL_q[0] = acos(abs(RL_pos[2]) / RL_a) + acos(l1 / RL_a) - PI / 2.0;
    else
        RL_q[0] = -acos(abs(RL_pos[2]) / RL_a) + acos(l1 / RL_a) - PI / 2.0;


    RL_c = sqrt(pow(RL_pos[0], 2) + pow(RL_pos[1] - l1 * cos(RL_q[0]), 2) + pow(RL_pos[2] - l1 * sin(RL_q[0]), 2));

    RL_q[2] = -acos((pow(RL_c, 2) - pow(l2, 2) - pow(l3, 2)) / (2.0 * l2 * l3));


    RL_theta = acos((pow(l2, 2) + pow(RL_c, 2) - pow(l3, 2)) / (2 * l2 * RL_c));

    RL_d = sqrt(pow((RL_pos[1] - l1 * cos(RL_q[0])), 2) + pow((RL_pos[2] - l1 * sin(RL_q[0])), 2));

    if (RL_pos[0] >= 0)
        RL_gamma = atan(abs(RL_pos[0]) / RL_d);
    else
        RL_gamma = atan(-abs(RL_pos[0]) / RL_d);

    RL_q[1] = (RL_theta - RL_gamma);

    joint_angle[0] = RL_q[0];
    joint_angle[1] = RL_q[1];
    joint_angle[2] = RL_q[2];

    //===================Rear Right===================//
    
    RR_pos[0] = pos[3] + 0.35;
    RR_pos[1] = pos[4] + 0.115;
    RR_pos[2] = pos[5];

    double RR_a = sqrt(pow(RR_pos[1], 2) + pow(RR_pos[2], 2));

    if (RR_pos[1] >= 0.0)
        RR_q[0] = acos(abs(RR_pos[2]) / RR_a) - acos(l1 / RR_a) + PI / 2.0;
    else 
        RR_q[0] = -acos(abs(RR_pos[2]) / RR_a) - acos(l1 / RR_a) + PI / 2.0;
    

    RR_c = sqrt(pow(RR_pos[0], 2) + pow(RR_pos[1] + l1 * cos(RR_q[0]), 2) + pow(RR_pos[2] + l1 * sin(RR_q[0]), 2));

    RR_q[2] = -acos((pow(RR_c, 2) - pow(l2, 2) - pow(l3, 2)) / (2.0 * l2 * l3));


    RR_theta = acos((pow(l2, 2) + pow(RR_c, 2) - pow(l3, 2)) / (2 * l2 * RR_c));

    RR_d = sqrt(pow((RR_pos[1] + l1 * cos(RR_q[0])), 2) + pow((RR_pos[2] + l1 * sin(RR_q[0])), 2));


    if (RR_pos[0] >= 0)
        RR_gamma = atan(abs(RR_pos[0]) / RR_d);
    else
        RR_gamma = atan(-abs(RR_pos[0]) / RR_d);


    RR_q[1] = (RR_theta - RR_gamma);


    joint_angle[3] = RR_q[0];
    joint_angle[4] = RR_q[1];
    joint_angle[5] = RR_q[2];

    //===================Front Left===================//
    FL_pos[0] = pos[6] - 0.35;
    FL_pos[1] = pos[7] - 0.115;
    FL_pos[2] = pos[8];

    double FL_a = sqrt(pow(FL_pos[1], 2) + pow(FL_pos[2], 2));

    if (FL_pos[1] >= 0.0)
        FL_q[0] = acos(abs(FL_pos[2]) / FL_a) + acos(l1 / FL_a) - PI / 2.0;
    else
        FL_q[0] = -acos(abs(FL_pos[2]) / sqrt(pow(FL_pos[1], 2) + pow(FL_pos[2], 2))) + acos(l1 / sqrt(pow(FL_pos[1], 2) + pow(FL_pos[2], 2))) - PI / 2.0;


    FL_c = sqrt(pow(FL_pos[0], 2) + pow(FL_pos[1] - l1 * cos(FL_q[0]), 2) + pow(FL_pos[2] - l1 * sin(FL_q[0]), 2));

    FL_q[2] = -acos((pow(FL_c, 2) - pow(l2, 2) - pow(l3, 2)) / (2.0 * l2 * l3));


    FL_theta = acos((pow(l2, 2) + pow(FL_c, 2) - pow(l3, 2)) / (2 * l2 * FL_c));

    FL_d = sqrt(pow((FL_pos[1] - l1 * cos(FL_q[0])), 2) + pow((FL_pos[2] - l1 * sin(FL_q[0])), 2));


    if (FL_pos[0] >= 0.0)
        FL_gamma = atan(abs(FL_pos[0]) / FL_d);
    else
        FL_gamma = atan(-abs(FL_pos[0]) / FL_d);

    FL_q[1] = (FL_theta - FL_gamma);


    joint_angle[6] = FL_q[0];
    joint_angle[7] = FL_q[1];
    joint_angle[8] = FL_q[2];

    //===================Front Right===================//

    FR_pos[0] = pos[9] - 0.35;
    FR_pos[1] = pos[10] + 0.115;
    FR_pos[2] = pos[11];

    double FR_a = sqrt(pow(FR_pos[1], 2) + pow(FR_pos[2], 2));

    if (FR_pos[1] >= 0.0)
        FR_q[0] = acos(abs(FR_pos[2]) / FR_a) - acos(l1 / FR_a) + PI / 2.0;
    else
        FR_q[0] = -acos(abs(FR_pos[2]) / FR_a) - acos(l1 / FR_a) + PI / 2.0;
   
    FR_c = sqrt(pow(FR_pos[0], 2) + pow(FR_pos[1] + l1 * cos(FR_q[0]), 2) + pow(FR_pos[2] + l1 * sin(FR_q[0]), 2));
    FR_q[2] = -acos((pow(FR_c, 2) - pow(l2, 2) - pow(l3, 2)) / (2.0 * l2 * l3));

    FR_theta = acos((pow(l2, 2) + pow(FR_c, 2) - pow(l3, 2)) / (2 * l2 * FR_c));
    FR_d = sqrt(pow((FR_pos[1] + l1 * cos(FR_q[0])), 2) + pow((FR_pos[2] + l1 * sin(FR_q[0])), 2));

    if (FR_pos[0] >= 0.0)
        FR_gamma = atan(abs(FR_pos[0]) / FR_d);
    else
        FR_gamma = atan(-abs(FR_pos[0]) / FR_d);

    FR_q[1] = (FR_theta - FR_gamma);

    joint_angle[9]  = FR_q[0];
    joint_angle[10] = FR_q[1];
    joint_angle[11] = FR_q[2];

    return joint_angle;

}

MatrixNd CRobot::Jac(VectorNd q) {


    VectorNd Position = VectorNd::Zero(12);
    MatrixNd RL_T_I1, RL_T_I2, RL_T_I3, RL_T_IE, R_I0, RL_R_I1, RL_R_I2, RL_R_I3;
    Vector3d RL_n_1, RL_n_2, RL_n_3, RL_r_I_I1, RL_r_I_I2, RL_r_I_I3, RL_r_I_IE;
    Vector3d RL_n_I_1, RL_n_I_2, RL_n_I_3;
    MatrixNd Jac_RL = MatrixNd::Zero(3, 3);

    MatrixNd RR_T_I1, RR_T_I2, RR_T_I3, RR_T_IE, RR_R_I1, RR_R_I2, RR_R_I3;
    Vector3d RR_n_1, RR_n_2, RR_n_3, RR_r_I_I1, RR_r_I_I2, RR_r_I_I3, RR_r_I_IE;
    Vector3d RR_n_I_1, RR_n_I_2, RR_n_I_3;
    MatrixNd Jac_RR = MatrixNd::Zero(3, 3);

    MatrixNd FL_T_I1, FL_T_I2, FL_T_I3, FL_T_IE, FL_R_I1, FL_R_I2, FL_R_I3;
    Vector3d FL_n_1, FL_n_2, FL_n_3, FL_r_I_I1, FL_r_I_I2, FL_r_I_I3, FL_r_I_IE;
    Vector3d FL_n_I_1, FL_n_I_2, FL_n_I_3;
    MatrixNd Jac_FL = MatrixNd::Zero(3, 3);

    MatrixNd FR_T_I1, FR_T_I2, FR_T_I3, FR_T_IE, FR_R_I1, FR_R_I2, FR_R_I3;
    Vector3d FR_n_1, FR_n_2, FR_n_3, FR_r_I_I1, FR_r_I_I2, FR_r_I_I3, FR_r_I_IE;
    Vector3d FR_n_I_1, FR_n_I_2, FR_n_I_3;
    MatrixNd Jac_FR = MatrixNd::Zero(3, 3);

    Vector3d I_r_1E, I_r_2E, I_r_3E;
    MatrixNd jac = MatrixNd::Zero(12, 12);

    
    FK(q);

    //RL
    RL_T_I1 = RL_I2B * RL_B2HR;
    RL_T_I2 = RL_T_I1 * RL_HR2HP;
    RL_T_I3 = RL_T_I2 * RL_HP2KN;
    RL_T_IE = RL_T_I3 * RL_KN2T;


    R_I0 = RL_I2B.block(0, 0, 3, 3);

    RL_R_I1 = RL_T_I1.block(0, 0, 3, 3);
    RL_R_I2 = RL_T_I2.block(0, 0, 3, 3);
    RL_R_I3 = RL_T_I3.block(0, 0, 3, 3);

    RL_r_I_I1 = RL_T_I1.block(0, 3, 3, 1);
    RL_r_I_I2 = RL_T_I2.block(0, 3, 3, 1);
    RL_r_I_I3 = RL_T_I3.block(0, 3, 3, 1);

    RL_n_1 << 1, 0, 0;
    RL_n_2 << 0, 1, 0;
    RL_n_3 << 0, 1, 0;


    RL_n_I_1 = R_I0 * RL_n_1;
    RL_n_I_2 = RL_R_I1 * RL_n_2;
    RL_n_I_3 = RL_R_I2 * RL_n_3;
   


    RL_r_I_IE = RL_T_IE.block(0, 3, 3, 1);

    jac.block(0, 0, 3, 1) << RL_n_I_1.cross(RL_r_I_IE - RL_r_I_I1);
    jac.block(0, 1, 3, 1) << RL_n_I_2.cross(RL_r_I_IE - RL_r_I_I2);
    jac.block(0, 2, 3, 1) << RL_n_I_3.cross(RL_r_I_IE - RL_r_I_I3);


    //RR
    RR_T_I1 = RR_I2B * RR_B2HR;
    RR_T_I2 = RR_T_I1 * RR_HR2HP;
    RR_T_I3 = RR_T_I2 * RR_HP2KN;

    RR_R_I1 = RR_T_I1.block(0, 0, 3, 3);
    RR_R_I2 = RR_T_I2.block(0, 0, 3, 3);
    RR_R_I3 = RR_T_I3.block(0, 0, 3, 3);

    RR_r_I_I1 = RR_T_I1.block(0, 3, 3, 1);
    RR_r_I_I2 = RR_T_I2.block(0, 3, 3, 1);
    RR_r_I_I3 = RR_T_I3.block(0, 3, 3, 1);

    RR_n_1 << 1, 0, 0;
    RR_n_2 << 0, 1, 0;
    RR_n_3 << 0, 1, 0;


    RR_n_I_1 = R_I0 * RR_n_1;
    RR_n_I_2 = RR_R_I1 * RR_n_2;
    RR_n_I_3 = RR_R_I2 * RR_n_3;

    RR_T_IE = RR_T_I3 * RL_KN2T;

    RR_r_I_IE = RR_T_IE.block(0, 3, 3, 1);


    jac.block(3, 3, 3, 1) << RR_n_I_1.cross(RR_r_I_IE - RR_r_I_I1);
    jac.block(3, 4, 3, 1) << RR_n_I_2.cross(RR_r_I_IE - RR_r_I_I2);
    jac.block(3, 5, 3, 1) << RR_n_I_3.cross(RR_r_I_IE - RR_r_I_I3);

    //FL
    FL_T_I1 = FL_I2B * FL_B2HR;
    FL_T_I2 = FL_T_I1 * FL_HR2HP;
    FL_T_I3 = FL_T_I2 * FL_HP2KN;

    FL_R_I1 = FL_T_I1.block(0, 0, 3, 3);
    FL_R_I2 = FL_T_I2.block(0, 0, 3, 3);
    FL_R_I3 = FL_T_I3.block(0, 0, 3, 3);

    FL_r_I_I1 = FL_T_I1.block(0, 3, 3, 1);
    FL_r_I_I2 = FL_T_I2.block(0, 3, 3, 1);
    FL_r_I_I3 = FL_T_I3.block(0, 3, 3, 1);

    FL_n_1 << 1, 0, 0;
    FL_n_2 << 0, 1, 0;
    FL_n_3 << 0, 1, 0;


    FL_n_I_1 = R_I0 * FL_n_1;
    FL_n_I_2 = FL_R_I1 * FL_n_2;
    FL_n_I_3 = FL_R_I2 * FL_n_3;

    FL_T_IE = FL_T_I3 * FL_KN2T;

    FL_r_I_IE = FL_T_IE.block(0, 3, 3, 1);


    jac.block(6, 6, 3, 1) << FL_n_I_1.cross(FL_r_I_IE - FL_r_I_I1);
    jac.block(6, 7, 3, 1) << FL_n_I_2.cross(FL_r_I_IE - FL_r_I_I2);
    jac.block(6, 8, 3, 1) << FL_n_I_3.cross(FL_r_I_IE - FL_r_I_I3);


    //FR
    FR_T_I1 = FR_I2B * FR_B2HR;
    FR_T_I2 = FR_T_I1 * FR_HR2HP;
    FR_T_I3 = FR_T_I2 * FR_HP2KN;

    FR_R_I1 = FR_T_I1.block(0, 0, 3, 3);
    FR_R_I2 = FR_T_I2.block(0, 0, 3, 3);
    FR_R_I3 = FR_T_I3.block(0, 0, 3, 3);

    FR_r_I_I1 = FR_T_I1.block(0, 3, 3, 1);
    FR_r_I_I2 = FR_T_I2.block(0, 3, 3, 1);
    FR_r_I_I3 = FR_T_I3.block(0, 3, 3, 1);

    FR_n_1 << 1, 0, 0;
    FR_n_2 << 0, 1, 0;
    FR_n_3 << 0, 1, 0;


    FR_n_I_1 = R_I0 * FR_n_1;
    FR_n_I_2 = FR_R_I1 * FR_n_2;
    FR_n_I_3 = FR_R_I2 * FR_n_3;

    FR_T_IE = FR_T_I3 * FR_KN2T;

    FR_r_I_IE = FR_T_IE.block(0, 3, 3, 1);


    jac.block(9, 9, 3, 1) << FR_n_I_1.cross(FR_r_I_IE - FR_r_I_I1);
    jac.block(9, 10, 3, 1) << FR_n_I_2.cross(FR_r_I_IE - FR_r_I_I2);
    jac.block(9, 11, 3, 1) << FR_n_I_3.cross(FR_r_I_IE - FR_r_I_I3);



    return jac;

}

void CRobot::coefficient_5thPoly(double *init_x, double *final_x, double tf, double *output) {
    double temp_t1 = 0;
    double temp_t2 = tf;
    //   MatrixXd R(6,1), A(6,6), P(6,1);

    R << init_x[0], final_x[0], init_x[1], final_x[1], init_x[2], final_x[2];

    A << 1, temp_t1, pow(temp_t1, 2), pow(temp_t1, 3), pow(temp_t1, 4), pow(temp_t1, 5),
            1, temp_t2, pow(temp_t2, 2), pow(temp_t2, 3), pow(temp_t2, 4), pow(temp_t2, 5),
            0, 1, 2 * pow(temp_t1, 1), 3 * pow(temp_t1, 2), 4 * pow(temp_t1, 3), 5 * pow(temp_t1, 4),
            0, 1, 2 * pow(temp_t2, 1), 3 * pow(temp_t2, 2), 4 * pow(temp_t2, 3), 5 * pow(temp_t2, 4),
            0, 0, 2, 6 * pow(temp_t1, 1), 12 * pow(temp_t1, 2), 20 * pow(temp_t1, 3),
            0, 0, 2, 6 * pow(temp_t2, 1), 12 * pow(temp_t2, 2), 20 * pow(temp_t2, 3);

    P = A.inverse() * R;

    output[0] = P(0, 0);
    output[1] = P(1, 0);
    output[2] = P(2, 0);
    output[3] = P(3, 0);
    output[4] = P(4, 0);
    output[5] = P(5, 0);
}

void CRobot::Torque_off(void) {
    cout<<"torque off"<<endl;
    for (int i = 0; i < nDOF; ++i) {
        joint[i].torque = 0;
    }
}


void CRobot::WalkReady_Pos_Traj(void) {
    if (wr_cnt == 0) {
        moving_done_flag = false;
        _c << 1, 1, 1, 1;
        contact_num = 4;

        init_RL_foot_pos = act_RL_foot_pos;
        init_RR_foot_pos = act_RR_foot_pos;
        init_FL_foot_pos = act_FL_foot_pos;
        init_FR_foot_pos = act_FR_foot_pos;

        com_pos = init_com_pos;
        com_vel = init_com_vel;

        RL_foot_pos = init_RL_foot_pos;
        RR_foot_pos = init_RR_foot_pos;
        FL_foot_pos = init_FL_foot_pos;
        FR_foot_pos = init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        base_ori << 0, 0, 0;
        base_ori_dot << 0, 0, 0;
        tmp_com_pos << 0, 0, 0;

        lpf_tar_pitch_ang = 0;

        fc_weight = 0;

        // for actual pos & vel
        pos_alpha = 1;
        vel_alpha = 1;

        wr_cnt++;
    } else if (wr_cnt <= walk_ready_cnt) {
        com_pos = init_com_pos;
        com_vel = init_com_vel;

        RL_foot_pos = init_RL_foot_pos + (tar_init_RL_foot_pos - init_RL_foot_pos) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)* (double) (wr_cnt) * dt));
        RR_foot_pos = init_RR_foot_pos + (tar_init_RR_foot_pos - init_RR_foot_pos) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)* (double) (wr_cnt) * dt));
        FL_foot_pos = init_FL_foot_pos + (tar_init_FL_foot_pos - init_FL_foot_pos) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)* (double) (wr_cnt) * dt));
        FR_foot_pos = init_FR_foot_pos + (tar_init_FR_foot_pos - init_FR_foot_pos) / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)* (double) (wr_cnt) * dt));

        RL_foot_vel = (tar_init_RL_foot_pos - init_RL_foot_pos)* (PI2 / (walk_ready_time * 2)) / 2.0 * (sin(PI2 / (walk_ready_time * 2) * (double) (wr_cnt) * dt));
        RR_foot_vel = (tar_init_RR_foot_pos - init_RR_foot_pos)* (PI2 / (walk_ready_time * 2)) / 2.0 * (sin(PI2 / (walk_ready_time * 2) * (double) (wr_cnt) * dt));
        FL_foot_vel = (tar_init_FL_foot_pos - init_FL_foot_pos)* (PI2 / (walk_ready_time * 2)) / 2.0 * (sin(PI2 / (walk_ready_time * 2) * (double) (wr_cnt) * dt));
        FR_foot_vel = (tar_init_FR_foot_pos - init_FR_foot_pos)* (PI2 / (walk_ready_time * 2)) / 2.0 * (sin(PI2 / (walk_ready_time * 2) * (double) (wr_cnt) * dt));

        fc_weight = 1 / 2.0 * (1 - cos(PI2 / (walk_ready_time * 2)* (double) (wr_cnt) * dt));

        if (wr_cnt == 1) {
            pos_alpha = 0.02;
            vel_alpha = 0.02;
        }
        wr_cnt++;
    } else if (wr_cnt <= walk_ready_cnt * 2) {
        com_pos =init_com_pos+ (tar_init_com_pos - init_com_pos) / 2.0* (1- cos(PI2 / (walk_ready_time * 2)* (double) (wr_cnt- walk_ready_cnt)* dt));
        com_vel = (tar_init_com_pos - init_com_pos)* (PI2 / (walk_ready_time * 2)) / 2.0 * (sin(PI2 / (walk_ready_time * 2)* (double) (wr_cnt - walk_ready_cnt) * dt));

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;

        fc_weight = 1;

        wr_cnt++;

        if (wr_cnt == walk_ready_cnt * 2) {
            walk_ready_moving_done_flag = true;
            cout << "!! Walk Ready Done !!" << endl;

            moving_done_flag = true;
        }
    } else {
        com_pos = tar_init_com_pos;
        com_vel = tar_init_com_vel;

        RL_foot_pos = tar_init_RL_foot_pos;
        RR_foot_pos = tar_init_RR_foot_pos;
        FL_foot_pos = tar_init_FL_foot_pos;
        FR_foot_pos = tar_init_FR_foot_pos;

        RL_foot_vel = tar_init_RL_foot_vel;
        RR_foot_vel = tar_init_RR_foot_vel;
        FL_foot_vel = tar_init_FL_foot_vel;
        FR_foot_vel = tar_init_FR_foot_vel;
    }
    base_pos = com_pos + base_offset;
}

void CRobot::FK2(void) {
    act_RL_foot_pos = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, true);
    act_RR_foot_pos = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, true);
    act_FL_foot_pos = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, true);
    act_FR_foot_pos = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, true);

    act_RL_foot_pos_local = act_RL_foot_pos - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);
    act_RR_foot_pos_local = act_RR_foot_pos - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);
    act_FL_foot_pos_local = act_FL_foot_pos - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);
    act_FR_foot_pos_local = act_FR_foot_pos - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);
    
//    printf("--> RL= ( %3f / %3f/ %3f ) \n", act_RL_foot_pos(0), act_RL_foot_pos(1), act_RL_foot_pos(2));
//    printf("--> RR= ( %3f / %3f/ %3f ) \n", act_RR_foot_pos(0), act_RR_foot_pos(1), act_RR_foot_pos(2));
//    printf("--> FL= ( %3f / %3f/ %3f ) \n", act_FL_foot_pos(0), act_FL_foot_pos(1), act_FL_foot_pos(2));
//    printf("--> FR= ( %3f / %3f/ %3f ) \n", act_FR_foot_pos(0), act_FR_foot_pos(1), act_FR_foot_pos(2));
    
//    printf("--> RL(L)= ( %3f / %3f/ %3f ) \n", act_RL_foot_pos_local(0), act_RL_foot_pos_local(1), act_RL_foot_pos_local(2));
//    printf("--> RR(L)= ( %3f / %3f/ %3f ) \n", act_RR_foot_pos_local(0), act_RR_foot_pos_local(1), act_RR_foot_pos_local(2));
//    printf("--> FL(L)= ( %3f / %3f/ %3f ) \n", act_FL_foot_pos_local(0), act_FL_foot_pos_local(1), act_FL_foot_pos_local(2));
//    printf("--> FR(L)= ( %3f / %3f/ %3f ) \n", act_FR_foot_pos_local(0), act_FR_foot_pos_local(1), act_FR_foot_pos_local(2));
    //cout << "=========================" << endl;
}

void CRobot::Get_act_com(void) {
    // ============== Get COM Position & Orientation ============ //
    if (contact_num != 0) {
        tmp_act_base_pos(0) = (_c(0) * (RL_foot_pos[0] - act_RL_foot_pos_local[0]) + _c(1) * (RR_foot_pos[0] - act_RR_foot_pos_local[0]) + _c(2) * (FL_foot_pos[0] - act_FL_foot_pos_local[0]) + _c(3) * (FR_foot_pos[0] - act_FR_foot_pos_local[0])) / contact_num;
        tmp_act_base_pos(1) = (_c(0) * (RL_foot_pos[1] - act_RL_foot_pos_local[1]) + _c(1) * (RR_foot_pos[1] - act_RR_foot_pos_local[1]) + _c(2) * (FL_foot_pos[1] - act_FL_foot_pos_local[1]) + _c(3) * (FR_foot_pos[1] - act_FR_foot_pos_local[1])) / contact_num;
        tmp_act_base_pos(2) = (_c(0) * (RL_foot_pos[2] - act_RL_foot_pos_local[2]) + _c(1) * (RR_foot_pos[2] - act_RR_foot_pos_local[2]) + _c(2) * (FL_foot_pos[2] - act_FL_foot_pos_local[2]) + _c(3) * (FR_foot_pos[2] - act_FR_foot_pos_local[2])) / contact_num;

        tmp_act_base_vel(0) = -(_c(0) * actual_EP_vel[0] + _c(1) * actual_EP_vel[3] + _c(2) * actual_EP_vel[6] + _c(3) * actual_EP_vel[9]) / contact_num;
        tmp_act_base_vel(1) = -(_c(0) * actual_EP_vel[1] + _c(1) * actual_EP_vel[4] + _c(2) * actual_EP_vel[7] + _c(3) * actual_EP_vel[10]) / contact_num;
        tmp_act_base_vel(2) = -(_c(0) * actual_EP_vel[2] + _c(1) * actual_EP_vel[5] + _c(2) * actual_EP_vel[8] + _c(3) * actual_EP_vel[11]) / contact_num;

    } else {
        tmp_act_base_pos(0) = ((RL_foot_pos[0] - act_RL_foot_pos_local[0])+ (RR_foot_pos[0] - act_RR_foot_pos_local[0])+ (FL_foot_pos[0] - act_FL_foot_pos_local[0])+ (FR_foot_pos[0] - act_FR_foot_pos_local[0])) / 4;
        tmp_act_base_pos(1) = ((RL_foot_pos[1] - act_RL_foot_pos_local[1])+ (RR_foot_pos[1] - act_RR_foot_pos_local[1])+ (FL_foot_pos[1] - act_FL_foot_pos_local[1])+ (FR_foot_pos[1] - act_FR_foot_pos_local[1])) / 4;
        tmp_act_base_pos(2) = ((RL_foot_pos[2] - act_RL_foot_pos_local[2])+ (RR_foot_pos[2] - act_RR_foot_pos_local[2])+ (FL_foot_pos[2] - act_FL_foot_pos_local[2])+ (FR_foot_pos[2] - act_FR_foot_pos_local[2])) / 4;

        tmp_act_base_vel(0) = -(actual_EP_vel[0] + actual_EP_vel[3] + actual_EP_vel[6] + actual_EP_vel[9]) / 4;
        tmp_act_base_vel(1) = -(actual_EP_vel[1] + actual_EP_vel[4] + actual_EP_vel[7] + actual_EP_vel[10]) / 4;
        tmp_act_base_vel(2) = -(actual_EP_vel[2] + actual_EP_vel[5] + actual_EP_vel[8] + actual_EP_vel[11]) / 4;
    }

    if (move_cnt == 0 && CommandFlag != GOTO_WALK_READY_POS) {
        act_base_pos = tmp_act_base_pos;
        act_base_vel = tmp_act_base_vel;
        pre_act_com_vel = act_base_vel;
    }

    act_base_pos = (1 - pos_alpha) * act_base_pos + pos_alpha * tmp_act_base_pos;
    act_base_vel = (1 - vel_alpha) * act_base_vel + vel_alpha * tmp_act_base_vel;
    
    act_com_pos = act_base_pos - tmp_com_pos - base_offset;
    act_com_vel = act_base_vel; //(0.90*lpf_base_alpha)*pre_act_com_vel + ((1 - 0.90)*lpf_base_alpha)*act_base_vel;

    tmp_act_com_acc = (act_com_vel - pre_act_com_vel) / dt;
    pre_act_com_vel = act_com_vel;

    act_com_acc = (1 - 0.02) * act_com_acc + 0.02 * tmp_act_com_acc;

    tmp_act_base_ori << IMURoll, IMUPitch, IMUYaw - init_IMUYaw;
    tmp_act_base_ori_dot << IMURoll_dot, IMUPitch_dot, IMUYaw_dot;

    // low pass filter
    const double tmp_base_alpha = 1;
    act_base_ori = (1 - tmp_base_alpha) * act_base_ori + tmp_base_alpha * tmp_act_base_ori;
    act_base_ori_dot = (1 - tmp_base_alpha) * act_base_ori_dot + tmp_base_alpha * tmp_act_base_ori_dot;

    if (act_base_ori(0) > 40 * D2R) {
        act_base_ori(0) = 40 * D2R;
    } else if (act_base_ori(0) < -40 * D2R) {
        act_base_ori(0) = -40 * D2R;
    }

    if (act_base_ori(1) > 40 * D2R) {
        act_base_ori(1) = 40 * D2R;
    } else if (act_base_ori(1) < -40 * D2R) {
        act_base_ori(1) = -40 * D2R;
    }
}

VectorNd CRobot::IK1(VectorNd EP) {
    const double L1 = 0.105;
    const double L2 = 0.305;
    const double L3 = 0.309; //0.305;

    static double x = 0;
    static double y = 0;
    static double z = 0;

    ROT_Y << cos(base_ori(1) * 1), 0, sin(base_ori(1) * 1)\
            , 0, 1, 0\
            , -sin(base_ori(1) * 1), 0, cos(base_ori(1) * 1);

    //    ROT_Y << 1, 0, 0,
    //    		 0, 1, 0,
    //    		 0, 0, 1;

    tmp_foot_pos << -(EP[0] - RL_base2hip_pos(0)), EP[1] - RL_base2hip_pos(1), EP[2] - RL_base2hip_pos(2);
    tmp_foot_pos2 = ROT_Y * tmp_foot_pos;

    //    x = tmp_foot_pos(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
    //    y = tmp_foot_pos(1);
    //    z = tmp_foot_pos(2); ///cos(base_ori(1));

    x = tmp_foot_pos2(0);
    y = tmp_foot_pos2(1);
    z = tmp_foot_pos2(2);

    //    cout << "tmp_foot_pos = " << tmp_foot_pos.transpose() << endl;
    //    cout << "tmp_foot_pos2 = " << tmp_foot_pos2.transpose() << endl;
    //    x = -(EP[0] - RL_base2hip_pos(0));
    //    y = EP[1] - RL_base2hip_pos(1);
    //    z = EP[2] - RL_base2hip_pos(2);

    if (z < (-0.6))
        z = -0.6;

    //    cout << "[RL1] x = " << x << ", y = " << y << ", z = " << z << endl;
    //    cout << "[RL2] x = " << x << ", y = " << y << ", z = " << z << endl;

    target_pos[0] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    target_pos[1] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[2] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    //	cout << "[RL] q[0] = " << target_pos[0]*R2D << ", q[1] = " << target_pos[1]*R2D << ", q[2] = " << target_pos[2]*R2D << endl;

    tmp_foot_pos << -(EP[3] - RR_base2hip_pos(0)), EP[4] - RR_base2hip_pos(1), EP[5]- RR_base2hip_pos(2);
    tmp_foot_pos2 = ROT_Y * tmp_foot_pos;

    x = tmp_foot_pos2(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
    y = tmp_foot_pos2(1);
    z = tmp_foot_pos2(2); ///cos(base_ori(1));

    //    x = tmp_foot_pos2(0);
    //    y = tmp_foot_pos2(1);
    //    z = tmp_foot_pos2(2);

    //    x = -(EP[3] - RR_base2hip_pos(0));
    //    y = EP[4] - RR_base2hip_pos(1);
    //    z = EP[5] - RR_base2hip_pos(2);

    if (z < (-0.6))
        z = -0.6;

    target_pos[3] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); //PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[4] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[5] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    tmp_foot_pos << -(EP[6] - FL_base2hip_pos(0)), EP[7] - FL_base2hip_pos(1), EP[8]- FL_base2hip_pos(2);
    tmp_foot_pos2 = ROT_Y * tmp_foot_pos;

    x = tmp_foot_pos2(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
    y = tmp_foot_pos2(1);
    z = tmp_foot_pos2(2); ///cos(base_ori(1));
    //    x = tmp_foot_pos2(0);
    //    y = tmp_foot_pos2(1);
    //    z = tmp_foot_pos2(2);

    //    x = -(EP[6] - FL_base2hip_pos(0));
    //    y = EP[7] - FL_base2hip_pos(1);
    //    z = EP[8] - FL_base2hip_pos(2);

    if (z < (-0.6))
        z = -0.6;

    target_pos[7] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    target_pos[8] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[9] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    tmp_foot_pos << -(EP[9] - FR_base2hip_pos(0)), EP[10] - FR_base2hip_pos(1), EP[11] - FR_base2hip_pos(2);
    tmp_foot_pos2 = ROT_Y * tmp_foot_pos;

    x = tmp_foot_pos2(0); // + 0.4*tan(base_ori(1));//tmp_foot_pos2(0);
    y = tmp_foot_pos2(1);
    z = tmp_foot_pos2(2); ///cos(base_ori(1));
    //    x = tmp_foot_pos2(0);
    //    y = tmp_foot_pos2(1);
    //    z = tmp_foot_pos2(2);

    if (z < (-0.6))
        z = -0.6;

    target_pos[10] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); //PI/2 - acos(L1/sqrt(pow(y,2) + pow(z,2))) - atan(abs(y)/abs(z)); //-((atan(y/abs(z)) - PI/2 + acos(L1/(sqrt(pow(y,2) + pow(z,2))))));
    target_pos[11] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[12] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    //    target_pos[6] = 0;
    //    cout << "tar_pos = " << target_pos.transpose()*R2D << endl;

    return target_pos;
}

VectorNd CRobot::Get_COM(VectorNd base, VectorNd q) {
    const double m_body = 24.333;
    const double m_hp = 1.4;
    const double m_thigh = 3.209;
    const double m_calf = 0.634;
    const double m_leg = m_hp + m_thigh + m_calf;
    const double m_robot = m_leg * 4 + m_body;

    // Transformation & Rotation matrix (Base)
    R_w2base_R << 1, 0, 0, 0\
            , 0, cos(base(3)), -sin(base(3)), 0\
            , 0, sin(base(3)), cos(base(3)), 0\
            , 0, 0, 0, 1;
    R_w2base_P << cos(base(4)), 0, sin(base(4)), 0\
            , 0, 1, 0, 0\
            , -sin(base(4)), 0, cos(base(4)), 0\
            , 0, 0, 0, 1;
    R_w2base_Y << cos(base(5)), -sin(base(5)), 0, 0\
            , sin(base(5)), cos(base(5)), 0, 0\
            , 0, 0, 1, 0\
            , 0, 0, 0, 1;

    R_w2base = R_w2base_Y * R_w2base_P * R_w2base_R;
    T_w2base << 1, 0, 0, base(0)\
            , 0, 1, 0, base(1)\
            , 0, 0, 1, base(2)\
            , 0, 0, 0, 1;

    // Transformation & Rotation matrix (Leg)
    // RL
    TR_RL_base2hp << 1, 0, 0, -0.35\
            , 0, cos(q(0)), -sin(q(0)), 0.115\
            , 0, sin(q(0)), cos(q(0)), -0.053\
            , 0, 0, 0, 1;

    TR_RL_hp2thigh << cos(q(1)), 0, sin(q(1)), 0.0\
            , 0, 1, 0, 0.105\
            , -sin(q(1)), 0, cos(q(1)), 0.0\
            , 0, 0, 0, 1;

    TR_RL_thigh2calf << cos(q(2)), 0, sin(q(2)), 0.0\
            , 0, 1, 0, 0.0\
            , -sin(q(2)), 0, cos(q(2)), -0.305\
            , 0, 0, 0, 1;

    p_RL_base2hp_com = TR_RL_base2hp * p_RL_hp_com;
    p_RL_base2thigh_com = TR_RL_base2hp * TR_RL_hp2thigh * p_RL_thigh_com;
    p_RL_base2calf_com = TR_RL_base2hp * TR_RL_hp2thigh * TR_RL_thigh2calf * p_RL_calf_com;

    p_RL_com = (m_hp * p_RL_base2hp_com + m_thigh * p_RL_base2thigh_com + m_calf * p_RL_base2calf_com) / (m_leg);

    //    cout << "p_RL_com = " << p_RL_com << endl;

    // RR
    TR_RR_base2hp << 1, 0, 0, -0.35\
             , 0, cos(q(3)), -sin(q(3)), -0.115\
            , 0, sin(q(3)), cos(q(3)), -0.053\
            , 0, 0, 0, 1;

    TR_RR_hp2thigh << cos(q(4)), 0, sin(q(4)), 0.0\
            , 0, 1, 0, -0.105\
            , -sin(q(4)), 0, cos(q(4)), 0.0\
            , 0, 0, 0, 1;

    TR_RR_thigh2calf << cos(q(5)), 0, sin(q(5)), 0.0\
            , 0, 1, 0, 0.0\
            , -sin(q(5)), 0, cos(q(5)), -0.305\
            , 0, 0, 0, 1;

    p_RR_base2hp_com = TR_RR_base2hp * p_RR_hp_com;
    p_RR_base2thigh_com = TR_RR_base2hp * TR_RR_hp2thigh * p_RR_thigh_com;
    p_RR_base2calf_com = TR_RR_base2hp * TR_RR_hp2thigh * TR_RR_thigh2calf * p_RR_calf_com;

    p_RR_com = (m_hp * p_RR_base2hp_com + m_thigh * p_RR_base2thigh_com + m_calf * p_RR_base2calf_com) / (m_leg);

    //    cout << "p_RR_com = " << p_RR_com << endl;

    // FL
    TR_FL_base2hp << 1, 0, 0, 0.35\
            , 0, cos(q(0)), -sin(q(0)), 0.115\
            , 0, sin(q(0)), cos(q(0)), -0.053\
            , 0, 0, 0, 1;

    TR_FL_hp2thigh << cos(q(1)), 0, sin(q(1)), 0.0\
            , 0, 1, 0, 0.105\
            , -sin(q(1)), 0, cos(q(1)), 0.0\
            , 0, 0, 0, 1;

    TR_FL_thigh2calf << cos(q(2)), 0, sin(q(2)), 0.0\
            , 0, 1, 0, 0.0\
            , -sin(q(2)), 0, cos(q(2)), -0.305\
            , 0, 0, 0, 1;

    p_FL_base2hp_com = TR_FL_base2hp * p_FL_hp_com;
    p_FL_base2thigh_com = TR_FL_base2hp * TR_FL_hp2thigh * p_FL_thigh_com;
    p_FL_base2calf_com = TR_FL_base2hp * TR_FL_hp2thigh * TR_FL_thigh2calf * p_FL_calf_com;

    p_FL_com = (m_hp * p_FL_base2hp_com + m_thigh * p_FL_base2thigh_com + m_calf * p_FL_base2calf_com) / (m_leg);

    //    cout << "p_FL_com = " << p_FL_com << endl;

    // FR
    TR_FR_base2hp << 1, 0, 0, 0.35\
            , 0, cos(q(3)), -sin(q(3)), -0.115\
            , 0, sin(q(3)), cos(q(3)), -0.053\
            , 0, 0, 0, 1;

    TR_FR_hp2thigh << cos(q(4)), 0, sin(q(4)), 0.0\
            , 0, 1, 0, -0.105\
            , -sin(q(4)), 0, cos(q(4)), 0.0\
            , 0, 0, 0, 1;

    TR_FR_thigh2calf << cos(q(5)), 0, sin(q(5)), 0.0\
            , 0, 1, 0, 0.0\
            , -sin(q(5)), 0, cos(q(5)), -0.305\
            , 0, 0, 0, 1;

    p_FR_base2hp_com = TR_FR_base2hp * p_FR_hp_com;
    p_FR_base2thigh_com = TR_FR_base2hp * TR_FR_hp2thigh * p_FR_thigh_com;
    p_FR_base2calf_com = TR_FR_base2hp * TR_FR_hp2thigh * TR_FR_thigh2calf * p_FR_calf_com;

    p_FR_com = (m_hp * p_FR_base2hp_com + m_thigh * p_FR_base2thigh_com + m_calf * p_FR_base2calf_com) / (m_leg);

    //    cout << "p_FR_com = " << p_FR_com << endl;

    // COM from base
    p_robot_com_from_base = (m_body * p_base2body_com + m_leg * (p_RL_com + p_RR_com + p_FL_com + p_FR_com)) / (m_robot);
    p_robot_com_from_w = T_w2base * R_w2base * p_robot_com_from_base;
    p_robot_com = p_robot_com_from_w.block<3, 1>(0, 0);

    printf("--> p_robot_com= ( %3f / %3f/ %3f ) \n", p_robot_com(0), p_robot_com(1), p_robot_com(2));
    return p_robot_com;
}

void CRobot::set_simul_para(void) {
    if (CommandFlag == NO_ACT || CommandFlag == GOTO_WALK_READY_POS || CommandFlag == NOMAL_TROT_WALKING || CommandFlag == TEST_FLAG) {
        foot_height = 0.10;
        _alpha = 0.001;
        dsp_time = 0.20;
        fsp_time = 0.10; //0.07
        step_time = dsp_time + fsp_time;
        dsp_cnt = 200;
        fsp_cnt = 100; //70;
        step_cnt = dsp_cnt + fsp_cnt;

        tar_Kp_q << 400, 100, 100, 400, 100, 100, 400, 100, 100, 400, 100, 100;
        tar_Kd_q << 15, 5, 5, 15, 5, 5, 15, 5, 5, 15, 5, 5;

        tar_Kp_t << 2000, 0, 500, 2000, 0, 500, 2000, 0, 500, 2000, 0, 500;
        tar_Kd_t << 10, 0, 5, 10, 0, 5, 10, 0, 5, 10, 0, 5;

        // for gain scheduling
        tar_Kp_q_low << 400, 50, 50, 400, 50, 50, 400, 50, 50, 400, 50, 50;
        tar_Kd_q_low << 15, 4, 4, 15, 4, 4, 15, 4, 4, 15, 4, 4;

        tar_Kp_t_low << 2000, 0, 0, 2000, 0, 0, 2000, 0, 0, 2000, 0, 0;
        tar_Kd_t_low << 10, 0, 0, 10, 0, 0, 10, 0, 0, 10, 0, 0;

        tar_Kp_x << 1, 0, 0\
                   , 0, 50, 0\
                    , 0, 0, 1;
        tar_Kd_x << 0.01, 0, 0\
                   , 0, 5, 0\
                   , 0, 0, 0.1;
        
        tar_Kp_w << 1000, 0, 0\
                , 0, 1000, 0\
                , 0, 0, 0;
        tar_Kd_w << 10, 0, 0\
                , 0, 10, 0\
                , 0, 0, 0;
        
    } else if (CommandFlag == STAIR_WALKING) {
        foot_height = 0.10;
        _alpha = 0.001;
        dsp_time = 0.20;
        fsp_time = 0.50; //0.07
        step_time = dsp_time + fsp_time;
        dsp_cnt = 200;
        fsp_cnt = 500; //70;
        step_cnt = dsp_cnt + fsp_cnt;

        tar_Kp_q << 400, 20, 20, 400, 20, 20, 400, 20, 20, 400, 20, 20;
        tar_Kd_q << 15, 2, 2, 15, 2, 2, 15, 2, 2, 15, 2, 2;

        tar_Kp_t << 1000, 0, 100, 1000, 0, 100, 1000, 0, 100, 1000, 0, 100;
        tar_Kd_t << 10, 0, 1, 10, 0, 1, 10, 0, 1, 10, 0, 1;

        // for gain scheduling
        tar_Kp_q_low << 400, 10, 10, 400, 10, 10, 400, 10, 10, 400, 10, 10;
        tar_Kd_q_low << 15, 1, 1, 15, 1, 1, 15, 1, 1, 15, 1, 1;

        tar_Kp_t_low << 1000, 0, 0, 1000, 0, 0, 1000, 0, 0, 1000, 0, 0;
        tar_Kd_t_low << 10, 0, 0, 10, 0, 0, 10, 0, 0, 10, 0, 0;

        tar_Kp_x << 1, 0, 0, 0, 50, 0, 0, 0, 30;
        tar_Kd_x << 0.01, 0, 0, 0, 5, 0, 0, 0, 3;
        
        tar_Kp_w << 1000, 0, 0\
                , 0, 1000, 0\
                , 0, 0, 0;
        tar_Kd_w << 10, 0, 0\
                , 0, 10, 0\
                , 0, 0, 0;
        
    } else if (CommandFlag == FLYING_TROT_RUNNING || CommandFlag == PRONK_JUMP) {
        swing_foot_height = 0.06; // flying trot
        _alpha = 0.0001;
        
        // =============== Flying trot parameters initialize =============== //
        ts = 0.20; //0.22; //0.25;
        tf = 0.05; //0.07;
        ft_step_time = ts + tf;

        ts_cnt = 200; //220;
        tf_cnt = 50;
        ft_step_cnt = ts_cnt + tf_cnt;

        h_0 = tar_init_com_pos(2);
        v_0 = 0;
        a_0 = 0;

        v_1 = 0.2; //0.10; //0.10; //0.15;
        a_1 = -GRAVITY;

        h_2 = tar_init_com_pos(2);
        v_2 = -0.0; //-0.05; // -0.3
        a_2 = -GRAVITY;

        h_3 = tar_init_com_pos(2);
        v_3 = 0;
        a_3 = 0;

        h_1 = 0.5 * GRAVITY * tf * tf - v_1 * tf + h_2;

        // =============== Flying trot parameters initialize END =============== //
        // for gain scheduling
        tar_Kp_q_low << 300, 100, 100, 300, 100, 100, 300, 100, 100, 300, 100, 100;
        tar_Kd_q_low << 10, 5, 5, 10, 5, 5, 10, 5, 5, 10, 5, 5;

        tar_Kp_t_low << 2000, 3000, 1000, 2000, 3000, 1000, 2000, 3000, 1000, 2000, 3000, 1000;
        tar_Kd_t_low << 15, 20, 10, 15, 20, 10, 15, 20, 10, 15, 20, 10;

        tar_Kp_x << 5, 0, 0\
                , 0, 50, 0\
                , 0, 0, 5;
        tar_Kd_x << 0.5, 0, 0\
                , 0, 5, 0\
                , 0, 0, 0.1;
        tar_Kp_w << 1000, 0, 0\
                , 0, 1000, 0\
                , 0, 0, 0;
        tar_Kd_w << 10, 0, 0\
                , 0, 10, 0\
                , 0, 0, 0;
    }
}

void CRobot::set_act_robot_para(void) {
    if (CommandFlag == NO_ACT || CommandFlag == GOTO_WALK_READY_POS || CommandFlag == NOMAL_TROT_WALKING || CommandFlag == TEST_FLAG) {
        foot_height = 0.06; //0.10;
        _alpha = 0.001; //0.01; // 0.001
        dsp_time = 0.20;
        fsp_time = 0.10; //0.07
        step_time = dsp_time + fsp_time;
        dsp_cnt = 200;
        fsp_cnt = 100; //70;
        step_cnt = dsp_cnt + fsp_cnt;

        tar_Kp_q << 500, 150, 100, 500, 150, 100, 500, 150, 100, 500, 150, 100;
        tar_Kd_q << 10, 6, 5, 10, 6, 5, 10, 6, 5, 10, 6, 5;

        tar_Kp_t << 1000, 0, 100, 1000, 0, 100, 1000, 0, 100, 1000, 0, 100;
        tar_Kd_t << 10, 0, 1, 10, 0, 1, 10, 0, 1, 10, 0, 1;

        // for gain scheduling
        tar_Kp_q_low << 500, 80, 50, 500, 80, 50, 500, 80, 50, 500, 80, 50;
        tar_Kd_q_low << 10, 4, 2, 10, 4, 2, 10, 4, 2, 10, 4, 2;

        tar_Kp_t_low << 1000, 0, 0, 1000, 0, 0, 1000, 0, 0, 1000, 0, 0;
        tar_Kd_t_low << 10, 0, 0, 10, 0, 0, 10, 0, 0, 10, 0, 0;

        // ========= QP Gain ========= //
        tar_Kp_x << 1, 0, 0\
                , 0, 50, 0\
                , 0, 0, 1;
        tar_Kd_x << 0.01, 0, 0\
                , 0, 0.5, 0\
                , 0, 0, 0.01;
        tar_Kp_w << 1000, 0, 0\
                , 0, 1000, 0\
                , 0, 0, 0;
        tar_Kd_w << 10, 0, 0\
                , 0, 10, 0\
                , 0, 0, 0;

    } else if (CommandFlag == STAIR_WALKING) {
        foot_height = 0.12; //0.10;
        _alpha = 0.001; // 0.001
        dsp_time = 0.20;
        fsp_time = 0.60; //0.07
        step_time = dsp_time + fsp_time;
        dsp_cnt = 200;
        fsp_cnt = 600; //70;
        step_cnt = dsp_cnt + fsp_cnt;

        tar_Kp_q << 500, 20, 20, 500, 20, 20, 500, 20, 20, 500, 20, 20;
        tar_Kd_q << 10, 2, 2, 10, 2, 2, 10, 2, 2, 10, 2, 2;

        tar_Kp_t << 2000, 0, 100, 2000, 0, 100, 2000, 0, 100, 2000, 0, 100;
        tar_Kd_t << 20, 0, 1, 20, 0, 1, 20, 0, 1, 20, 0, 1;

        // for gain scheduling
        tar_Kp_q_low << 500, 10, 10, 500, 10, 10, 500, 10, 10, 500, 10, 10;
        tar_Kd_q_low << 10, 1, 1, 10, 1, 1, 10, 1, 1, 10, 1, 1;

        tar_Kp_t_low << 2000, 0, 0, 2000, 0, 0, 2000, 0, 0, 2000, 0, 0;
        tar_Kd_t_low << 20, 0, 0, 20, 0, 0, 20, 0, 0, 20, 0, 0;

        // ========= QP Gain ========= //
        tar_Kp_x << 1, 0, 0\
                , 0, 50, 0\
                , 0, 0, 10;
        tar_Kd_x << 0.1, 0, 0\
                , 0, 5, 0\
                , 0, 0, 1;
        tar_Kp_w << 1000, 0, 0\
                , 0, 1000, 0\
                , 0, 0, 0;
        tar_Kd_w << 10, 0, 0\
                , 0, 10, 0\
                , 0, 0, 0;

    } else if (CommandFlag == FLYING_TROT_RUNNING || CommandFlag == PRONK_JUMP) {

        swing_foot_height = 0.06; // flying trot
        _alpha = 0.001;

        // =============== Flying trot parameters initialize =============== //
        ts = 0.20; //0.22; //0.25;
        tf = 0.05; //0.07;
        ft_step_time = ts + tf;

        ts_cnt = 200; //220;
        tf_cnt = 50;
        ft_step_cnt = ts_cnt + tf_cnt;

        h_0 = tar_init_com_pos(2);
        v_0 = 0;
        a_0 = 0;

        v_1 = 0.20; //0.10; //0.15;
        a_1 = -GRAVITY;

        h_2 = tar_init_com_pos(2);
        v_2 = -0.0; //-0.05; // -0.3
        a_2 = -GRAVITY;

        h_3 = tar_init_com_pos(2);
        v_3 = 0;
        a_3 = 0;

        h_1 = 0.5 * GRAVITY * tf * tf - v_1 * tf + h_2;

        // =============== Flying trot parameters initialize END =============== //

        // for gain scheduling
        tar_Kp_q_low << 200, 100, 100, 200, 100, 100, 0, 200, 100, 100, 200, 100, 100;
        tar_Kd_q_low << 10, 5, 5, 10, 5, 5, 0, 10, 5, 5, 10, 5, 5;

        tar_Kp_t_low << 2000, 0, 0, 2000, 0, 0, 2000, 0, 0, 2000, 0, 0;
        tar_Kd_t_low << 20, 0, 0, 20, 0, 0, 20, 0, 0, 20, 0, 0;

        tar_Kp_x << 1, 0, 0\
                , 0, 30, 0\
                , 0, 0, 1;
        tar_Kd_x << 0.1, 0, 0\
                , 0, 3, 0\
                , 0, 0, 0.1;
        tar_Kp_w << 1000, 0, 0\
                , 0, 500, 0\
                , 0, 0, 0;
        tar_Kd_w << 10, 0, 0\
                , 0, 5, 0\
                , 0, 0, 0;
    }
}

void CRobot::QP_Con_Init(void) {
    cout << "QP Init Start !! " << endl;
    // Selection matrix
    S_mat.block<6, 18>(0, 0) = MatrixNd::Zero(6, 18);
    S_mat.block<12, 6>(6, 0) = MatrixNd::Zero(12, 6);
    S_mat.block<12, 12>(6, 6) = MatrixNd::Identity(12, 12);

    des_x_2dot << 0, 0, 0;
    des_w_dot << 0, 0, 0;

    //    _m = 46;//43;//53; //48.5; //kg
    _m = 53;
    //    _I_g << 1.7214, -0.0038, -0.1540,
    //            -0.0038, 4.9502, -0.0006,
    //            -0.1540, -0.0006, 5.1152;

    //    _I_g << 1.7214, 0, 0,
    //            0, 4.9502, 0,
    //            0, 0, 5.1152;

    //    _I_g << 1.5, 0, 0,
    //            0, 4.0, 0,
    //            0, 0, 2.0;
    //     _I_g << 1.5, 0, 0,
    //            0, 5.0, 0,
    //            0, 0, 2.0;

    _I_g << 1.5, 0, 0\
            , 0, 5.0, 0\
            , 0, 0, 2.0;

    //    _I_g << 0.7, 0, 0,
    //           0, 4.0, 0,
    //           0, 0, 2.0;

    _A.block<3, 3>(0, 0) = MatrixNd::Identity(3, 3);
    _A.block<3, 3>(0, 3) = MatrixNd::Identity(3, 3);
    _A.block<3, 3>(0, 6) = MatrixNd::Identity(3, 3);
    _A.block<3, 3>(0, 9) = MatrixNd::Identity(3, 3);

    tar_RL_foot_pos_local = tar_init_RL_foot_pos - init_base_pos;
    tar_RR_foot_pos_local = tar_init_RR_foot_pos - init_base_pos;
    tar_FL_foot_pos_local = tar_init_FL_foot_pos - init_base_pos;
    tar_FR_foot_pos_local = tar_init_FR_foot_pos - init_base_pos;

//    cout << "tar_RL_foot_pos_local = " << tar_RL_foot_pos_local.transpose() << endl;
//    cout << "tar_RR_foot_pos_local = " << tar_RR_foot_pos_local.transpose() << endl;
//    cout << "tar_FL_foot_pos_local = " << tar_FL_foot_pos_local.transpose() << endl;
//    cout << "tar_FR_foot_pos_local = " << tar_FR_foot_pos_local.transpose() << endl;

    p_com_oross_pro << 0, -tar_RL_foot_pos_local(2), tar_RL_foot_pos_local(1), 0, -tar_RR_foot_pos_local(2), tar_RR_foot_pos_local(1), 0, -tar_FL_foot_pos_local(2), tar_FL_foot_pos_local(1), 0, -tar_FR_foot_pos_local(2), tar_FR_foot_pos_local(1)\
            , tar_RL_foot_pos_local(2), 0, -tar_RL_foot_pos_local(0), tar_RR_foot_pos_local(2), 0, -tar_RR_foot_pos_local(0), tar_FL_foot_pos_local(2), 0, -tar_FL_foot_pos_local(0), tar_FR_foot_pos_local(2), 0, -tar_FR_foot_pos_local(0)\
            , -tar_RL_foot_pos_local(1), tar_RL_foot_pos_local(0), 0, -tar_RR_foot_pos_local(1), tar_RR_foot_pos_local(0), 0, -tar_FL_foot_pos_local(1), tar_FL_foot_pos_local(0), 0, -tar_FR_foot_pos_local(1), tar_FR_foot_pos_local(0), 0;

    _A.block<3, 12>(3, 0) = p_com_oross_pro;

    _g << 0, 0, 9.81;

    _b << _m * (des_x_2dot + _g), _I_g * des_w_dot;

//    cout << "_b = " << _b << endl;
//    //    fx_max = 50;
//    //    fx_min = -50;
//    //    fy_max = 50;
//    //    fy_min = -50;
//
    _P = _A.transpose() * _S * _A + _alpha * _W;
    _q = -_A.transpose() * _S * _b;

    // ===================== OSQP  ====================== //
    int jj = 0;
    int kk = 0;
    int max_jj = 0;
//
    // ===================== P_x ====================== //
    for (unsigned int i = 0; i < P_nnz; ++i) {
        P_x[i] = _P(jj, kk);
        jj = jj + 1;

        if (jj > max_jj) {
            jj = 0;
            kk = kk + 1;
            max_jj = max_jj + 1;
        }
        //        cout << "i = " << i << ", P_x = " << P_x[i] << endl;
    }

    // ===================== P_i ====================== //
    jj = 0;
    max_jj = 0;

    for (unsigned int i = 0; i < P_nnz; ++i) {
        P_i[i] = jj;
        jj = jj + 1;

        if (jj > max_jj) {
            jj = 0;
            max_jj = max_jj + 1;
        }
        //        cout << "i = " << i << ", P_i = " << P_i[i] << endl;
    }

    // ===================== P_p ====================== //
    P_p[0] = 0;
    for (unsigned int i = 1; i < A_nnz + 1; ++i) {
        P_p[i] = P_p[i - 1] + i;

        //        cout << "i = " << i-1 << ", P_p = " << P_p[i-1] << endl;
    }
    //    cout << "i = " << A_nnz << ", P_p = " << P_p[A_nnz] << endl;

    // ===================== A_x ====================== //

    for (unsigned int i = 0; i < A_nnz; ++i) {
        A_x[i] = 1;

        //        cout << "i = " << i << ", A_x = " << A_x[i] << endl;
    }

    // ===================== A_i ====================== //

    for (unsigned int i = 0; i < A_nnz; ++i) {
        A_i[i] = i;

        //        cout << "i = " << i << ", A_i = " << A_i[i] << endl;
    }

    // ===================== A_p ====================== //
    jj = 0;
    for (unsigned int i = 0; i < A_nnz + 1; ++i) {
        A_p[i] = jj;

        jj = jj + 1;

        //        cout << "i = " << i << ", A_p = " << A_p[i] << endl;
    }
//    //        cout << "i = " << A_nnz << ", A_p = " << A_p[A_nnz] << endl;
//    // ===================== G_l & G_u ====================== //

    //    fz_max = 500;
    //    fz_min = 0;

    _c << 1, 1, 1, 1;

    if (_c(0) == 0) {
        fz_RL_max = 0;
        fz_RL_min = 0;
    } else {
        fz_RL_max = max_Fext_z;
        fz_RL_min = 0;
    }

    if (_c(1) == 0) {
        fz_RR_max = 0;
        fz_RR_min = 0;
    } else {
        fz_RR_max = max_Fext_z;
        fz_RR_min = 0;
    }

    if (_c(2) == 0) {
        fz_FL_max = 0;
        fz_FL_min = 0;
    } else {
        fz_FL_max = max_Fext_z;
        fz_FL_min = 0;
    }

    if (_c(3) == 0) {
        fz_FR_max = 0;
        fz_FR_min = 0;
    } else {
        fz_FR_max = max_Fext_z;
        fz_FR_min = 0;
    }

    Fc << 0, 0, 0, 0, 0, 0, 0, 0, 140, 0, 0, 140, 0, 0, 140, 0, 0, 140;

    _d_u << mu * abs(Fc(2 + 6)), mu * abs(Fc(2 + 6)), fz_RL_max, mu * abs(Fc(5 + 6)), mu * abs(Fc(5 + 6)), fz_RR_max, mu * abs(Fc(8 + 6)), mu * abs(Fc(8 + 6)), fz_FL_max, mu * abs(Fc(11 + 6)), mu * abs(Fc(11 + 6)), fz_FR_max;
    _d_l << -mu * abs(Fc(2 + 6)), -mu * abs(Fc(2 + 6)), fz_RL_min, -mu * abs(Fc(5 + 6)), -mu * abs(Fc(5 + 6)), fz_RR_min, -mu * abs(Fc(8 + 6)), -mu * abs(Fc(8 + 6)), fz_FL_min, -mu * abs(Fc(11 + 6)), -mu * abs(Fc(11 + 6)), fz_FR_min;

    //    c_vec << _c(0),_c(0),_c(0),_c(1),_c(1),_c(1),_c(2),_c(2),_c(2),_c(3),_c(3),_c(3);

    //    cout << "c_vec = " << c_vec.transpose() << endl;

    for (unsigned int i = 0; i < A_nnz; ++i) {
        l[i] = _d_l(i);
        u[i] = _d_u(i);

        //        cout << "i = " << i << ", G_l = " << G_l[i] << endl;
        //        cout << "i = " << i << ", G_u = " << G_u[i] << endl;
        //        cout << "==============================" << endl;
    }

    for (unsigned int i = 0; i < A_nnz; ++i) {
        q[i] = _q(i);
    }

    // Populate data
    if (QP_data) {
        QP_data->n = n;
        QP_data->m = m;
        QP_data->P = csc_matrix(QP_data->n, QP_data->n, P_nnz, P_x, P_i, P_p);
        QP_data->q = q;
        QP_data->A = csc_matrix(QP_data->m, QP_data->n, A_nnz, A_x, A_i, A_p);
        QP_data->l = l;
        QP_data->u = u;
    }

    // Define solver settings as default
    if (QP_settings) {
        osqp_set_default_settings(QP_settings);
        QP_settings->alpha = 1; // Change alpha parameter
    }

    // Setup workspace
    QP_exitflag = osqp_setup(&QP_work, QP_data, QP_settings);

    // Solve Problem
    osqp_solve(QP_work);

    cout << "[RL] x = " << QP_work->solution->x[0] << ", y = "
            << QP_work->solution->x[1] << ", z = " << QP_work->solution->x[2]
            << endl;
    cout << "[RR] x = " << QP_work->solution->x[3] << ", y = "
            << QP_work->solution->x[4] << ", z = " << QP_work->solution->x[5]
            << endl;
    cout << "[FL] x = " << QP_work->solution->x[6] << ", y = "
            << QP_work->solution->x[7] << ", z = " << QP_work->solution->x[8]
            << endl;
    cout << "[FR] x = " << QP_work->solution->x[9] << ", y = "
            << QP_work->solution->x[10] << ", z = " << QP_work->solution->x[11]
            << endl;

    cout << "QP Init Done !! " << endl;

    // Cleanup
    // ==================== OSQP TEST END =================== //

}