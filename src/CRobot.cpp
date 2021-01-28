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

    m_pModel = getModel;
    m_pModel->gravity = Vector3d(0., 0., -9.81);
    nDOF = m_pModel->dof_count - 6; //* get Degree of freedom, Except x,y,z,roll,pitch,yaw of the robot
    joint = new JOINT[nDOF]; //* only joint of the robot excepting x,y,z,roll,pitch,yaw of the robot

    RobotState = VectorNd::Zero(19);
    RobotStatedot = VectorNd::Zero(18);
    RobotState2dot = VectorNd::Zero(18);
    BasePosOri = VectorNd::Zero(6);
    BaseVel = VectorNd::Zero(6);
    //    JointAngle = VectorNd::Zero(nDOF);
    //    JointVel = VectorNd::Zero(nDOF);

    base.ID = m_pModel->GetBodyId("BODY");
    RL.ID = m_pModel->GetBodyId("RL_CALF");
    RR.ID = m_pModel->GetBodyId("RR_CALF");
    FL.ID = m_pModel->GetBodyId("FL_CALF");
    FR.ID = m_pModel->GetBodyId("FR_CALF");


    QQ << 0, 0, 0, 1;
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);

    for (unsigned int i = 0; i < nDOF; ++i) {
        joint[i].torque = 0;
    }

    base.currentX = 0;
    base.currentY = 0;
    base.currentZ = 0;
    base.currentRoll = 0;
    base.currentPitch = 0;
    base.currentYaw = 0;
    base.currentXvel = 0;
    base.currentYvel = 0;
    base.currentZvel = 0;
    base.currentRollvel = 0;
    base.currentPitchvel = 0;
    base.currentYawvel = 0;
}

void CRobot::ComputeTorqueControl(void) {

    RobotState(AXIS_X) = base.currentX;
    RobotState(AXIS_Y) = base.currentY;
    RobotState(AXIS_Z) = base.currentZ;
    RobotState(AXIS_Roll) = base.currentRoll;
    RobotState(AXIS_Pitch) = base.currentPitch;
    RobotState(AXIS_Yaw) = base.currentYaw;
    RobotStatedot(AXIS_X) = base.currentXvel;
    RobotStatedot(AXIS_Y) = base.currentYvel;
    RobotStatedot(AXIS_Z) = base.currentZvel;
    RobotStatedot(AXIS_Roll) = base.currentRollvel;
    RobotStatedot(AXIS_Pitch) = base.currentPitchvel;
    RobotStatedot(AXIS_Yaw) = base.currentYawvel;

    for (int i = 0; i < nDOF; ++i) {
        RobotState(6 + i) = actual_joint_pos[i];
        RobotStatedot(6 + i) = actual_joint_vel[i];
    }

    Math::Quaternion QQ(0, 0, 0, 1);
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);
    CalcPointJacobian6D(*m_pModel, RobotState, base.ID, Originbase, J_BASE, true);
    CalcPointJacobian(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, J_RL, true);
    CalcPointJacobian(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, J_RR, true);
    CalcPointJacobian(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, J_FL, true);
    CalcPointJacobian(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, J_FR, true);

    J_RL2 = J_RL.block(0, 6, 3, 3);
    J_RR2 = J_RR.block(0, 9, 3, 3);
    J_FL2 = J_FL.block(0, 12, 3, 3);
    J_FR2 = J_FR.block(0, 15, 3, 3);

    J_A.block(0, 0, 3, 3) = J_RL2;
    J_A.block(3, 3, 3, 3) = J_RR2;
    J_A.block(6, 6, 3, 3) = J_FL2;
    J_A.block(9, 9, 3, 3) = J_FR2;

//    std::cout << "J_A(RBDL)" << endl << J_A << std::endl;
//    cout<<"---------------------"<<endl;
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

//void CRobot::Init_Pos_Traj_HS(void) {
//    if (cnt_HS == 0) {
//        cycle_time_HS = 3.0;
//        //goal_joint_pos_HS << 0 * D2R, 45 * D2R, -90 * D2R;
//        goal_EP_pos_HS << 0.0, 0.105, -0.45;
//
//        for (int i = 0; i < NUM_OF_ELMO; ++i) {
//            target_EP_pos_HS[i] = actual_EP_pos_local[i];
//            target_EP_vel_HS[i] = 0;
//            init_EP_pos_HS[i] = target_EP_pos_HS[i];
//        }
//        cnt_HS++;
//    } else if (cnt_HS < (unsigned int) (cycle_time_HS / dt)) {
//        for (int i = 0; i < NUM_OF_ELMO; ++i) {
//            target_EP_pos_HS[i] = init_EP_pos_HS[i] + (goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * (1 - cos(PI / cycle_time_HS * cnt_HS * dt));
//            target_EP_vel_HS[i] = (goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * PI / cycle_time_HS * sin(PI / cycle_time_HS * cnt_HS * dt);
//        }
//        cnt_HS++;
//    } else {
//        for (int i = 0; i < NUM_OF_ELMO; ++i) {
//            target_EP_pos_HS[i] = goal_EP_pos_HS[i];
//            target_EP_vel_HS[i] = 0;
//        }
//    }
//    //    target_joint_pos_HS = IK_HS(target_EP_pos_HS);
//    //    target_joint_vel_HS = J_A.inverse() * target_EP_vel_HS;
//}

//void CRobot::Home_Pos_Traj_HS(void) {
//    if (cnt_HS == 0) {
//        cycle_time_HS = 3.0;
//        goal_EP_pos_HS << 0.0, 0.105, -0.45;
//        target_EP_pos_HS = actual_EP_pos_local;
//        init_EP_pos_HS = target_EP_pos_HS;
//        target_EP_vel_HS << 0, 0, 0;
//        cnt_HS++;
//    } else if (cnt_HS < (unsigned int) (cycle_time_HS / dt)) {
//        for (int i = 0; i < NUM_OF_ELMO; ++i) {
//            target_EP_pos_HS[i] = init_EP_pos_HS[i]+(goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * (1 - cos(PI / cycle_time_HS * cnt_HS * dt));
//            target_EP_vel_HS[i] = (goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * PI / cycle_time_HS * sin(PI / cycle_time_HS * cnt_HS * dt);
//        }
//        cnt_HS++;
//    } else {
//        target_EP_pos_HS = goal_EP_pos_HS;
//        target_EP_vel_HS << 0, 0, 0;
//    }
//    //    target_joint_pos_HS = IK_HS(target_EP_pos_HS);
//    //    target_joint_vel_HS = J_A.inverse() * target_EP_vel_HS;
//}

//void CRobot::Cycle_Test_Pos_Traj_HS(void) {
//    if (cnt_HS == 0) {
//        cycle_time_HS = 3.0;
//        alpha = 10 * D2R;
//        goal_joint_pos_HS << 0.0, 0.0, target_joint_pos_HS(2) + alpha;
//
//        //target_joint_pos_HS = actual_joint_pos_HS;
//        init_joint_pos_HS = target_joint_pos_HS;
//        target_joint_vel_HS << 0, 0, 0;
//        cnt_HS++;
//    } else if (cnt_HS < (unsigned int) (cycle_time_HS / dt)) {
//        for (int i = 0; i < NUM_OF_ELMO; ++i) {
//            target_joint_pos_HS[i] = init_joint_pos_HS[i]+(goal_joint_pos_HS[i] - init_joint_pos_HS[i]) / 2.0 * (1 - cos(2 * PI / cycle_time_HS * cnt_HS * dt));
//            target_joint_vel_HS[i] = (goal_joint_pos_HS[i] - init_joint_pos_HS[i]) / 2.0 * (2.0 * PI) / cycle_time_HS * sin(2 * PI / cycle_time_HS * cnt_HS * dt);
//        }
//        cnt_HS++;
//    } else {
//        if (stop_flag == false) {
//            cnt_HS = 1;
//        } else {
//            target_joint_pos_HS = init_joint_pos_HS;
//            target_joint_vel_HS << 0, 0, 0;
//        }
//    }
//}

//void CRobot::Joystick_Pos_Traj_HS(void) {
//    double pos_limit_x_u = 0.25;
//    double pos_limit_x_l = -0.20;
//
//    double pos_limit_y_u = 0.11;
//    double pos_limit_y_l = 0.10;
//
//    double pos_limit_z_u = -0.3;
//    double pos_limit_z_l = -0.45;
//    double lamda = 0.8;
//    //double lamda = 0.5;
//    VectorNd tmp1_target_EP_pos_HS = VectorNd::Zero(3);
//
//    if (JoyMode == JOYMODE_HOME) {
//        if (cnt_HS == 0) {
//            cycle_time_HS = 3.0;
//            init_EP_pos_HS = actual_EP_pos_local;
//            goal_EP_pos_HS << 0.0, 0.105, -0.45;
//            target_EP_pos_HS = init_EP_pos_HS;
//            target_EP_vel_HS << 0, 0, 0;
//            cnt_HS++;
//
//        } else if (cnt_HS < (unsigned int) (cycle_time_HS / dt)) {
//            for (int i = 0; i < NUM_OF_ELMO; ++i) {
//                target_EP_pos_HS[i] = init_EP_pos_HS[i]+(goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * (1 - cos(PI / cycle_time_HS * cnt_HS * dt));
//                target_EP_vel_HS[i] = (goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * PI / cycle_time_HS * sin(PI / cycle_time_HS * cnt_HS * dt);
//            }
//            cnt_HS++;
//        } else {
//            target_EP_pos_HS = goal_EP_pos_HS;
//            target_EP_vel_HS << 0, 0, 0;
//        }
//    } else if (JoyMode == JOYMODE_MOVE || JoyMode == JOYMODE_WALK) {
//        tmp1_target_EP_pos_HS[0] = target_EP_pos_HS[0] + lamda * joy_vel_x*dt;
//        tmp1_target_EP_pos_HS[1] = target_EP_pos_HS[1] + lamda * joy_vel_y*dt;
//
//        if ((tmp1_target_EP_pos_HS[0]) > pos_limit_x_u || pos_limit_x_l > (tmp1_target_EP_pos_HS[0])) {
//            joy_vel_x = 0.0;
//            tmp1_target_EP_pos_HS[0] = target_EP_pos_HS[0];
//        } else {
//            // tmp2_target_EP_pos_HS[0] = tmp1_target_EP_pos_HS[0];
//        }
//        if ((tmp1_target_EP_pos_HS[1]) > pos_limit_y_u || pos_limit_y_l > (tmp1_target_EP_pos_HS[1])) {
//            joy_vel_y = 0.0;
//            tmp1_target_EP_pos_HS[1] = target_EP_pos_HS[1];
//        } else {
//            //tmp2_target_EP_pos_HS[1] = tmp1_target_EP_pos_HS[1];
//        }
//        //target_EP_pos_HS[0] = tmp2_target_EP_pos_HS[0];
//        //target_EP_pos_HS[1] = tmp2_target_EP_pos_HS[1];
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////////            
//        if (JoyMode == JOYMODE_MOVE) {
//            // tmp1_target_EP_pos_HS[2] = target_EP_pos_HS[2] + lamda * joy_vel_z*dt;
//            //            if ((tmp1_target_EP_pos_HS[2]) > pos_limit_z_u || pos_limit_z_l > (tmp1_target_EP_pos_HS[2])) {
//            //                joy_vel_z = 0.0;
//            //                tmp1_target_EP_pos_HS[2] = target_EP_pos_HS[2];
//            //            } else {
//            ////                tmp2_target_EP_pos_HS[2] = tmp1_target_EP_pos_HS[2];
//            //            }
//            //  target_EP_pos_HS[2] = tmp2_target_EP_pos_HS[2];
//        }
//        else if (JoyMode == JOYMODE_WALK) {
//            walk_time = cnt_HS*dt;
//
//            if (cnt_HS == 0) {
//                init_EP_pos_HS[2] = target_EP_pos_HS[2];
//                goal_EP_pos_HS[2] = init_EP_pos_HS[2] + foot_height_HS;
//                target_EP_pos_HS[2] = init_EP_pos_HS[2];
//                SF_EP_Traj_Gen_HS(step_time_HS, init_EP_pos_HS, goal_EP_pos_HS);
//
//                cnt_HS++;
//            } else if (cnt_HS < tsp_cnt_HS) {
//                t2 = walk_time;
//                if (cnt_HS < tsp_cnt_HS / 2) {
//                    t1 = t2;
//                    target_EP_pos_HS[2] = z_up[5] * pow(t1, 5) + z_up[4] * pow(t1, 4) + z_up[3] * pow(t1, 3) + z_up[2] * pow(t1, 2) + z_up[1] * pow(t1, 1) + z_up[0];
//                    target_EP_vel_HS[2] = 5 * z_up[5] * pow(t1, 4) + 4 * z_up[4] * pow(t1, 3) + 3 * z_up[3] * pow(t1, 2) + 2 * z_up[2] * pow(t1, 1) + z_up[1];
//                } else {
//                    t1 = t2 - tsp_time_HS / 2.0;
//                    target_EP_pos_HS[2] = z_down[5] * pow(t1, 5) + z_down[4] * pow(t1, 4) + z_down[3] * pow(t1, 3) + z_down[2] * pow(t1, 2) + z_down[1] * pow(t1, 1) + z_down[0];
//                    target_EP_vel_HS[2] = 5 * z_down[5] * pow(t1, 4) + 4 * z_down[4] * pow(t1, 3) + 3 * z_down[3] * pow(t1, 2) + 2 * z_down[2] * pow(t1, 1) + z_down[1];
//                }
//                cnt_HS++;
//            } else if (cnt_HS < fsp_cnt_HS + (tsp_cnt_HS + fsp_cnt_HS)*3) {
//                target_EP_pos_HS[2] = init_EP_pos_HS[2];
//                target_EP_vel_HS[2] = 0;
//                cnt_HS++;
//            } else {
//                if (walk_stop_flag != true) {
//                    cnt_HS = 1;
//                }
//            }
//        }
//    }
//    //    target_joint_pos_HS = IK_HS(target_EP_pos_HS);
//    //    target_joint_vel_HS = J_A.inverse() * target_EP_vel_HS;
//}

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


//void CRobot::Cartesian_Controller(void) {
//
//    Cart_Controller_HS[0] = 0;
//    Cart_Controller_HS[1] = 0;
//    Cart_Controller_HS[2] = 0;
//    Cart_Controller_HS[3] = 0;
//    Cart_Controller_HS[4] = 0;
//    Cart_Controller_HS[5] = 0;
//
//    for (int i = 0; i < NUM_OF_ELMO; ++i) {
//        Cart_Controller_HS[i + 6] = kp_EP_HS[i] * (actual_EP_pos_local[i] - target_EP_pos_HS[i]) + kd_EP_HS[i] * (actual_EP_vel_local_HS[i] - target_EP_vel_HS[i]);
//    }
//}

//void CRobot::Joint_Controller(void) {
//
//    Joint_Controller_HS[0] = 0;
//    Joint_Controller_HS[1] = 0;
//    Joint_Controller_HS[2] = 0;
//    Joint_Controller_HS[3] = 0;
//    Joint_Controller_HS[4] = 0;
//    Joint_Controller_HS[5] = 0;
//
//    for (int i = 0; i < NUM_OF_ELMO; ++i) {
//        Joint_Controller_HS[i + 6] = kp_joint_HS[i] * (target_joint_pos_HS[i] - actual_joint_pos[i]) + kd_joint_HS[i] * (target_joint_vel_HS[i] - actual_joint_vel[i]);
//    }
//}

void CRobot::Mode_Change(void) {
    if (tmp_Mode_Change_flag == true) {
        if (cnt_mode_change < 3000) {
            Mode_Change_flag = false;
            //cnt_HS = 0; //?
        } else {
            Mode_Change_flag = true;
            tmp_Mode_Change_flag = false;
            cnt_mode_change = 0;
            //cnt_HS = 0;
        }
        cnt_mode_change++;
    }
}

//void CRobot::Controller_Change(void) {
//    unsigned int target_cnt_change = 3000;
//
//    if (Mode_Change_flag == true) {
//
//        if (cnt_Control_change == 0) {
//            target_kp_joint_HS = kp_joint_HS;
//            target_kd_joint_HS = kd_joint_HS;
//            target_kp_EP_HS = kp_EP_HS;
//            target_kd_EP_HS = kd_EP_HS;
//
//            init_kp_joint_HS = target_kp_joint_HS;
//            init_kd_joint_HS = target_kd_joint_HS;
//            init_kp_EP_HS = target_kp_EP_HS;
//            init_kd_EP_HS = target_kd_EP_HS;
//
//            if (CommandFlag == GOTO_INIT_POS_HS || CommandFlag == GOTO_JOYSTICK_POS_HS) {
//                Cart_Controller_HS = VectorNd::Zero(9);
//                goal_kp_joint_HS = abs_kp_joint_HS;
//                goal_kd_joint_HS = abs_kd_joint_HS;
//                goal_kp_EP_HS = VectorNd::Zero(3);
//                goal_kd_EP_HS = VectorNd::Zero(3);
//            } else if (CommandFlag == GOTO_WALK_READY_POS_HS) {
//                Joint_Controller_HS = VectorNd::Zero(9);
//                goal_kp_joint_HS = VectorNd::Zero(3);
//                goal_kd_joint_HS = VectorNd::Zero(3);
//                goal_kp_EP_HS = abs_kp_EP_HS;
//                goal_kd_EP_HS = abs_kd_EP_HS;
//            }
//            cnt_Control_change++;
//        } else if (cnt_Control_change < target_cnt_change) {
//            target_kp_joint_HS = init_kp_joint_HS + (goal_kp_joint_HS - init_kp_joint_HS) / 2.0 * (1 - cos(PI / target_cnt_change * cnt_Control_change));
//            target_kd_joint_HS = init_kd_joint_HS + (goal_kd_joint_HS - init_kd_joint_HS) / 2.0 * (1 - cos(PI / target_cnt_change * cnt_Control_change));
//            target_kp_EP_HS = init_kp_EP_HS + (goal_kp_EP_HS - init_kp_EP_HS) / 2.0 * (1 - cos(PI / target_cnt_change * cnt_Control_change));
//            target_kd_EP_HS = init_kd_EP_HS + (goal_kd_EP_HS - init_kd_EP_HS) / 2.0 * (1 - cos(PI / target_cnt_change * cnt_Control_change));
//            cnt_Control_change++;
//        } else {
//            target_kp_joint_HS = goal_kp_joint_HS;
//            target_kd_joint_HS = goal_kd_joint_HS;
//            target_kp_EP_HS = goal_kp_EP_HS;
//            target_kd_EP_HS = goal_kd_EP_HS;
//        }
//
//        kp_joint_HS = target_kp_joint_HS;
//        kd_joint_HS = target_kd_joint_HS;
//        kp_EP_HS = target_kp_EP_HS;
//        kd_EP_HS = target_kd_EP_HS;
//    }
//
//}

void CRobot::SF_EP_Traj_Gen_HS(double travel_time, VectorNd init_EP_pos, VectorNd goal_EP_pos) {

    //******* Z trajectory ********///
    init_x[0] = init_EP_pos(2);
    init_x[1] = 0;
    init_x[2] = 0;
    //    final_x[0] = init_EP_pos(2) + foot_height_HS;
    final_x[0] = goal_EP_pos(2);
    final_x[1] = 0;
    final_x[2] = 0;
    coefficient_5thPoly(init_x, final_x, travel_time / 2.0, z_up);

    //    init_x[0] = init_EP_pos(2) + foot_height_HS;
    init_x[0] = goal_EP_pos(2);
    init_x[1] = 0;
    init_x[2] = 0;
    final_x[0] = init_EP_pos(2);
    final_x[1] = 0;
    final_x[2] = 0;
    coefficient_5thPoly(init_x, final_x, travel_time / 2.0, z_down);

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