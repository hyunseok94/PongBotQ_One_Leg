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
    
    //std::cout<<J_A<<std::endl;
    
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
    CTC_Torque = Joint_Controller_HS + G_term + C_term;

    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        joint[i].torque = CTC_Torque(6 + i);
    }
}

void CRobot::Init_Pos_Traj_HS(void) {
    if (cnt_HS == 0) {
        cycle_time_HS = 3.0;
        //goal_joint_pos_HS << 0 * D2R, 45 * D2R, -90 * D2R;
        goal_EP_pos_HS << 0.0, 0.105, -0.45;

        for (int i = 0; i < NUM_OF_ELMO; ++i) {
            target_EP_pos_HS[i] = actual_EP_pos_local_HS[i];
            target_EP_vel_HS[i] = 0;
            init_EP_pos_HS[i] = target_EP_pos_HS[i];
        }
        cnt_HS++;
    } else if (cnt_HS < (unsigned int) (cycle_time_HS / dt)) {
        for (int i = 0; i < NUM_OF_ELMO; ++i) {
            target_EP_pos_HS[i] = init_EP_pos_HS[i] + (goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * (1 - cos(PI / cycle_time_HS * cnt_HS * dt));
            target_EP_vel_HS[i] = (goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * PI / cycle_time_HS * sin(PI / cycle_time_HS * cnt_HS * dt);
        }
        cnt_HS++;
    } else {
        for (int i = 0; i < NUM_OF_ELMO; ++i) {
            target_EP_pos_HS[i] = goal_EP_pos_HS[i];
            target_EP_vel_HS[i] = 0;
        }
    }
    target_joint_pos_HS = IK_HS(target_EP_pos_HS);
    target_joint_vel_HS = J_A.inverse() * target_EP_vel_HS;
}

void CRobot::Home_Pos_Traj_HS(void) {
    if (cnt_HS == 0) {
        cycle_time_HS = 3.0;
        goal_EP_pos_HS << 0.0, 0.105, -0.45;
        target_EP_pos_HS = actual_EP_pos_local_HS;
        init_EP_pos_HS = target_EP_pos_HS;
        target_EP_vel_HS << 0, 0, 0;
        cnt_HS++;
    } else if (cnt_HS < (unsigned int) (cycle_time_HS / dt)) {
        for (int i = 0; i < NUM_OF_ELMO; ++i) {
            target_EP_pos_HS[i] = init_EP_pos_HS[i]+(goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * (1 - cos(PI / cycle_time_HS * cnt_HS * dt));
            target_EP_vel_HS[i] = (goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * PI / cycle_time_HS * sin(PI / cycle_time_HS * cnt_HS * dt);
        }
        cnt_HS++;
    } else {
        target_EP_pos_HS = goal_EP_pos_HS;
        target_EP_vel_HS << 0, 0, 0;
    }
    target_joint_pos_HS = IK_HS(target_EP_pos_HS);
    target_joint_vel_HS = J_A.inverse() * target_EP_vel_HS;
}

void CRobot::Cycle_Test_Pos_Traj_HS(void) {
    if (cnt_HS == 0) {
        cycle_time_HS = 3.0;
        alpha = 10 * D2R;
        goal_joint_pos_HS << 0.0, 0.0, target_joint_pos_HS(2) + alpha;

        //target_joint_pos_HS = actual_joint_pos_HS;
        init_joint_pos_HS = target_joint_pos_HS;
        target_joint_vel_HS << 0, 0, 0;
        cnt_HS++;
    } else if (cnt_HS < (unsigned int) (cycle_time_HS / dt)) {
        for (int i = 0; i < NUM_OF_ELMO; ++i) {
            target_joint_pos_HS[i] = init_joint_pos_HS[i]+(goal_joint_pos_HS[i] - init_joint_pos_HS[i]) / 2.0 * (1 - cos(2 * PI / cycle_time_HS * cnt_HS * dt));
            target_joint_vel_HS[i] = (goal_joint_pos_HS[i] - init_joint_pos_HS[i]) / 2.0 * (2.0 * PI) / cycle_time_HS * sin(2 * PI / cycle_time_HS * cnt_HS * dt);
        }
        cnt_HS++;
    } else {
        if (stop_flag == false) {
            cnt_HS = 1;
        } else {
            target_joint_pos_HS = init_joint_pos_HS;
            target_joint_vel_HS << 0, 0, 0;
        }
    }
}

void CRobot::Joystick_Pos_Traj_HS(void) {
    double pos_limit_x_u = 0.25;
    double pos_limit_x_l = -0.20;
    
    double pos_limit_y_u = 0.11;
    double pos_limit_y_l = 0.10;
    
    double pos_limit_z_u = -0.3;
    double pos_limit_z_l = -0.45;
    double lamda = 0.8;
    //double lamda = 0.5;
    VectorNd tmp1_target_EP_pos_HS = VectorNd::Zero(3);

    if (JoyMode == JOYMODE_HOME) {
        if (cnt_HS == 0) {
            cycle_time_HS = 3.0;
            init_EP_pos_HS = actual_EP_pos_local_HS;
            goal_EP_pos_HS << 0.0, 0.105, -0.45;
            target_EP_pos_HS = init_EP_pos_HS;
            target_EP_vel_HS << 0, 0, 0;
            cnt_HS++;

        } else if (cnt_HS < (unsigned int) (cycle_time_HS / dt)) {
            for (int i = 0; i < NUM_OF_ELMO; ++i) {
                target_EP_pos_HS[i] = init_EP_pos_HS[i]+(goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * (1 - cos(PI / cycle_time_HS * cnt_HS * dt));
                target_EP_vel_HS[i] = (goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * PI / cycle_time_HS * sin(PI / cycle_time_HS * cnt_HS * dt);
            }
            cnt_HS++;
        } else {
            target_EP_pos_HS = goal_EP_pos_HS;
            target_EP_vel_HS << 0, 0, 0;
        }
    }
    else if (JoyMode == JOYMODE_MOVE || JoyMode == JOYMODE_WALK) {
        tmp1_target_EP_pos_HS[0] = target_EP_pos_HS[0] + lamda * joy_vel_x*dt;
        tmp1_target_EP_pos_HS[1] = target_EP_pos_HS[1] + lamda * joy_vel_y*dt;

        if ((tmp1_target_EP_pos_HS[0]) > pos_limit_x_u || pos_limit_x_l > (tmp1_target_EP_pos_HS[0])) {
            joy_vel_x = 0.0;
            tmp1_target_EP_pos_HS[0] = target_EP_pos_HS[0];
        } else {
           // tmp2_target_EP_pos_HS[0] = tmp1_target_EP_pos_HS[0];
        }
        if ((tmp1_target_EP_pos_HS[1]) > pos_limit_y_u || pos_limit_y_l > (tmp1_target_EP_pos_HS[1])) {
            joy_vel_y = 0.0;
            tmp1_target_EP_pos_HS[1] = target_EP_pos_HS[1];
        } else {
            //tmp2_target_EP_pos_HS[1] = tmp1_target_EP_pos_HS[1];
        }
            //target_EP_pos_HS[0] = tmp2_target_EP_pos_HS[0];
            //target_EP_pos_HS[1] = tmp2_target_EP_pos_HS[1];

////////////////////////////////////////////////////////////////////////////////////////////////////            
        if (JoyMode == JOYMODE_MOVE) {
           // tmp1_target_EP_pos_HS[2] = target_EP_pos_HS[2] + lamda * joy_vel_z*dt;
//            if ((tmp1_target_EP_pos_HS[2]) > pos_limit_z_u || pos_limit_z_l > (tmp1_target_EP_pos_HS[2])) {
//                joy_vel_z = 0.0;
//                tmp1_target_EP_pos_HS[2] = target_EP_pos_HS[2];
//            } else {
////                tmp2_target_EP_pos_HS[2] = tmp1_target_EP_pos_HS[2];
//            }
          //  target_EP_pos_HS[2] = tmp2_target_EP_pos_HS[2];
        } 
        else if (JoyMode == JOYMODE_WALK) {
            walk_time=cnt_HS*dt;

            if (cnt_HS == 0) {
                init_EP_pos_HS[2] = target_EP_pos_HS[2];
                goal_EP_pos_HS[2] = init_EP_pos_HS[2] + foot_height_HS;
                target_EP_pos_HS[2] = init_EP_pos_HS[2];
                SF_EP_Traj_Gen_HS(step_time_HS, init_EP_pos_HS, goal_EP_pos_HS);
               
                cnt_HS++;
            } else if (cnt_HS < tsp_cnt_HS) {
                t2=walk_time;
                if (cnt_HS < tsp_cnt_HS / 2) {
                    t1 = t2;
                    target_EP_pos_HS[2] = z_up[5] * pow(t1, 5) + z_up[4] * pow(t1, 4) + z_up[3] * pow(t1, 3) + z_up[2] * pow(t1, 2) + z_up[1] * pow(t1, 1) + z_up[0];
                    target_EP_vel_HS[2] = 5 * z_up[5] * pow(t1, 4) + 4 * z_up[4] * pow(t1, 3) + 3 * z_up[3] * pow(t1, 2) + 2 * z_up[2] * pow(t1, 1) + z_up[1];
                } else {
                    t1 = t2 - tsp_time_HS / 2.0;
                    target_EP_pos_HS[2] = z_down[5] * pow(t1, 5) + z_down[4] * pow(t1, 4) + z_down[3] * pow(t1, 3) + z_down[2] * pow(t1, 2) + z_down[1] * pow(t1, 1) + z_down[0];
                    target_EP_vel_HS[2] = 5 * z_down[5] * pow(t1, 4) + 4 * z_down[4] * pow(t1, 3) + 3 * z_down[3] * pow(t1, 2) + 2 * z_down[2] * pow(t1, 1) + z_down[1];
                }
                cnt_HS++;
            }
            else if (cnt_HS < fsp_cnt_HS + (tsp_cnt_HS + fsp_cnt_HS)*3) {
                target_EP_pos_HS[2] = init_EP_pos_HS[2];
                target_EP_vel_HS[2] = 0;
                cnt_HS++;
            }
            else {
                if(walk_stop_flag!=true){
                   cnt_HS = 1;
                }
            }
        }
    }
    target_joint_pos_HS = IK_HS(target_EP_pos_HS);
    target_joint_vel_HS = J_A.inverse() * target_EP_vel_HS;
}

VectorNd CRobot::FK_HS(VectorNd joint_pos) {
    VectorNd EP_pos_HS(3);
    const double L1 = 0.1045;
    const double L2 = 0.305;
    const double L3 = 0.309;

    static double q1 = 0;
    static double q2 = 0;
    static double q3 = 0;

    //FL_EP
    q1 = joint_pos[0];
    q2 = joint_pos[1];
    q3 = joint_pos[2];

    EP_pos_HS[0] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
    EP_pos_HS[1] = L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
    EP_pos_HS[2] = L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

    return EP_pos_HS;
}

VectorNd CRobot::IK_HS(VectorNd EP_pos) {
    VectorNd joint_pos(3);

    const double L1 = 0.1045;
    const double L2 = 0.305;
    const double L3 = 0.309;

    static double x = 0;
    static double y = 0;
    static double z = 0;
    static double q1_cal = 0;

    x = -EP_pos[0];
    y = EP_pos[1];
    z = EP_pos[2];

    joint_pos[0] = atan2(y, abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    joint_pos[1] = -(-atan2(x, sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    joint_pos[2] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    //RL_joints
    //    x = EP_pos[0];
    //    y = EP_pos[1];
    //    z = EP_pos[2];
    //    if (actual_joint_pos_HS[0] >= 0) {
    //        joint_pos[0] = (acos(-z / sqrt(pow(y, 2) + pow(z, 2))) + acos(L1 / sqrt(pow(y, 2) + pow(z, 2))) - PI / 2);
    //    } else {
    //        joint_pos[0] = -(PI - (acos(-y / sqrt(pow(y, 2) + pow(z, 2))) + acos(L1 / sqrt(pow(y, 2) + pow(z, 2)))));
    //    }
    //    q1_cal = joint_pos[0];
    //    joint_pos[1] = (acos((pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow((y - L1 * cos(q1_cal)), 2) + pow(z - L1 * sin(q1_cal), 2)) / (2 * L2 * sqrt(pow(x, 2) + pow((y - L1 * cos(q1_cal)), 2) + pow((z - L1 * sin(q1_cal)), 2)))) - atan(x / sqrt(pow(y, 2) + pow(z, 2) - pow(L1, 2))));
    //    joint_pos[2] = -(acos((pow(x, 2) + pow((y - L1 * cos(q1_cal)), 2) + pow((z - L1 * sin(q1_cal)), 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3)));


    return joint_pos;
}

MatrixNd CRobot::Jacobian_HS(VectorNd joint_pos) {
    MatrixNd J(3, 3);

    const double L1 = 0.1045;
    const double L2 = 0.305;
    const double L3 = 0.309;

    static double q1 = 0;
    static double q2 = 0;
    static double q3 = 0;

    //FL_EP
    q1 = joint_pos[0];
    q2 = joint_pos[1];
    q3 = joint_pos[2];

    J(0, 0) = 0;
    J(0, 1) = -L2 * cos(q2) + L3 * (sin(q2) * sin(q3)) - L3 * (cos(q3) * cos(q2));
    J(0, 2) = -L3 * (cos(q2) * cos(q3)) + L3 * (sin(q3) * sin(q2));

    J(1, 0) = -L1 * sin(q1) + L2 * cos(q2) * cos(q1) - L3 * (cos(q1) * sin(q2) * sin(q3)) + L3 * (cos(q2) * cos(q3) * cos(q1));
    J(1, 1) = -L2 * sin(q2) * sin(q1) - L3 * (sin(q1) * cos(q2) * sin(q3)) - L3 * (sin(q2) * cos(q3) * sin(q1));
    J(1, 2) = -L3 * (sin(q1) * sin(q2) * cos(q3)) - L3 * (cos(q2) * sin(q3) * sin(q1));

    J(2, 0) = L1 * cos(q1) + L2 * sin(q1) * cos(q2) + L3 * (sin(q1) * cos(q2) * cos(q3)) - L3 * (sin(q1) * sin(q2) * sin(q3));
    J(2, 1) = L2 * cos(q1) * sin(q2) + L3 * (cos(q1) * sin(q2) * cos(q3)) + L3 * (cos(q1) * cos(q2) * sin(q3));
    J(2, 2) = L3 * (cos(q1) * cos(q2) * sin(q3)) + L3 * (cos(q1) * sin(q2) * cos(q3));

    return J;
}

void CRobot::Cartesian_Controller(void) {

    Cart_Controller_HS[0] = 0;
    Cart_Controller_HS[1] = 0;
    Cart_Controller_HS[2] = 0;
    Cart_Controller_HS[3] = 0;
    Cart_Controller_HS[4] = 0;
    Cart_Controller_HS[5] = 0;

    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        Cart_Controller_HS[i + 6] = kp_EP_HS[i] * (actual_EP_pos_local_HS[i] - target_EP_pos_HS[i]) + kd_EP_HS[i] * (actual_EP_vel_local_HS[i] - target_EP_vel_HS[i]);
    }
}

void CRobot::Joint_Controller(void) {

    Joint_Controller_HS[0] = 0;
    Joint_Controller_HS[1] = 0;
    Joint_Controller_HS[2] = 0;
    Joint_Controller_HS[3] = 0;
    Joint_Controller_HS[4] = 0;
    Joint_Controller_HS[5] = 0;

    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        Joint_Controller_HS[i + 6] = kp_joint_HS[i] * (target_joint_pos_HS[i] - actual_joint_pos[i]) + kd_joint_HS[i] * (target_joint_vel_HS[i] - actual_joint_vel[i]);
    }
}

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

void CRobot::Controller_Change(void) {
    unsigned int target_cnt_change = 3000;

    if (Mode_Change_flag == true) {

        if (cnt_Control_change == 0) {
            target_kp_joint_HS = kp_joint_HS;
            target_kd_joint_HS = kd_joint_HS;
            target_kp_EP_HS = kp_EP_HS;
            target_kd_EP_HS = kd_EP_HS;

            init_kp_joint_HS = target_kp_joint_HS;
            init_kd_joint_HS = target_kd_joint_HS;
            init_kp_EP_HS = target_kp_EP_HS;
            init_kd_EP_HS = target_kd_EP_HS;

            if (CommandFlag == GOTO_INIT_POS_HS || CommandFlag == GOTO_JOYSTICK_POS_HS) {
                Cart_Controller_HS = VectorNd::Zero(9);
                goal_kp_joint_HS = abs_kp_joint_HS;
                goal_kd_joint_HS = abs_kd_joint_HS;
                goal_kp_EP_HS = VectorNd::Zero(3);
                goal_kd_EP_HS = VectorNd::Zero(3);
            } else if (CommandFlag == GOTO_WALK_READY_POS_HS) {
                Joint_Controller_HS = VectorNd::Zero(9);
                goal_kp_joint_HS = VectorNd::Zero(3);
                goal_kd_joint_HS = VectorNd::Zero(3);
                goal_kp_EP_HS = abs_kp_EP_HS;
                goal_kd_EP_HS = abs_kd_EP_HS;
            }
            cnt_Control_change++;
        } else if (cnt_Control_change < target_cnt_change) {
            target_kp_joint_HS = init_kp_joint_HS + (goal_kp_joint_HS - init_kp_joint_HS) / 2.0 * (1 - cos(PI / target_cnt_change * cnt_Control_change));
            target_kd_joint_HS = init_kd_joint_HS + (goal_kd_joint_HS - init_kd_joint_HS) / 2.0 * (1 - cos(PI / target_cnt_change * cnt_Control_change));
            target_kp_EP_HS = init_kp_EP_HS + (goal_kp_EP_HS - init_kp_EP_HS) / 2.0 * (1 - cos(PI / target_cnt_change * cnt_Control_change));
            target_kd_EP_HS = init_kd_EP_HS + (goal_kd_EP_HS - init_kd_EP_HS) / 2.0 * (1 - cos(PI / target_cnt_change * cnt_Control_change));
            cnt_Control_change++;
        } else {
            target_kp_joint_HS = goal_kp_joint_HS;
            target_kd_joint_HS = goal_kd_joint_HS;
            target_kp_EP_HS = goal_kp_EP_HS;
            target_kd_EP_HS = goal_kd_EP_HS;
        }

        kp_joint_HS = target_kp_joint_HS;
        kd_joint_HS = target_kd_joint_HS;
        kp_EP_HS = target_kp_EP_HS;
        kd_EP_HS = target_kd_EP_HS;
    }

}

void CRobot::SF_EP_Traj_Gen_HS(double travel_time, VectorNd init_EP_pos, VectorNd goal_EP_pos) {

    //******* Z trajectory ********///
    init_x[0] = init_EP_pos(2);
    init_x[1] = 0;
    init_x[2] = 0;
    //    final_x[0] = init_EP_pos(2) + foot_height_HS;
    final_x[0] = goal_EP_pos(2);
    final_x[1] = 0;
    final_x[2] = 0;
    coefficient_5thPoly_HS(init_x, final_x, travel_time / 2.0, z_up);

    //    init_x[0] = init_EP_pos(2) + foot_height_HS;
    init_x[0] = goal_EP_pos(2);
    init_x[1] = 0;
    init_x[2] = 0;
    final_x[0] = init_EP_pos(2);
    final_x[1] = 0;
    final_x[2] = 0;
    coefficient_5thPoly_HS(init_x, final_x, travel_time / 2.0, z_down);

}

void CRobot::coefficient_5thPoly_HS(double *init_x, double *final_x, double tf, double *output) {
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