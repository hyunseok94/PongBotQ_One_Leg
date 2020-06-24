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
    return th; // [rad]
}

double CRobot::Count2RadDot(int Gear_Ratio, INT32 CountPerSec) {
    double th_dot = (double) CountPerSec * 2 * PI / (2048 * Gear_Ratio);
    return th_dot; // [rad]
}

double CRobot::Count2Rad_ABS(int _Resolution, INT32 Count) {
    double th = (double) Count * 2 * PI / (_Resolution);
    return th;
}

INT32 CRobot::Count_tf(int _Ratio, INT32 _Count_in) {
    INT32 _Count_out = (INT32) _Count_in / (_Ratio);
    return _Count_out;
}

INT16 CRobot::Tor2Cur(double OutputTorque, double _Kt, int _Gear, double _ratedCur) {
    INT16 inputCurrent = (OutputTorque /_Gear)/_Kt /_ratedCur * 1000;
    
    return inputCurrent;
}

void CRobot::setRobotModel(Model* getModel) {

    m_pModel = getModel;
    m_pModel->gravity = Vector3d(0., 0., -9.81);
    nDOF = m_pModel->dof_count - 6; //* get Degree of freedom, Except x,y,z,roll,pitch,yaw of the robot
    joint = new JOINT[nDOF]; //* only joint of the robot excepting x,y,z,roll,pitch,yaw of the robot
    RobotState = VectorNd::Zero(10);
    RobotStatedot = VectorNd::Zero(9);
    RobotState2dot = VectorNd::Zero(9);
    BasePosOri = VectorNd::Zero(6);
    BaseVel = VectorNd::Zero(6);
    JointAngle = VectorNd::Zero(nDOF);
    JointVel = VectorNd::Zero(nDOF);

    base.ID = m_pModel->GetBodyId("BASE");
    EP.ID = m_pModel->GetBodyId("CALF");

    QQ << 0, 0, 0, 1;
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);

    for (unsigned int i = 0; i < NUM_OF_ELMO; ++i) {
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
        RobotState(6 + i) = actual_joint_pos_HS[i];
        RobotStatedot(6 + i) = actual_joint_vel_HS[i];
    }

    Math::Quaternion QQ(0, 0, 0, 1);
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);
    CalcPointJacobian6D(*m_pModel, RobotState, base.ID, Originbase, J_BASE, true);
    CalcPointJacobian(*m_pModel, RobotState, EP.ID, EP_OFFSET, J_EP, true);

    //J_A<< J_BASE\
            , J_EP;
    J_A.block(0, 0, 6, 9) = J_BASE;
    J_A.block(6, 0, 3, 9) = J_EP;
    //std::cout<<J_A<<std::endl;
    J_A_EP = J_EP.block(0, 6, 3, 3);

    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);

    C_term = hatNonLinearEffects - G_term;

    Joint_Controller();
    Cartesian_Controller();
    Controller_Change();

//    CTC_Torque = Joint_Controller_HS + C_term + G_term - J_A.transpose() * (Cart_Controller_HS);
    //CTC_Torque = Joint_Controller_HS - J_A.transpose() * (Cart_Controller_HS);
    
    //CTC_Torque = Joint_Controller_HS;
    //CTC_Torque = G_term + C_term;
    CTC_Torque = Joint_Controller_HS + G_term + C_term;

    //CTC_Torque = C_term + G_term - J_A.transpose() * (Cart_Controller_HS);
    //CTC_Torque = G_term;
    //CTC_Torque = Joint_Controller_HS + G_term + C_term;

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
    target_joint_vel_HS = J_A_EP.inverse() * target_EP_vel_HS;
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
    target_joint_vel_HS = J_A_EP.inverse() * target_EP_vel_HS;
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

    //    x = -EP_pos[0];
    //    y = EP_pos[1];
    //    z = EP_pos[2];
    //
    //    joint_pos[0] = atan2(y , abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    //    joint_pos[1] = -(-atan2(x, sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    //    joint_pos[2] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    //RL_joints
    x = EP_pos[0];
    y = EP_pos[1];
    z = EP_pos[2];
    if (actual_joint_pos_HS[0] >= 0) {
        joint_pos[0] = (acos(-z / sqrt(pow(y, 2) + pow(z, 2))) + acos(L1 / sqrt(pow(y, 2) + pow(z, 2))) - PI / 2);
    } else {
        joint_pos[0] = -(PI - (acos(-y / sqrt(pow(y, 2) + pow(z, 2))) + acos(L1 / sqrt(pow(y, 2) + pow(z, 2)))));
    }
    q1_cal = joint_pos[0];
    joint_pos[1] = (acos((pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow((y - L1 * cos(q1_cal)), 2) + pow(z - L1 * sin(q1_cal), 2)) / (2 * L2 * sqrt(pow(x, 2) + pow((y - L1 * cos(q1_cal)), 2) + pow((z - L1 * sin(q1_cal)), 2)))) - atan(x / sqrt(pow(y, 2) + pow(z, 2) - pow(L1, 2))));
    joint_pos[2] = -(acos((pow(x, 2) + pow((y - L1 * cos(q1_cal)), 2) + pow((z - L1 * sin(q1_cal)), 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3)));


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
        Joint_Controller_HS[i + 6] = kp_joint_HS[i] * (target_joint_pos_HS[i] - actual_joint_pos_HS[i]) + kd_joint_HS[i] * (target_joint_vel_HS[i] - actual_joint_vel_HS[i]);
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

            if (CommandFlag == GOTO_INIT_POS_HS) {
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