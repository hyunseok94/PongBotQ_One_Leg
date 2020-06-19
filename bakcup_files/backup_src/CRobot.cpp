#include <stdio.h>
#include <math.h>
#include "CRobot.h"

CRobot::CRobot() {
}

CRobot::CRobot(const CRobot& orig) {
}

CRobot::~CRobot() {
}

void CRobot::setRobotModel(Model* getModel) {

    m_pModel = getModel;
    m_pModel->gravity = Vector3d(0., 0., -9.81);
    nDOF = m_pModel->dof_count - 6; //* get Degree of freedom, Except x,y,z,roll,pitch,yaw of the robot
    joint = new JOINT[nDOF]; //* only joint of the robot excepting x,y,z,roll,pitch,yaw of the robot
    RobotState = VectorNd::Zero(20);
    RobotStatedot = VectorNd::Zero(19);
    RobotState2dot = VectorNd::Zero(19);
    BasePosOri = VectorNd::Zero(6);
    BaseVel = VectorNd::Zero(6);
    JointAngle = VectorNd::Zero(nDOF);
    JointVel = VectorNd::Zero(nDOF);

    base.ID = m_pModel->GetBodyId("REAR_BODY");
    front_body.ID = m_pModel->GetBodyId("FRONT_BODY");
    FR.ID = m_pModel->GetBodyId("FR_CALF");
    FL.ID = m_pModel->GetBodyId("FL_CALF");
    RR.ID = m_pModel->GetBodyId("RR_CALF");
    RL.ID = m_pModel->GetBodyId("RL_CALF");
    //QQ << 0, 0, 0, 1;


    //m_pModel->SetQuaternion(base.ID, QQ, RobotState);
    //QQ=m_pModel->GetQuaternion(base.ID,Base_Quat_Info);
}
