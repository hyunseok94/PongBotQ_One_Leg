
#include <stdio.h>
#include <math.h>
#include "CRobot.h"

#include <time.h> // DH

CRobot::CRobot() {
}

CRobot::CRobot(const CRobot& orig) {
}

CRobot::~CRobot() {
}

void CRobot::setRobotModel(Model* getModel) {
    cout << endl << "Set Robot Model Start !!" << endl;

//    cout << "[1]" << endl;
    RBDL_Init(getModel);
//    cout << "[2]" << endl;
    PARA_Init();
//    cout << "[3]" << endl;
    NMPC_Init();
//    cout << "[4]" << endl;
    OSQP_Init();
//    cout << "[5]" << endl;

    cout << "Set Robot Model End !!" << endl;
}


void CRobot::NMPC_Init(void){
    // ================= DDP ====================//

    nmpc_run_flag = false;
    
    // moment of inertia tensor
//    MIT << 1.64, 0,    0,
//            0,   5.34, 0,
//            0,   0,    6.03; 
    
    MIT << 1.64, 0,    0,
            0,   5.34, 0,
            0,   0,    6.03; 
    
    inv_MIT = MIT.inverse();
    
//    cout << "inv_MIT = " << inv_MIT << endl;
  
//    ddp.Q_vec << 1, 10, 0.1,
//                 10, 1, 0,
//                 0.01, 0.1, 0.001,
//                 0.1, 0.01, 0.0;
//    ddp.Q_vec << 1, 1, 1,
//                 100, 100, 0,
//                 0.01, 0.01, 0.01,
//                 10, 10, 0.1;
    
    ddp.Q_vec << 1, 1, 100,
                 10, 10, 0,
                 0.01, 0.01, 10,
                 0.01, 0.01, 0.01;
    
    for(int i=0;i<Nx;++i){
        ddp.Q(i,i) = ddp.Q_vec(i);
    }
    
    ddp.Q_f = ddp.Q;
    ddp.R_vec = VectorNd::Ones(Nu)*pow(10,-6); //-4
   
    for(int i=0;i<Nu;++i){
        ddp.R(i,i) = ddp.R_vec(i);
    }
        
    ddp.f_init = F_ext.block(6, 0, 12, 1);
//    ddp.p_foot_init << RL.init_goal_pos_global,RR.init_goal_pos_global,FL.init_goal_pos_global,FR.init_goal_pos_global; 
    ddp.u_init = ddp.f_init;//, ddp.p_foot_init;
    ddp.x_init << com.init_pos, base.init_Euler_Ang, com.init_vel, base.init_Ang_Vel;
    ddp.x_ref  = ddp.x_init;//<< com.init_goal_pos, base.init_goal_Euler_Ang, com.init_goal_vel, base.init_goal_Ang_Vel;
    ddp.u_ref  = VectorNd::Zero(Nu);//, ddp.p_foot_init;
    ddp.u_mat_init = Kron(MatrixNd::Ones(1,Nh-1), ddp.u_init);
    ddp.x_des_tilde = Kron(MatrixNd::Ones(1,Nh), ddp.x_ref);
    ddp.u_des_tilde = Kron(MatrixNd::Ones(1,Nh), ddp.u_ref);
    RL.des_pos_global_tilde = Kron(MatrixNd::Ones(1,Nh), RL.des_pos_global);
    RR.des_pos_global_tilde = Kron(MatrixNd::Ones(1,Nh), RR.des_pos_global);
    FL.des_pos_global_tilde = Kron(MatrixNd::Ones(1,Nh), FL.des_pos_global);
    FR.des_pos_global_tilde = Kron(MatrixNd::Ones(1,Nh), FR.des_pos_global);
    ddp.c_ref << CONTACT_ON, CONTACT_ON, CONTACT_ON, CONTACT_ON; // RL, RR, FL, FR
    ddp.c_tilde = Kron(MatrixNd::Ones(1,Nh), ddp.c_ref);
    ddp.out_u_mat = ddp.u_mat_init;
    
    // ================= DDP Setting End ====================//
    
    // =============== 1 time DDP ===================== //
    ddp.u = ddp.u_mat_init;
    
    ddp.x_mat = PB_Dynamics(ddp.x_init,ddp.u);
    DDP_Process(ddp.x_mat,ddp.u);
    // =============== 1 time DDP End ===================== //
}

// Operating every 100ms
void CRobot::NMPC_Process(void){
    cout << "=========== [NMPC PROCESS] ===========" << endl;
    
    ddp.x << com.act_pos, base.act_Euler_Ang, com.act_vel, base.act_Ang_Vel;
    
//    cout << "com.act_pos = " << com.act_pos.transpose() << endl;
//    cout << "base.act_Euler_Ang = " << base.act_Euler_Ang.transpose()*180/PI << endl;
//    cout << "com.act_vel = " << com.act_vel.transpose() << endl;
//    cout << "base.act_Ang_Vel = " << base.act_Ang_Vel.transpose()*180/PI << endl;
//    cout << "ddp.x = " << ddp.x << endl;
//    ddp.x = ddp.out_x_mat.block(0,1,Nx,1);
    
    ddp.x_ref << com.des_pos, base.des_Euler_Ang, com.des_vel, base.des_Ang_Vel;
    
    // Update State, Input, and Contact values
    for(int i=0;i<Nh-1;++i){
        ddp.x_des_tilde.block(0,i,Nx,1) = ddp.x_des_tilde.block(0,i+1,Nx,1); 
        ddp.u_des_tilde.block(0,i,Nu,1) = ddp.u_des_tilde.block(0,i+1,Nu,1);
        ddp.c_tilde.block(0,i,4,1) = ddp.c_tilde.block(0,i+1,4,1); 
        
        RL.des_pos_global_tilde.block(0,i,3,1) = RL.des_pos_global_tilde.block(0,i+1,3,1);
        RR.des_pos_global_tilde.block(0,i,3,1) = RR.des_pos_global_tilde.block(0,i+1,3,1);
        FL.des_pos_global_tilde.block(0,i,3,1) = FL.des_pos_global_tilde.block(0,i+1,3,1);
        FR.des_pos_global_tilde.block(0,i,3,1) = FR.des_pos_global_tilde.block(0,i+1,3,1);
        
        RL.des_vel_global_tilde.block(0,i,3,1) = RL.des_vel_global_tilde.block(0,i+1,3,1);
        RR.des_vel_global_tilde.block(0,i,3,1) = RR.des_vel_global_tilde.block(0,i+1,3,1);
        FL.des_vel_global_tilde.block(0,i,3,1) = FL.des_vel_global_tilde.block(0,i+1,3,1);
        FR.des_vel_global_tilde.block(0,i,3,1) = FR.des_vel_global_tilde.block(0,i+1,3,1);
    }
    ddp.x_des_tilde.block(0,Nh-1,Nx,1) = ddp.x_ref;
    ddp.u_des_tilde.block(0,Nh-1,Nu,1) = ddp.u_ref;
    ddp.c_tilde.block(0,Nh-1,4,1) = ddp.c_ref;
    
    RL.des_pos_global_tilde.block(0,Nh-1,3,1) = RL.des_pos_global;
    RR.des_pos_global_tilde.block(0,Nh-1,3,1) = RR.des_pos_global;
    FL.des_pos_global_tilde.block(0,Nh-1,3,1) = FL.des_pos_global;
    FR.des_pos_global_tilde.block(0,Nh-1,3,1) = FR.des_pos_global;
    
    RL.des_vel_global_tilde.block(0,Nh-1,3,1) = RL.des_vel_global;
    RR.des_vel_global_tilde.block(0,Nh-1,3,1) = RR.des_vel_global;
    FL.des_vel_global_tilde.block(0,Nh-1,3,1) = FL.des_vel_global;
    FR.des_vel_global_tilde.block(0,Nh-1,3,1) = FR.des_vel_global;
    
    for(int i=0;i<Nh-2;++i){
        ddp.u.block(0,i,Nu,1) = ddp.out_u_mat.block(0,i+1,Nu,1); 
    }
    
    
    ddp.x_mat = PB_Dynamics(ddp.x,ddp.u);
    
    pre_F_ext = ddp.out_u_mat.block(0,0,Nu,1);
    
    // ==================== DDP Process ===================== //
    DDP_Process(ddp.x_mat, ddp.u);
    // ================================================= //
    
    
    cout << "[ddp out] u = "   <<  ddp.out_u_mat.block(0,0,Nu,1).transpose() << endl;
//    cout << "[ddp out] x = "   <<  ddp.out_x_mat.block(0,1,Nx,1).transpose() << endl;
//    cout << "[ddp out] J = "   <<  ddp.out_J << endl;
    
//    coef = coef_3nd_poly();
    
//    F_ext << VectorNd::Zero(6), ddp.out_u_mat.block(0,0,Nu,1);
    
//    cout << "F_ext = " << F_ext << endl;
    
    


    
    cout << "========================================" << endl << endl;
}

void CRobot::DDP_Process(MatrixNd x_mat, MatrixNd u_mat){
//    cout << "=========== [DDP PROCESS] ============" << endl;
    clock_t start, end;
    double result;
    
    static double J_pre = 0;
    const int maxIter = 50;
    
    J_pre = 0;
    
    ddp.out_x_mat = x_mat;
    ddp.out_u_mat = u_mat;
    
    for(unsigned int i=0;i<maxIter;++i){
        cout << "iter =" << i << endl;
//        start = clock();
        Finite_Diff(ddp.out_x_mat,ddp.out_u_mat,ddp.x_des_tilde,ddp.u_des_tilde);
//        end = clock(); 
//        result = (double)(end - start)/1000;
//        cout << "Finite_Diff : "<< result <<" (ms)"<< endl;

//        start = clock();
        Backward_Pass();
//        end = clock(); 
//        result = (double)(end - start)/1000;
//        cout << "Backward_Pass : "<< result <<" (ms)"<< endl;
        
//        start = clock();
        Forward_Pass(ddp.out_x_mat,ddp.out_u_mat);  
//        end = clock(); 
//        result = (double)(end - start)/1000;
//        cout << "Forward_Pass : "<< result <<" (ms)"<< endl;
        
//        start = clock();
        ddp.out_J = cal_J(ddp.out_x_mat,ddp.out_u_mat);
//        end = clock(); 
//        result = (double)(end - start)/1000;
//        cout << "out_J : "<< result <<" (ms)"<< endl;
//        cout << "========================" << endl<< endl;
        
        cout << "ddp.out_J = " << ddp.out_J << endl; 
        
        
        if(abs(ddp.out_J - J_pre) < 0.001){
//            cout << " Final iter = " << i << endl;
            cout << "============ J_err < 0.001 ============" << endl<< endl;
            break;
        }
        
        J_pre = ddp.out_J;
        
    }
}
double CRobot::cal_J(MatrixNd x_mat, MatrixNd u_mat){
    double J = 0;
    
    for(int i=0;i<Nh-1;++i){
        J = J + PB_cost(x_mat.block(0,i,Nx,1),u_mat.block(0,i,Nu,1),i);
    }
    J = J + PB_cost_final(x_mat.block(0,Nh-1,Nx,1));
    
    return J;
}

double CRobot::PB_cost(VectorNd x_vec, VectorNd u_vec, int i){
    double c1, c2;

    x_err = x_vec - ddp.x_des_tilde.block(0,i,Nx,1);
    
//    cout << "x_vec = " << x_vec <<endl;
//    cout << "ddp.x_des_tilde.block(0,i,Nx,1) = " << ddp.x_des_tilde.block(0,i,Nx,1) <<endl;
//    cout << "x_err = " << x_err <<endl;
//    cout << "==========================================" << endl << endl;
    
    u_err = u_vec - ddp.u_des_tilde.block(0,i,Nu,1);
    
    c1 = 0.5*x_err.transpose()*ddp.Q*x_err;
    c2 = 0.5*u_err.transpose()*ddp.R*u_err;
    
//    cout << "x_err.transpose() = " << x_err.transpose() << endl;
//    cout << "x_err.transpose()*ddp.Q = " << x_err.transpose()*ddp.Q << endl;
//    cout << "ddp.Q = " << ddp.Q << endl;
//    cout << "c1 = " << c1 << endl;

    return c1 + c2;
}

double CRobot::PB_cost_final(VectorNd x_vec){
    static double c1;
    
    x_err = x_vec - ddp.x_des_tilde.block(0,Nh-1,Nx,1);
    
    c1 = 0.5*x_err.transpose()*ddp.Q_f*x_err;

    return c1;
}

void CRobot::Forward_Pass(MatrixNd x_mat, MatrixNd u_mat){
    
    ddp.out_x_mat.block(0,0,Nx,1) = x_mat.block(0,0,Nx,1);
    
    for(int i=0;i<Nh-1;++i){
        dx = ddp.out_x_mat.block(0,i,Nx,1) - x_mat.block(0,i,Nx,1);
        for(int j=0;j<Nu;++j){
            tmp_k(j) = k[j][i];
            for(int k=0;k<Nx;++k){
                tmp_K(j,k) = K[j][k][i];
            }
        }
        
        du = tmp_k + tmp_K*dx;
        
        Get_u_min_max(ddp.c_tilde.block(0,i,4,1), u_mat.block(0,i,Nu,1));
        
//        [u_min,u_max] = u_min_max_cal(c_ref,u_mat(:,i));
        
        // Naive Clamping
        for(int j=0;j<Nu;++j){
            du(j)=max(ddp.u_min(j)-u_mat(j,i),min(du(j),ddp.u_max(j)-u_mat(j,i)));
        }
//        du = max(ddp.u_min-u_mat.block(0,i,Nu,1),min(du,ddp.u_max-u_mat.block(0,i,Nu,1)));
        
        ddp.out_u_mat.block(0,i,Nu,1) = u_mat.block(0,i,Nu,1) + du;
        ddp.out_x_mat.block(0,i+1,Nx,1) = PB_Dynamics_once(ddp.out_x_mat.block(0,i,Nx,1),ddp.out_u_mat.block(0,i,Nu,1),i);
    }
}

void CRobot::Get_u_min_max(VectorNd _c, VectorNd _u){
    const double max_Fz = 1000;
    const double mu = 0.6;
    
    Fz_vec << _u(2),_u(5),_u(8),_u(11);
    
    if(_c(0) == CONTACT_ON){
        max_Fz_vec(0) = max_Fz;
    }
    else{
        max_Fz_vec(0) = 0;
        Fz_vec(0) = 0;
    }
    
    if(_c(1) == CONTACT_ON){
        max_Fz_vec(1) = max_Fz;
    }
    else{
        max_Fz_vec(1) = 0;
        Fz_vec(1) = 0;
    }
    
    if(_c(2) == CONTACT_ON){
        max_Fz_vec(2) = max_Fz;
    }
    else{
        max_Fz_vec(2) = 0;
        Fz_vec(2) = 0;
    }
    
    if(_c(3) == CONTACT_ON){
        max_Fz_vec(3) = max_Fz;
    }
    else{
        max_Fz_vec(3) = 0;
        Fz_vec(3) = 0;
    }
    
    ddp.u_min << -mu*Fz_vec(0),-mu*Fz_vec(0),0,
                 -mu*Fz_vec(1),-mu*Fz_vec(1),0,
                 -mu*Fz_vec(2),-mu*Fz_vec(2),0,
                 -mu*Fz_vec(3),-mu*Fz_vec(3),0;
    
    ddp.u_max <<  mu*Fz_vec(0), mu*Fz_vec(0),max_Fz_vec(0),
                  mu*Fz_vec(1), mu*Fz_vec(1),max_Fz_vec(1),
                  mu*Fz_vec(2), mu*Fz_vec(2),max_Fz_vec(2),
                  mu*Fz_vec(3), mu*Fz_vec(3),max_Fz_vec(3);
}

void CRobot::Backward_Pass(void){
    
    for(unsigned int i=0;i<Nx;++i){
        Vx(i) = cx[i][Nh-1];
        
        for(unsigned int j=0;j<Nx;++j){
            Vxx(i,j) = cxx[i][j][Nh-1];
        }
    }
    
    for(int i=Nh-2;i>=0;--i){
        //  Array to Eigen Matrix
        Array2Eigen(i); 
                
        Qx = tmp_cx + tmp_fx.transpose()*Vx;
        Qu = tmp_cu + tmp_fu.transpose()*Vx;
        Qxx = tmp_cxx + tmp_fx.transpose()*Vxx*tmp_fx;
        Quu = tmp_cuu + tmp_fu.transpose()*Vxx*tmp_fu;
        Qux = tmp_cux + tmp_fu.transpose() * Vxx*tmp_fx;
        
        tmp_k = -Quu.inverse()*Qu;
        tmp_K = -Quu.inverse()*Qux;

        Vx = Qx - tmp_K.transpose()*Quu*tmp_k;
        Vxx = Qxx - tmp_K.transpose()*Quu*tmp_K;
        
        for(int j=0;j<Nu;++j){
            k[j][i] = tmp_k(j);
            
            for(int k=0;k<Nx;++k){
                K[j][k][i] = tmp_K(j,k);
            }
        }
    }
}


void CRobot::Array2Eigen(int i){
    for(int j=0;j<Nx;++j){
        tmp_cx(j) = cx[j][i];
    }

    for(int j=0;j<Nu;++j){
        tmp_cu(j) = cu[j][i];
    }

    for(int j=0;j<Nx;++j){
        for(int k=0;k<Nx;++k){
            tmp_cxx(j,k) = cxx[j][k][i];
        }
    }

    for(int j=0;j<Nu;++j){
        for(int k=0;k<Nu;++k){
            tmp_cuu(j,k) = cuu[j][k][i];
        }
    }

    for(int j=0;j<Nu;++j){
        for(int k=0;k<Nx;++k){
            tmp_cux(j,k) = cux[j][k][i];
        }
    }

    for(int j=0;j<Nx;++j){
        for(int k=0;k<Nx;++k){
            tmp_fx(j,k) = fx[j][k][i];
        }
    }

    for(int j=0;j<Nx;++j){
        for(int k=0;k<Nu;++k){
            tmp_fu(j,k) = fu[j][k][i];
        }
    }
}

void CRobot::Finite_Diff(MatrixNd x_mat,MatrixNd u_mat, MatrixNd x_des_mat, MatrixNd u_des_mat){
    const double del_x = 0.00001;
    const double del_u = 0.00001;
       
    // Find fx
    for(unsigned int i=0;i<Nx;++i){
        tmp_x_vec = VectorNd::Zero(Nx);
        tmp_x_vec(i) = 1;
        
        x_pur1 = x_mat + del_x*Kron(MatrixNd::Ones(1,Nh), tmp_x_vec);
        x_pur2 = x_mat - del_x*Kron(MatrixNd::Ones(1,Nh), tmp_x_vec);
                
        for(unsigned int j=0;j<Nh-1;++j){
            x_pur_mat1.block(0,j,Nx,1) = PB_Dynamics_once(x_pur1.block(0,j,Nx,1),u_mat.block(0,j,Nu,1),j);
            x_pur_mat2.block(0,j,Nx,1) = PB_Dynamics_once(x_pur2.block(0,j,Nx,1),u_mat.block(0,j,Nu,1),j);
        }
        x_pur_mat1.block(0,Nh-1,Nx,1) = PB_Dynamics_once(x_pur1.block(0,Nh-1,Nx,1),VectorNd::Zero(Nu),Nh-1);
        x_pur_mat2.block(0,Nh-1,Nx,1) = PB_Dynamics_once(x_pur2.block(0,Nh-1,Nx,1),VectorNd::Zero(Nu),Nh-1);
        
        tmp_fx2 = (x_pur_mat1 - x_pur_mat2)/(2*del_x);
        
        for(unsigned int j = 0;j<Nx;++j){
            for(unsigned int k = 0;k<Nh;++k){
                fx[j][i][k] = tmp_fx2(j,k);
            }
        }
    }

    // Find fu
    for(unsigned int i=0;i<Nu;++i){
        tmp_u_vec = VectorNd::Zero(Nu);
        tmp_u_vec(i) = 1;
        
        u_pur1 = u_mat + del_u*Kron(MatrixNd::Ones(1,Nh-1), tmp_u_vec);
        u_pur2 = u_mat - del_u*Kron(MatrixNd::Ones(1,Nh-1), tmp_u_vec);
                
        for(unsigned int j=0;j<Nh-1;++j){
            x_pur_mat1.block(0,j,Nx,1) = PB_Dynamics_once(x_mat.block(0,j,Nx,1),u_pur1.block(0,j,Nu,1),j);
            x_pur_mat2.block(0,j,Nx,1) = PB_Dynamics_once(x_mat.block(0,j,Nx,1),u_pur2.block(0,j,Nu,1),j);
        }
        x_pur_mat1.block(0,Nh-1,Nx,1) = PB_Dynamics_once(x_mat.block(0,Nh-1,Nx,1),VectorNd::Zero(Nu),Nh-1);
        x_pur_mat2.block(0,Nh-1,Nx,1) = PB_Dynamics_once(x_mat.block(0,Nh-1,Nx,1),VectorNd::Zero(Nu),Nh-1);
        
        tmp_fx2 = (x_pur_mat1 - x_pur_mat2)/(2*del_u);
        
        for(unsigned int j = 0;j<Nx;++j){
            for(unsigned int k = 0;k<Nh;++k){
                fu[j][i][k] = tmp_fx2(j,k);
            }
        }
    }
    
    // cx
    for(unsigned int i=0;i<Nh-1;++i){
        tmp_cx = ddp.Q*(x_mat.block(0,i,Nx,1) - x_des_mat.block(0,i,Nx,1));
        
        for(unsigned int j=0;j<Nx;++j){
            cx[j][i] = tmp_cx(j);
        }
    }
    tmp_cx = ddp.Q_f*(x_mat.block(0,Nh-1,Nx,1) - x_des_mat.block(0,Nh-1,Nx,1));
        
    for(unsigned int j=0;j<Nx;++j){
        cx[j][Nh-1] = tmp_cx(j);
    }
    
    // cu
    for(unsigned int i=0;i<Nh-1;++i){
        tmp_cu = ddp.R*(u_mat.block(0,i,Nu,1) - u_des_mat.block(0,i,Nu,1));
        
        for(unsigned int j=0;j<Nu;++j){
            cu[j][i] = tmp_cu(j);
        }
    }
    
    // cxx
    for(unsigned int i=0;i<Nx;++i){
        for(unsigned int j=0;j<Nx;++j){
            for(unsigned int k=0;k<Nh-1;++k){
                cxx[i][j][k] = ddp.Q(i,j);
            }
        }
    }
    
    for(unsigned int i=0;i<Nx;++i){
        for(unsigned int j=0;j<Nx;++j){
            cxx[i][j][Nh-1] = ddp.Q_f(i,j);
        }
    }
    
    // cuu
    for(unsigned int i=0;i<Nu;++i){
        for(unsigned int j=0;j<Nu;++j){
            for(unsigned int k=0;k<Nh-1;++k){
                cuu[i][j][k] = ddp.R(i,j);
            }
        }
    }
    
    for(unsigned int i=0;i<Nu;++i){
        for(unsigned int j=0;j<Nx;++j){
            for(unsigned int k=0;k<Nh-1;++k){
                cux[i][j][k] = 0;
            }
        }
    }    
}

VectorNd CRobot::PB_Dynamics_once(VectorNd x_vec,VectorNd u_vec, int i){
    
//    cout << "============= [PB_Dynamics_once] ============" << endl;

        com.tmp_des_pos        = x_vec.block(0,0,3,1);
        base.tmp_des_Euler_Ang = x_vec.block(3,0,3,1);
        p_com_dot              = x_vec.block(6,0,3,1);
        base.tmp_des_Ang_Vel   = x_vec.block(9,0,3,1);
        
        RL.tmp_f = u_vec.block(0,0,3,1);
        RR.tmp_f = u_vec.block(3,0,3,1);
        FL.tmp_f = u_vec.block(6,0,3,1);
        FR.tmp_f = u_vec.block(9,0,3,1);

        RL.tmp_des_pos_local = RL.des_pos_global_tilde.block(0,i,3,1) - com.tmp_des_pos;
        RR.tmp_des_pos_local = RR.des_pos_global_tilde.block(0,i,3,1) - com.tmp_des_pos;
        FL.tmp_des_pos_local = FL.des_pos_global_tilde.block(0,i,3,1) - com.tmp_des_pos;
        FR.tmp_des_pos_local = FR.des_pos_global_tilde.block(0,i,3,1) - com.tmp_des_pos;

        inv_E << 1,  sin(base.tmp_des_Euler_Ang(1))*sin(base.tmp_des_Euler_Ang(2))/cos(base.tmp_des_Euler_Ang(2)),  -cos(base.tmp_des_Euler_Ang(1))*sin(base.tmp_des_Euler_Ang(2))/cos(base.tmp_des_Euler_Ang(2)),
                 0,  cos(base.tmp_des_Euler_Ang(1)),                                                                 sin(base.tmp_des_Euler_Ang(1)),
                 0, -sin(base.tmp_des_Euler_Ang(1))/cos(base.tmp_des_Euler_Ang(2)),                                  cos(base.tmp_des_Euler_Ang(1))/cos(base.tmp_des_Euler_Ang(2));

        Euler_Ang_dot = inv_E * base.tmp_des_Ang_Vel;
        
        H_dot = Cross_prod(RL.tmp_des_pos_local,RL.tmp_f) + Cross_prod(RR.tmp_des_pos_local,RR.tmp_f) + Cross_prod(FL.tmp_des_pos_local,FL.tmp_f) + Cross_prod(FR.tmp_des_pos_local,FR.tmp_f);

        Ang_Vel_dot = inv_MIT*(H_dot - Cross_prod(base.tmp_des_Ang_Vel,MIT*base.tmp_des_Ang_Vel));
    
        p_com_2dot = (RL.tmp_f + RR.tmp_f + FL.tmp_f + FR.tmp_f)/M - gravity_vec;

        x_dot << p_com_dot, Euler_Ang_dot, p_com_2dot, Ang_Vel_dot;

        x_next_vec = x_vec + x_dot*Ts;
        
    return x_next_vec;
}


MatrixNd CRobot::PB_Dynamics(VectorNd x_vec,MatrixNd u_mat){
            
    x_next_mat.block(0, 0, Nx, 1) = x_vec;
    
    for(unsigned int i = 0;i<Nh-1;++i){
        
        com.tmp_des_pos        = x_next_mat.block(0,i,3,1);
        base.tmp_des_Euler_Ang = x_next_mat.block(3,i,3,1);
        p_com_dot              = x_next_mat.block(6,i,3,1);
        base.tmp_des_Ang_Vel   = x_next_mat.block(9,i,3,1);
        
        RL.tmp_f = u_mat.block(0,i,3,1);
        RR.tmp_f = u_mat.block(3,i,3,1);
        FL.tmp_f = u_mat.block(6,i,3,1);
        FR.tmp_f = u_mat.block(9,i,3,1);

        RL.tmp_des_pos_local = RL.des_pos_global_tilde.block(0,i,3,1) - com.tmp_des_pos;
        RR.tmp_des_pos_local = RR.des_pos_global_tilde.block(0,i,3,1) - com.tmp_des_pos;
        FL.tmp_des_pos_local = FL.des_pos_global_tilde.block(0,i,3,1) - com.tmp_des_pos;
        FR.tmp_des_pos_local = FR.des_pos_global_tilde.block(0,i,3,1) - com.tmp_des_pos;

        inv_E << 1,  sin(base.tmp_des_Euler_Ang(1))*sin(base.tmp_des_Euler_Ang(2))/cos(base.tmp_des_Euler_Ang(2)),  -cos(base.tmp_des_Euler_Ang(1))*sin(base.tmp_des_Euler_Ang(2))/cos(base.tmp_des_Euler_Ang(2)),
                 0,  cos(base.tmp_des_Euler_Ang(1)),                                                                 sin(base.tmp_des_Euler_Ang(1)),
                 0, -sin(base.tmp_des_Euler_Ang(1))/cos(base.tmp_des_Euler_Ang(2)),                                  cos(base.tmp_des_Euler_Ang(1))/cos(base.tmp_des_Euler_Ang(2));
        Euler_Ang_dot = inv_E * base.tmp_des_Ang_Vel;
        
        H_dot = Cross_prod(RL.tmp_des_pos_local,RL.tmp_f) + Cross_prod(RR.tmp_des_pos_local,RR.tmp_f) + Cross_prod(FL.tmp_des_pos_local,FL.tmp_f) + Cross_prod(FR.tmp_des_pos_local,FR.tmp_f);

        Ang_Vel_dot = inv_MIT*(H_dot - Cross_prod(base.tmp_des_Ang_Vel,MIT*base.tmp_des_Ang_Vel));
    
        p_com_2dot = (RL.tmp_f + RR.tmp_f + FL.tmp_f + FR.tmp_f)/M - gravity_vec;

        x_dot << p_com_dot, Euler_Ang_dot, p_com_2dot, Ang_Vel_dot;

        x_next_mat.block(0,i+1,Nx,1) = x_next_mat.block(0,i,Nx,1) + x_dot*Ts;
        
    }
    
    return x_next_mat;
}


// ==================================================================== //

void CRobot::StateUpdate(void) {
    RobotState(AXIS_X) = base.des_pos(0); //base.currentX;
    RobotState(AXIS_Y) = base.des_pos(1); //base.currentY;
    RobotState(AXIS_Z) = base.des_pos(2); //base.currentZ;
    RobotState(AXIS_Roll)  = base.des_Euler_Ang(0); //act_base_ori(0);//base_ori(0); //base.currentRoll;
    RobotState(AXIS_Pitch) = base.des_Euler_Ang(1); //act_base_ori(1);//base_ori(1); //base.currentPitch;
    RobotState(AXIS_Yaw)   = base.des_Euler_Ang(2); //base.currentYaw;

    RobotStatedot(AXIS_X) = base.des_vel(0); //base.currentXvel;
    RobotStatedot(AXIS_Y) = base.des_vel(1); //base.currentYvel;
    RobotStatedot(AXIS_Z) = base.des_vel(2); //base.currentZvel;
    RobotStatedot(AXIS_Roll)  = base.des_Ang_Vel(0); //base.currentRollvel;
    RobotStatedot(AXIS_Pitch) = base.des_Ang_Vel(1); //base.currentPitchvel;
    RobotStatedot(AXIS_Yaw)   = base.des_Ang_Vel(2); //base.currentYawvel;

    for (int nJoint = 0; nJoint < JOINT_NUM; ++nJoint) {
        RobotState(6 + nJoint) = joint->act_pos[nJoint];
        RobotStatedot(6 + nJoint) = joint->act_vel[nJoint];
        RobotState2dot(6 + nJoint) = joint->act_acc[nJoint];
    }
    
    base.des_Euler_Ang_quad = Math::Quaternion::fromXYZAngles(base.des_Euler_Ang);
    Math::Quaternion QQ(base.des_Euler_Ang_quad);
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);
    
    FK1();
    
    Get_act_com();
}


void CRobot::ComputeTorqueControl() {

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
    
    J_RL2 << J_RL.block(0, 6, 3, 3);
    J_RR2 << J_RR.block(0, 9, 3, 3);
    J_FL2 << J_FL.block(0, 12, 3, 3);
    J_FR2 << J_FR.block(0, 15, 3, 3);
    
    RL.act_q_dot << joint->act_vel[0], joint->act_vel[1],  joint->act_vel[2];
    RR.act_q_dot << joint->act_vel[3], joint->act_vel[4],  joint->act_vel[5];
    FL.act_q_dot << joint->act_vel[6], joint->act_vel[7],  joint->act_vel[8];
    FR.act_q_dot << joint->act_vel[9], joint->act_vel[10], joint->act_vel[11];
    
//    cout << "RL.act_q_dot = " << RL.act_q_dot << endl;
    
    RL.act_vel_local = J_RL2*RL.act_q_dot;
    RR.act_vel_local = J_RR2*RR.act_q_dot;
    FL.act_vel_local = J_FL2*FL.act_q_dot;
    FR.act_vel_local = J_FR2*FR.act_q_dot;
    
    actual_EP_vel << RL.act_vel_local, RR.act_vel_local, FL.act_vel_local, FR.act_vel_local;
    actual_EP << RL.act_pos_local, RR.act_pos_local, FL.act_pos_local, FR.act_pos_local;
        
    com.des_pos_now = ddp.x_des_tilde.block(0,0,3,1);
    com.des_vel_now = ddp.x_des_tilde.block(6,0,3,1);
    
    RL.des_pos_global_now = RL.des_pos_global_tilde.block(0,0,3,1);
    RR.des_pos_global_now = RR.des_pos_global_tilde.block(0,0,3,1);
    FL.des_pos_global_now = FL.des_pos_global_tilde.block(0,0,3,1);
    FR.des_pos_global_now = FR.des_pos_global_tilde.block(0,0,3,1);
    
    RL.des_vel_global_now = RL.des_vel_global_tilde.block(0,0,3,1);
    RR.des_vel_global_now = RR.des_vel_global_tilde.block(0,0,3,1);
    FL.des_vel_global_now = FL.des_vel_global_tilde.block(0,0,3,1);
    FR.des_vel_global_now = FR.des_vel_global_tilde.block(0,0,3,1);
            
    RL.des_pos_local = RL.des_pos_global_now - com.des_pos_now;
    RR.des_pos_local = RR.des_pos_global_now - com.des_pos_now;
    FL.des_pos_local = FL.des_pos_global_now - com.des_pos_now;
    FR.des_pos_local = FR.des_pos_global_now - com.des_pos_now;
    
//    cout << "RL.des_pos_global = " << RL.des_pos_global << endl;
//    cout << "RR.des_pos_global = " << RR.des_pos_global << endl;
//    cout << "FL.des_pos_global = " << FL.des_pos_global << endl;
//    cout << "FR.des_pos_global = " << FR.des_pos_global << endl;
//    cout << "================================" << endl << endl;
    
    target_EP << RL.des_pos_local, RR.des_pos_local, FL.des_pos_local ,FR.des_pos_local;
    
    joint->des_pos = IK1(target_EP,base.des_Euler_Ang);
    
    RL.des_vel_local = RL.des_vel_global_now - com.des_vel_now;
    RR.des_vel_local = RR.des_vel_global_now - com.des_vel_now;
    FL.des_vel_local = FL.des_vel_global_now - com.des_vel_now;
    FR.des_vel_local = FR.des_vel_global_now - com.des_vel_now;
    
    target_EP_vel << RL.des_vel_local, RR.des_vel_local, FL.des_vel_local ,FR.des_vel_local;
    
    for (int i = 0; i < JOINT_NUM; ++i) {
        pd_con_task[i + 6] = Kp_t[i]*(target_EP[i] - actual_EP[i]) + Kd_t[i]*(0 - actual_EP_vel[i]);
        pd_con_joint[i + 6] = Kp_q[i]*(joint->des_pos[i] - joint->act_pos[i]) + Kd_q[i]*(joint->des_vel[i] - joint->act_vel[i]);

//        tmp_data1[i] = Fc(i + 7);
//        tmp_data1[i + 25] = target_EP_vel[i];
    }
    
    
    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term);
    C_term = hatNonLinearEffects - G_term;
    
//    F_ext << 0,0,0,0,0,0, 0,0,110, 0,0,110, 0,0,110, 0,0,110; // Tmp
    
    // ============== Get F_ext ============== //
    
//    for(int i=0;i<Nu;++i){
//        tmp_F_ext(i) = ddp.u(i,0) + (ddp.out_u_mat(i,0) - ddp.u(i,0))/2.0*(1-cos(PI2/(Ts*2)*(double)(NMPC_thread_cnt)*dt));
//    }
    
//    cout << "NMPC_thread_cnt = " << NMPC_thread_cnt << endl;
//    
//    cout << "next u = " << ddp.out_u_mat.block(0,0,Nu,1).transpose() << endl;
//    
//    cout << "pre u = " << pre_F_ext.transpose() << endl;
    
//    tmp_F_ext = ddp.u.block(0,0,Nu,1) + (ddp.out_u_mat.block(0,0,Nu,1) - ddp.u.block(0,0,Nu,1))/2.0*(1-cos(PI2/(Ts*2)*(double)(NMPC_thread_cnt)*dt));
    
    tmp_F_ext = pre_F_ext + (ddp.out_u_mat.block(0,0,Nu,1) - pre_F_ext)/2.0*(1-cos(PI2/(Ts*2)*(double)(NMPC_thread_cnt)*dt));
    
    
//    cout << "tmp_F_ext(2) = " << tmp_F_ext(2) << endl;
    
//    tmp_data1(17) = NMPC_thread_cnt;
            
//    cout << "tmp_F_ext = " << tmp_F_ext.transpose() << endl;
            
    F_ext << VectorNd::Zero(6), tmp_F_ext;
    
//    F_ext << VectorNd::Zero(6), VectorNd::Zero(12);

//    CTC_Torque = fc_weight * (C_term + G_term - J_A.transpose() * (Fc - pd_con_task)) + pd_con_joint;
    
//    CTC_Torque = (C_term + G_term - J_A.transpose() * (F_ext - pd_con_task)) + pd_con_joint;

    CTC_Torque = C_term + G_term - J_A.transpose() * (F_ext - pd_con_task) + pd_con_joint;
    
//    cout << "CTC_Torque = " << CTC_Torque.transpose() << endl;
    
    for (int nJoint = 0; nJoint < JOINT_NUM; nJoint++) {
        joint->torque[nJoint] = CTC_Torque(6 + nJoint);
        
        tmp_data1(nJoint+1) = tmp_F_ext(nJoint);
        //        tmp_data1[nJoint + 25] = joint[nJoint].torque;
    }
    
    
//    cout << "ddp.x_des_tilde = " << ddp.x_des_tilde << endl;
    
    tmp_data1(13) = ddp.x_des_tilde(2,0); // com z pos.
    tmp_data1(14) = com.act_pos(2);
    tmp_data1(15) = ddp.x_des_tilde(8,0); // com z vel.
    tmp_data1(16) = com.act_vel(2);
    
    tmp_data1(17) = ddp.c_tilde(0,0)*100;
    
    
    base.des_pos = com.des_pos_now - com.offset;
    base.des_vel = com.des_vel;
    
//    cout << "=================================" << endl;


}



// =========================================================//





void CRobot::Home_Pos_PD_Con(void) {
    const double Kp_home = 500;
    const double Kd_home = 5;
    
//    ddp.c_ref << CONTACT_ON,CONTACT_ON,CONTACT_ON,CONTACT_ON;
    
//    cout << "ddp.c_ref = " << ddp.c_ref << endl;
    
    for(unsigned int i=0;i<JOINT_NUM;++i){
        joint->torque[i] = Kp_home * (joint->init_pos[i] * D2R - joint->act_pos[i]) + Kd_home * (0 - joint->act_vel[i]);
    }
}

void CRobot::Pronk_Jump(void) {
// =============== Pronk Initialize =============== //
//    cout << "[Pronk_Jump]" ;
    
    const double jump_ready_height = 0.35;
    
    const double jump_ready_time = 2.0;
    const int    jump_ready_cnt = 2000;
    const double jump_stance_time = 0.300;
    const int    jump_stance_cnt = 300;
    const double jump_flight_time = 0.30;
    const int    jump_flight_cnt = 300;
    const double jump_landing_time = 0.1;
    const int    jump_landing_cnt = 100;

    const double tmp_t1 = jump_ready_cnt;
    const double tmp_t2 = tmp_t1 + jump_stance_cnt;
    const double tmp_t3 = tmp_t2 + jump_flight_cnt;
    const double tmp_t4 = tmp_t3 + jump_landing_cnt;
    const double tmp_t5 = tmp_t4 + jump_ready_cnt;
    
    static double tmp_t_vec[3];

    // =============== Pronk Initialize END =============== //
    
    if (pr_cnt == 0) {
        ddp.c_ref << CONTACT_ON,CONTACT_ON,CONTACT_ON,CONTACT_ON;
        jump_phase = FRONK_JUMP_READY;
    }
    else if (pr_cnt == tmp_t1) {
        ddp.c_ref << CONTACT_ON,CONTACT_ON,CONTACT_ON,CONTACT_ON;
        jump_phase = FRONK_TAKE_OFF;
    }
    else if (pr_cnt == tmp_t2) {
        ddp.c_ref << CONTACT_OFF,CONTACT_OFF,CONTACT_OFF,CONTACT_OFF;
        jump_phase = FRONK_FLIGHT;
    }
    else if (pr_cnt == tmp_t3) {
        ddp.c_ref << CONTACT_ON,CONTACT_ON,CONTACT_ON,CONTACT_ON;
        jump_phase = FRONK_LAND;
    }
    else if (pr_cnt == tmp_t4) {
        ddp.c_ref << CONTACT_ON,CONTACT_ON,CONTACT_ON,CONTACT_ON;
        jump_phase = FRONK_FINAL;
    }
    else if (pr_cnt == tmp_t5) {
        ddp.c_ref << CONTACT_ON,CONTACT_ON,CONTACT_ON,CONTACT_ON;
        jump_phase = FRONK_NONE;
        
        move_done_flag = true;
    }
    
    
    
    switch(jump_phase){
    
        case FRONK_JUMP_READY :
//            cout << "======== [[FRONK_JUMP_READY]] =======" << endl;
            tmp_cnt = pr_cnt;
            tmp_time = (double)pr_cnt*dt;
            
            if(tmp_cnt == 0){
                com.des_pos = com.init_goal_pos;
                com.des_vel = com.init_goal_vel;
                
                RL.des_pos_global = RL.init_goal_pos_global;
                RR.des_pos_global = RR.init_goal_pos_global;
                FL.des_pos_global = FL.init_goal_pos_global;
                FR.des_pos_global = FR.init_goal_pos_global;
                
                tmp_t_vec[0] = jump_stance_time;
                tmp_t_vec[1] = jump_flight_time;
                tmp_t_vec[2] = jump_landing_time;
//                tmp_t_vec[3] = jump_stance_time;
                
                Jump_COM_Z_Traj_Gen(jump_ready_height, tmp_t_vec);
            }
            else{
                com.des_pos(2) = com.init_goal_pos(2) + (jump_ready_height - com.init_goal_pos(2))/2.0*(1-cos(PI2/(jump_ready_time*2)*tmp_time));
                com.des_vel(2) = (jump_ready_height - com.init_goal_pos(2))/2.0*PI2/(jump_ready_time*2)*(sin(PI2/(jump_ready_time*2)*tmp_time));
                com.des_acc(2) = 0;
            }
            
//            cout << "com.des_pos(2) = " << com.des_pos(2) << endl;

        break;
    
        case FRONK_TAKE_OFF :
//            cout << "======== [[FRONK_TAKE_OFF]] =======" << endl;
            tmp_cnt = pr_cnt - tmp_t1;
            tmp_time = (double)tmp_cnt*dt;
            
            com.des_pos(2) = fifth_order_poly(jump_z1, tmp_time);
            com.des_vel(2) = fifth_order_poly_dot(jump_z1, tmp_time);
            com.des_acc(2) = fifth_order_poly_2dot(jump_z1, tmp_time);
        
        break;
        
        case FRONK_FLIGHT :
//            cout << "======== [[FRONK_FLIGHT]] =======" << endl;
            tmp_cnt = pr_cnt - tmp_t2;
            tmp_time = (double)tmp_cnt*dt;
            
            com.des_pos(2) = fifth_order_poly(jump_z2, tmp_time);
            com.des_vel(2) = fifth_order_poly_dot(jump_z2, tmp_time);
            com.des_acc(2) = fifth_order_poly_2dot(jump_z2, tmp_time);
            
            RL.des_pos_global(2) = RL.init_goal_pos_global(2) + (0.10 - RL.init_goal_pos_global(2))/2.0*(1-cos(PI2/(jump_flight_time)*tmp_time));
            RR.des_pos_global(2) = RR.init_goal_pos_global(2) + (0.10 - RR.init_goal_pos_global(2))/2.0*(1-cos(PI2/(jump_flight_time)*tmp_time));
            FL.des_pos_global(2) = FL.init_goal_pos_global(2) + (0.10 - FL.init_goal_pos_global(2))/2.0*(1-cos(PI2/(jump_flight_time)*tmp_time));
            FR.des_pos_global(2) = FR.init_goal_pos_global(2) + (0.10 - FR.init_goal_pos_global(2))/2.0*(1-cos(PI2/(jump_flight_time)*tmp_time));
            
            RL.des_vel_global(2) = (0.10 - RL.init_goal_pos_global(2))/2.0*PI2/(jump_flight_time)*(sin(PI2/(jump_flight_time)*tmp_time));
            RR.des_vel_global(2) = (0.10 - RR.init_goal_pos_global(2))/2.0*PI2/(jump_flight_time)*(sin(PI2/(jump_flight_time)*tmp_time));
            FL.des_vel_global(2) = (0.10 - FL.init_goal_pos_global(2))/2.0*PI2/(jump_flight_time)*(sin(PI2/(jump_flight_time)*tmp_time));
            FR.des_vel_global(2) = (0.10 - FR.init_goal_pos_global(2))/2.0*PI2/(jump_flight_time)*(sin(PI2/(jump_flight_time)*tmp_time));
            
            
//            cout << "RL.des_pos_global(2) = " << RL.des_pos_global(2) << endl;
        
        break;
        
        case FRONK_LAND :
//            cout << "======== [[FRONK_LAND]] =======" << endl;
            tmp_cnt = pr_cnt - tmp_t3;
            tmp_time = (double)tmp_cnt*dt;
            
            com.des_pos(2) = fifth_order_poly(jump_z4, tmp_time);
            com.des_vel(2) = fifth_order_poly_dot(jump_z4, tmp_time);
            com.des_acc(2) = fifth_order_poly_2dot(jump_z4, tmp_time);
            
            RL.des_pos_global(2) = RL.init_goal_pos_global(2);
            RR.des_pos_global(2) = RR.init_goal_pos_global(2);
            FL.des_pos_global(2) = FL.init_goal_pos_global(2);
            FR.des_pos_global(2) = FR.init_goal_pos_global(2);
            
            RL.des_vel_global(2) = 0;
            RR.des_vel_global(2) = 0;
            FL.des_vel_global(2) = 0;
            FR.des_vel_global(2) = 0;
        
        break;
        
        case FRONK_FINAL :
//            cout << "======== [[FRONK_FINAL]] =======" << endl;
            tmp_cnt = pr_cnt - tmp_t4;
            tmp_time = (double)tmp_cnt*dt;
            
            com.des_pos(2) = jump_ready_height + (com.init_goal_pos(2) - jump_ready_height)/2.0*(1-cos(PI2/(jump_ready_time*2)*tmp_time));
            com.des_vel(2) = (com.init_goal_pos(2) - jump_ready_height)/2.0*PI2/(jump_ready_time*2)*(sin(PI2/(jump_ready_time*2)*tmp_time));
            com.des_acc(2) = 0;
        
        break;
        
        default :
            com.des_pos(2) = com.init_goal_pos(2);
            com.des_vel(2) = 0;
            com.des_acc(2) = 0;
            
        break;
    
        
    
    }
    
    
    
    
    
    pr_cnt++;
}

double CRobot::fifth_order_poly(double c[], double t) {
    static double y = 0;
    y = c[5] * pow(t, 5) + c[4] * pow(t, 4) + c[3] * pow(t, 3) + c[2] * pow(t, 2) + c[1] * pow(t, 1) + c[0];
    return y;
}

double CRobot::fifth_order_poly_dot(double c[], double t) {
    static double y = 0;
    y = 5 * c[5] * pow(t, 4) + 4 * c[4] * pow(t, 3) + 3 * c[3] * pow(t, 2) + 2 * c[2] * pow(t, 1) + 1 * c[1];
    return y;
}

double CRobot::fifth_order_poly_2dot(double c[], double t) {
    static double y = 0;
    y = 20 * c[5] * pow(t, 3) + 12 * c[4] * pow(t, 2) + 6 * c[3] * pow(t, 1) + 2 * c[2];
    return y;
}


void CRobot::Jump_COM_Z_Traj_Gen(double h0, double t[3]) {

    const double jump_h_0 = h0;
    const double jump_v_0 = 0;
    const double jump_a_0 = 0;

    const double jump_v_1 = 0.6;
    const double jump_a_1 = -GRAVITY;

    const double jump_h_2 = jump_h_0-0.00;
    const double jump_v_2 = -0.5; //-0.05;
    const double jump_a_2 = -GRAVITY;

    const double jump_h_3 = jump_h_0;
    const double jump_v_3 = 0;
    const double jump_a_3 = 0;

    const double jump_h_1 = 0.5*GRAVITY*t[1]*t[1] - jump_v_1*t[1] + jump_h_2;

    // =============== Stance phase (First) =============== //
    init_x[0] = jump_h_0;
    init_x[1] = jump_v_0;
    init_x[2] = jump_a_0;

    final_x[0] = jump_h_1;
    final_x[1] = jump_v_1;
    final_x[2] = jump_a_1;

    coefficient_5thPoly(init_x, final_x, t[0], jump_z1);

    // =============== Flight phase =============== //
    init_x[0] = jump_h_1;
    init_x[1] = jump_v_1;
    init_x[2] = jump_a_1;

    final_x[0] = jump_h_2;
    final_x[1] = jump_v_2;
    final_x[2] = jump_a_2;

    coefficient_5thPoly(init_x, final_x, t[1], jump_z2);
    
    // =============== Stance phase (First) =============== //
    init_x[0] = jump_h_2;
    init_x[1] = jump_v_2;
    init_x[2] = jump_a_2;

    final_x[0] = jump_h_1;
    final_x[1] = jump_v_1;
    final_x[2] = jump_a_1;

    coefficient_5thPoly(init_x, final_x, t[0], jump_z3);

    // =============== Landing phase =============== //
    init_x[0] = jump_h_2;
    init_x[1] = jump_v_2;
    init_x[2] = jump_a_2;

    final_x[0] = jump_h_3;
    final_x[1] = jump_v_3;
    final_x[2] = jump_a_3;

    coefficient_5thPoly(init_x, final_x, t[2], jump_z4);

}


void CRobot::coefficient_5thPoly(double *init_x, double *final_x, double tf, double *output) {
    R << init_x[0], final_x[0], init_x[1], final_x[1], init_x[2], final_x[2];

    static double temp_t1, temp_t2;
    
    temp_t1 = 0;
    temp_t2 = tf;

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

void CRobot::WalkReady_Pos_Traj(void) {
    if (wr_cnt == 0) {
//        cout << "[1]" << endl;
        move_done_flag = false;
        
//        RL.CT = true;
//        RR.CT = true;
//        FL.CT = true;
//        FR.CT = true;
        ddp.c_ref << CONTACT_ON,CONTACT_ON,CONTACT_ON,CONTACT_ON;

        RL.init_pos_global = RL.act_pos_global;
        RR.init_pos_global = RR.act_pos_global;
        FL.init_pos_global = FL.act_pos_global;
        FR.init_pos_global = FR.act_pos_global;
        
        RL.des_pos_global = RL.init_pos_global;
        RR.des_pos_global = RR.init_pos_global;
        FL.des_pos_global = FL.init_pos_global;
        FR.des_pos_global = FR.init_pos_global;
        
        RL.des_pos_global_tilde = Kron(MatrixNd::Ones(1,Nh), RL.des_pos_global);
        RR.des_pos_global_tilde = Kron(MatrixNd::Ones(1,Nh), RR.des_pos_global);
        FL.des_pos_global_tilde = Kron(MatrixNd::Ones(1,Nh), FL.des_pos_global);
        FR.des_pos_global_tilde = Kron(MatrixNd::Ones(1,Nh), FR.des_pos_global);
        
        RL.des_vel_global = RL.init_vel_global;
        RR.des_vel_global = RR.init_vel_global;
        FL.des_vel_global = FL.init_vel_global;
        FR.des_vel_global = FR.init_vel_global;

        com.des_pos = com.init_pos;
        com.des_vel = com.init_vel;
        
        nmpc_run_flag = true;
        
//        base.des_Euler_Ang << 0,0,0;
//        base.des_Ang_Vel   << 0,0,0;

        wr_cnt++;
    } 
    else if (wr_cnt <= walk_ready_cnt) {
//        cout << "[2]" << endl;
        tmp_time = (double) (wr_cnt) * dt;
        com.des_pos = com.init_pos;
        com.des_vel = com.init_vel;
        
        RL.des_pos_global = RL.init_pos_global + (RL.init_goal_pos_global - RL.init_pos_global)/2.0*(1 - cos(PI2/(walk_ready_time * 2)*tmp_time));
        RR.des_pos_global = RR.init_pos_global + (RR.init_goal_pos_global - RR.init_pos_global)/2.0*(1 - cos(PI2/(walk_ready_time * 2)*tmp_time));
        FL.des_pos_global = FL.init_pos_global + (FL.init_goal_pos_global - FL.init_pos_global)/2.0*(1 - cos(PI2/(walk_ready_time * 2)*tmp_time));
        FR.des_pos_global = FR.init_pos_global + (FR.init_goal_pos_global - FR.init_pos_global)/2.0*(1 - cos(PI2/(walk_ready_time * 2)*tmp_time));
        
        RL.des_vel_global = (RL.init_goal_pos_global - RL.init_pos_global)/2.0*PI2/(walk_ready_time * 2)*(sin(PI2/(walk_ready_time * 2)*tmp_time));
        RR.des_vel_global = (RR.init_goal_pos_global - RR.init_pos_global)/2.0*PI2/(walk_ready_time * 2)*(sin(PI2/(walk_ready_time * 2)*tmp_time));
        FL.des_vel_global = (FL.init_goal_pos_global - FL.init_pos_global)/2.0*PI2/(walk_ready_time * 2)*(sin(PI2/(walk_ready_time * 2)*tmp_time));
        FR.des_vel_global = (FR.init_goal_pos_global - FR.init_pos_global)/2.0*PI2/(walk_ready_time * 2)*(sin(PI2/(walk_ready_time * 2)*tmp_time));
        
        wr_cnt++;
        
   } else if (wr_cnt <= walk_ready_cnt * 2) {
//        cout << "[3]" << endl;
        tmp_time = (double) (wr_cnt - walk_ready_cnt) * dt;
        com.des_pos = com.init_pos + (com.init_goal_pos - com.init_pos)/2.0*(1 - cos(PI2/(walk_ready_time * 2)*tmp_time));
        com.des_vel = (com.init_goal_pos - com.init_pos)/2.0*PI2/(walk_ready_time * 2)*(sin(PI2/(walk_ready_time * 2)*tmp_time));

        RL.des_pos_global = RL.init_goal_pos_global;
        RR.des_pos_global = RR.init_goal_pos_global;
        FL.des_pos_global = FL.init_goal_pos_global;
        FR.des_pos_global = FR.init_goal_pos_global;

        RL.des_vel_global = RL.init_goal_vel_global;
        RR.des_vel_global = RL.init_goal_vel_global;
        FL.des_vel_global = RL.init_goal_vel_global;
        FR.des_vel_global = RL.init_goal_vel_global;
        
        wr_cnt++;
//
        if (wr_cnt == walk_ready_cnt * 2) {
            walk_ready_move_done_flag = true;
            cout << "!! Walk Ready Done !!" << endl;

            move_done_flag = true;
        }
   } else {
//       cout << "[4 finish !]" << endl;
       
        com.des_pos = com.init_goal_pos;
        com.des_vel = com.init_goal_vel;

        RL.des_pos_global = RL.init_goal_pos_global;
        RR.des_pos_global = RR.init_goal_pos_global;
        FL.des_pos_global = FL.init_goal_pos_global;
        FR.des_pos_global = FR.init_goal_pos_global;

        RL.des_vel_global = RL.init_goal_vel_global;
        RR.des_vel_global = RL.init_goal_vel_global;
        FL.des_vel_global = RL.init_goal_vel_global;
        FR.des_vel_global = RL.init_goal_vel_global;
    }
}

void CRobot::Get_act_com(void) {
    // ============== Get COM Position & Orientation ============ //
    VectorNd _c = VectorNd::Zero(4);
    static double contact_num = 0;
    const double  pos_alpha = 0.02;
    const double  vel_alpha = 0.02;
    VectorNd tmp_act_base_pos = VectorNd::Zero(3);
    VectorNd tmp_act_base_vel = VectorNd::Zero(3);
    VectorNd tmp_act_com_acc = VectorNd::Zero(3);
    VectorNd pre_com_act_vel = VectorNd::Zero(3);
    
    _c[0] = ddp.c_tilde(0,0);//RL.CT;
    _c[1] = ddp.c_tilde(1,0);//RR.CT;
    _c[2] = ddp.c_tilde(2,0);//FL.CT;
    _c[3] = ddp.c_tilde(3,0);//FR.CT;
    
    contact_num = 0;

    for(unsigned int i = 0;i<4;++i){
        if(_c[i] == CONTACT_ON){
            contact_num++;
        }
    }
    
//    cout << "contact_num = " << contact_num << endl;
    
//    if (contact_num != 0) {
//        tmp_act_base_pos = (_c(0)*(RL.des_pos_global-RL.act_pos_local)+_c(1)*(RR.des_pos_global-RR.act_pos_local)+_c(2)*(FL.des_pos_global-FL.act_pos_local)+_c(3)*(FR.des_pos_global-FR.act_pos_local))/contact_num;   
//        tmp_act_base_vel = (_c(0)*(-RL.act_vel_local)+_c(1)*(-RR.act_vel_local)+_c(2)*(-FL.act_vel_local)+_c(3)*(-FR.act_vel_local))/contact_num;
//        
////        cout << "tmp_act_base_vel = " << tmp_act_base_vel << endl;
//    } else {
//        tmp_act_base_pos = ((RL.des_pos_global-RL.act_pos_local)+(RR.des_pos_global-RR.act_pos_local)+(FL.des_pos_global-FL.act_pos_local)+(FR.des_pos_global-FR.act_pos_local))/4;   
//        tmp_act_base_vel = ((-RL.act_vel_local)+(-RR.act_vel_local)+(-FL.act_vel_local)+(-FR.act_vel_local))/4;
//    }
    
    if(contact_num == 0){
        contact_num = 4;
        _c[0] = 1;
        _c[1] = 1;
        _c[2] = 1;
        _c[3] = 1;
        
    }
    
    VectorNd pin_point = VectorNd::Zero(2);
    
    pin_point(0) = (_c(0)*(RL.des_pos_global(0))+_c(1)*(RR.des_pos_global(0))+_c(2)*(FL.des_pos_global(0))+_c(3)*(FR.des_pos_global(0)))/contact_num;
    pin_point(1) = (_c(0)*(RL.des_pos_global(1))+_c(1)*(RR.des_pos_global(1))+_c(2)*(FL.des_pos_global(1))+_c(3)*(FR.des_pos_global(1)))/contact_num;
    
    tmp_act_base_pos(0) = com.des_pos_now(2)*(IMUPitch) + pin_point(0);
    tmp_act_base_pos(1) = com.des_pos_now(2)*(-IMURoll) + pin_point(1);
    tmp_act_base_pos(2) = (_c(0)*(RL.des_pos_global(2)-RL.act_pos_local(2))+_c(1)*(RR.des_pos_global(2)-RR.act_pos_local(2))+_c(2)*(FL.des_pos_global(2)-FL.act_pos_local(2))+_c(3)*(FR.des_pos_global(2)-FR.act_pos_local(2)))/contact_num;   

    tmp_act_base_pos(0) = com.des_pos_now(2)*(IMUPitch_dot);
    tmp_act_base_pos(1) = com.des_pos_now(2)*(-IMURoll_dot);
    tmp_act_base_vel(2) = (_c(0)*(-RL.act_vel_local(2))+_c(1)*(-RR.act_vel_local(2))+_c(2)*(-FL.act_vel_local(2))+_c(3)*(-FR.act_vel_local(2)))/contact_num;
    
    
//    if (contact_num != 0) {
//        tmp_act_base_pos(0) = com.des_pos(2)*(-IMURoll) + (_c(0)*(RL.des_pos_global)+_c(1)*(RR.des_pos_global)+_c(2)*(FL.des_pos_global)+_c(3)*(FR.des_pos_global))/contact_num;
//        
//        tmp_act_base_pos = (_c(0)*(RL.des_pos_global-RL.act_pos_local)+_c(1)*(RR.des_pos_global-RR.act_pos_local)+_c(2)*(FL.des_pos_global-FL.act_pos_local)+_c(3)*(FR.des_pos_global-FR.act_pos_local))/contact_num;   
//        tmp_act_base_vel = (_c(0)*(-RL.act_vel_local)+_c(1)*(-RR.act_vel_local)+_c(2)*(-FL.act_vel_local)+_c(3)*(-FR.act_vel_local))/contact_num;
//        
////        cout << "tmp_act_base_vel = " << tmp_act_base_vel << endl;
//    } else {
//        tmp_act_base_pos = ((RL.des_pos_global-RL.act_pos_local)+(RR.des_pos_global-RR.act_pos_local)+(FL.des_pos_global-FL.act_pos_local)+(FR.des_pos_global-FR.act_pos_local))/4;   
//        tmp_act_base_vel = ((-RL.act_vel_local)+(-RR.act_vel_local)+(-FL.act_vel_local)+(-FR.act_vel_local))/4;
//    }

    // LPF
    base.act_pos = (1 - pos_alpha) * base.act_pos + pos_alpha*tmp_act_base_pos;
    base.act_vel = (1 - vel_alpha) * base.act_vel + vel_alpha*tmp_act_base_vel;


//    //    cout << "act_base_pos(0) = " << act_base_pos(0) << ", act_base_pos(1) = " << act_base_pos(1) << ", act_base_pos(2) = " << act_base_pos(2) << endl;
//    //    cout << "act_base_pos2(0) = " << act_base_pos2(0) << ", act_base_pos2(1) = " << act_base_pos2(1) << ", act_base_pos2(2) = " << act_base_pos2(2) << endl;
//    //    cout << "act_RL_foot_pos_local[2] = " << act_RL_foot_pos_local[2] << endl;
//    //    cout << "===============================================================" << endl;

    com.act_pos = base.act_pos + com.offset;
    com.act_vel = base.act_vel;
    
    tmp_act_com_acc = (com.act_vel - pre_com_act_vel)/dt;
    
    pre_com_act_vel = com.act_vel;
    
    com.act_acc = (1 - 0.02)*com.act_acc + 0.02*tmp_act_com_acc;
    
    base.act_Euler_Ang << IMURoll, IMUPitch, IMUYaw;
    base.act_Ang_Vel << IMURoll_dot, IMUPitch_dot, IMUYaw_dot;


//    // low pass filter
//    const double tmp_base_alpha = 1;
//    act_base_ori = (1 - tmp_base_alpha) * act_base_ori + tmp_base_alpha*tmp_act_base_ori;
//    act_base_ori_dot = (1 - tmp_base_alpha) * act_base_ori_dot + tmp_base_alpha*tmp_act_base_ori_dot;

    if (base.act_Euler_Ang(0) > 40 * D2R) {
        base.act_Euler_Ang(0) = 40 * D2R;
    } else if (base.act_Euler_Ang(0) < -40 * D2R) {
        base.act_Euler_Ang(0) = -40 * D2R;
    }

    if (base.act_Euler_Ang(1) > 40 * D2R) {
        base.act_Euler_Ang(1) = 40 * D2R;
    } else if (base.act_Euler_Ang(1) < -40 * D2R) {
        base.act_Euler_Ang(1) = -40 * D2R;
    }
}




void CRobot::FK1(void) {

    RL.act_pos_global = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RL.ID, EP_OFFSET_RL, true);
    RR.act_pos_global = CalcBodyToBaseCoordinates(*m_pModel, RobotState, RR.ID, EP_OFFSET_RR, true);
    FL.act_pos_global = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FL.ID, EP_OFFSET_FL, true);
    FR.act_pos_global = CalcBodyToBaseCoordinates(*m_pModel, RobotState, FR.ID, EP_OFFSET_FR, true);
    
//    cout << "RL.act_pos_global = " << RL.act_pos_global.transpose() << endl;
//    cout << "RR.act_pos_global = " << RR.act_pos_global.transpose() << endl;
//    cout << "FL.act_pos_global = " << FL.act_pos_global.transpose() << endl;
//    cout << "FR.act_pos_global = " << FR.act_pos_global.transpose() << endl;

    RL.act_pos_local = RL.act_pos_global - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);
    RR.act_pos_local = RR.act_pos_global - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);
    FL.act_pos_local = FL.act_pos_global - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);
    FR.act_pos_local = FR.act_pos_global - CalcBodyToBaseCoordinates(*m_pModel, RobotState, base.ID, Originbase, true);
}



VectorNd CRobot::Get_COM(VectorNd base_pos, VectorNd base_ori, VectorNd q) {
    const double m_body = 26.0;
    const double m_hp = 1.534;
    const double m_thigh = 3.32;
    const double m_calf = 0.738;
    const double m_leg = m_hp + m_thigh + m_calf;
    const double m_robot = m_leg * 4 + m_body;
    
    VectorNd p_base2body_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_hp_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_RL_calf_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_hp_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_RR_calf_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_hp_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_FL_calf_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_hp_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_thigh_com = VectorNd::Zero(4, 1);
    VectorNd p_FR_calf_com = VectorNd::Zero(4, 1);
    
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

    // Link com position
    p_base2body_com << -0.01304, 0, 0.04958, 1;
    p_RL_hp_com     <<  0.0, -0.00546, 0.0, 1;
    p_RL_thigh_com  <<  0.0, -0.01146, -0.02836, 1;
    p_RL_calf_com   <<  0.05246, 0.00259, -0.11592, 1;
    p_RR_hp_com     <<  0.0, 0.00546, 0.0, 1;
    p_RR_thigh_com  <<  0.0, 0.01146, -0.02836, 1;
    p_RR_calf_com   <<  0.05246, -0.00259, -0.11592, 1;
    p_FL_hp_com     <<  0.0, -0.00546, 0.0, 1;
    p_FL_thigh_com  <<  0.0, -0.01146, -0.02836, 1;
    p_FL_calf_com   <<  0.05246, 0.00259, -0.11592, 1;
    p_FR_hp_com     <<  0.0, 0.00546, 0.0, 1;
    p_FR_thigh_com  <<  0.0, 0.01146, -0.02836, 1;
    p_FR_calf_com   <<  0.05246, -0.00259, -0.11592, 1;

    // Transformation & Rotation matrix (Base)
    R_w2base_R << 1, 0, 0, 0,
                  0, cos(base_ori(0)), -sin(base_ori(0)), 0,
                  0, sin(base_ori(0)), cos(base_ori(0)), 0,
                  0, 0, 0, 1;
    R_w2base_P << cos(base_ori(1)), 0, sin(base_ori(1)), 0,
                  0, 1, 0, 0,
                  -sin(base_ori(1)), 0, cos(base_ori(1)), 0,
                  0, 0, 0, 1;
    R_w2base_Y << cos(base_ori(2)), -sin(base_ori(2)), 0, 0,
                  sin(base_ori(2)), cos(base_ori(2)), 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1;

    R_w2base = R_w2base_Y * R_w2base_P * R_w2base_R;
    T_w2base << 1, 0, 0, base_pos(0),
                0, 1, 0, base_pos(1),
                0, 0, 1, base_pos(2),
                0, 0, 0, 1;

    // Transformation & Rotation matrix (Leg)
    // RL
    TR_RL_base2hp << 1, 0, 0, -0.35,
                     0, cos(q(0)), -sin(q(0)), 0.115,
                     0, sin(q(0)), cos(q(0)), -0.053,
                     0, 0, 0, 1;

    TR_RL_hp2thigh << cos(q(1)), 0, sin(q(1)), 0.0,
                          0, 1, 0, 0.105,
                          -sin(q(1)), 0, cos(q(1)), 0.0,
                          0, 0, 0, 1;

    TR_RL_thigh2calf << cos(q(2)), 0, sin(q(2)), 0.0,
                        0, 1, 0, 0.0,
                        -sin(q(2)), 0, cos(q(2)), -0.305,
                        0, 0, 0, 1;

    p_RL_base2hp_com = TR_RL_base2hp*p_RL_hp_com;
    p_RL_base2thigh_com = TR_RL_base2hp * TR_RL_hp2thigh*p_RL_thigh_com;
    p_RL_base2calf_com = TR_RL_base2hp * TR_RL_hp2thigh * TR_RL_thigh2calf*p_RL_calf_com;

    p_RL_com = (m_hp * p_RL_base2hp_com + m_thigh * p_RL_base2thigh_com + m_calf * p_RL_base2calf_com) / (m_leg);

    //    cout << "p_RL_com = " << p_RL_com << endl;

    // RR
    TR_RR_base2hp << 1, 0, 0, -0.35,
            0, cos(q(3)), -sin(q(3)), -0.115,
            0, sin(q(3)), cos(q(3)), -0.053,
            0, 0, 0, 1;

    TR_RR_hp2thigh << cos(q(4)), 0, sin(q(4)), 0.0,
            0, 1, 0, -0.105,
            -sin(q(4)), 0, cos(q(4)), 0.0,
            0, 0, 0, 1;

    TR_RR_thigh2calf << cos(q(5)), 0, sin(q(5)), 0.0,
            0, 1, 0, 0.0,
            -sin(q(5)), 0, cos(q(5)), -0.305,
            0, 0, 0, 1;

    p_RR_base2hp_com = TR_RR_base2hp*p_RR_hp_com;
    p_RR_base2thigh_com = TR_RR_base2hp * TR_RR_hp2thigh*p_RR_thigh_com;
    p_RR_base2calf_com = TR_RR_base2hp * TR_RR_hp2thigh * TR_RR_thigh2calf*p_RR_calf_com;

    p_RR_com = (m_hp * p_RR_base2hp_com + m_thigh * p_RR_base2thigh_com + m_calf * p_RR_base2calf_com) / (m_leg);

    //    cout << "p_RR_com = " << p_RR_com << endl;

    // FL
    TR_FL_base2hp << 1, 0, 0, 0.35,
            0, cos(q(6)), -sin(q(6)), 0.115,
            0, sin(q(6)), cos(q(6)), -0.053,
            0, 0, 0, 1;

    TR_FL_hp2thigh << cos(q(7)), 0, sin(q(7)), 0.0,
            0, 1, 0, 0.105,
            -sin(q(7)), 0, cos(q(7)), 0.0,
            0, 0, 0, 1;

    TR_FL_thigh2calf << cos(q(8)), 0, sin(q(8)), 0.0,
            0, 1, 0, 0.0,
            -sin(q(8)), 0, cos(q(8)), -0.305,
            0, 0, 0, 1;

    p_FL_base2hp_com = TR_FL_base2hp*p_FL_hp_com;
    p_FL_base2thigh_com = TR_FL_base2hp * TR_FL_hp2thigh*p_FL_thigh_com;
    p_FL_base2calf_com = TR_FL_base2hp * TR_FL_hp2thigh * TR_FL_thigh2calf*p_FL_calf_com;

    p_FL_com = (m_hp * p_FL_base2hp_com + m_thigh * p_FL_base2thigh_com + m_calf * p_FL_base2calf_com) / (m_leg);

    //    cout << "p_FL_com = " << p_FL_com << endl;

    // FR
    TR_FR_base2hp << 1, 0, 0, 0.35,
            0, cos(q(9)), -sin(q(9)), -0.115,
            0, sin(q(9)), cos(q(9)), -0.053,
            0, 0, 0, 1;

    TR_FR_hp2thigh << cos(q(10)), 0, sin(q(10)), 0.0,
            0, 1, 0, -0.105,
            -sin(q(10)), 0, cos(q(10)), 0.0,
            0, 0, 0, 1;

    TR_FR_thigh2calf << cos(q(11)), 0, sin(q(11)), 0.0,
            0, 1, 0, 0.0,
            -sin(q(11)), 0, cos(q(11)), -0.305,
            0, 0, 0, 1;

    p_FR_base2hp_com = TR_FR_base2hp*p_FR_hp_com;
    p_FR_base2thigh_com = TR_FR_base2hp * TR_FR_hp2thigh*p_FR_thigh_com;
    p_FR_base2calf_com = TR_FR_base2hp * TR_FR_hp2thigh * TR_FR_thigh2calf*p_FR_calf_com;

    p_FR_com = (m_hp * p_FR_base2hp_com + m_thigh * p_FR_base2thigh_com + m_calf * p_FR_base2calf_com) / (m_leg);

    // COM from base
    p_robot_com_from_base = (m_body * p_base2body_com + m_leg * (p_RL_com + p_RR_com + p_FL_com + p_FR_com)) / (m_robot);
    p_robot_com_from_w = T_w2base * R_w2base*p_robot_com_from_base;
    p_robot_com = p_robot_com_from_w.block<3, 1>(0, 0);

    cout << "p_robot_com = " << p_robot_com << endl;

    return p_robot_com;
}

VectorNd CRobot::IK1(VectorNd EP,VectorNd base_ori) {
    const double L1 = 0.105;
    const double L2 = 0.305;
    const double L3 = 0.309; //0.305;

    static double x = 0;
    static double y = 0;
    static double z = 0;
    
    MatrixNd ROT_Y = MatrixNd::Zero(3, 3);
    VectorNd tmp_foot_pos = VectorNd::Zero(3);
    VectorNd target_pos = VectorNd::Zero(12);
    
    ROT_Y << cos(base_ori(1)*1), 0, sin(base_ori(1)*1),
            0, 1, 0,
            -sin(base_ori(1)*1), 0, cos(base_ori(1)*1);


    tmp_foot_pos << -(EP[0] - RL_base2hip_pos(0)),
                      EP[1] - RL_base2hip_pos(1),
                      EP[2] - RL_base2hip_pos(2);

    tmp_foot_pos = ROT_Y*tmp_foot_pos;

    x = tmp_foot_pos(0);
    y = tmp_foot_pos(1);
    z = tmp_foot_pos(2);

    if (z < (-0.6)) z = -0.6;

    target_pos[0] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    target_pos[1] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[2] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    tmp_foot_pos << -(EP[3] - RR_base2hip_pos(0)),
                      EP[4] - RR_base2hip_pos(1),
                      EP[5] - RR_base2hip_pos(2);

    tmp_foot_pos = ROT_Y*tmp_foot_pos;

    x = tmp_foot_pos(0); 
    y = tmp_foot_pos(1);
    z = tmp_foot_pos(2); 

    if (z < (-0.6)) z = -0.6;

    target_pos[3] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); 
    target_pos[4] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[5] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    tmp_foot_pos << -(EP[6] - FL_base2hip_pos(0)),
                      EP[7] - FL_base2hip_pos(1),
                      EP[8] - FL_base2hip_pos(2);

    tmp_foot_pos = ROT_Y*tmp_foot_pos;

    x = tmp_foot_pos(0); 
    y = tmp_foot_pos(1);
    z = tmp_foot_pos(2); 

    if (z < (-0.6)) z = -0.6;

    target_pos[6] = atan(y / abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    target_pos[7] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[8] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    tmp_foot_pos << -(EP[9] - FR_base2hip_pos(0)),
                      EP[10] - FR_base2hip_pos(1),
                      EP[11] - FR_base2hip_pos(2);

    tmp_foot_pos = ROT_Y*tmp_foot_pos;

    x = tmp_foot_pos(0); 
    y = tmp_foot_pos(1);
    z = tmp_foot_pos(2); 


    if (z < (-0.6)) z = -0.6;

    target_pos[9] = PI / 2 + atan(y / abs(z)) - acos(L1 / sqrt(pow(y, 2) + pow(z, 2))); 
    target_pos[10] = -(-atan(x / sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    target_pos[11] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    return target_pos;
}



void CRobot::Torque_off(void) {
    for(int i = 0; i < JOINT_NUM; ++i) {
        joint->torque[i] = 0;
        joint->des_vel[i] = 0;
        joint->des_acc[i] = 0;
    }
}


MatrixNd CRobot::Kron(MatrixNd AA, MatrixNd BB) {

    MatrixNd Out = MatrixNd::Zero(AA.rows() * BB.rows(), AA.cols() * BB.cols());
    //    cout << "AA = " << AA << endl;
    //    cout << "BB = " << BB << endl;
    //    cout << "Out(0,0) = " << Out(0,0) << endl;

    for (int i = 0; i < AA.rows(); i++) {
        for (int j = 0; j < AA.cols(); j++) {
            for (int k = 0; k < BB.rows(); k++) {
                for (int l = 0; l < BB.cols(); l++) {
                    Out(i * BB.rows() + k, j * BB.cols() + l) = AA(i, j) * BB(k, l);
                }
            }
        }
    }

    return Out;
}


void CRobot::OSQP_Init(void){
    // =============== OSQP TEST ===============//
    
//    // Load problem data
//    c_float P_x[3] = {4.0, 1.0, 2.0, };
//    c_int P_nnz = 3;
//    c_int P_i[3] = {0, 0, 1, };
//    c_int P_p[3] = {0, 1, 3, };
//    c_float q[2] = {1.0, 1.0, };
//    c_float A_x[4] = {1.0, 1.0, 1.0, 1.0, };
//    c_int A_nnz = 4;
//    c_int A_i[4] = {0, 1, 0, 2, };
//    c_int A_p[3] = {0, 2, 4, };
//    c_float l[3] = {1.0, 0.0, 0.0, };
//    c_float u[3] = {1.0, 0.7, 0.7, };
//    c_int n = 2;
//    c_int m = 3;
//
//    // Exitflag
//    c_int exitflag = 0;
//
//    // Workspace structures
//    OSQPWorkspace *work;
//    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
//    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));
//
//    // Populate data
//    if (data) {
//        data->n = n;
//        data->m = m;
//        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
//        data->q = q;
//        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
//        data->l = l;
//        data->u = u;
//    }
//
//    // Define solver settings as default
//    if (settings) {
//        osqp_set_default_settings(settings);
//        settings->alpha = 1.0; // Change alpha parameter
//    }
//
//    // Setup workspace
//    exitflag = osqp_setup(&work, data, settings);
//
//    // Solve Problem
//    osqp_solve(work);
//    
//    cout << "solution x[0]: " << work->solution->x[0] << endl;
//    cout << "solution x[1]: " << work->solution->x[1] << endl;
//    
//
//    // Cleanup
//    if (data) {
//        if (data->A) c_free(data->A);
//        if (data->P) c_free(data->P);
//        c_free(data);
//    }
//    if (settings) c_free(settings);
    
}

void CRobot::RBDL_Init(Model* getModel){
    // ================= For RBDL =================== //
    m_pModel = getModel;
    m_pModel->gravity = Vector3d(0., 0., -9.81);
    nDOF = m_pModel->dof_count - 6; //* get Degree of freedom, Except x,y,z,roll,pitch,yaw of the robot
    joint = new JOINT[nDOF]; //* only joint of the robot excepting x,y,z,roll,pitch,yaw of the robot
    RobotState = VectorNd::Zero(19);
    RobotStatedot = VectorNd::Zero(18);
    RobotState2dot = VectorNd::Zero(18);

    base.ID = m_pModel->GetBodyId("BODY");
    RL.ID = m_pModel->GetBodyId("RL_CALF");
    RR.ID = m_pModel->GetBodyId("RR_CALF");
    FL.ID = m_pModel->GetBodyId("FL_CALF");
    FR.ID = m_pModel->GetBodyId("FR_CALF");

    QQ << 0, 0, 0, 1;
    m_pModel->SetQuaternion(base.ID, QQ, RobotState);
    
    // ================= RBDL Setting Done =================== //
}

void CRobot::PARA_Init(void){
    joint->init_pos << 0, 45, -90, 0, 45, -90, 0, 45, -90, 0, 45, -90;

    com.height = 0.4;
    
    com.init_goal_pos        << 0,0,com.height;
    com.init_goal_vel        << 0,0,0;
    base.init_goal_Euler_Ang << 0,0,0;
    base.init_goal_Ang_Vel   << 0,0,0;
    
    base.init_pos       << 0, 0, com.height;
    base.init_vel       << 0, 0, 0;
    base.init_Euler_Ang << 0, 0, 0;
    base.init_Ang_Vel   << 0, 0, 0;
    
    base.des_pos = base.init_pos;
    base.des_vel = base.init_vel;
    base.des_Euler_Ang << 0, 0, 0;
    base.des_Ang_Vel   << 0, 0, 0;
    
    RL_base2hip_pos << -0.350,  0.115, -0.0;
    RR_base2hip_pos << -0.350, -0.115, -0.0;
    FL_base2hip_pos <<  0.350,  0.115, -0.0;
    FR_base2hip_pos <<  0.350, -0.115, -0.0;
    
    init_RL_hip2EP << 0, 0.105,-com.height;
    init_RR_hip2EP << 0,-0.105,-com.height;
    init_FL_hip2EP << 0, 0.105,-com.height;
    init_FR_hip2EP << 0,-0.105,-com.height;
    
    RL.init_goal_pos_global = base.init_pos + RL_base2hip_pos + init_RL_hip2EP;
    RR.init_goal_pos_global = base.init_pos + RR_base2hip_pos + init_RR_hip2EP;
    FL.init_goal_pos_global = base.init_pos + FL_base2hip_pos + init_FL_hip2EP;
    FR.init_goal_pos_global = base.init_pos + FR_base2hip_pos + init_FR_hip2EP;
    
    RL.des_pos_global = RL.init_goal_pos_global;
    RR.des_pos_global = RR.init_goal_pos_global;
    FL.des_pos_global = FL.init_goal_pos_global;
    FR.des_pos_global = FR.init_goal_pos_global;
    
//    cout << "RL.des_pos_global = " << RL.des_pos_global << endl;
//    cout << "RR.des_pos_global = " << RR.des_pos_global << endl;
//    cout << "FL.des_pos_global = " << FL.des_pos_global << endl;
//    cout << "FR.des_pos_global = " << FR.des_pos_global << endl;
//    cout << "================================" << endl << endl;
                
    for (unsigned int i = 0; i < JOINT_NUM; ++i) {
        joint->torque[i] = 0;
    }
    
    init_EP << RL.init_pos_global - base.init_pos, RR.init_pos_global - base.init_pos, FL.init_pos_global - base.init_pos, FR.init_pos_global - base.init_pos;
    joint->des_pos = IK1(init_EP,base.des_Euler_Ang);
    
//    cout << "joint->des_pos = " << joint->des_pos*R2D << endl;
    
    com.init_pos = Get_COM(base.des_pos, base.des_Euler_Ang, joint->des_pos);
    
    com.offset = com.init_pos - base.init_pos; // Base to Actual COM position
    
    double tmp_fz = g*M/4; 
//    F_ext << 0,0,0,0,0,0, 0,0,110, 0,0,110, 0,0,110, 0,0,110; // init F_ext
    F_ext << 0,0,0,0,0,0, 0,0,tmp_fz, 0,0,tmp_fz, 0,0,tmp_fz, 0,0,tmp_fz; // init F_ext
    
    gravity_vec << 0,0,g;
    
    // =========== Flag initialization ============ //
    move_done_flag = true;
    walk_ready_move_done_flag = false;
  
    // ============ GAIN Setting ============ //
//    Kp_q << 400, 100, 100,
//            400, 100, 100,
//            400, 100, 100,
//            400, 100, 100;

//    Kd_q << 15, 5, 5,
//            15, 5, 5,
//            15, 5, 5,
//            15, 5, 5;

    Kp_q << 100, 10, 10,
            100, 10, 10,
            100, 10, 10,
            100, 10, 10;
    
    Kd_q << 10, 1, 1,
            10, 1, 1,
            10, 1, 1,
            10, 1, 1;

    Kp_t << 100, 100, 100,
            100, 100, 100,
            100, 100, 100,
            100, 100, 100;

    Kd_t << 1, 1, 1,
            1, 1, 1,
            1, 1, 1,
            1, 1, 1;
}


VectorNd CRobot::Cross_prod(VectorNd x,VectorNd y){
    out(0) =  x(1)*y(2)-x(2)*y(1);
    out(1) = -x(0)*y(2)+x(2)*y(0);
    out(2) =  x(0)*y(1)-x(1)*y(0);

    return out;
}
