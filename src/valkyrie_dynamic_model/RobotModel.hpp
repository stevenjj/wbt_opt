#ifndef ROBOT_MODEL
#define ROBOT_MODEL

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class Valkyrie_Dyn_Model;
class Valkyrie_Kin_Model;

using namespace sejong;

class RobotModel{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static RobotModel* GetRobotModel();
    virtual ~RobotModel(void);

    bool getMassInertia(sejong::Matrix & A);

    virtual bool getInverseMassInertia(sejong::Matrix & Ainv) ;
    virtual bool getGravity(Vector & grav) ;
    virtual bool getCoriolis(Vector & coriolis) ;

    virtual void getCentroidJacobian(sejong::Matrix & Jcent);
    virtual void getCentroidInertia(sejong::Matrix & Icent);
    virtual void getPosition(const Vector & q,
                             int link_id, Vect3 & pos) ;
    virtual void getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J) const ;
    virtual void getCoMPosition(const Vector & q, Vect3 & com_pos, bool update= false);
    virtual void getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel);

    void getFullJacobianDot(const Vector & q, const Vector & qdot, int link_id, sejong::Matrix & J) const ;
    void getOrientation(const Vector & q,
                        int link_id, sejong::Quaternion & ori) ;
    void getVelocity(const Vector & q, const Vector &qdot,
                     int link_id, Vect3 & vel) ;
    void getAngVel(const Vector & q, const Vector & qdot,
                   int link_id, Vect3 & ang_vel);

    void getCentroidVelocity(sejong::Vector & centroid_vel);
    void getCoMJacobian(const Vector & q, sejong::Matrix & J);
    void UpdateKinematics(const Vector & q, const Vector &qdot);
    void UpdateModel(const sejong::Vector & q, const sejong::Vector & qdot);
    void UpdateModel(int timestep, const sejong::Vector & q, const sejong::Vector & qdot);    

protected:
    Valkyrie_Dyn_Model* dyn_model_;
    Valkyrie_Kin_Model* kin_model_;

    RigidBodyDynamics::Model* model_;
private:
    RobotModel();
    int last_timestep_model_update = -1;
};

#endif
