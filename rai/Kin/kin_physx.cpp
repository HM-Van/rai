/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


/**
 * @file
 * @ingroup group_ors
 */
/**
 * @ingroup group_ors
 * @{
 */


#ifdef MLR_PHYSX

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <physx/PxPhysicsAPI.h>
#include <physx/extensions/PxExtensionsAPI.h>
#include <physx/extensions/PxDefaultErrorCallback.h>
#include <physx/extensions/PxDefaultAllocator.h>
#include <physx/extensions/PxDefaultSimulationFilterShader.h>
#include <physx/extensions/PxDefaultCpuDispatcher.h>
#include <physx/extensions/PxShapeExt.h>
#include <physx/foundation/PxMat33.h>
#include <physx/pvd/PxVisualDebugger.h>
#include <physx/physxvisualdebuggersdk/PvdConnectionFlags.h>
//#include <PxMat33Legacy.h>
#include <physx/extensions/PxSimpleFactory.h>
#pragma GCC diagnostic pop

#include "kin_physx.h"
#include "frame.h"
#include <Gui/opengl.h>

using namespace physx;

static PxFoundation* mFoundation = NULL;
static PxPhysics* mPhysics = NULL;
static PxCooking* mCooking = NULL;
static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;


// ============================================================================
/**
 * @brief Connect ors with PhysX and add a cmaera.
 *
 * See bindOrsToOpenGL for a similar function.
 *
 * @param graph the graph PhysX is going to use.
 * @param gl the gl output.
 * @param physx the PhyxXInteface which handles the ors graph.
 */
void bindOrsToPhysX(mlr::KinematicWorld& graph, OpenGL& gl, PhysXInterface& physx) {
//  physx.create(graph);
  
  MLR_MSG("I don't understand this: why do you need a 2nd opengl window? (This is only for sanity check in the example.)")
  gl.add(glStandardScene, NULL);
  gl.add(physx);
  gl.setClearColors(1., 1., 1., 1.);
  
  mlr::Frame* glCamera = graph.getFrameByName("glCamera");
  if(glCamera) {
    gl.camera.X = glCamera->X;
  } else {
    gl.camera.setPosition(10., -15., 8.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();
  }
  gl.update();
}

// ============================================================================

void PxTrans2OrsTrans(mlr::Transformation& f, const PxTransform& pose) {
  f.pos.set(pose.p.x, pose.p.y, pose.p.z);
  f.rot.set(pose.q.w, pose.q.x, pose.q.y, pose.q.z);
}

PxTransform conv_Transformation2PxTrans(const mlr::Transformation& f) {
  return PxTransform(PxVec3(f.pos.x, f.pos.y, f.pos.z), PxQuat(f.rot.x, f.rot.y, f.rot.z, f.rot.w));
}

PxTransform Id_PxTrans() {
  return PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(0.f, 0.f, 0.f, 1.f));
}

arr conv_PxVec3_arr(const PxVec3& v){
  return {v.x, v.y, v.z};
}

// ============================================================================
//stuff from Samples/PxToolkit

namespace PxToolkit {
PxConvexMesh* createConvexMesh(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, PxConvexFlags flags) {
  PxConvexMeshDesc convexDesc;
  convexDesc.points.count     = vertCount;
  convexDesc.points.stride    = sizeof(PxVec3);
  convexDesc.points.data      = verts;
  convexDesc.flags        = flags;
  
  PxDefaultMemoryOutputStream buf;
  if(!cooking.cookConvexMesh(convexDesc, buf))
    return NULL;
    
  PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  return physics.createConvexMesh(input);
}

PxTriangleMesh* createTriangleMesh32(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, const PxU32* indices32, PxU32 triCount) {
  PxTriangleMeshDesc meshDesc;
  meshDesc.points.count     = vertCount;
  meshDesc.points.stride      = 3*sizeof(float);
  meshDesc.points.data      = verts;
  
  meshDesc.triangles.count    = triCount;
  meshDesc.triangles.stride   = 3*sizeof(uint);
  meshDesc.triangles.data     = indices32;
  
  PxDefaultMemoryOutputStream writeBuffer;
  bool status = cooking.cookTriangleMesh(meshDesc, writeBuffer);
  if(!status)
    return NULL;
    
  PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
  return physics.createTriangleMesh(readBuffer);
}
}
// ============================================================================

struct sPhysXInterface {
  PxScene* gScene = NULL;
  mlr::Array<PxRigidActor*> actors;
  mlr::Array<mlr::BodyType> actorTypes;
  mlr::Array<PxD6Joint*> joints;
  OpenGL *gl=NULL;

  debugger::comm::PvdConnection* connection = NULL;
  
  void addLink(mlr::Frame *b, physx::PxMaterial *material);
  void addJoint(mlr::Joint *jj);

  void lockJoint(PxD6Joint *joint, mlr::Joint *ors_joint);
  void unlockJoint(PxD6Joint *joint, mlr::Joint *ors_joint);
};

// ============================================================================

PhysXInterface::PhysXInterface(mlr::KinematicWorld& _world): world(_world), s(NULL) {
  s = new sPhysXInterface;

  if(!mFoundation) {
    mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
    mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale());
    PxCookingParams cookParams(mPhysics->getTolerancesScale());
    cookParams.skinWidth = .001f;
    mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, cookParams);
    if(!mCooking) HALT("PxCreateCooking failed!");
    if(!mPhysics) HALT("Error creating PhysX3 device.");

    if(!PxInitExtensions(*mPhysics))
      HALT("PxInitExtensions failed!");
  }

  //PxExtensionVisualDebugger::connect(mPhysics->getPvdConnectionManager(),"localhost",5425, 10000, true);

  //-- Create the scene
  PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.f, 0.f, -9.8f);

  if(!sceneDesc.cpuDispatcher) {
    PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if(!mCpuDispatcher) {
      cerr << "PxDefaultCpuDispatcherCreate failed!" << endl;
    }
    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }
  if(!sceneDesc.filterShader) {
    sceneDesc.filterShader  = gDefaultFilterShader;
  }

  s->gScene = mPhysics->createScene(sceneDesc);
  if(!s->gScene) {
    cerr << "createScene failed!" << endl;
  }

  s->gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);
  s->gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

  //-- Create objects
  PxMaterial* mMaterial = mPhysics->createMaterial(10.f, 10.f, 0.1f);

  //Create ground plane
  //PxReal d = 0.0f;
  PxTransform pose = PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)));

  PxRigidStatic* plane = mPhysics->createRigidStatic(pose);
  CHECK(plane, "create plane failed!");

  PxShape* planeShape = plane->createShape(PxPlaneGeometry(), *mMaterial);
  CHECK(planeShape, "create shape failed!");
  s->gScene->addActor(*plane);

  //-- create Kin equivalent in PhysX
  // loop through kin
  s->actors.resize(world.frames.N); s->actors.setZero();
  s->actorTypes.resize(world.frames.N); s->actorTypes.setZero();
  for(mlr::Frame *a : world.getLinks()) s->addLink(a, mMaterial);
//  for(mlr::Joint *j : world.fwdActiveJoints) s->addJoint(j);

  /// save data for the PVD
  if(mlr::getParameter<bool>("physx_debugger", false)) {
    const char* filename = "pvd_capture.pxd2";
    PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();

    s->connection = PxVisualDebuggerExt::createConnection(mPhysics->getPvdConnectionManager(), filename, connectionFlags);
    mPhysics->getVisualDebugger()->setVisualDebuggerFlags(PxVisualDebuggerFlag::eTRANSMIT_CONTACTS | PxVisualDebuggerFlag::eTRANSMIT_CONSTRAINTS);
  }
}

PhysXInterface::~PhysXInterface() {
  ShutdownPhysX();
  delete s;
}

void PhysXInterface::step(double tau, bool withKinematicPush) {
  //-- push positions of all kinematic objects
  if(withKinematicPush){
    for(mlr::Frame *b: world.frames) if(b->inertia && b->inertia->type==mlr::BT_kinematic) {
      ((PxRigidDynamic*)s->actors(b->ID))->setKinematicTarget(conv_Transformation2PxTrans(b->X));
    }
  }

  //-- dynamic simulation
  s->gScene->simulate(tau);
  
  //...perform useful work here using previous frame's state data
  while(!s->gScene->fetchResults()) {
  }
  
  //-- pull state of all objects
  pullFromPhysx();
}

void PhysXInterface::setArticulatedBodiesKinematic(){
  HALT("NOT SURE IF THIS IS DESIRED");
  for(mlr::Joint* j:world.fwdActiveJoints) if(j->type!=mlr::JT_free){
    if(j->from()->inertia && j->from()->inertia->type==mlr::BT_dynamic) j->from()->inertia->type=mlr::BT_kinematic;
    if(j->frame.inertia   && j->frame.inertia->type==mlr::BT_dynamic) j->frame.inertia->type=mlr::BT_kinematic;
  }
  for(mlr::Frame *b: world.frames) if(s->actors(b->ID) && b->inertia){
    if(b->inertia->type==mlr::BT_kinematic)
      ((PxRigidDynamic*)s->actors(b->ID))->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
    if(b->inertia->type==mlr::BT_dynamic)
      ((PxRigidDynamic*)s->actors(b->ID))->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, false);
  }
}

/**
 * @brief Create the PhysX interface which then can be used by OpenGL.
 *
 * - setup some physx stuff
 * - create PhysX equivalent to the ors graph
 */

void sPhysXInterface::addJoint(mlr::Joint *jj) {
  HALT("REALLY?");
  while(joints.N <= jj->frame.ID)
    joints.append(NULL);

//  cout <<"ADDING JOINT " <<jj->frame.parent->name <<'-' <<jj->frame.name <<endl;

  mlr::Transformation rel;
  mlr::Frame *from = jj->frame.getUpwardLink(rel);

  if(!jj->frame.inertia || !from || !from->inertia) return;
  CHECK(jj->frame.inertia, "this joint belongs to a frame '" <<jj->frame.name <<"' without inertia");
  CHECK(from, "this joint ('" <<jj->frame.name <<"') links from NULL");
  CHECK(from->inertia, "this joint ('" <<jj->frame.name <<"') links from a frame '" <<from->name <<"' without inertia");

  PxTransform A = conv_Transformation2PxTrans(rel);
  PxTransform B = Id_PxTrans();
  switch(jj->type) {
    case mlr::JT_free: //do nothing
      break;
    case mlr::JT_hingeX:
    case mlr::JT_hingeY:
    case mlr::JT_hingeZ: {

      PxD6Joint *desc = PxD6JointCreate(*mPhysics, actors(from->ID), A, actors(jj->frame.ID), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      if(jj->frame.ats.find<arr>("drive")) {
        arr drive_values = jj->frame.ats.get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, true);
        desc->setDrive(PxD6Drive::eTWIST, drive);
      }
      
      if(jj->frame.ats.find<arr>("limit")) {
        desc->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);

        arr limits = jj->frame.ats.get<arr>("limit");
        PxJointAngularLimitPair limit(limits(0), limits(1), 0.1f);
        limit.restitution = limits(2);
          //limit.spring = limits(3);
          //limit.damping= limits(4);
        //}
        desc->setTwistLimit(limit);
      } else {
        desc->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
      }

      if(jj->frame.ats.find<arr>("drive")) {
        arr drive_values = jj->frame.ats.get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, false);
        desc->setDrive(PxD6Drive::eTWIST, drive);
        //desc->setDriveVelocity(PxVec3(0, 0, 0), PxVec3(5e-1, 0, 0));
      }
      joints(jj->frame.ID) = desc;
    }
    break;
    case mlr::JT_rigid: {
      // PxFixedJoint* desc =
      PxFixedJointCreate(*mPhysics, actors(jj->from()->ID), A, actors(jj->frame.ID), B.getInverse());
      // desc->setProjectionLinearTolerance(1e10);
      // desc->setProjectionAngularTolerance(3.14);
    }
    break;
    case mlr::JT_trans3: {
      break; 
    }
    case mlr::JT_transXYPhi: {
      PxD6Joint *desc = PxD6JointCreate(*mPhysics, actors(jj->from()->ID), A, actors(jj->frame.ID), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      desc->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      desc->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
      desc->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

      joints(jj->frame.ID) = desc;
      break;
    }
    case mlr::JT_transX:
    case mlr::JT_transY:
    case mlr::JT_transZ:
    {
      PxD6Joint *desc = PxD6JointCreate(*mPhysics, actors(jj->from()->ID), A, actors(jj->frame.ID), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      if(jj->frame.ats.find<arr>("drive")) {
        arr drive_values = jj->frame.ats.get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, true);
        desc->setDrive(PxD6Drive::eX, drive);
      }
      
      if(jj->frame.ats.find<arr>("limit")) {
        desc->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);

        arr limits = jj->frame.ats.get<arr>("limit");
        PxJointLinearLimit limit(mPhysics->getTolerancesScale(), limits(0), 0.1f);
        limit.restitution = limits(2);
        //if(limits(3)>0) {
          //limit.spring = limits(3);
          //limit.damping= limits(4);
        //}
        desc->setLinearLimit(limit);
      }
      else {
        desc->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      }
      joints(jj->frame.ID) = desc;
    }
    break;
    default:
      NIY;
  }
}
void sPhysXInterface::lockJoint(PxD6Joint *joint, mlr::Joint *ors_joint) {
  joint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
  joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
  joint->setTwistLimit(PxJointAngularLimitPair(joint->getTwist()-.001, joint->getTwist()+.001));
}
void sPhysXInterface::unlockJoint(PxD6Joint *joint, mlr::Joint *ors_joint) {
  switch(ors_joint->type) {
    case mlr::JT_hingeX:
    case mlr::JT_hingeY:
    case mlr::JT_hingeZ:
      //joint->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
      //joint->setLinearLimit(PxJointLimit(ors_joint->Q.rot.getRad(), ors_joint->Q.rot.getRad()));
      joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
      break;
    case mlr::JT_transX:
    case mlr::JT_transY:
    case mlr::JT_transZ:
      //joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLOCKED);
      joint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      break;
    default:
      break;
  }
}

void sPhysXInterface::addLink(mlr::Frame *b, physx::PxMaterial *mMaterial) {
  PxRigidDynamic* actor=NULL;

  mlr::BodyType type = mlr::BT_static;
  if(b->joint) type = mlr::BT_kinematic;
  if(b->inertia) type = b->inertia->type;
  actorTypes(b->ID) = type;

  switch(type) {
  case mlr::BT_static:
    actor = (PxRigidDynamic*) mPhysics->createRigidStatic(conv_Transformation2PxTrans(b->X));
    break;
  case mlr::BT_dynamic:
    actor = mPhysics->createRigidDynamic(conv_Transformation2PxTrans(b->X));
    break;
  case mlr::BT_kinematic:
    actor = mPhysics->createRigidDynamic(conv_Transformation2PxTrans(b->X));
    actor->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
    break;
  case mlr::BT_none:
    HALT("this shoudn't be none BT!?")
        //      actor = mPhysics->createRigidDynamic(OrsTrans2PxTrans(b->X));
        break;
  }
  CHECK(actor, "create actor failed!");

  FrameL parts = {b};
  b->getRigidSubFrames(parts);

  cout <<MLR_HERE <<"adding '" <<b->name <<"' as " <<type <<" with parts";
  for(auto* p:parts) cout <<' ' <<p->name; cout <<endl;

  for(mlr::Frame *p: parts){
    mlr::Shape *s = p->shape;
    if(!s) continue;
    if(s->frame.name.startsWith("coll_")) continue; //these are the 'pink' collision boundary shapes..
    PxGeometry* geometry;
    switch(s->type()) {
      case mlr::ST_box: {
        geometry = new PxBoxGeometry(.5*s->size(0), .5*s->size(1), .5*s->size(2));
      }
      break;
      case mlr::ST_sphere: {
        geometry = new PxSphereGeometry(s->size(3));
      }
      break;
      case mlr::ST_capsule: {
        geometry = new PxCapsuleGeometry(s->size(3), .5*s->size(2));
      }
      break;
      case mlr::ST_cylinder:
      case mlr::ST_ssBox:
      case mlr::ST_mesh: {
        // Note: physx can't decompose meshes itself.
        // Physx doesn't support triangle meshes in dynamic objects! See:
        // file:///home/mtoussai/lib/PhysX/Documentation/PhysXGuide/Manual/Shapes.html
        // We have to decompose the meshes "by hand" and feed them to PhysX.

        // PhysX uses float for the vertices
        floatA Vfloat;

        Vfloat.clear();
        copy(Vfloat, s->mesh().V); //convert vertices from double to float array..
        PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
            *mPhysics, *mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
            PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eINFLATE_CONVEX);
        geometry = new PxConvexMeshGeometry(triangleMesh);
      }
      break;
      case mlr::ST_marker: {
        geometry = NULL;
      }
      break;
      default:
        NIY;
    }
    if(geometry) {
      PxShape* shape = NULL;
      if(&s->frame==b){
        shape = actor->createShape(*geometry, *mMaterial);
      }else{
        shape = actor->createShape(*geometry, *mMaterial, conv_Transformation2PxTrans(s->frame.Q));
      }
      CHECK(shape, "create shape failed!");
    }
    //actor = PxCreateDynamic(*mPhysics, OrsTrans2PxTrans(s->X), *geometry, *mMaterial, 1.f);
  }
  if(type != mlr::BT_static) {
    if(b->inertia && b->inertia->mass>0.) {
      PxRigidBodyExt::setMassAndUpdateInertia(*actor, b->inertia->mass);
    }else{
      PxRigidBodyExt::updateMassAndInertia(*actor, 1.f);
    }
    actor->setAngularDamping(0.75);
//    actor->setLinearVelocity(PxVec3(b->X.vel.x, b->X.vel.y, b->X.vel.z));
//    actor->setAngularVelocity(PxVec3(b->X.angvel.x, b->X.angvel.y, b->X.angvel.z));
  }
  gScene->addActor(*actor);
  actor->userData = b;

  actors(b->ID) = actor;
}

void PhysXInterface::pullFromPhysx(mlr::KinematicWorld *K, arr& vels) {
  if(!K) K=&world;
  if(&vels) vels.resize(K->frames.N, 2, 3).setZero();
  for(mlr::Frame *f : K->frames){
    if(s->actors.N <= f->ID) continue;
    PxRigidActor* a = s->actors(f->ID);
    if(!a) continue;
    PxTrans2OrsTrans(f->X, a->getGlobalPose());
    if(&vels && a->getType() == PxActorType::eRIGID_DYNAMIC) {
      PxRigidBody *px_body = (PxRigidBody*) a;
      vels(f->ID, 0, {}) = conv_PxVec3_arr(px_body->getLinearVelocity());
      vels(f->ID, 1, {}) = conv_PxVec3_arr(px_body->getAngularVelocity());
    }
#if 0
    if(a->getType() == PxActorType::eRIGID_DYNAMIC) {
      PxRigidBody *px_body = (PxRigidBody*) a;
      PxVec3 vel = px_body->getLinearVelocity();
      PxVec3 angvel = px_body->getAngularVelocity();
      mlr::Vector newvel(vel[0], vel[1], vel[2]);
      mlr::Vector newangvel(angvel[0], angvel[1], angvel[2]);
      mlr::Body *b = world.bodies(a_COUNT);
      b->force = b->mass * ((b->X.vel - newvel)/tau);
      b->torque = b->mass * ((b->X.angvel - newangvel)/tau);
      b->X.vel = newvel;
      b->X.angvel = newangvel;
    }
#endif
  }
  K->calc_Q_from_BodyFrames();
  K->calc_q_from_Q();
}

void PhysXInterface::pushToPhysx(mlr::KinematicWorld *K, mlr::KinematicWorld *Kt_1, double tau, bool onlyKinematic) {
  if(!K) K=&world;
  for(mlr::Frame *f : K->frames){
    if(s->actors.N <= f->ID) continue;
    PxRigidActor* a = s->actors(f->ID);
    if(!a) continue; //f is not an actor

    if(f->inertia && f->inertia->type!= s->actorTypes(f->ID)){
      LOG(0) <<"SWITCHING INERTIA TYPE! " <<f->name;
      s->actorTypes(f->ID) = f->inertia->type;
      if(f->inertia->type==mlr::BT_kinematic) ((PxRigidDynamic*)a)->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
      if(f->inertia->type==mlr::BT_dynamic){
        ((PxRigidDynamic*)a)->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, false);
        if(Kt_1){
          mlr::Vector v = (f->X.pos - Kt_1->frames(f->ID)->X.pos)/tau;
          ((PxRigidDynamic*)a)->setLinearVelocity(PxVec3(v.x, v.y, v.z));

          mlr::Vector w = (f->X.rot.getJacobian() * (f->X.rot.getArr4d() - Kt_1->frames(f->ID)->X.rot.getArr4d()))/tau;
          ((PxRigidDynamic*)a)->setAngularVelocity(PxVec3(w.x, w.y, w.z));

          LOG(0) <<"  switch velocity=" <<v <<"  angular=" <<w <<"  position=" <<f->X.pos;
        }else{
          LOG(-1) <<"I could not set velocities as K_{t-1} was not given";
        }
      }
    }

    bool isKinematic = (f->joint && !f->inertia) || (f->inertia && f->inertia->type==mlr::BT_kinematic);
    if(onlyKinematic && !isKinematic) continue;
    ((PxRigidDynamic*)a)->setKinematicTarget(conv_Transformation2PxTrans(f->X));
    //    cout <<"pushing " <<f->name <<" : " <<f->X <<endl;
  }
}

void PhysXInterface::ShutdownPhysX() {
//  s->mMaterial
//  s->plane
//  s->planeShape

  for(PxRigidActor* a: s->actors) if(a){
    s->gScene->removeActor(*a);
    a->release();
  }
  if(s->connection){
    s->connection->release();
    s->connection=NULL;
  }
  if(s->gScene){
    s->gScene->release();
    s->gScene = NULL;
  }
  if(s->gl){
    delete s->gl;
    s->gl=NULL;
  }

  mCooking->release();
  mPhysics->release();
//  mFoundation->release();
}

void DrawActor(PxRigidActor* actor, mlr::Frame *frame) {
  PxU32 nShapes = actor->getNbShapes();
  PxShape** shapes=new PxShape*[nShapes];
  //cout <<"#shapes=" <<nShapes;
  
  actor->getShapes(shapes, nShapes);
  while(nShapes--) {
    PxShape *shape = shapes[nShapes];

    // use the color of the first shape of the body for the entire body
    mlr::Shape *s = frame->shape;
    if(!s) s = frame->outLinks.elem(0)->shape;
    if(s) glColor(s->mesh().C);

    mlr::Transformation f;
    double mat[16];
    PxTrans2OrsTrans(f, PxShapeExt::getGlobalPose(*shape, *actor));
    glLoadMatrixd(f.getAffineMatrixGL(mat));
    //cout <<"drawing shape " <<body->name <<endl;
    switch(shape->getGeometryType()) {
      case PxGeometryType::eBOX: {
        PxBoxGeometry g;
        shape->getBoxGeometry(g);
        //glutSolidCube(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
        glDrawBox(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
      } break;
      case PxGeometryType::eSPHERE: {
        PxSphereGeometry g;
        shape->getSphereGeometry(g);
        glutSolidSphere(g.radius, 10, 10);
      } break;
      case PxGeometryType::eCAPSULE: {
        PxCapsuleGeometry g;
        shape->getCapsuleGeometry(g);
        glDrawCappedCylinder(g.radius, g.halfHeight*2);
      } break;
      case PxGeometryType::eCONVEXMESH: {
#if 1
        PxConvexMeshGeometry g;
        shape->getConvexMeshGeometry(g);
        floatA Vfloat((float*)g.convexMesh->getVertices(), 3*g.convexMesh->getNbVertices()); //reference
        mlr::Mesh mesh;
        copy(mesh.V,Vfloat);
        mesh.V.reshape(g.convexMesh->getNbVertices(),3);
        mesh.makeConvexHull();
        mesh.glDraw(NoOpenGL);
#else
        s->mesh.glDraw();
#endif
      } break;
      
      default:
        MLR_MSG("can't draw this type");
    }
  }
  delete [] shapes;
}

void PhysXInterface::glDraw(OpenGL&) {
  uint i=0;
  for(PxRigidActor* a: s->actors){
    if(a) DrawActor(a, world.frames(i));
    i++;
  }
}

void PhysXInterface::watch(bool pause, const char *txt){
  if(!s->gl){
    s->gl = new OpenGL("PHYSX direct");
    s->gl->add(glStandardScene, NULL);
    s->gl->add(*this);
    s->gl->camera.setDefault();
  }
  if(pause) s->gl->watch(txt);
  else s->gl->update(txt);
}

void PhysXInterface::addForce(mlr::Vector& force, mlr::Frame* b) {
  PxVec3 px_force = PxVec3(force.x, force.y, force.z);
  PxRigidBody *actor = (PxRigidBody*) (s->actors(b->ID)); // dynamic_cast fails for missing RTTI in physx
  actor->addForce(px_force);
}

void PhysXInterface::addForce(mlr::Vector& force, mlr::Frame* b, mlr::Vector& pos) {
  PxVec3 px_force = PxVec3(force.x, force.y, force.z);
  PxVec3 px_pos = PxVec3(pos.x, pos.y, pos.z);
  PxRigidBody *actor = (PxRigidBody*)(s->actors(b->ID));
  PxRigidBodyExt::addForceAtPos(*actor, px_force, px_pos);
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

#else //MLR_PHYSX

#include "kin_physx.h"
PhysXInterface::PhysXInterface(mlr::KinematicWorld& _world) : world(_world), s(NULL) { NICO }
PhysXInterface::~PhysXInterface() { NICO }
  
void PhysXInterface::step(double tau) { NICO }
void PhysXInterface::pushToPhysx() { NICO }
void PhysXInterface::pullFromPhysx(double tau) { NICO }
void PhysXInterface::setArticulatedBodiesKinematic() { NICO }
void PhysXInterface::ShutdownPhysX() { NICO }
void PhysXInterface::glDraw(OpenGL&) { NICO }
void PhysXInterface::addForce(mlr::Vector& force, mlr::Frame* b) { NICO }
void PhysXInterface::addForce(mlr::Vector& force, mlr::Frame* b, mlr::Vector& pos) { NICO }

void glPhysXInterface(void *classP) { NICO }
void bindOrsToPhysX(mlr::KinematicWorld& graph, OpenGL& gl, PhysXInterface& physx) { NICO }

#endif
/** @} */

RUN_ON_INIT_BEGIN(kin_physx)
mlr::Array<PxRigidActor*>::memMove=true;
mlr::Array<PxD6Joint*>::memMove=true;
mlr::Array<mlr::BodyType>::memMove=true;
RUN_ON_INIT_END(kin_physx)
