#pragma once

#include "PhysicsEngine.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	///Plane class
	class Plane : public StaticActor
	{
	public:
		//A plane with default paramters: XZ plane centred at (0,0,0)
		Plane(PxVec3 normal=PxVec3(0.f, 1.f, 0.f), PxReal distance=0.f) 
			: StaticActor(PxTransformFromPlaneEquation(PxPlane(normal, distance)))
		{
			CreateShape(PxPlaneGeometry());
		}
	};

	///Sphere class
	class Sphere : public DynamicActor
	{
	public:
		//a sphere with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m
		// - denisty: 1kg/m^3
		Sphere(const PxTransform& pose=PxTransform(PxIdentity), PxReal radius=1.f, PxReal density=1.f) 
			: DynamicActor(pose)
		{ 
			CreateShape(PxSphereGeometry(radius), density);
		}
	};

	class Swing : public DynamicActor
	{
	public:
		Swing(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(6.5f, 0.5f, 0.5f), PxReal density = 1.0f)
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);

			GetShape(0)->setLocalPose(PxTransform(PxVec3(0.f, 3.5f, 5.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			GetShape(1)->setLocalPose(PxTransform(PxVec3(0.f, 3.5f, -5.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			GetShape(2)->setLocalPose(PxTransform(PxVec3(0.f, 9.5f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));
		}
	};

	class Stadium : public StaticActor
	{
	public:
		Stadium(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(15.5f, 3.5f, 200.5f), PxReal density = 1.0f)
			: StaticActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(15.5f, 15.5f, 200.5f), density);
			CreateShape(PxBoxGeometry(15.5f, 15.5f, 200.5f), density);

			GetShape(0)->setLocalPose(PxTransform(PxVec3(100.0f, .5f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			GetShape(1)->setLocalPose(PxTransform(PxVec3(-150.0f, .5f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			GetShape(2)->setLocalPose(PxTransform(PxVec3(.0f, .5f, 100.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));
			GetShape(3)->setLocalPose(PxTransform(PxVec3(0.0f, .5f, -150.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));


		}
	};
	class GoalShape : public DynamicActor
	{
	public:
		GoalShape(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(6.5f, 0.5f, 0.5f), PxReal density = 1.0f)
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(13.f, .1f,11.f), density);

			GetShape(0)->setLocalPose(PxTransform(PxVec3(.0f, 6.5f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			GetShape(1)->setLocalPose(PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			GetShape(2)->setLocalPose(PxTransform(PxVec3(0.f, 12.5f, 6.5f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));
			GetShape(3)->setLocalPose(PxTransform(PxVec3(0.f, 12.5f, -6.5f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));
			GetShape(4)->setLocalPose(PxTransform(PxVec3(0.f, 18.5f, -12.5f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			GetShape(5)->setLocalPose(PxTransform(PxVec3(0.f, 18.5f, 12.5f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			GetShape(6)->setLocalPose(PxTransform(PxVec3(0.f, 27.f, 0), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));


		}
	};

	class Catapult : public StaticActor {
	public:
		Catapult(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = 1.0f)
			: StaticActor(pose) 
		{
			CreateShape(PxBoxGeometry(3.5f,1.f,1.f), density);
			CreateShape(PxBoxGeometry(3.5f, 1.f, 1.f), density);
			CreateShape(PxBoxGeometry(3.5f, 1.f, 1.f), density);
			CreateShape(PxBoxGeometry(3.5f, 1.f, 1.f), density);

			GetShape(0)->setLocalPose(PxTransform(PxVec3(0.0f, 1.0f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));
			GetShape(1)->setLocalPose(PxTransform(PxVec3(8.0f, 1.0f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));
			GetShape(2)->setLocalPose(PxTransform(PxVec3(8.0f, 3.0f, 0.f), PxQuat(PxPi / 2, PxVec3(1.f, 0.f, 0.f))));
			GetShape(3)->setLocalPose(PxTransform(PxVec3(0.0f, 3.0f, 0.f), PxQuat(PxPi / 2, PxVec3(1.f, 0.f, 0.f))));
		}
	};

	class CatapultArm : public DynamicActor {
	public:
		CatapultArm(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = 1.0f)
			: DynamicActor(pose) {
			CreateShape(PxBoxGeometry(PxVec3(7.5f, .5f, .5f)), 0.25f);//Main arm
			CreateShape(PxBoxGeometry(PxVec3(1.5f, 1.6f, .1f)), 0.25f);//Base of Box
			CreateShape(PxBoxGeometry(PxVec3(1.5f, .7, .1f)), 0.25f);//Back of Box
			CreateShape(PxBoxGeometry(PxVec3(.1f, .7, 1.5f)), 0.25f);//right of Box
			CreateShape(PxBoxGeometry(PxVec3(.1f, .7, 1.5f)), 0.25f);//left of Box


			GetShape(1)->setLocalPose(PxTransform(PxVec3(6.5f, .5f, .0f), PxQuat(PxPi / 2, PxVec3(1.f, 0.f, 0.f))));
			GetShape(2)->setLocalPose(PxTransform(PxVec3(8.f, 1.f, .0f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));

			GetShape(3)->setLocalPose(PxTransform(PxVec3(6.5f, 1.f, 1.5f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));
			GetShape(4)->setLocalPose(PxTransform(PxVec3(6.5f, 1.f, -1.5f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));
		}

	};
	class Cat {
	public:
		CatapultArm* catsarm;
		Catapult* cats;
		Cat(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f)) {

			catsarm = new CatapultArm(PxTransform(PxVec3(0.f, 0.f, 0.f)));
			cats = new Catapult(PxTransform(PxVec3(0.f, 0.f, 0.f)));


		}

	};
	class Trebuchet : public DynamicActor
	{
	public:
		Trebuchet(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = 2.0f)
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(PxVec3(3.5f,.5f,.5f)), density);//left
			CreateShape(PxBoxGeometry(PxVec3(3.5f,.5f, .5f)), density);//right
			CreateShape(PxBoxGeometry(PxVec3(3.5f, .5f, .5f)), density);//middle
			CreateShape(PxBoxGeometry(PxVec3(3.5f, .5f, .5f)), density);//up left
			CreateShape(PxBoxGeometry(PxVec3(3.5f, .5f, 0.5f)), density);//up right
			CreateShape(PxBoxGeometry(PxVec3(0.45f, .45f, 0.5f)), density);//middle left
			CreateShape(PxBoxGeometry(PxVec3(0.45f, .45f, 0.5f)), density);//middle right


			//0 = left 1 = right 2 = middle 3 = up left 4 = up right = 5 = middle left 6 = middle left
			GetShape(0)->setLocalPose(PxTransform(PxVec3(0.0f, 1.5f, 0.f)));
			GetShape(1)->setLocalPose(PxTransform(PxVec3(0.0f, 1.5f, 4.f)));
			GetShape(2)->setLocalPose(PxTransform(PxVec3(.0f, 1.5f, 2.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));
			GetShape(3)->setLocalPose(PxTransform(PxVec3(0.0f, 5.5f, 0.0f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			GetShape(4)->setLocalPose(PxTransform(PxVec3(.0f, 5.5f, 4.0f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			GetShape(5)->setLocalPose(PxTransform(PxVec3(0.0f, 8.5f, 3.0f)));
			GetShape(6)->setLocalPose(PxTransform(PxVec3(0.0f, 8.5f, 1.f)));

			
		}
	};

	class TrebuchetArm : public DynamicActor
	{
	public:
		TrebuchetArm(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(PxVec3(8.5f, .5f, .5f)), 0.25f);//Main arm
			CreateShape(PxBoxGeometry(PxVec3(1.5f, 1.6f, .1f)), 0.25f);//Base of Box
			CreateShape(PxBoxGeometry(PxVec3(1.5f, .7, .1f)), 0.25f);//Back of Box
			
			GetShape(1)->setLocalPose(PxTransform(PxVec3(7.5f, .5f, .0f), PxQuat(PxPi / 2, PxVec3(1.f, 0.f, 0.f))));
			GetShape(2)->setLocalPose(PxTransform(PxVec3(9.f, 1.f, .0f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));
			

		}
	};

	class FullTreb {
	public:

		TrebuchetArm* Fullarm;
		Trebuchet* Mainframe;
		Sphere* Ball;
		FullTreb(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f))
		{
			Fullarm = new TrebuchetArm(PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxPi / 2, PxVec3(1.f, 0.f, 0.f))));
			Mainframe = new Trebuchet(PxTransform(PxVec3(-20.f, 1.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));

			//counterJoint = new RevoluteJoint(trebarm, PxTransform(PxVec3(-8.0f, -1.0f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.0f, 0.f))), counterWeight, PxTransform(PxVec3(0.0f, .5f, 0.f)));
			//counterJoint->SetLimits(-PxPi / 2, PxPi / 4);

			//RevoluteJoint joint(Fullarm, PxTransform(PxVec3(-4.5f, 0.5f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, -1.0f, 0.f))), Mainframe, PxTransform(PxVec3(0.0f, 8.5f, 2.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))));
		}
	};

	///Box class
	class Box : public DynamicActor
	{
	public:
		//a Box with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m x 1m x 1m
		// - denisty: 1kg/m^3
		Box(const PxTransform& pose=PxTransform(PxIdentity), PxVec3 dimensions=PxVec3(.5f,.5f,.5f), PxReal density=1.f) 
			: DynamicActor(pose)
		{ 
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	///Box class
	class Base : public DynamicActor
	{
	public:
		//a Box with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m x 1m x 1m
		// - denisty: 1kg/m^3
		Base(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.1f, .1f, .1f), PxReal density = 100.f)
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	class Boxx : public StaticActor
	{
	public:
		//a Box with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m x 1m x 1m
		// - denisty: 1kg/m^3
		PxReal* increase;

		Boxx(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.5f, .5f, .5f), PxReal density = 1.0f)
			: StaticActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	class Capsule : public DynamicActor
	{
	public:
		Capsule(const PxTransform& pose=PxTransform(PxIdentity), PxVec2 dimensions=PxVec2(1.f,1.f), PxReal density=1.f) 
			: DynamicActor(pose)
		{
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
		}
	};

	
	///The ConvexMesh class
	class ConvexMesh : public DynamicActor
	{
	public:
		//constructor
		ConvexMesh(const std::vector<PxVec3>& verts, const PxTransform& pose=PxTransform(PxIdentity), PxReal density=1.f)
			: DynamicActor(pose)
		{
			PxConvexMeshDesc mesh_desc;
			mesh_desc.points.count = (PxU32)verts.size();
			mesh_desc.points.stride = sizeof(PxVec3);
			mesh_desc.points.data = &verts.front();
			mesh_desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
			mesh_desc.vertexLimit = 256;

			CreateShape(PxConvexMeshGeometry(CookMesh(mesh_desc)), density);
		}

		//mesh cooking (preparation)
		PxConvexMesh* CookMesh(const PxConvexMeshDesc& mesh_desc)
		{
			PxDefaultMemoryOutputStream stream;

			if(!GetCooking()->cookConvexMesh(mesh_desc, stream))
				throw new Exception("ConvexMesh::CookMesh, cooking failed.");

			PxDefaultMemoryInputData input(stream.getData(), stream.getSize());

			return GetPhysics()->createConvexMesh(input);
		}
	};

	///The TriangleMesh class
	class TriangleMesh : public StaticActor
	{
	public:
		//constructor
		TriangleMesh(const std::vector<PxVec3>& verts, const std::vector<PxU32>& trigs, const PxTransform& pose=PxTransform(PxIdentity))
			: StaticActor(pose)
		{
			PxTriangleMeshDesc mesh_desc;
			mesh_desc.points.count = (PxU32)verts.size();
			mesh_desc.points.stride = sizeof(PxVec3);
			mesh_desc.points.data = &verts.front();
			mesh_desc.triangles.count = (PxU32)trigs.size();
			mesh_desc.triangles.stride = 3*sizeof(PxU32);
			mesh_desc.triangles.data = &trigs.front();

			CreateShape(PxTriangleMeshGeometry(CookMesh(mesh_desc)));
		}

		//mesh cooking (preparation)
		PxTriangleMesh* CookMesh(const PxTriangleMeshDesc& mesh_desc)
		{
			PxDefaultMemoryOutputStream stream;

			if(!GetCooking()->cookTriangleMesh(mesh_desc, stream))
				throw new Exception("TriangleMesh::CookMesh, cooking failed.");

			PxDefaultMemoryInputData input(stream.getData(), stream.getSize());

			return GetPhysics()->createTriangleMesh(input);
		}
	};

	class FixedJoint : public Joint
	{
	public:
		FixedJoint(Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1) {
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0->Get();
			

			joint = (PxJoint*)PxFixedJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1->Get(), localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
		//	joint->setBreakForce(100, 100);
		}

	};

	//Distance joint with the springs switched on
	class DistanceJoint : public Joint
	{
	public:
		DistanceJoint(Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
		{
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0->Get();

			joint = (PxJoint*)PxDistanceJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1->Get(), localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
			((PxDistanceJoint*)joint)->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
			Damping(1.f);
			Stiffness(1.f);
		}

		void Stiffness(PxReal value)
		{
			((PxDistanceJoint*)joint)->setStiffness(value);
		}

		PxReal Stiffness()
		{
			return ((PxDistanceJoint*)joint)->getStiffness();		
		}

		void Damping(PxReal value)
		{
			((PxDistanceJoint*)joint)->setDamping(value);
		}

		PxReal Damping()
		{
			return ((PxDistanceJoint*)joint)->getDamping();
		}
	};

	///Revolute Joint
	class RevoluteJoint : public Joint
	{
	public:
		RevoluteJoint(Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
		{
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0->Get();

			joint = PxRevoluteJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1->Get(), localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION,true);
		}

		void DriveVelocity(PxReal value)
		{
			//wake up the attached actors
			PxRigidDynamic *actor_0, *actor_1;
			((PxRevoluteJoint*)joint)->getActors((PxRigidActor*&)actor_0, (PxRigidActor*&)actor_1);
			if (actor_0)
			{
				if (actor_0->isSleeping())
					actor_0->wakeUp();
			}
			if (actor_1)
			{
				if (actor_1->isSleeping())
					actor_1->wakeUp();
			}
			((PxRevoluteJoint*)joint)->setDriveVelocity(value);
			((PxRevoluteJoint*)joint)->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
		}

		PxReal DriveVelocity()
		{
			return ((PxRevoluteJoint*)joint)->getDriveVelocity();
		}

		void SetLimits(PxReal lower, PxReal upper)
		{
			((PxRevoluteJoint*)joint)->setLimit(PxJointAngularLimitPair(lower, upper));
			((PxRevoluteJoint*)joint)->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
		}
	};
}