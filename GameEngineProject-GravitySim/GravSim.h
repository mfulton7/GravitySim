#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>

#include "OgreManualObject.h"
#include "btBulletDynamicsCommon.h"
#include "btHeightfieldTerrainShape.h"
#include "BaseApplication.h"



//this is where the environement and objects will be built
//along with the code to setup and affect gravity
class GravSim : public BaseApplication
{
public:
	//default ctor and destructor
	GravSim();
	virtual ~GravSim();
	float gravitymultiplier = 50;
	float speed = 100.f;
	bool isReseting = false;

protected:
	//create and destroy scene
	virtual void createScene();
	virtual void destroyScene();

	//frame stuff
	virtual void createFrameListener();
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	bool frameStarted(const Ogre::FrameEvent &evt);

	//cube stuff for testing physics
	//void CreateCube(const btVector3 &Position, btScalar Mass, const btVector3 &scale, char * name);
	void CreateCube(const btVector3 & Position, btScalar Mass, const btVector3 &scale);
	Ogre::ManualObject *createCubeMesh(Ogre::String name, Ogre::String matName);

	//check to see if gravity is turned on 
	bool GravD;
	bool GravU;
	bool GravL;
	bool GravR;

	//all items in the scene
	std::vector<btRigidBody*> physicsObjects;

private:
	//create terrain for testing
	void defineTerrain(long x, long y);
	void initBlendMaps(Ogre::Terrain* terrain);
	void configureTerrainDefaults(Ogre::Light* light);
	bool mTerrainsImported;
	Ogre::TerrainGroup* mTerrainGroup;
	Ogre::TerrainGlobalOptions* mTerrainGlobals;

	//physics sim
	void createBulletSim(void);

	OgreBites::Label* mInfoLabel;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* overlappingPairCache;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	btCollisionShape* groundShape;
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	//function burrowed from ablodget past project for testing
	void CreateBall(const btVector3 &Position, btScalar Mass, const btVector3 &scale);

	//get input from player
	bool getInput(const Ogre::FrameEvent& evt);
	
};