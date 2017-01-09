#include "GravSim.h"


//most of the following is boiler plate stuff that i took from the bullet tutorial on 
//the class website, this is just to set up a basic physics sim and an environemnt to test on
//my code to modify gravity and stuff will come later and will be marked as different

class MyMotionState : public btMotionState {
public:
	MyMotionState(const btTransform &initialpos, Ogre::SceneNode *node) {
		mVisibleobj = node;
		mPos1 = initialpos;
	}
	virtual ~MyMotionState() {    }
	void setNode(Ogre::SceneNode *node) {
		mVisibleobj = node;
	}
	virtual void getWorldTransform(btTransform &worldTrans) const {
		worldTrans = mPos1;
	}
	virtual void setWorldTransform(const btTransform &worldTrans) {
		if (NULL == mVisibleobj) return; // silently return before we set a node
		btQuaternion rot = worldTrans.getRotation();
		mVisibleobj->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
		btVector3 pos = worldTrans.getOrigin();
		// TODO **** XXX need to fix this up such that it renders properly since this doesnt know the scale of the node
		// also the getCube function returns a cube that isnt centered on Z
		mVisibleobj->setPosition(pos.x(), pos.y() + 5, pos.z() - 5);
	}
protected:
	Ogre::SceneNode *mVisibleobj;
	btTransform mPos1;
};

GravSim::GravSim() : mTerrainGroup(0), mTerrainGlobals(0), mInfoLabel(0)
{

}

GravSim::~GravSim()
{

}

//void GravSim::CreateCube(const btVector3 &Position, btScalar Mass, const btVector3 &scale, char * name)
void GravSim::CreateCube(const btVector3 &Position, btScalar Mass, const btVector3 &scale)
{
	// empty ogre vectors for the cubes size and position
	Ogre::Vector3 size = Ogre::Vector3::ZERO;
	Ogre::Vector3 pos = Ogre::Vector3::ZERO;
	Ogre::SceneNode *boxNode;
	Ogre::Entity *boxentity;
	// Convert the bullet physics vector to the ogre vector
	pos.x = Position.getX();
	pos.y = Position.getY();
	pos.z = Position.getZ();
	//boxentity = mSceneMgr->createEntity(name, "cube.mesh");
	boxentity = mSceneMgr->createEntity("cube.mesh");
	//boxentity->setScale(Vector3(scale.x,scale.y,scale.z));
	boxentity->setCastShadows(true);
	boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	boxNode->attachObject(boxentity);
	boxNode->scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));
	//boxNode->setScale(Vector3(0.1,0.1,0.1));
	Ogre::AxisAlignedBox boundingB = boxentity->getBoundingBox();
	//Ogre::AxisAlignedBox boundingB = boxNode->_getWorldAABB();
	boundingB.scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));
	size = boundingB.getSize()*0.95f;
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(Position);
	MyMotionState *MotionState = new MyMotionState(Transform, boxNode);
	//Give the rigid body half the size
	// of our cube and tell it to create a BoxShape (cube)
	btVector3 HalfExtents(size.x*0.5f, size.y*0.5f, size.z*0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(Mass, LocalInertia);
	btRigidBody *RigidBody = new btRigidBody(Mass, MotionState, Shape, LocalInertia);

	// Store a pointer to the Ogre Node so we can update it later
	RigidBody->setUserPointer((void *)(boxNode));

	// Add it to the physics world
	dynamicsWorld->addRigidBody(RigidBody);
	collisionShapes.push_back(Shape);

	//RigidBody->activate();

	//add it to list of physics objects
	physicsObjects.push_back(RigidBody);
}


//burrowed from ablodget
void GravSim::CreateBall(const btVector3 &Position, btScalar Mass, const btVector3 &scale){
	// empty ogre vectors for the cubes size and position
	Ogre::Vector3 size = Ogre::Vector3::ZERO;
	Ogre::Vector3 pos = Ogre::Vector3::ZERO;
	Ogre::SceneNode *ballNode;
	Ogre::Entity *ballEntity;
	// Convert the bullet physics vector to the ogre vector
	pos.x = Position.getX();
	pos.y = Position.getY();
	pos.z = Position.getZ();
	ballEntity = mSceneMgr->createEntity("sphere.mesh");

	ballEntity->setCastShadows(true);
	ballNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();

	//Give the sphere texture!
	ballEntity->setMaterialName("Examples/sphere");

	ballNode->attachObject(ballEntity);
	ballNode->scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));
	Ogre::AxisAlignedBox boundingB = ballEntity->getBoundingBox();

	boundingB.scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));
	size = boundingB.getSize()*0.95f;
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(Position);
	MyMotionState *MotionState = new MyMotionState(Transform, ballNode);
	//Give the rigid body half the size
	// of our cube and tell it to create a BoxShape (cube)
	btVector3 HalfExtents(size.x*0.5f, size.y*0.5f, size.z*0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(Mass, LocalInertia);
	btRigidBody *RigidBody = new btRigidBody(Mass, MotionState, Shape, LocalInertia);

	// Store a pointer to the Ogre Node so we can update it later
	RigidBody->setUserPointer((void *)(ballNode));

	// Add it to the physics world
	dynamicsWorld->addRigidBody(RigidBody);
	collisionShapes.push_back(Shape);

	//Get camera direction
	Ogre::Vector3 camDirection = mCamera->getDerivedDirection();
	//Convert the vector to bullet's so you can give velocity
	btVector3 initVelocity = btVector3(camDirection.x, camDirection.y, camDirection.z);
	//Define a speed
	
	//Give the bullet vector a speed
	initVelocity.normalize();
	initVelocity *= speed;

	//btVector3 initVelocity = btVector3(50, -50, 50);
	RigidBody->setLinearVelocity(initVelocity);

}

void GravSim::createBulletSim(void)
{
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	dispatcher = new   btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
	//dynamicsWorld->setGravity(btVector3(0, -100, 0));
	//set gravity to zero
	dynamicsWorld->setGravity(btVector3(0,-50,0));
	{
		///create a few basic rigid bodies
		// start with ground plane, 1500, 1500
		Ogre::Terrain * pTerrain = mTerrainGroup->getTerrain(0, 0);
		float* terrainHeightData = pTerrain->getHeightData();
		Ogre::Vector3 terrainPosition = pTerrain->getPosition();
		float * pDataConvert = new float[pTerrain->getSize() *pTerrain->getSize()];
		for (int i = 0; i<pTerrain->getSize(); i++)
			memcpy(
			pDataConvert + pTerrain->getSize() * i, // source
			terrainHeightData + pTerrain->getSize() * (pTerrain->getSize() - i - 1), // target
			sizeof(float)*(pTerrain->getSize()) // size
			);

		float metersBetweenVertices = pTerrain->getWorldSize() / (pTerrain->getSize() - 1); //edit: fixed 0 -> 1 on 2010-08-13
		btVector3 localScaling(metersBetweenVertices, 1, metersBetweenVertices);

		btHeightfieldTerrainShape* groundShape = new btHeightfieldTerrainShape(
			pTerrain->getSize(),
			pTerrain->getSize(),
			pDataConvert,
			1/*ignore*/,
			pTerrain->getMinHeight(),
			pTerrain->getMaxHeight(),
			1,
			PHY_FLOAT,
			true);

		groundShape->setUseDiamondSubdivision(true);
		groundShape->setLocalScaling(localScaling);

		btRigidBody * mGroundBody = new btRigidBody(0, new btDefaultMotionState(), groundShape);

		mGroundBody->getWorldTransform().setOrigin(
			btVector3(
			terrainPosition.x,
			terrainPosition.y + (pTerrain->getMaxHeight() - pTerrain->getMinHeight()) / 2,
			terrainPosition.z));

		mGroundBody->getWorldTransform().setRotation(
			btQuaternion(
			Ogre::Quaternion::IDENTITY.x,
			Ogre::Quaternion::IDENTITY.y,
			Ogre::Quaternion::IDENTITY.z,
			Ogre::Quaternion::IDENTITY.w));

		dynamicsWorld->addRigidBody(mGroundBody);
		collisionShapes.push_back(groundShape);

		//CreateCube(btVector3(2623, 500, 750), 1.0f, btVector3(0.3, 0.3, 0.3), "Cube0");
		//CreateCube(btVector3(2263, 150, 1200), 1.0f, btVector3(0.2, 0.2, 0.2), "Cube1");
		//CreateCube(btVector3(2053, 100, 1210), 1.0f, btVector3(0.2, 0.2, 0.2), "Cube2");
		
		//CreateCube(btVector3(2253, 200, 1210), 1.0f, btVector3(0.2, 0.2, 0.2), "Cube3");
		//CreateCube(btVector3(2453, 190, 1210), 1.0f, btVector3(0.2, 0.2, 0.2), "Cube4");
		//CreateCube(btVector3(2280, 300, 1110), 4.0f, btVector3(0.4, 0.4, 0.4), "Cube5");
		


		//CreateCube(btVector3(1963, 150, 1660),1.0f,btVector3(0.2,0.2,0.2),"Cube1");

	}


}

Ogre::ManualObject* GravSim::createCubeMesh(Ogre::String name, Ogre::String matName)
{
	Ogre::ManualObject* cube = new Ogre::ManualObject(name);

	cube->begin(matName);

	cube->position(0.5, -0.5, 1.0); cube->normal(0.408248, -0.816497, 0.408248); cube->textureCoord(1, 0);
	cube->position(-0.5, -0.5, 0.0); cube->normal(-0.408248, -0.816497, -0.408248); cube->textureCoord(0, 1);
	cube->position(0.5, -0.5, 0.0); cube->normal(0.666667, -0.333333, -0.666667); cube->textureCoord(1, 1);
	cube->position(-0.5, -0.5, 1.0); cube->normal(-0.666667, -0.333333, 0.666667); cube->textureCoord(0, 0);
	cube->position(0.5, 0.5, 1.0); cube->normal(0.666667, 0.333333, 0.666667); cube->textureCoord(1, 0);
	cube->position(-0.5, -0.5, 1.0); cube->normal(-0.666667, -0.333333, 0.666667); cube->textureCoord(0, 1);
	cube->position(0.5, -0.5, 1.0); cube->normal(0.408248, -0.816497, 0.408248); cube->textureCoord(1, 1);
	cube->position(-0.5, 0.5, 1.0); cube->normal(-0.408248, 0.816497, 0.408248); cube->textureCoord(0, 0);
	cube->position(-0.5, 0.5, 0.0); cube->normal(-0.666667, 0.333333, -0.666667); cube->textureCoord(0, 1);
	cube->position(-0.5, -0.5, 0.0); cube->normal(-0.408248, -0.816497, -0.408248); cube->textureCoord(1, 1);
	cube->position(-0.5, -0.5, 1.0); cube->normal(-0.666667, -0.333333, 0.666667); cube->textureCoord(1, 0);
	cube->position(0.5, -0.5, 0.0); cube->normal(0.666667, -0.333333, -0.666667); cube->textureCoord(0, 1);
	cube->position(0.5, 0.5, 0.0); cube->normal(0.408248, 0.816497, -0.408248); cube->textureCoord(1, 1);
	cube->position(0.5, -0.5, 1.0); cube->normal(0.408248, -0.816497, 0.408248); cube->textureCoord(0, 0);
	cube->position(0.5, -0.5, 0.0); cube->normal(0.666667, -0.333333, -0.666667); cube->textureCoord(1, 0);
	cube->position(-0.5, -0.5, 0.0); cube->normal(-0.408248, -0.816497, -0.408248); cube->textureCoord(0, 0);
	cube->position(-0.5, 0.5, 1.0); cube->normal(-0.408248, 0.816497, 0.408248); cube->textureCoord(1, 0);
	cube->position(0.5, 0.5, 0.0); cube->normal(0.408248, 0.816497, -0.408248); cube->textureCoord(0, 1);
	cube->position(-0.5, 0.5, 0.0); cube->normal(-0.666667, 0.333333, -0.666667); cube->textureCoord(1, 1);
	cube->position(0.5, 0.5, 1.0); cube->normal(0.666667, 0.333333, 0.666667); cube->textureCoord(0, 0);

	cube->triangle(0, 1, 2);      cube->triangle(3, 1, 0);
	cube->triangle(4, 5, 6);      cube->triangle(4, 7, 5);
	cube->triangle(8, 9, 10);      cube->triangle(10, 7, 8);
	cube->triangle(4, 11, 12);   cube->triangle(4, 13, 11);
	cube->triangle(14, 8, 12);   cube->triangle(14, 15, 8);
	cube->triangle(16, 17, 18);   cube->triangle(16, 19, 17);
	cube->end();

	return cube;
}

void GravSim::createScene()
{
	//////////////////////////////////////////////////
	GravU = false;
	GravD = false;
	GravL = false;
	GravR = false;
	/////////////////////////////////////////////////
	mCamera->setPosition(Ogre::Vector3(1863, 60, 1650));
	mCamera->lookAt(Ogre::Vector3(2263, 50, 1200));
	mCamera->setNearClipDistance(.1);

	bool infiniteClip =
		mRoot->getRenderSystem()->getCapabilities()->hasCapability(
		Ogre::RSC_INFINITE_FAR_PLANE);

	if (infiniteClip)
		mCamera->setFarClipDistance(0);
	else
		mCamera->setFarClipDistance(50000);

	mSceneMgr->setAmbientLight(Ogre::ColourValue(.2, .2, .2));

	Ogre::Vector3 lightDir(.55, -.3, .75);
	lightDir.normalise();

	Ogre::Light* light = mSceneMgr->createLight("TestLight");
	light->setType(Ogre::Light::LT_DIRECTIONAL);
	light->setDirection(lightDir);
	light->setDiffuseColour(Ogre::ColourValue::White);
	light->setSpecularColour(Ogre::ColourValue(.4, .4, .4));

	// Fog
	Ogre::ColourValue fadeColour(.9, .9, .9);
	mWindow->getViewport(0)->setBackgroundColour(fadeColour);

	//mSceneMgr->setFog(Ogre::FOG_EXP2, fadeColour, 0.002);

	// Terrain
	mTerrainGlobals = OGRE_NEW Ogre::TerrainGlobalOptions();

	mTerrainGroup = OGRE_NEW Ogre::TerrainGroup(
		mSceneMgr,
		Ogre::Terrain::ALIGN_X_Z,
		513, 12000.0);
	mTerrainGroup->setFilenameConvention(Ogre::String("terrain"), Ogre::String("dat"));
	mTerrainGroup->setOrigin(Ogre::Vector3::ZERO);

	configureTerrainDefaults(light);

	for (long x = 0; x <= 0; ++x)
		for (long y = 0; y <= 0; ++y)
			defineTerrain(x, y);

	mTerrainGroup->loadAllTerrains(true);

	if (mTerrainsImported)
	{
		Ogre::TerrainGroup::TerrainIterator ti = mTerrainGroup->getTerrainIterator();

		while (ti.hasMoreElements())
		{
			Ogre::Terrain* t = ti.getNext()->instance;
			initBlendMaps(t);
		}
	}

	mTerrainGroup->freeTemporaryResources();

	// Sky Techniques
	// mSceneMgr->setSkyBox(true, "Examples/SpaceSkyBox", 300, false);
	mSceneMgr->setSkyDome(true, "Examples/CloudySky", 5, 8);
	// Ogre::Plane plane;
	// plane.d = 1000;
	// plane.normal = Ogre::Vector3::NEGATIVE_UNIT_Y;

	// mSceneMgr->setSkyPlane(
	//   true, plane, "Examples/SpaceSkyPlane", 1500, 40, true, 1.5, 150, 150);
	createBulletSim();
	//isReseting = false;
}

void GravSim::createFrameListener()
{
	BaseApplication::createFrameListener();

	mInfoLabel = mTrayMgr->createLabel(OgreBites::TL_TOP, "TerrainInfo", "", 350);
}

void GravSim::destroyScene()
{
	OGRE_DELETE mTerrainGroup;
	OGRE_DELETE mTerrainGlobals;
}

bool GravSim::frameStarted(const Ogre::FrameEvent &evt)
{
	//if (GravON){
	//	for (int i = 0; i < physicsObjects.size(); ++i){
	//		//physicsObjects[i]->applyForce(btVector3(0, -100, 0), btVector3(0, 0, 0));
	//		
	//	}
	//}
	//	mKeyboard->capture();
	//	mMouse->capture();
	// update physics simulation
	//dynamicsWorld->stepSimulation(evt.timeSinceLastFrame,10);
	dynamicsWorld->stepSimulation(evt.timeSinceLastFrame);
	//add forces to all items in scene

	return true;
}

bool GravSim::frameRenderingQueued(const Ogre::FrameEvent& fe)
{

	bool ret = BaseApplication::frameRenderingQueued(fe);
	dynamicsWorld->stepSimulation(fe.timeSinceLastFrame);
	////////////////////////////////////////////////////
	//process input
	if (!getInput(fe))
		return false;

	////if (mTerrainGroup->isDerivedDataUpdateInProgress())
	//{
		mTrayMgr->moveWidgetToTray(mInfoLabel, OgreBites::TL_TOP, 0);
		mInfoLabel->show();

		/*if (mTerrainsImported)
			mInfoLabel->setCaption("Building terrain...");
		else
			mInfoLabel->setCaption("Updating textures...");*/
		if (GravU || GravD || GravL || GravR)
			mInfoLabel->setCaption("Gravity is on");
		else
			mInfoLabel->setCaption("Gravity is off");
//	}
	/*else
	{
		mTrayMgr->removeWidgetFromTray(mInfoLabel);
		mInfoLabel->hide();

		if (mTerrainsImported)
		{
			mTerrainGroup->saveAllTerrains(true);
			mTerrainsImported = false;
		}
	}*/

	return ret;
}

void getTerrainImage(bool flipX, bool flipY, Ogre::Image& img)
{
	img.load("terrain.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

	if (flipX)
		img.flipAroundY();
	if (flipY)
		img.flipAroundX();

}

void GravSim::defineTerrain(long x, long y)
{
	Ogre::String filename = mTerrainGroup->generateFilename(x, y);

	bool exists =
		Ogre::ResourceGroupManager::getSingleton().resourceExists(
		mTerrainGroup->getResourceGroup(),
		filename);

	if (exists)
		mTerrainGroup->defineTerrain(x, y);
	else
	{
		Ogre::Image img;
		getTerrainImage(x % 2 != 0, y % 2 != 0, img);
		mTerrainGroup->defineTerrain(x, y, &img);

		mTerrainsImported = true;
	}
}

void GravSim::initBlendMaps(Ogre::Terrain* terrain)
{
	Ogre::Real minHeight0 = 70;
	Ogre::Real fadeDist0 = 40;
	Ogre::Real minHeight1 = 70;
	Ogre::Real fadeDist1 = 15;

	Ogre::TerrainLayerBlendMap* blendMap0 = terrain->getLayerBlendMap(1);
	Ogre::TerrainLayerBlendMap* blendMap1 = terrain->getLayerBlendMap(2);

	float* pBlend0 = blendMap0->getBlendPointer();
	float* pBlend1 = blendMap1->getBlendPointer();

	for (Ogre::uint16 y = 0; y < terrain->getLayerBlendMapSize(); ++y)
	{
		for (Ogre::uint16 x = 0; x < terrain->getLayerBlendMapSize(); ++x)
		{
			Ogre::Real tx, ty;

			blendMap0->convertImageToTerrainSpace(x, y, &tx, &ty);
			Ogre::Real height = terrain->getHeightAtTerrainPosition(tx, ty);
			Ogre::Real val = (height - minHeight0) / fadeDist0;
			val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
			*pBlend0++ = val;

			val = (height - minHeight1) / fadeDist1;
			val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
			*pBlend1++ = val;
		}
	}

	blendMap0->dirty();
	blendMap1->dirty();
	blendMap0->update();
	blendMap1->update();

}

void GravSim::configureTerrainDefaults(Ogre::Light* light)
{
	mTerrainGlobals->setMaxPixelError(8);
	mTerrainGlobals->setCompositeMapDistance(3000);

	mTerrainGlobals->setLightMapDirection(light->getDerivedDirection());
	mTerrainGlobals->setCompositeMapAmbient(mSceneMgr->getAmbientLight());
	mTerrainGlobals->setCompositeMapDiffuse(light->getDiffuseColour());

	Ogre::Terrain::ImportData& importData = mTerrainGroup->getDefaultImportSettings();
	importData.terrainSize = 513;
	importData.worldSize = 12000.0;
	importData.inputScale = 600;
	importData.minBatchSize = 33;
	importData.maxBatchSize = 65;

	importData.layerList.resize(3);
	importData.layerList[0].worldSize = 100;
	importData.layerList[0].textureNames.push_back(
		"dirt_grayrocky_diffusespecular.dds");
	importData.layerList[0].textureNames.push_back(
		"dirt_grayrocky_normalheight.dds");
	importData.layerList[1].worldSize = 30;
	importData.layerList[1].textureNames.push_back(
		"grass_green-01_diffusespecular.dds");
	importData.layerList[1].textureNames.push_back(
		"grass_green-01_normalheight.dds");
	importData.layerList[2].worldSize = 200;
	importData.layerList[2].textureNames.push_back(
		"growth_weirdfungus-03_diffusespecular.dds");
	importData.layerList[2].textureNames.push_back(
		"growth_weirdfungus-03_normalheight.dds");

}
///////////////////////////////////////////////////////////////////////////////
bool GravSim::getInput(const Ogre::FrameEvent& evt){

	//turn on gravity -down
	if (mKeyboard->isKeyDown(OIS::KC_NUMPAD2))
	{
		//
		//if gravity is not enabled
		if (GravD == false){
			//turn on regular gravity
			dynamicsWorld->setGravity(btVector3(0, -gravitymultiplier, 0));
			//dynamicsWorld->getGravity()
			//test
			//CreateCube(btVector3(2000, 400, 1222), 2.0f, btVector3(0.8, 0.2, 0.3), "Plank1");
			//set flag so gravity is true
			GravD = true;
			GravU = false;
			GravL = false;
			GravR = false;
			//dynamicsWorld->setGravity(btVector3(0, -100, 0));
			//dynamicsWorld->
			//physicsObjects[4]->setGravity(btVector3(0, -100, 0));
			//for (int i = 0; i < physicsObjects.size(); ++i){
			//physicsObjects[i]->applyForce(btVector3(0, -100, 0), btVector3(0, 0, 0));
			//	physicsObjects[i]->setGravity(btVector3(0, -100, 0));
			//btRigidBody *RigidBody = static_cast<Ogre::Entity*>(mSceneMgr->getSceneNode("Cube0")->getAttachedObject("RigidBody"));
			//		->applyForce(btVector3(0, -100, 0));
			//}
			}
		//if gravity is already enabled set it back to null
		else{
			dynamicsWorld->setGravity(btVector3(0, -gravitymultiplier, 0));
			GravD = false;
		}
		//
	}

	//turn on gravity -up
	
	if (mKeyboard->isKeyDown(OIS::KC_NUMPAD8)){
		if (!GravU){
			dynamicsWorld->setGravity(btVector3(0, gravitymultiplier, 0));
			GravU = true;
			GravD = false;
			GravL = false;
			GravR = false;
		}
	else{
		dynamicsWorld->setGravity(btVector3(0, -gravitymultiplier, 0));
		GravU = false;
	}
	}

	//turn on gravity -left
	if (mKeyboard->isKeyDown(OIS::KC_NUMPAD4)){
		if (!GravL){
			dynamicsWorld->setGravity(btVector3(-gravitymultiplier, 0, 0));
			GravL = true;
			GravU = false;
			GravD = false;
			GravR = false;
		}
		else{
			dynamicsWorld->setGravity(btVector3(0, -gravitymultiplier, 0));
			GravL = false;
		}
	}

	//turn on gravity -right
	if (mKeyboard->isKeyDown(OIS::KC_NUMPAD6)){
		if (!GravR){
			dynamicsWorld->setGravity(btVector3(gravitymultiplier, 0, 0));
			GravR = true;
			GravU = false;
			GravL = false;
			GravD = false;
		}
		else{
			dynamicsWorld->setGravity(btVector3(0, -gravitymultiplier, 0));
			GravR = false;
		}
	}

	//set gravity in the direction the camera is looking
	if (mKeyboard->isKeyDown(OIS::KC_NUMPAD0)){
		//get vector the camera is looking at
		Ogre::Vector3 d	= mCamera->getDerivedDirection();
		//convert to btvector
		btVector3 d2;
	
		/*d2.setX(d.x);
		d2.setY(d.y);
		d2.setZ(d.z);*/

		d2.setX(d.x * gravitymultiplier);
		d2.setY(d.y * gravitymultiplier);
		d2.setZ(d.z * gravitymultiplier);

		//get point  a distance away from cam
		//variable for distance
		int dist = 100;
		Ogre::Vector3 p = mCamera->getDerivedPosition();

		//set gravity in direction
		dynamicsWorld->setGravity(d2);
		GravR = false;
		GravU = false;
		GravL = false;
		GravD = false;

	}
	//spawn a cube 
	if (mKeyboard->isKeyDown(OIS::KC_NUMPAD7)){
		//Get camera position
		Ogre::Vector3 camPosition = mCamera->getDerivedPosition();
		//Convert the vector to bullet's so you pass it to CreateBall
		
		//get where the camera is looking
		Ogre::Vector3 sight = mCamera->getDirection();
		//set position of spawned object to be x units in front of camera
		/*position.x = position.x * sight.x;
		position.y = position.y * sight.y;
		position.z = position.z * sight.z;*/
		float offset = 400;
		float tmpx = camPosition.x + (sight.x * offset);
		float tmpy = camPosition.y + (sight.y * offset);
		float tmpz = camPosition.z + (sight.z * offset);

		//btVector3 position = btVector3(camPosition.x, camPosition.y, camPosition.z);
		btVector3 position = btVector3(tmpx, tmpy, tmpz);
		CreateCube(position, 1.0f, btVector3(0.3, 0.3, 0.3));
	}

	//function to reset scene
	if (mKeyboard->isKeyDown(OIS::KC_NUMPAD9)){
		if (isReseting == false){
			isReseting = true;
			//destroyScene();
			//createScene();
		}
	}
	//derived from ablodget
	//throw a ball
	if (mKeyboard->isKeyDown(OIS::KC_RSHIFT))
	{
		//Get camera position
		Ogre::Vector3 camPosition = mCamera->getDerivedPosition();
		//Convert the vector to bullet's so you pass it to CreateBall
		btVector3 position = btVector3(camPosition.x, camPosition.y, camPosition.z);

		CreateBall(btVector3(position), 5.0f, btVector3(0.2, 0.2, 0.2));
		//break;
	}
	

	return true;
}
///////////////////////////////////////////////////////////////////////////////

#if Ogre_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
	INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
#else
	int main(int argc, char *argv[])
#endif
	{
		// Create application object
		GravSim app;

		try {
			app.go();
		}
		catch (Ogre::Exception& e) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
			MessageBox(NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
			std::cerr << "An exception has occured: " <<
				e.getFullDescription().c_str() << std::endl;
#endif
		}

		return 0;
	}

#ifdef __cplusplus
}
#endif