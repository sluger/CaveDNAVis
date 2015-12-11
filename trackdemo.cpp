/*!
 *  sluger - CaveDNAVis
 *  adapted code from
 *  Adrian Haffegee, June 2005
 *  simple application using CAVESceneManager
 */

#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGMultiDisplayWindow.h>
// needed for loading files
#include <OpenSG/OSGSceneFileHandler.h>

// CAVESceneManager includes:
#include <OSGCSM/OSGCAVESceneManager.h>
#include <OSGCSM/OSGCAVEConfig.h>

// some tracking bits - remove these later if move tracking elsewhere
#include <trackdAPI_CC.h>

// headers from application utilities library
#include <OSGCSM/appctrl.h>

#include <GL/freeglut.h>

// CUSTOM LIBS
#include <OpenSG/OSGSceneFileHandler.h>
#include <OpenSG/OSGTextureBackground.h>
#include <OpenSG/OSGGradientBackground.h>
#include <OpenSG/OSGPointLight.h>
#include <OpenSG/OSGSpotLight.h>
#include <ctime>
#include <OpenSG/OSGSkyBackground.h>
#include <OpenSG/OSGOSGSceneFileType.h>
#include <OpenSG/OSGTextureBackground.h>
#include <OpenSG/OSGComponentTransform.h>
#include <OpenSG/OSGMaterialGroup.h>
#include <OpenSG/OSGImage.h>
#include <OpenSG/OSGSimpleTexturedMaterial.h>
#include <OpenSG/OSGLine.h>
#include <OpenSG/OSGIntersectAction.h>

#include <boost/bind.hpp>
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

OSG_USING_NAMESPACE
using namespace OSGCSM;

// The CAVESceneManager to manage the cave applications
CAVESceneManager *mgr;
NodeRefPtr scene = NULL;

// various control flags
bool followHead = true;

//tracking vars
TrackerReader* tracker = NULL;
ControllerReader* controller = NULL;
float lastLR=0, lastUD=0;
void InitTracker(CAVEConfig &cfg);
void CheckTracker();
#define MAXBUTTONS 8
int buttonValues[MAXBUTTONS];

// forward defines
int setupGLUT( int *argc, char *argv[] );

// ================= FORWARD DECLS =================
// forward declaration so we have debug info about dna models
void printBasePairHashMap ();

// ================= CUSTOM GLOBALS =================
// dna sequence string
const std::string dnaStr = "TGCATGCTAGCTAGCTAGTTTTTTCTGACTGACTAAAAAACA";

// sun light
NodeRecPtr sun;
PointLightRecPtr sunLight;

// loaded basepair models
NodeRecPtr at;
NodeRecPtr ta;
NodeRecPtr cg;
NodeRecPtr gc;

// dna sequence starting pointer
NodeRecPtr dna;

// transformation for dna sequence
NodeRecPtr dnaTrans;

// last basepair in dna sequence
NodeRecPtr dock;

// tool
NodeRecPtr toolTrans;

// material
MaterialGroupRecPtr mg;

// possible basepairs
enum BasePairType {AT = 0, TA = 1, CG = 2, GC = 3};

// store basepairs in hash map
boost::unordered_map<NodeRecPtr,BasePairType> basePairPtrs;

// translation of basepairs
float rot = 12;
float x = -35;
float y = 105;
float z = 45;

// TOOL
float toolLength = 100;
ComponentTransformRecPtr toolCT;
Vec3f toolChg = Vec3f(0,0,0);

ComponentTransformRecPtr sunCT;
SimpleMaterialRecPtr sunMat;

// start scene coordinates
float sceneOriginX = 0;
float sceneOriginY = 0;
float sceneOriginZ = 0;

float toolOriginX = 0;
float toolOriginY = 100;
float toolOriginZ = -toolLength;

float dnaOriginX = -100;
float dnaOriginY = -100;
float dnaOriginZ = 0;

Color3f red = Color3f(1,0,0);
Color3f yellow = Color3f(1,1,0);
Color3f green = Color3f(0,1,0);
Color3f blue = Color3f(0,0,1);

// movement
Vec3f userPos;

// ================= CUSTOM METHODS =================

/*// SKYBOX:
SkyBackgroundTransitPtr getSkyBackground() {

 OSG::TextureObjChunkRecPtr back_texture = getTexture("opensg/textures/back.png");
 OSG::TextureObjChunkRecPtr left_texture = getTexture("opensg/textures/left.png");
 OSG::TextureObjChunkRecPtr right_texture = getTexture("opensg/textures/right.png");
 OSG::TextureObjChunkRecPtr front_texture = getTexture("opensg/textures/front.png");
 OSG::TextureObjChunkRecPtr top_texture = getTexture("opensg/textures/top.png");
 OSG::TextureObjChunkRecPtr bot_texture = getTexture("opensg/textures/bot.png");

 OSG::SkyBackgroundRecPtr skybg = OSG::SkyBackground::create();
 skybg->setBackTexture(back_texture);
 skybg->setLeftTexture(left_texture);
 skybg->setRightTexture(right_texture);
 skybg->setFrontTexture(front_texture);
 skybg->setTopTexture(top_texture);
 skybg->setBottomTexture(bot_texture);

 return SkyBackgroundTransitPtr(skybg);
}*/

// MATERIAL: mat
MaterialGroupRecPtr createMaterial (Color3f diffuse, Color3f ambient, Color3f specular, float transparency) {
	SimpleMaterialRecPtr m = SimpleMaterial::create();
	m->setDiffuse(diffuse);
	m->setAmbient(ambient);
	m->setSpecular(specular);
	m->setEmission(Color3f(0.5f, 0.5f, 0.5f));
	m->setTransparency(transparency);
	MaterialGroupRecPtr mg = MaterialGroup::create();
	mg->setMaterial(m);

	return mg;
}

// BASE PAIR: (trans -> basepair wrl)+
NodeRecPtr createBasePair (NodeRecPtr cur, NodeRecPtr bp, BasePairType t, Vec3f trans, Vec3f axis, int angle=0) {
	// transform node
	ComponentTransformRecPtr bpCT = ComponentTransform::create();
	bpCT->setTranslation(trans);
	bpCT->setRotation(Quaternion(axis,osgDegree2Rad(angle)));

	NodeRecPtr bpTrans = makeNodeFor(bpCT);

	//Color3f red = Color3f(1,0,0);
	//Color3f yellow = Color3f(1,1,0);
	//Color3f green = Color3f(0,1,0);
	//Color3f blue = Color3f(0,0,1);
	//float transparency = 0;

	//NodeRecPtr bpMat = Node::create();
	//MaterialGroupRecPtr mg;

	//// material node, created by enum type
	//switch (t) {
	//case 0:
	//	mg = createMaterial(red,red,Color3f(1,1,1),transparency);
	//	break;
	//case 1:
	//	mg = createMaterial(yellow,yellow,Color3f(1,1,1),transparency);
	//	break;
	//case 2:
	//	mg = createMaterial(green,green,Color3f(1,1,1),transparency);
	//	break;
	//case 3:
	//	mg = createMaterial(blue,blue,Color3f(1,1,1),transparency);
	//	break;
	//	default:
	//		std::cerr << "Base Pair not included!" << std::endl;
	//}

	//bpMat->setCore(mg); // set material core
	//cur->addChild(bpMat);
	cur->addChild(bpTrans);

	bpTrans->addChild(bp);
	//bpMat->addChild(bpTrans);

	// remember basepair for object
	basePairPtrs[bp] = t;

	return bp;
}

// MODELS: wrl files loaded from filesystem
// TODO: change path for vrc to local
void loadModels () {
	std::cout << "Reading at model..." << std::endl;
	at = SceneFileHandler::the()->read("opensg/models/at.wrl");
	// TODO: uncomment
	std::cout << "Reading ta model..." << std::endl;
	ta = SceneFileHandler::the()->read("opensg/models/ta.wrl");
	std::cout << "Reading cg model..." << std::endl;
	cg = SceneFileHandler::the()->read("opensg/models/cg.wrl");
	std::cout << "Reading gc model..." << std::endl;
	gc = SceneFileHandler::the()->read("opensg/models/gc.wrl");
}

// SPHERE: mat -> trans -> geo
// LIGHT:
NodeRecPtr setupLightSphere () {
	// light sphere geo
	GeometryRecPtr sunGeo = makeSphereGeo(6, 7);
	sun = Node::create();
	sun->setCore(sunGeo);

	// light sphere trans
	sunCT = ComponentTransform::create();
	// initial transformation
	sunCT->setTranslation(Vec3f(0,0,-toolLength/2));

	NodeRecPtr sunTrans = Node::create();
	sunTrans->setCore(sunCT);
	sunTrans->addChild(sun);


	// light sphere material
	sunMat = SimpleMaterial::create();
	sunMat->setDiffuse(Color3f(1,1,1));
	sunMat->setAmbient(Color3f(1, 1, 1));
	sunMat->setSpecular(Color3f(1,1,1));

	MaterialGroupRecPtr sunMgCore = MaterialGroup::create();
	sunMgCore->setMaterial(sunMat);

	NodeRecPtr sunMg = Node::create();
	sunMg->setCore(sunMgCore);
	sunMg->addChild(sunTrans);

	// point light
	sunLight = PointLight::create();
	//sunLight->setAttenuation(1,0,2);

	// point light color information
	sunLight->setDiffuse(Color4f(1,1,1,1));
	sunLight->setAmbient(Color4f(0.2f,0.2f,0.2f,1));
	sunLight->setSpecular(Color4f(1,1,1,1));

	// point light beacon
	sunLight->setBeacon(sun);

	return sunMg;
}

// CYLINDER: mat -> trans -> geo
NodeRecPtr setupCylinder () {
	float transparency = 0;
	Color3f gray = Color3f(0.5,0.5,0.5);

	// mat
	NodeRecPtr cylinderMat = Node::create();
	MaterialGroupRecPtr mg = createMaterial(gray,gray,Color3f(1,1,1),transparency);
	cylinderMat->setCore(mg); // set material core

	// trans
	ComponentTransformRecPtr cylinderCT = ComponentTransform::create();
	// initial transformation
	//cylinderCT->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(90)));
	//cylinderCT->setTranslation(Vec3f(0,0,0));

	//cylinderTrans->setScale(Vec3f(2,2,2));
	NodeRecPtr cylinderTrans = Node::create();
	cylinderTrans->setCore(cylinderCT);
	cylinderMat->addChild(cylinderTrans);

	// geo
	GeometryRecPtr cylinderGeo = makeCylinderGeo(toolLength,1,5,1,1,1);
	NodeRecPtr cylinder = Node::create();
	cylinder->setCore(cylinderGeo);
	cylinderTrans->addChild(cylinder);

	return cylinderMat;
}

// TOOL: trans -> group -> (lightsphere & cylinder)
void createTool() {
	toolCT = ComponentTransform::create();
	toolCT->setTranslation(Vec3f(toolOriginX,toolOriginY,toolOriginZ));
	toolCT->setCenter(Vec3f(0,0,toolLength/2));
	//toolCT->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(90)));
	//toolTrans->setScale(Vec3f(2,2,2));
	toolTrans = Node::create();
	toolTrans->setCore(toolCT);

	// combine both, cylinder and light source
	NodeRecPtr combinedGrp = Node::create();
	combinedGrp->setCore(Group::create());

	toolTrans->addChild(combinedGrp);

	// add cylinder and light
	combinedGrp->addChild(setupLightSphere());
	//combinedGrp->addChild(setupCylinder());
}

// DNA: trans -> group -> dna
void createDNA () {
	// load custom models
	loadModels();

	// create dna hierarchically
	dna = Node::create();
	dna->setCore(Group::create());
	NodeRecPtr cur = dna;

	for (int i = 0; i < dnaStr.length(); i++) {
		if (dnaStr[i] == 'A') {
		  cur = createBasePair(cur, cloneTree(at), BasePairType(0), Vec3f(x,y,z),Vec3f(0,1,0),rot);
		} else if (dnaStr[i] == 'T') {
		  cur = createBasePair(cur, cloneTree(ta), BasePairType(1), Vec3f(x,y,z),Vec3f(0,1,0),rot);
		} else if (dnaStr[i] == 'C') {
		  cur = createBasePair(cur, cloneTree(cg), BasePairType(2), Vec3f(x,y,z),Vec3f(0,1,0),rot);
		} else if (dnaStr[i] == 'G') {
		  cur = createBasePair(cur, cloneTree(gc), BasePairType(3), Vec3f(x,y,z),Vec3f(0,1,0),rot);
		} else {
			std::cerr << "Character not a valid nucletoide!" << std::endl;
		}
		//x += addX; y += addY; z+=addZ; rot += addRot;
	}

	// last bp added
	dock = cur;

	// print hashmap
	printBasePairHashMap();

	// rotate dna
	ComponentTransformRecPtr dnaCT = ComponentTransform::create();
	// initial translation
	dnaCT->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(-90)));
	dnaCT->setTranslation(Vec3f(dnaOriginX,dnaOriginY,dnaOriginZ));
	dnaTrans = Node::create();
	dnaTrans->setCore(dnaCT);
	dnaTrans->addChild(dna);
}

void collissionDetection () {
	// check if player is falling
	 Line ray = Line(Pnt3f(sun->getToWorld()[3][0], sun->getToWorld()[3][1], sun->getToWorld()[3][2]),Vec3f(0, 0, -1));
	 IntersectActionRefPtr iAct = IntersectAction::create();
	 iAct->setLine(ray);
	 iAct->apply(scene);

	 NodeRecPtr n;
    //see if anything was hit
    if (iAct->didHit()){		 // get the hit point
		Pnt3f p = iAct->getHitPoint();
		std::cout << "HIT!!!!!! : " << p[0] << " " << p[1] << " " << p[2] << std::endl;

		//and the node that was hit
		n = iAct->getHitObject();
		if (n != NULL && n->getParent() != NULL && n->getParent()->getParent() != NULL && n->getParent()->getParent()->getParent()) {

			std::cout << "======================================================" << std::endl;
			std::cout << "n " << n << std::endl <<
				"n->getParent() " << n->getParent() << std::endl <<
				"n->getCore() " << n->getCore() << std::endl <<
				"n->getParent()->getParent() " << n->getParent()->getParent() << std::endl <<
				"n->getParent()->getParent()->getParent() " << n->getParent()->getParent()->getParent() << std::endl <<
				"n->getParent()->getParent()->getParent()->getId() " << n->getParent()->getParent()->getParent()->getId() << std::endl <<
				"n->getParent()->getParent()->getParent()->getToWorld() " << std::endl << n->getParent()->getParent()->getParent()->getToWorld() << std::endl <<
				"n->getParent()->getParent()->getParent()->getGroupId() " << n->getParent()->getParent()->getParent()->getGroupId() << std::endl;
				//std::cout << "BP: " << basePairPtrs.at((NodeRecPtr)n) << std::endl;
			std::cout << "======================================================" << std::endl;
			//std::cout << n->getParent()->getParent()->getParent() << "==" << dock << "?" << std::endl;


			if (basePairPtrs.count(NodeRecPtr(n->getParent()->getParent()->getParent()))) {
				BasePairType bp = basePairPtrs.at((NodeRecPtr)n->getParent()->getParent()->getParent());
				Color3f col;
				switch(bp) {
				case 0: col = red; std::cout << "AT" << std::endl; break;
				case 1: col = yellow; std::cout << "TA" << std::endl; break;
				case 2: col = blue; std::cout << "CG" << std::endl; break;
				case 3: col = green; std::cout << "GC" << std::endl; break;
				}

				sunMat->setDiffuse(col);
				sunMat->setAmbient(col);
				sunMat->setSpecular(col);

				mgr->redraw();
			}
		}
	} else {
		std::cout << "NO HIT!!!!!!!!!!!!!!!!!!!!" << std::endl;
		std::cout << "x:" << sun->getToWorld()[3][0] << " y:" << sun->getToWorld()[3][1] <<" z:" << sun->getToWorld()[3][2] <<std::endl;
	}
}
// build scene
NodeTransitPtr buildScene () {
	// ORIGINAL TORUS
    //NodeRecPtr customScenePtr = makeTorus(250, 1000, 320, 640);

	// root group
	NodeRecPtr customScenePtr = Node::create();
	//customScenePtr->setCore(Group::create());

	// add tool to scene
	createTool();
	// add tool to scene
	customScenePtr->addChild(toolTrans);
	// turn on light
	customScenePtr->setCore(sunLight);

	// add dna to scene
	createDNA();
	// add dna to scene
	customScenePtr->addChild(dnaTrans);

	return NodeTransitPtr(customScenePtr);
}

// debug info
void printBasePairHashMap () {
	// print out basepairs on concole
	std::cout << "HASHMAP:" << std::endl;
	boost::unordered_map<NodeRecPtr,BasePairType>::iterator end = basePairPtrs.end();
	for (boost::unordered_map<NodeRecPtr,BasePairType>::iterator it = basePairPtrs.begin(); it != end;){
		std::cout << it->first << " -> " << it->second << std::endl;
		it++;
	}
}

void display(void)
{
	CheckTracker();

	const float time = 1000.f * std::clock() / CLOCKS_PER_SEC;	// timer var

	// TODO: dna transform
	//ComponentTransformRecPtr dnaCT = ComponentTransformRecPtr::dcast(dnaTrans->getCore());
	//dnaCT->setTranslation(Vec3f(0,0,-2500+0.025f*time));
	//dnaCT->setRotation(Quaternion(Vec3f(0,1,0),osgDegree2Rad(270)+0.0001f*time));
	//bt->setScale(Vec3f(0.001,0.001,0.001));

	// TODO: add console translation and rotation
	//float pos[3];
	//tracker->trackdGetPosition(1, pos);
	float vec[3];// = toolCT->getTranslation();

	//toolCT->setTranslation(toolChg);


	// TOOL ROTATION
	// controller x: x -> y
	// controller y -> y
	// controller z -> x
	tracker->trackdGetPosition(1, vec);
	toolCT->setTranslation(Vec3f(toolChg[0]+vec[0]*32.8084*2,toolChg[1]+vec[1]*32.8084*2-32.8084*7,toolChg[2]+vec[2]*32.8084*2));

	//tracker->trackdGetEulerAngles(1, vec);
	//Quaternion Q = Quaternion(Vec3f(0,0,0),1);
	//Q *= Quaternion(Vec3f(1,0,0),((vec[0]+deltaRotX)*Pi)/180);
	//Q *= Quaternion(Vec3f(0,1,0),((vec[1]+deltaRotY)*Pi)/180);
	//Q *= Quaternion(Vec3f(0,0,1),((vec[2]+deltaRotZ)*Pi)/180);

	//toolCT->setRotation(Q);

	//toolCT->setRotation(Quaternion(Vec3f(0,1,0),osgDegree2Rad(270)+0.1f*time));

	commitChanges();
	mgr->redraw();

	//the changelist should be cleared - else things
	//could be copied multiple times
	OSG::Thread::getCurrentChangeList()->clear();

	// to ensure a black navigation window
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
}

int main(int argc, char **argv)
{
    OSG::preloadSharedObject((const OSG::TChar*)"OSGFileIO");
	OSG::preloadSharedObject((const OSG::TChar*)"OSGImageFileIO");

	CAVEConfig cfg;
	bool cfgIsSet = false;

	// ChangeList needs to be set for OpenSG 1.4
	ChangeList::setReadWriteDefault();
	osgInit(argc,argv);

	// evaluate intial params
	for(int a=1 ; a<argc ; ++a)
	{
		if( argv[a][0] == '-' )
		{
			if ( strcmp(argv[a],"-f") == 0 )
			{
				char* cfgFile = argv[a][2] ? &argv[a][2] : &argv[++a][0];
				if (!cfg.loadFile(cfgFile))
				{
					printf("\nInvalid config file ('%s') - exiting\n", cfgFile);
					exit(1);
				}
				cfgIsSet = true;
			}
		} else {
			// try and load the scene
			printf("\nLoading file '%s'", argv[a]);
			scene = SceneFileHandler::the()->read(argv[a],0);
		}
	}

	// load the CAVE setup config file if it was not loaded already:
	if ( ! cfgIsSet )
	{
		// try and load from default config file...
		if ( ! cfg.loadFile("cve.cfg") )
		{
			printf("\nNo config file given, and no or error in default ('cve.cfg') - exiting\n");
			exit(1);
		}
	}

	cfg.printConfig();

	// start servers for video rendering
	if ( startServers(cfg) < 0 )
	{
		printf("\nFailed to start servers.... exiting");
		exit (1);
	}

	int winid = setupGLUT(&argc, argv);

	// initalize the tracker
	InitTracker(cfg);

	// Create a window for the app to write to
	// the second parameter is the broadcast-address.
	// on a cluster, you have to use an address broadcasting to all nodes:
	MultiDisplayWindowRefPtr mwin = createAppWindow(cfg,"10.0.255.255");

	// create the scene
	if (scene == NULL)
		scene = buildScene();

	commitChanges();
	// create the CAVESceneManager helper + tell it what to manage
	mgr = new CAVESceneManager(&cfg);
	mgr->setWindow(mwin );
	mgr->setRoot  (scene);

	// set an initial user position (in case no tracker being used)
	mgr->setUserTransform(Vec3f(0,0,0), Quaternion(Vec3f(0,0,0),0) );
	mgr->setTranslation(Vec3f(0,0,0));
	mgr->showAll();

	// CUSTOM CHANTGES
	// turn off headlight
	mgr->setHeadlight(false);

	// set near and far clipping pane
	mgr->setNearClippingPlane(0.01);
	mgr->setFarClippingPlane(10000);

	mgr->getWindow()->init();
	//mgr->turnWandOff();

	mgr->redraw();
	std::cout << "Entering main loop..." << std::endl;

	glutMainLoop();
	return 0;
}

void cleanup()
{
	scene = NULL;
	delete mgr;
	delete tracker;
	delete controller;
}

void reshape(int w, int h)
{
	mgr->resize(w, h);
	glutPostRedisplay();
}

void keyboard(unsigned char k, int x, int y)
{
	Real32 ed;
	switch(k)
	{
		case 'q':
		case 27:
			std::cout << "Exiting program on user request..." << std::endl;
			cleanup();
			exit(0);
			break;
		case 'z':
			ed = mgr->getEyeSeparation() * .9f;
			std::cout << "ED: " << ed << std::endl;
			mgr->setEyeSeparation(ed);
			break;
		case 'x':
			ed = mgr->getEyeSeparation() * 1.1f;
			std::cout << "ED: " << ed << std::endl;;
			mgr->setEyeSeparation(ed);
			break;
		case 'h':
			followHead=!followHead;
			std::cout << "following head: " << std::boolalpha << followHead << std::endl;;
			break;
		default:
			std::cout << "Key '" << k << "' ignored" << std::endl;
	}
}

int setupGLUT(int *argc, char *argv[])
{
	// ititalize glut + set display mode
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE );

	const int winid = glutCreateWindow("OpenSG CSMDemo + TrackdAPI");
	// initalize all relevant callback functions
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(display);
	return winid;
}

void printControllerStats(int i)
{
	if (!tracker)
		return;

	std::cout << std::showpoint << "Sensor " << i;
	float vec[3];
	std::cout.precision(5);
	tracker->trackdGetPosition(i, vec);
	std::cout << " position(" << vec[0] << ',' << vec[1] << ',' << vec[2];
	tracker->trackdGetEulerAngles(i, vec);
	std::cout << " euler angles(" << vec[0] << ',' << vec[1] << ',' << vec[2] << '\n';
}

void ButtonChanged(int button, int newState)
{
	if (button == 0)
	{
		if (newState==1 && tracker)
		{
			const int numSensors = tracker->trackdGetNumberOfSensors();

			for (int i=0; i < numSensors; ++i)
				printControllerStats(i);
		}
	}
	if (button == 1 && newState == 1)
	  {
	    // TODO: add at bp
		collissionDetection();
	  }
	  if (button == 2 && newState == 1)
	  {
	    // TODO: add ta bp
	  }
	  if (button == 3 && newState == 1)
	  {
	    // TODO: add cg bp
	  }
	  if (button == 4 && newState == 1)
	  {
	    // TODO: add gc bp
	  }
	if (button == 5 && newState == 1)
	  {
	    mgr->setTranslation(Vec3f(sceneOriginX,sceneOriginY,sceneOriginZ));
		//toolCT->setTranslation(Vec3f(toolOriginX,toolOriginY,toolOriginZ));
		toolChg = Vec3f(toolOriginX,toolOriginY,toolOriginZ);
	  }
}

void InitTracker(CAVEConfig &cfg)
{
	for (int i=0; i < MAXBUTTONS; i++)
		buttonValues[i] = 0;

	try
	{
		const int trackerKey = cfg.getTrackerKey();
		const int controllerKey = cfg.getControllerKey();
		if (trackerKey != 0)
			tracker = new TrackerReader(trackerKey);

		if (controllerKey != 0)
			controller = new ControllerReader(controllerKey);
	}
	catch(const TrackdError& te)
	{
		std::cout << "ERROR: " << te.what() << '\n';
		return;
	}
}

void CheckTracker()
{
	/* If connected to a shared mem segment for a controller/wand */
	if (controller)
	{
		float ValLR = controller->trackdGetValuator(0); //LR
		float ValFB = controller->trackdGetValuator(1); //UD
		Vec3f transChg = Vec3f(0,0,0);
		float rotateAngle = 0;

		if (fabsf(ValLR) > 0.01)
		{
			transChg[0] += (ValLR*10);
			toolChg[0] += (ValLR*10);
		}

		if (fabsf(ValFB) > 0.01)
		{
			// also check if we need to go up or down
			/*float orn[3];
			tracker->trackdGetEulerAngles(1,orn);
			float b = cosf((orn[0]/180)*Pi);
			transChg[2] -= (b*(ValFB*10));
			float a= sinf((orn[0]/180)*Pi);
			transChg[1] += (a*(ValFB*10));
			// set the rotation based on the y axis rotation of wand
			rotateAngle = (orn[1]/180)*Pi;*/

			// CUSTOM: only go front and back
			transChg[2] -= (ValFB*10);
			toolChg[2] -= (ValFB*10);
		}

		// create a matrix of out of the transformation
		Matrix transMtx,rotateMtx;
		transMtx.setIdentity();
		transMtx.setTranslate(transChg);
		rotateMtx.setIdentity();
		rotateMtx.setRotate(Quaternion(Vec3f(0,1,0),rotateAngle));

		// get the managers translation, multiply it by the change and put it back
		Matrix mgrMtx;
		mgrMtx.setTransform(mgr->getTranslation());
		mgrMtx.mult(rotateMtx);
		mgrMtx.mult(transMtx);
		//mgr->setUserTransform(Vec3f(transChg[0], 0, transChg[2]), Q);

		Quaternion Q, Q1;
		Vec3f T, T1;
		mgrMtx.getTransform(T, Q, T1, Q1);

		mgr->setTranslation(T);

		if(followHead && tracker)
		{
			float pos[3], orn[3];
			tracker->trackdGetPosition(0,pos);
			tracker->trackdGetEulerAngles(0,orn);

			// set up a quaternion for orientation diferences in the head
			Quaternion Q = Quaternion(Vec3f(0,0,0),1);
			Q *= Quaternion(Vec3f(1,0,0),(orn[0]*Pi)/180);
			Q *= Quaternion(Vec3f(0,1,0),(orn[1]*Pi)/180);
			Q *= Quaternion(Vec3f(0,0,1),(orn[2]*Pi)/180);

			// adjusting the translation by 30 for feet to cm conversion, really
			// should be handled through CSM config....
			userPos = Vec3f(pos[0]*32.8084, pos[1]*32.8084, pos[2]*32.8084);
			mgr->setUserTransform(userPos, Q);
			//mgr->setUserTransform(Vec3f(pos[0]*30, pos[1]*30, pos[2]*30), Q);
		}

		int numButtons = controller->trackdGetNumberOfButtons();
		// a bit of a bug(??) in soem trackd libs, where num buttons can be too high
		if (numButtons > MAXBUTTONS)
		{
			numButtons = MAXBUTTONS;
		}

		for (int i=0; i<numButtons; i++)
		{
			int buttVal = controller->trackdGetButton(i);
			if (buttVal != buttonValues[i])
			{
				ButtonChanged(i, buttVal);
				buttonValues[i] = buttVal;
			}
		}
	}
}
