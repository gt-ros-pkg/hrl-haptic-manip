// #include "ros/ros.h"
// #include "darpa_m3/SkinContact.h"
#include <iostream>
#include <set>
#include <algorithm>
#include <functional>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define MAX_CONTACTS 5          // maximum number of contact points per body                              
#define MAX_FEEDBACKNUM 5
#define GRAVITY REAL(0)

#endif


struct MyFeedback {
  dJointFeedback fb;
  bool first;
};
static int doFeedback=1;
static MyFeedback feedbacks[MAX_FEEDBACKNUM];
//static MyFeedback fixed_feedback[1];
static int fbnum=0;
//static MyObject obj[NUM];                                                                                 
static dJointGroupID contactgroup;                                                                        
static int selected = -1;       // selected object          

using namespace std;
dBody *env;
dWorld *world;
dSpace *space;
dPlane *ground;
dJointGroup joints;
dBody *link1;
//dCylinder *g_link1;
dBox *g_link1;
dBody *link2;
//dCylinder *g_link2;
dBox *g_link2;
dBody *link3;
//dCylinder *g_link3;
dBox *g_link3;
dHingeJoint *hinge1;
dHingeJoint *hinge2;
dHingeJoint *hinge3;
dBody *obst;
dBox *g_obst;
dSliderJoint *slider1;


static dJointID plane2d;
dHingeJoint *hinge;
dFixedJoint *fixed_joint;
dFixedJoint *fixed_joint2;
dContactJoint *jt_contact;
int dir;

struct Box {
    dBody body;
    dBox geom;
    Box() :
        body(*world),
        geom(*space, 0.2, 0.2, 0.2)
    {
        dMass mass;
        mass.setBox(1, 0.2, 0.2, 0.2);
        body.setMass(mass);
        geom.setData(this);
        geom.setBody(body);
    }
    void draw() const
    {
        dVector3 lengths;
        geom.getLengths(lengths);
        dsSetTexture(DS_WOOD);
        dsSetColor(0,1,0);
        dsDrawBox(geom.getPosition(), geom.getRotation(), lengths);
    }
};

struct Cylinder {
    dBody body;
    dCylinder geom;
  Cylinder(dReal density, dReal rad, dReal len) :
        body(*world),
        geom(*space, rad, len)
    {
        dMass mass;
        mass.setCylinder(density, 1, rad, len);
        body.setMass(mass);
	//body.setRotation();
        geom.setData(this);
        geom.setBody(body);
    }
    // void draw() const
    // {
    //     dVector3 lengths;
    //     geom.getLengths(lengths);
    //     dsSetTexture(DS_WOOD);
    //     dsSetColor(0,1,0);
    //     dsDrawBox(geom.getPosition(), geom.getRotation(), lengths);
    // }
};

set<Cylinder*> cylinders;
set<Box*> boxes;
set<Box*> to_remove;

void makeObst(dReal x, dReal y, dReal z, dReal friction, dReal density, dReal rad, dReal len)
{
  Cylinder *cylinder = new Cylinder(density, rad, len);
  cylinder->body.setPosition(x,y,z);
  cylinders.insert(cylinder);



}

void dropBox()
{
    Box *box = new Box();
    
    dReal px = (rand() / float(RAND_MAX)) * 2 - 1;
    dReal py = (rand() / float(RAND_MAX)) * 2 - 1;
    dReal pz = 3;
    box->body.setPosition(px, py, pz);
    
    boxes.insert(box);
}

void queueRemoval(dGeomID g)
{
    Box *b = (Box*)dGeomGetData(g);
    to_remove.insert(b);
}

void removeQueued()
{
    while (!to_remove.empty()) {
        Box *b = *to_remove.begin();
        to_remove.erase(b);
        boxes.erase(b);
        delete b;
    }
}




static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i;
  // if (o1->body && o2->body) return;                                                                                 

  // exit without doing anything if the two bodies are connected by a joint                                            
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  //  printf("got up to checking for connection\n");
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;
  //  printf("got past checking for connection\n");
  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box                                         
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = 0.01;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.001;
    contact[i].surface.bounce_vel = 0.001;
    contact[i].surface.soft_cfm = 0.01;
  }

  //  printf("got past making contacts\n");  

  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact))) 
  {
    //    printf("got into checking for collisions\n");
    for (i=0; i<numc; i++) 
    {
      //      printf("num of collisions is %i \n", i);
      //      dJointID c = dJointCreateContact (*world,contactgroup,contact+i);
      dJointID c = dJointCreateContact (*world,joints.id(),contact+i);
      dJointAttach (c,b1,b2);

      if (fbnum<MAX_FEEDBACKNUM)
	{
	  //	  printf("fbnum is %i \n", fbnum);
	  feedbacks[fbnum].first = 0;// b1==obj[selected].bod;
	  dJointSetFeedback (c,&feedbacks[fbnum++].fb);
	}
      else fbnum++;

    }
  }
}

void
simLoop(int pause)
{
    if (!pause) {
        const dReal timestep = 0.001;

        space->collide(0, nearCallback);
        
        world->step(timestep);
	joints.empty();

	if (fbnum>MAX_FEEDBACKNUM)
	  printf("joint feedback buffer overflow!\n");
	else
	  {
	    printf("got in here");
	    dVector3 sum = {0, 0, 0};
	    printf("num of feedback %i\n", fbnum);
	    for (int i=0; i<fbnum; i++) {
	      printf("force 1 %f %f %f\n", feedbacks[i].fb.f1[0], feedbacks[i].fb.f1[1], feedbacks[i].fb.f1[2]);
	      printf("force 2 %f %f %f\n", feedbacks[i].fb.f2[0], feedbacks[i].fb.f2[1], feedbacks[i].fb.f2[2]);
	    }
	  }
	//        printf("force joint %f %f %f\n", fixed_feedback[0].fb.f1[0], fixed_feedback[0].fb.f1[1], fixed_feedback[0].fb.f1[2]);
	//        printf("force joint 2 %f %f %f\n", fixed_feedback[0].fb.f2[0], fixed_feedback[0].fb.f2[1], fixed_feedback[0].fb.f2[2]);
	//	hinge->addTorque(0);	
	doFeedback = 0;
	fbnum = 0;
        joints.clear();

    }
    hinge3->addTorque(3);
    dVector3 lengths;

    // the moving platform
    // kbox->getLengths(lengths);
    // dsSetTexture(DS_WOOD);
    // dsSetColor(.3, .3, 1);
    // dsDrawBox(kbox->getPosition(), kbox->getRotation(), lengths);

    g_link1->getLengths(lengths);
    dsSetTexture(DS_WOOD);
    dsSetColor(1, 0, 0);
    dsDrawBox(g_link1->getPosition(), g_link1->getRotation(), lengths);

    g_link2->getLengths(lengths);
    dsSetTexture(DS_WOOD);
    dsSetColor(0, 1, 0);
    dsDrawBox(g_link2->getPosition(), g_link2->getRotation(), lengths);

    g_link3->getLengths(lengths);
    dsSetTexture(DS_WOOD);
    dsSetColor(0, 0, 1);
    dsDrawBox(g_link3->getPosition(), g_link3->getRotation(), lengths);

    dReal len, rad;
    // g_obst->getParams(&rad, &len);
    // dsSetTexture(DS_WOOD);
    // dsSetColor(0, 1, 1);
    // dsDrawCylinder(g_obst->getPosition(), g_obst->getRotation(), len, rad);

    g_obst->getLengths(lengths);
    dsSetTexture(DS_WOOD);
    dsSetColor(0, 1, 1);
    dsDrawBox(g_obst->getPosition(), g_obst->getRotation(), lengths);


    //    printf("got up to drawing\n");

    //    printf("got past all drawing but boxes\n");
    // and the boxes
    //    for_each(boxes.begin(), boxes.end(), mem_fun(&Box::draw));
    //    printf("got past drawing boxes too\n");
}

void command(int c)
{
    switch (c) {
        case ' ':
            dropBox();
            break;
    }
}

int main(int argc, char **argv)
{
  // ros::init(argc, argv, "sim_arm");
  // ros::NodeHandle n;
  // ros::Publisher skin_pub = n.advertise<darpa_m3::SkinContact>("/skin/contacts", 100);
  // ros::Rate loop_rate(1000);

    dInitODE();
    dir = 1;
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = 0;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    
    cout << endl << "*** Press SPACE to drop boxes **" << endl;
    
    space = new dSimpleSpace();
    //    ground = new dPlane(*space, 0, 0, 1, 0);
    
    world = new dWorld;
    world->setGravity(0, 0, 0);
    
//link1
    dReal rad1(0.3);
    dReal len1(2);
    link1 = new dBody(*world);
    dMass mass_l1;
    //    mass_l1.setCylinder(1, 3, rad1, len1);
    mass_l1.setBox(1, 2, 0.2, 0.2);
    link1->setMass(mass_l1);
    //    g_link1 = new dCylinder(*space, rad1, len1);
    g_link1 = new dBox(*space, 2, 0.2, 0.2);
    g_link1->setBody(*link1);
    //    dMatrix3 rot_link1 = { 0,0,-1,0,1,0,0,0,0,1,0,0 };
    //    link1->setRotation(rot_link1);
    link1->setPosition(1, 0, 1); 

    //link2
    dReal rad2(0.3);
    dReal len2(2);
    link2 = new dBody(*world);
    dMass mass_l2;
    //    mass_l2.setCylinder(1, 3, rad2, len2);
    mass_l2.setBox(1, 2, 0.2, 0.2);
    link2->setMass(mass_l2);
    //    g_link2 = new dCylinder(*space, rad2, len2);
    g_link2 = new dBox(*space, 2, 0.2, 0.2);
    g_link2->setBody(*link2);
    //    dMatrix3 rot_link2 = { 0,0,-1,0,1,0,0,0,0,1,0,0 };
    //    link2->setRotation(rot_link2);
    link2->setPosition(3, 0, 1); 

    //link3
    dReal rad3(0.3);
    dReal len3(1);
    link3 = new dBody(*world);
    dMass mass_l3;
    //    mass_l3.setCylinder(1, 3, rad3, len3);
    mass_l3.setBox(1, 1, 0.2, 0.2);
    link3->setMass(mass_l3);
    //    g_link3 = new dCylinder(*space, rad3, len3);
    g_link3 = new dBox(*space, 1, 0.2, 0.2);
    g_link3->setBody(*link3);
    //    dMatrix3 rot_link3 = { 0,0,-1,0,1,0,0,0,0,1,0,0 };
    //    link3->setRotation(rot_link3);
    link3->setPosition(4.5, 0, 1); 

    //    obst                                                                                      
    obst = new dBody(*world);
    dMass mass_obst;
    mass_obst.setBox(1, 0.2, 0.2, 2.0);
    obst->setMass(mass_obst);
    g_obst = new dBox(*space, 0.2, 0.2, 2.0);
    g_obst->setBody(*obst);
    obst->setPosition(0, 1, 1.1);


    //    slider1 = new dSliderJoint(*world);
    //    slider1->attach(*obst, 0);
    //    slider1->setAxis(1, 0, 0);
    plane2d = dJointCreatePlane2D(world->id(), 0);
    dJointAttach(plane2d, obst->id(), 0);
    // dJointSetPlane2DXParam (plane2d, dParamFMax, 10);
    // dJointSetPlane2DYParam (plane2d, dParamFMax, 10);


    //    fixed_joint = new dFixedJoint(*world);
    //    fixed_joint->attach(*obst, 0);
    //    dJointSetFixed(fixed_joint->id());

    hinge1 = new dHingeJoint(*world);
    hinge1->attach(*link1, 0);
    hinge1->setAnchor(0, 0, 1);
    hinge1->setAxis(0, 0, 1);
    hinge1->addTorque(0);
    dJointSetHingeParam(hinge1->id(),dParamLoStop, 0);
    dJointSetHingeParam(hinge1->id(),dParamHiStop, 3.14);

    hinge2 = new dHingeJoint(*world);
    hinge2->attach(*link2, *link1);
    hinge2->setAnchor(2, 0, 1);
    hinge2->setAxis(0, 0, 1);
    dJointSetHingeParam(hinge2->id(),dParamLoStop, 0);
    dJointSetHingeParam(hinge2->id(),dParamHiStop, 3.14);

    
    hinge3 = new dHingeJoint(*world);
    hinge3->attach(*link3, *link2);
    hinge3->setAnchor(4, 0, 1);
    hinge3->setAxis(0, 0, 1);
    dJointSetHingeParam(hinge3->id(),dParamLoStop, -3.14/2.0);
    dJointSetHingeParam(hinge3->id(),dParamHiStop, 3.14/2.0);

    joints.create();


    dsSimulationLoop (argc, argv, 1280, 768, &fn);

    // while (ros::ok())    
    //   {
    // 	darpa_m3::SkinContact skin;
    // 	skin_pub.publish(skin);
    // 	ros::spinOnce();
    // 	loop_rate.sleep();
    //   }
    
    dCloseODE();
}
