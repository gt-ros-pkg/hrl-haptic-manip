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
static MyFeedback fixed_feedback[1];
static int fbnum=0;
//static MyObject obj[NUM];                                                                                 
static dJointGroupID contactgroup;                                                                        
static int selected = -1;       // selected object          

using namespace std;
dBody *env;
dWorld *world;
dSpace *space;
dPlane *ground;
dBody *kbody;
dBox *kbox;
dJointGroup joints;
dCylinder *kpole;
dCylinder *obst;
dBody *kobst;
dBody *matraca;
dBox *matraca_geom;
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
    void draw() const
    {
        dVector3 lengths;
        geom.getLengths(lengths);
        dsSetTexture(DS_WOOD);
        dsSetColor(0,1,0);
        dsDrawBox(geom.getPosition(), geom.getRotation(), lengths);
    }
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
      dJointID c = dJointCreateContact (*world,contactgroup,contact+i);
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




// void nearCallback(void *data, dGeomID g1, dGeomID g2)
// {
//     // if (g1 == ground->id()) {
//     //     queueRemoval(g2);
//     //     return;
//     // }
//     // if (g2 == ground->id()) {
//     //     queueRemoval(g1);
//     //     return;
//     // }

//     dBodyID b1 = dGeomGetBody(g1);
//     dBodyID b2 = dGeomGetBody(g2);
    
//     if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
//         return;

//     const int MAX_CONTACTS = 10;
//     dContact contact[MAX_CONTACTS];
//     int n = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
//     for (int i=0; i<n; ++i) {
//         contact[i].surface.mode = 0;
//         contact[i].surface.mu = 1;
// 	jt_contact = dContactJoint(*world, joints.id(), contact+i);
// 	//        dJointID j = dJointCreateContact (*world, joints.id(), contact+i);
//         dJointAttach(jt_contact, b1, b2);
// 	//	jt_contact->setFeedback(1);
//     }
// }


void
simLoop(int pause)
{
    if (!pause) {
        const dReal timestep = 0.0005;

        space->collide(0, nearCallback);
        
        world->step(timestep);
	

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
        printf("force joint %f %f %f\n", fixed_feedback[0].fb.f1[0], fixed_feedback[0].fb.f1[1], fixed_feedback[0].fb.f1[2]);
        printf("force joint 2 %f %f %f\n", fixed_feedback[0].fb.f2[0], fixed_feedback[0].fb.f2[1], fixed_feedback[0].fb.f2[2]);
	hinge->addTorque(100);	
	doFeedback = 0;
	fbnum = 0;
        joints.clear();

    }

    dVector3 lengths;

    // the moving platform
    // kbox->getLengths(lengths);
    // dsSetTexture(DS_WOOD);
    // dsSetColor(.3, .3, 1);
    // dsDrawBox(kbox->getPosition(), kbox->getRotation(), lengths);
    //    printf("got up to drawing\n");
    dReal length, radius;
    kpole->getParams(&radius, &length);
    dsSetTexture(DS_CHECKERED);
    dsSetColor(1, 1, 0);
    dsDrawCylinder(kpole->getPosition(), kpole->getRotation(), length, radius);


    obst->getParams(&radius, &length);
    dsSetTexture(DS_CHECKERED);
    dsSetColor(1, 1, 0);
    dsDrawCylinder(obst->getPosition(), obst->getRotation(), length, radius);
    
    // the matraca
    matraca_geom->getLengths(lengths);
    dsSetColor(1,0,0);
    dsSetTexture(DS_WOOD);
    dsDrawBox(matraca_geom->getPosition(), matraca_geom->getRotation(), lengths);

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
    ground = new dPlane(*space, 0, 0, 1, 0);
    
    world = new dWorld;
    world->setGravity(0, 0, 0);
    
    kbody = new dBody(*world);
    // kbody->setKinematic();
    // const dReal kx = 1, ky = 0, kz = .25;
    // kbody->setPosition(kx, ky, kz);
    // kbox = new dBox(*space, 3, 3, .5);
    // kbox->setBody(*kbody);
    kpole = new dCylinder(*space, .125, 1.5);
    kpole->setBody(*kbody);
    kpole->setPosition(0, 0, 1.5/2.0);
    env = 0;
    dGeomSetOffsetPosition(kpole->id(), 0, 0, 0.8);

    kobst = new dBody(*world);
    obst = new dCylinder(*space, 0.125, 1.5);
    obst->setBody(*kobst);
    obst->setPosition(0, -1.8, 1.5/2.0);
    dGeomSetOffsetPosition(obst->id(), 0, -1.8, 0.8);


    matraca = new dBody(*world);
    matraca->setPosition(0, 1, 1.5);
    matraca_geom = new dBox(*space, 0.2, 2, 0.5);
    matraca_geom->setBody(*matraca);
    dMass mass;
    mass.setBox(1, 0.5, 2, 0.75);
    matraca->setMass(mass);


    //    jt_contact = new dContactJoint(*world);

    fixed_joint = new dFixedJoint(*world);
    fixed_joint->attach(*kbody, 0);

    fixed_joint2 = new dFixedJoint(*world);
    fixed_joint2->attach(*kobst, 0);
    dJointSetFeedback (fixed_joint2->id(),&fixed_feedback[0].fb);


    //    fixed_joint2->

    hinge = new dHingeJoint(*world);
    hinge->attach(*kbody, *matraca);
    hinge->setAnchor(0, 0, 0.75+0.75/2);
    hinge->setAxis(0, 0, 1);
    hinge->addTorque(100);
    dsSimulationLoop (argc, argv, 640, 480, &fn);

    // while (ros::ok())    
    //   {
    // 	darpa_m3::SkinContact skin;
    // 	skin_pub.publish(skin);
    // 	ros::spinOnce();
    // 	loop_rate.sleep();
    //   }
    
    dCloseODE();
}
