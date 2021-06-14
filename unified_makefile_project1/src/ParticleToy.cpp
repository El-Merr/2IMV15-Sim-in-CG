// ParticleToy.cpp : Defines the entry point for the console application.
// make clean && make

#include "Particle.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "RailConstraint.h"
#include "CircularWireConstraint.h"
#include "GravityForce.h"
#include "imageio.h"
#include "Constraint.h"
#include "ConstraintSolver.h"
#include "Wall.h"
#include "FixedObject.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <iostream>


#define IX(i,j) ((i)+(N+2)*(j))

/* macros */

/* external definitions (from solver) */
extern void simulation_step( std::vector<Particle*> pVector, float dt, bool slomoBool, int scheme );
/* global variables */
// these are in FluidSolver.cpp
extern void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt );
extern void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt );
extern void add_objects ( std::vector<FixedObject*> objects );


static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;
static int mouse_particle_index;
static int scheme;

// New from demo.c
static float diff, visc;
static float force, source;
static int dvel;
static float * u, * v, * u_prev, * v_prev;
static float * dens, * dens_prev;


// spring constants
static float spring_ks = 0.01;
static float spring_kd = 0.0001;

// static Particle *pList;
static std::vector<Particle*> pVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;
bool slomo = false;
bool gravityArrows = false;
bool gravityActive = true;

static std::vector<Constraint*> constraints;
static ConstraintSolver* constraintSolver = NULL;

static std::vector<SpringForce*> springForce;
static RodConstraint * rodConstraint = NULL;
static CircularWireConstraint * circularWireConstraint = NULL;
static GravityForce * gravityForce = NULL;
static RailConstraint* railConstraint = NULL;
static std::vector<FixedObject*> fixedObjects;


Particle* mouseParticle = NULL;
SpringForce* mouseForce = NULL;
Wall* wall = NULL;

bool hold = false; //is the mouse held down?

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void init_system(int sceneNr)
{
    const double dist = 0.2;
    const Vec2f center(0.5, 0.5);
    const Vec2f offset(dist, 0.0);
    float defaultMass = 0.01;

    std::vector<Vec2f> pointVector;
    pointVector.push_back(center+Vec2f(-0.1+dist, -0.1));
    pointVector.push_back(center+Vec2f(0.1+dist, -0.1));
    pointVector.push_back(center+Vec2f(0.0+dist, 0.1));
    fixedObjects.push_back(new FixedObject(pointVector));

    std::vector<Vec2f> pointVector3;
    pointVector3.push_back(center+Vec2f(-dist, -dist)-offset);
    pointVector3.push_back(center+Vec2f(+dist, -dist)-offset);
    pointVector3.push_back(center+Vec2f(+dist, +dist)-offset);
    pointVector3.push_back(center+Vec2f(-dist, +dist)-offset);
    fixedObjects.push_back(new FixedObject(pointVector3));

    add_objects(fixedObjects);

    pVector.push_back(new Particle(Vec2f(N, N), 0.01));

}

static void free_data ( void )
{
	pVector.clear();
	constraints.clear();
	if (rodConstraint) {
		delete rodConstraint;
        rodConstraint = NULL;
	}
    springForce.clear();
	if (circularWireConstraint) {
		delete circularWireConstraint;
        circularWireConstraint = NULL;
	}
    if (gravityForce) {
        delete gravityForce;
        gravityForce = NULL;
    }
    if (constraintSolver) {
        delete constraintSolver;
        constraintSolver = NULL;
    }
    if (railConstraint) {
        delete railConstraint;
        railConstraint = NULL;
    }
    if (wall) {
        delete wall;
        wall = NULL;
    }
    fixedObjects.clear();
    //from demo.c
    if ( u ) free ( u );
    if ( v ) free ( v );
    if ( u_prev ) free ( u_prev );
    if ( v_prev ) free ( v_prev );
    if ( dens ) free ( dens );
    if ( dens_prev ) free ( dens_prev );
}

static void clear_data ( void )
{
	int ii, size = pVector.size();

	for(ii=0; ii<size; ii++){
		pVector[ii]->reset();
	}

	//from demo.c
    int i, size2=(N+2)*(N+2);

    for ( i=0 ; i<size2 ; i++ ) {
        u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
    }
}

static void clear_forces ( void )
{
    int ii, size = pVector.size();

    for(ii=0; ii<size; ii++){
        pVector[ii]->clear_force();
    }
}
/**
 * From demo.c
 */
static int allocate_data ( void )
{
    int size = (N+2)*(N+2);

    u			= (float *) malloc ( size*sizeof(float) );
    v			= (float *) malloc ( size*sizeof(float) );
    u_prev		= (float *) malloc ( size*sizeof(float) );
    v_prev		= (float *) malloc ( size*sizeof(float) );
    dens		= (float *) malloc ( size*sizeof(float) );
    dens_prev	= (float *) malloc ( size*sizeof(float) );

    if ( !u || !v || !u_prev || !v_prev || !dens || !dens_prev ) {
        fprintf ( stderr, "cannot allocate data\n" );
        return ( 0 );
    }

    return ( 1 );
}

/**
 * Handles mouse interaction during simulation
 * When the left mouse button is held down a spring force between
 * the mouse cursor and the 3rd particle is created
 */
void handle_mouse() {
    int i, j; // screen coords
    float x, y; // mouse coords

    if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0]
         && !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

    i = (int)((       mx /(float)win_x)*N);
    j = (int)(((win_y-my)/(float)win_y)*N);

    if ( i<1 || i>N || j<1 || j>N ) return;

    x = (float)2 * i / N - 1;
    y = (float)2 * j / N - 1;

    if (mouse_down[0]) {
        // when left mouse button is pressed and held, a spring force is applied between it and a given particle
        if (!hold) {
            // try to find a particle to drag, otherwise there is no particle in the scene
            try {
                // create a particle at the mouse position
                mouseParticle = new Particle(Vec2f(x, y), 0);

                // find the particle in pVector that is closest to the mouse
                Particle* dragParticle = pVector[0];
                float dist = sqrt( pow(mouseParticle->m_Position[0] - pVector[0]->m_Position[0], 2)
                                   + pow(mouseParticle->m_Position[1] - pVector[0]->m_Position[1], 2) );
                float new_dist = 0;
                for ( auto p : pVector ) {
                    new_dist = sqrt( pow(mouseParticle->m_Position[0] - p->m_Position[0], 2)
                                     + pow(mouseParticle->m_Position[1] - p->m_Position[1], 2) );
                    if (new_dist < dist) {
                        dist = new_dist;
                        dragParticle = p;
                    }
                }

                // create springforce between mouse and closest particle
                mouseForce = new SpringForce(mouseParticle, dragParticle, 0.6, 0.0004, 0.000001);
            } catch (...) {
                printf("There is no particle to drag");
            }

        }
        hold = true;
        mouseParticle->set_state(Vec2f(x, y), Vec2f(0.0, 0.0));
    }

    if (mouse_release[0]) {
//        printf("mouse released\n");
        hold = false;
        mouse_down[0] = false;
        mouse_release[0] = false;
        delete mouseParticle;
        delete mouseForce;
    }
}


/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( 0.0, 1.0, 0.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char * buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			sprintf(filename, "../snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			saveImageRGBA(filename, buffer, w, h);
			
			free(buffer);
		}
	}
	frame_number++;
	
	glutSwapBuffers ();
}

static void apply_constraints ( void )
{
    if (constraintSolver)
        constraintSolver->apply_constraint();
}

static void draw_particles ( void )
{
	int size = pVector.size();

	for(int ii=0; ii< size; ii++)
	{
		pVector[ii]->draw();
	}
}
/**
 * From demo.c
 */
static void draw_velocity ( void )
{
    int i, j;
    float x, y, h;

    h = 1.0f/N;

    glColor3f ( 1.0f, 1.0f, 1.0f );
    glLineWidth ( 1.0f );

    glBegin ( GL_LINES );

    for ( i=1 ; i<=N ; i++ ) {
        x = (i-0.5)*h;
        for ( j=1 ; j<=N ; j++ ) {
            y = (j-0.5f)*h;

            glVertex2f ( x, y );
            glVertex2f ( x+u[IX(i,j)], y+v[IX(i,j)] );
        }
    }

    glEnd ();
}
/**
 * From demo.c
 */
static void draw_density ( void )
{
    int i, j;
    float x, y, h, d00, d01, d10, d11;

    h = 1.0f/N;

    glBegin ( GL_QUADS );

    for ( i=0 ; i<=N ; i++ ) {
        x = (i-0.5f)*h;
        for ( j=0 ; j<=N ; j++ ) {
            y = (j-0.5f)*h;

            d00 = dens[IX(i,j)];
            d01 = dens[IX(i,j+1)];
            d10 = dens[IX(i+1,j)];
            d11 = dens[IX(i+1,j+1)];

            glColor3f ( d00, d00, d00 ); glVertex2f ( x, y );
            glColor3f ( d10, d10, d10 ); glVertex2f ( x+h, y );
            glColor3f ( d11, d11, d11 ); glVertex2f ( x+h, y+h );
            glColor3f ( d01, d01, d01 ); glVertex2f ( x, y+h );
        }
    }

    glEnd ();
}

static void draw_forces ( void )
{
	// change this to iteration over full set
    for(int spring=0; spring<springForce.size(); spring++)
    {
        springForce[spring]->draw();
    }
	if (gravityForce && gravityActive) {
        gravityForce->drawArrows = gravityArrows;
        gravityForce->draw();
    }
}

static void apply_forces ( void )
{
    for(int spring=0; spring<springForce.size(); spring++)
    {
        springForce[spring]->apply_spring();
    }

    if (gravityForce && gravityActive) {
        gravityForce->apply_gravity();
    }

    if (hold) {
        mouseForce->apply_spring();
    }

    if (wall) {
        wall->detectCollision(pVector);
    }
}

static void draw_constraints ( void )
{
	for (int ii=0; ii < constraints.size(); ii++) {
	    constraints[ii]->draw();
	}
	if (wall) {
        wall->draw();
    }
}

static void draw_fixed_objects (void ) {
    for (int i = 0; i < fixedObjects.size(); i ++) {
        fixedObjects[i]->DrawFixedObject();
    }
}

/**
 * Draw the direction of the particles forces.
 */
static void draw_direction ( void )
{
    for(int ii=0; ii<pVector.size(); ii++)
    {
        pVector[ii]->draw_arrows();
    }
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI (float * d, float * u, float * v)
{
	int i, j;
	int size = (N+2)*(N+2);

    for ( i=0 ; i<size ; i++ ) {
        u[i] = v[i] = d[i] = 0.0f;
    }

	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0] 
	&& !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

    //from demo.c
    i = (int)((       mx /(float)win_x)*N+1);
    j = (int)(((win_y-my)/(float)win_y)*N+1);

    if ( i<1 || i>N || j<1 || j>N ) return;

    if ( mouse_down[0] ) {
        u[IX(i,j)] = force * (mx-omx);
        v[IX(i,j)] = force * (omy-my);
    }

    if ( mouse_down[2] ) {
        d[IX(i,j)] = source;
    }
    //end mouse from demo.c

	if( mouse_release[0] ) {

	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	int ii, size = pVector.size();
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
	}
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key ) {
        case 'c':
        case 'C':
            clear_data();
            break;
        case 'd':
        case 'D':
            dump_frames = !dump_frames;
            break;

        case 'q':
        case 'Q':
            free_data();
            exit(0);
            break;
	    case 'g':
        case 'G':
	        gravityArrows = !gravityArrows;
	        break;
        case 'h':
        case 'H':
            gravityActive = !gravityActive;
            break;
        case 's': //slomo mode
        case 'S':
            slomo = !slomo;
            break;
        // from demo.c
        case 'v':
        case 'V':
            dvel = !dvel;
            break;
        case ' ':
            dsim = !dsim;
            break;

//        case '1':
//            free_data();
//            init_system(0);
//            break;
//
//        case '2':
//            free_data();
//            init_system(1);
//            break;
//
//	    case '3':
//            free_data();
//            init_system(2);
//            break;
//
//        case '4':
//            free_data();
//            init_system(3);
//            break;
//        case '5':
//            free_data();
//            init_system(4);
//            break;
//
//        case 'e':
//        case 'E':
//            scheme = 0;
//            break;
//        case 'm':
//        case 'M':
//            scheme = 1;
//            break;
//        case 'r':
//        case 'R':
//            scheme = 2;
//            break;
    }
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	if(!mouse_down[0]){hmx=x; hmy=y;}
	if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
    clear_forces();

	if ( dsim ) {
        handle_mouse();
        apply_forces();
        apply_constraints();
        simulation_step( pVector, dt, slomo, scheme );
	}
	else {
        // demo.c
        get_from_UI ( dens_prev, u_prev, v_prev );
        vel_step ( N, u, v, u_prev, v_prev, visc, dt );
        dens_step ( N, dens, dens_prev, u, v, diff, dt );
	    remap_GUI();
	}



	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

	draw_forces();
	draw_constraints();
	draw_particles();


	if (dsim) {
        draw_direction();
	}

	if (hold) {
	    mouseParticle->draw();
	    mouseForce->draw();
	}
	// demo.c
    if ( dvel ) draw_velocity ();
    else		draw_density ();
    // draw objects after fluid
    draw_fixed_objects();
	post_display ();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Particletoys!" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );


	printf ( "\n\nHow to use this application:\n\n" );

    printf ( "To change scenes, use the number keys 1-4\n\n" );

    printf ( "To change integration schemes:\n" );
    printf ( "\t Switch to the Euler method with the 'e' key\n" );
    printf ( "\t Switch to the Mid-point method with the 'm' key\n" );
    printf ( "\t Switch to the Runge-Kutta 4 method with the 'r' key\n" );

    printf ( "Other controls:\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
    printf ( "\t Toggle slow-motion with the 's' key\n" );
    printf ( "\t Toggle gravity with the 'h' key\n" );
    printf ( "\t Toggle gravity force arrows with the 'g' key\n" );
    printf ( "\t Reset the particles with the 'c' key\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	scheme = 2;

    //from demo.c
    if ( argc != 1 && argc != 6 ) {
        fprintf ( stderr, "usage : %s N dt diff visc force source\n", argv[0] );
        fprintf ( stderr, "where:\n" );\
		fprintf ( stderr, "\t N      : grid resolution\n" );
        fprintf ( stderr, "\t dt     : time step\n" );
        fprintf ( stderr, "\t diff   : diffusion rate of the density\n" );
        fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
        fprintf ( stderr, "\t force  : scales the mouse movement that generate a force\n" );
        fprintf ( stderr, "\t source : amount of density that will be deposited\n" );
        exit ( 1 );
    }

    if ( argc == 1 ) {
        N = 128;
        dt = 0.1f;
        d = 5.f;
        diff = 0.0f;
        visc = 0.0f;
        force = 5.0f;
        source = 100.0f;
        fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g diff=%g visc=%g force = %g source=%g\n",
                  N, dt, d, diff, visc, force, source );
    } else {
        N = atoi(argv[1]);
        dt = atof(argv[2]);
        diff = atof(argv[3]);
        visc = atof(argv[4]);
        force = atof(argv[5]);
        source = atof(argv[6]);
        d = atof(argv[7]);
    }

    printf ( "\n\nHow to use this demo:\n\n" );
    printf ( "\t Add densities with the right mouse button\n" );
    printf ( "\t Add velocities with the left mouse button and dragging the mouse\n" );
    printf ( "\t Toggle density/velocity display with the 'v' key\n" );
    printf ( "\t Clear the simulation by pressing the 'c' key\n" );
    printf ( "\t Quit by pressing the 'q' key\n" );

    dvel = 0;

    if ( !allocate_data () ) exit ( 1 );
    clear_data ();

    init_system(0);
	
	win_x = 900;
	win_y = 900;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}

