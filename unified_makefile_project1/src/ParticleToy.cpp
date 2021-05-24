// ParticleToy.cpp : Defines the entry point for the console application.
//

#include "Particle.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "GravityForce.h"
#include "imageio.h"
#include "Constraint.h"
#include "ConstraintSolver.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <iostream>

/* macros */

/* external definitions (from solver) */
extern void simulation_step( std::vector<Particle*> pVector, float dt);
/* global variables */

static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;
static int mouse_particle_index;

// spring constants
static float spring_ks = 0.001;
static float spring_kd = 0.00001;

// static Particle *pList;
static std::vector<Particle*> pVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

static std::vector<Constraint*> constraints;
static ConstraintSolver* constraintSolver = NULL;

static std::vector<SpringForce*> springForce;
static RodConstraint * rodConstraint = NULL;
static CircularWireConstraint * circularWireConstraint = NULL;
static GravityForce * gravityForce = NULL;

Particle* mouseParticle = NULL;
SpringForce* mouseForce = NULL;

bool hold = false; //is the mouse held down?

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

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
}

static void clear_data ( void )
{
	int ii, size = pVector.size();

	for(ii=0; ii<size; ii++){
		pVector[ii]->reset();
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
                mouseForce = new SpringForce(mouseParticle, dragParticle, 0.6, 0.00004, 0.0000001);
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

static void init_system(int sceneNr)
{
	const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);
	float defaultMass = 0.01;

	switch(sceneNr) {
        case 0: //default scene
            // Create three particles, attach them to each other, then add a
            // circular wire constraint to the first.
            pVector.push_back(new Particle(center + offset, defaultMass));
            pVector.push_back(new Particle(center + offset + offset, defaultMass));
            pVector.push_back(new Particle(center + offset + offset + offset, defaultMass));

            mouse_particle_index = 1; // sets the 2nd particle to be the mouse interaction particle.

            springForce.push_back(new SpringForce(pVector[0], pVector[1], dist, spring_ks, spring_kd));
            rodConstraint = new RodConstraint(pVector[1], pVector[2], dist);
            circularWireConstraint = new CircularWireConstraint(pVector[0], center, dist);

            constraints.push_back(circularWireConstraint);
			constraints.push_back(rodConstraint);
            constraintSolver = new ConstraintSolver(pVector, constraints);

            break;

        case 1: //cloth scene
            mouse_particle_index = 20;

            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {
                    pVector.push_back(new Particle(center - 2 * offset +
                        Vec2f(i * dist, j * dist), 0.001));
                }
            }
            int size = pVector.size();

            for(int ii=0; ii < size - 1; ii++) {
                if ((ii + 1) % 5 != 0) {
                    springForce.push_back(new SpringForce(pVector[ii], pVector[ii + 1], dist*2, spring_ks, spring_kd));
                }
                if (ii < 20 ) {
                    springForce.push_back(new SpringForce(pVector[ii], pVector[ii + 5], dist*2, spring_ks, spring_kd));
                }
                circularWireConstraint = new CircularWireConstraint(pVector[4], center + Vec2f(-3 * dist, 5 * dist), dist);
                auto circularWireConstraint2 = new CircularWireConstraint(pVector[24], center + Vec2f(3 * dist, 5 * dist), dist);
                constraints.push_back(circularWireConstraint);
                constraints.push_back(circularWireConstraint2);
                constraintSolver = new ConstraintSolver(pVector, constraints);
            }
    }
    gravityForce = new GravityForce(pVector);
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
	gluOrtho2D ( -1.0, 1.0, -1.0, 1.0 );
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

static void draw_forces ( void )
{
	// change this to iteration over full set
    for(int spring=0; spring<springForce.size(); spring++)
    {
        springForce[spring]->draw();
    }
	if (gravityForce)
	    gravityForce->draw();
}

static void apply_forces ( void )
{
    for(int spring=0; spring<springForce.size(); spring++)
    {
        springForce[spring]->apply_spring();
    }

    if (gravityForce) {
        gravityForce->apply_gravity();
    }

    if (hold) {
        mouseForce->apply_spring();
    }
}

static void draw_constraints ( void )
{
	for (int ii; ii < constraints.size(); ii++) {
	    constraints[ii]->draw();
	}
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI ()
{
	int i, j;
	// int size, flag;
//	int hi, hj;
//	float x, y;

	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0] 
	&& !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

	i = (int)((       mx /(float)win_x)*N);
	j = (int)(((win_y-my)/(float)win_y)*N);

	if ( i<1 || i>N || j<1 || j>N ) return;

//    x = (float)2 * i / N - 1;
//    y = (float)2 * j / N - 1;

	if ( mouse_down[0]) {
	}

	if ( mouse_down[2] ) {
	}

//	hi = (int)((       hmx /(float)win_x)*N);
//	hj = (int)(((win_y-hmy)/(float)win_y)*N);

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

        case ' ':
            dsim = !dsim;
            break;

        case '1':
            free_data();
            init_system(0);
            break;

        case '2':
            free_data();
            init_system(1);
            break;
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
	    simulation_step( pVector, dt );
	}
	else {
	    get_from_UI();
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
	if (hold) {
	    mouseParticle->draw();
	    mouseForce->draw();
	}

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

	if ( argc == 1 ) {
		N = 64;
		dt = 0.1f;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	
	init_system(0);
	
	win_x = 800;
	win_y = 800;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}

