#include "FixedObject.h"
#include <gfx/vec2.h>
#include "vector"

#define IX(i,j) ((i)+(N+2)*(j))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) {
#define END_FOR }}

static std::vector<FixedObject*> fixedObjects;

void add_source ( int N, float * x, float * s, float dt )
{
	int i, size=(N+2)*(N+2);
	for ( i=0 ; i<size ; i++ ) x[i] += dt*s[i];
}

void bnd_fixed_object ( int N, int b, float * x) {
    for ( int obj=0; obj < fixedObjects.size(); obj++ ) {
        auto points = fixedObjects[obj]->get_points();
        Vec2f p1, p2;
        for ( int point=0; point < points.size(); point++ ) {
            p1 = points[point] * N;
            p2 = (point == points.size() - 1 ? p2 = points[0] : p2 = points[point + 1]) * N;
//            if ( point == points.size() - 1 ) {    // when last point, connect to first
//                p2 = points[0];
//            } else {
//                p2 = points[point + 1];
//            }


            if ( std::abs(p1[0] - p2[0]) < std::abs(p1[1] - p2[1]) ) {
                if (p1[1] > p2[1]) {
                    Vec2f temp = p1;
                    p1 = p2;
                    p2 = temp;
                }
                float a = (p1[0] - p2[0]) / (p1[1] - p2[1]);
                float b = p1[0] - a * p1[1];
                for ( int py=p1[1]; py <= p2[1]; py++ ) {
                    int px = floor(a * py + b + 0.5);
//                printf("a: %f, b: %f, px: %d  py: %d\n", a, b, px, py);
                    x[IX(px  ,py)] = -x[IX(px+1,py+1)];
                    //b==1 ? -x[IX(px,py)] : x[IX(px ,py)];
//                x[IX(px  ,py+1)] = b==1 ? -x[IX(px,py+1)] : x[IX(px ,py+1)];
                }
            } else {
                if (p1[0] > p2[0]) {
                    Vec2f temp = p1;
                    p1 = p2;
                    p2 = temp;
                }

                float a = (p1[1] - p2[1]) / (p1[0] - p2[0]);
                float b = p1[1] - a * p1[0];
                for ( int px=p1[0]; px <= p2[0]; px++ ) {
                    int py = floor(a * px + b + 0.5);
//                printf("a: %f, b: %f, px: %d  py: %d\n", a, b, px, py);
                    x[IX(px  ,py)] = -x[IX(px+1,py+1)]; //b==1 ? -x[IX(px,py)] : x[IX(px ,py)];
//                x[IX(px  ,py+1)] = b==1 ? -x[IX(px,py+1)] : x[IX(px ,py+1)];
                }
            }



////            printf("point: %f , %f", p1[0], p1[1]);
//            p1 = p1 * N;
//            p2 = p2 * N;

//            printf("p1: %f, %f     p2: %f, %f\n", p1[0],p1[1], p2[0], p2[1]);

            // line y = ax + b
//            float a = (p1[1] - p2[1]) / (p1[0] - p2[0]);
//            float b = p1[1] - a * p1[0];
//            for ( int px=p1[0]; px <= p2[0]; px++ ) {
//                int py = floor(a * px + b);
////                printf("a: %f, b: %f, px: %d  py: %d\n", a, b, px, py);
//                x[IX(px  ,py)] = -x[IX(px,py)]; //b==1 ? -x[IX(px,py)] : x[IX(px ,py)];
////                x[IX(px  ,py+1)] = b==1 ? -x[IX(px,py+1)] : x[IX(px ,py+1)];
//            }


        }
    }
//    for ( int i=50; i <= 100; i++ ) {
//        int line = 75;
//        x[IX(50  ,i)] = b==1 ? -x[IX(51,i)] : x[IX(51,i)];        // horizontal
//        x[IX(100+1,i)] = b==1 ? -x[IX(100,i)] : x[IX(100,i)];
//        x[IX(i,50  )] = b==2 ? -x[IX(i,51)] : x[IX(i,51)];        // vertical
//        x[IX(i,100+1)] = b==2 ? -x[IX(i,100)] : x[IX(i,100)];
//    }
}

void set_bnd ( int N, int b, float * x )
{
	int i;

	for ( i=1 ; i<=N ; i++ ) {
		x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];
		x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];
		x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];
		x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];
	}

    bnd_fixed_object( N, b, x);

	x[IX(0  ,0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0  ,1)]);
	x[IX(0  ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0  ,N)]);
	x[IX(N+1,0  )] = 0.5f*(x[IX(N,0  )]+x[IX(N+1,1)]);
	x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);


}



void lin_solve ( int N, int b, float * x, float * x0, float a, float c )
{
	int i, j, k;

	for ( k=0 ; k<20 ; k++ ) {
		FOR_EACH_CELL
			x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
		END_FOR
		set_bnd ( N, b, x );
	}
}

void diffuse ( int N, int b, float * x, float * x0, float diff, float dt )
{
	float a=dt*diff*N*N;
	lin_solve ( N, b, x, x0, a, 1+4*a );
}

void advect ( int N, int b, float * d, float * d0, float * u, float * v, float dt )
{
	int i, j, i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dt0;

	dt0 = dt*N;
	FOR_EACH_CELL
		x = i-dt0*u[IX(i,j)]; y = j-dt0*v[IX(i,j)];
		if (x<0.5f) x=0.5f; if (x>N+0.5f) x=N+0.5f; i0=(int)x; i1=i0+1;
		if (y<0.5f) y=0.5f; if (y>N+0.5f) y=N+0.5f; j0=(int)y; j1=j0+1;
		s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
		d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])+
					 s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
	END_FOR
	set_bnd ( N, b, d );
}

void project ( int N, float * u, float * v, float * p, float * div )
{
	int i, j;

	FOR_EACH_CELL
		div[IX(i,j)] = -0.5f*(u[IX(i+1,j)]-u[IX(i-1,j)]+v[IX(i,j+1)]-v[IX(i,j-1)])/N;
		p[IX(i,j)] = 0;
	END_FOR	
	set_bnd ( N, 0, div ); set_bnd ( N, 0, p );

	lin_solve ( N, 0, p, div, 1, 4 );

	FOR_EACH_CELL
		u[IX(i,j)] -= 0.5f*N*(p[IX(i+1,j)]-p[IX(i-1,j)]);
		v[IX(i,j)] -= 0.5f*N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
	END_FOR
	set_bnd ( N, 1, u ); set_bnd ( N, 2, v );
}

void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt )
{
	add_source ( N, x, x0, dt );
	SWAP ( x0, x ); diffuse ( N, 0, x, x0, diff, dt );
	SWAP ( x0, x ); advect ( N, 0, x, x0, u, v, dt );
}

void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt )
{
	add_source ( N, u, u0, dt ); add_source ( N, v, v0, dt );
	SWAP ( u0, u ); diffuse ( N, 1, u, u0, visc, dt );
	SWAP ( v0, v ); diffuse ( N, 2, v, v0, visc, dt );
	project ( N, u, v, u0, v0 );
	SWAP ( u0, u ); SWAP ( v0, v );
	advect ( N, 1, u, u0, u0, v0, dt ); advect ( N, 2, v, v0, u0, v0, dt );
	project ( N, u, v, u0, v0 );
}

void add_objects ( std::vector<FixedObject*> objects ) {
    fixedObjects.clear();
    fixedObjects = objects;
}

