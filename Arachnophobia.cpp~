#include<stdio.h>
#include <iostream>
#include <cmath>
#include <GL/glut.h>
#include<vector>
#include<utility>
#include<time.h>
#include<stdlib.h>
#include<cstdlib>
#include <sys/types.h>
#include <unistd.h>
#include<signal.h>
#include <sys/prctl.h>
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alut.h>

using namespace std;

#define PI 3.141592653589f
#define DEG2RAD(deg) (deg * PI / 180)

// Function Declarations
void drawScene();
void draw_canon();
void update(int value);
void drawLaser(float x1,float y1,float x2,float y2,float theta);
void drawBall(float rad);
void drawBox(float len);
void drawLine(float len);
void drawArrow(float x,char color);
void drawverticalLine(float len);
void drawRectangle(float len);
void drawTriangle();
void initRendering();
void dragwithmouse(int x,int y);
void handleResize(int w, int h);
void drawSpider(float x,float y,int c);
void handleKeypress1(unsigned char key, int x, int y);
void handleKeypress2(int key, int x, int y);
void handleMouseclick(int button, int state, int x, int y);

// Global Variables
float ball_x = 1.0f;
float ball_y = 2.0f;
float ball_velx = 0.01f;
float ball_vely = 0.02f;
float ball_rad = 0.2f;
float box_len = 4.0f;
float tri_x = 0.0f;
float tri_y = 0.0f;
float r_x=-box_len/2+0.3f;
float r_y=-box_len/2+0.35f;
float g_x=box_len/2-0.3f;
float g_y=-box_len/2+0.35f;
float b_x=0.0f;
float b_y=-box_len/2+0.3f;
float b_theta=90.0f;
time_t curr_t;
time_t prev_t;
time_t prevs_t;
float maxs=0.01f;
float mins=0.01f;
time_t bcurr_t;
time_t bprev_t;
float theta = 0.0f;
int togcan=0;
int toggreen=0;
int togred=0;
int rotcan=0;
float mousex;
float mousey;
int finalscore=0;
int windowWidth;
int windowHeight;
int paused=0;
int gameover=0;
int sub=0;
float ti=100.0f;
char score[1000];
#define r 1
#define g 2
#define b 3
int cur=0;

typedef struct spiders {
    float x;
    float y;
    int c;
    float s;
    time_t t;
} spiders;
vector <spiders> spider;

typedef struct beams {
    float x1;
    float y1;
    float x2;
    float y2;
    int ref;
    float theta;
}beams;
vector <beams> laser;

//-----------sound related stuff------------------i

// Maximum data buffers we will need.
#define NUM_BUFFERS 3

// Maximum emissions we will need.
#define NUM_SOURCES 3

// These index the buffers and sources.
#define BATTLE      0
#define GUN1        1
#define GUN2        2

// Buffers hold sound data.
ALuint Buffers[NUM_BUFFERS];

// Sources are points of emitting sound.
ALuint Sources[NUM_SOURCES];

// Position of the source sounds.
ALfloat SourcesPos[NUM_SOURCES][3];

// Velocity of the source sounds.
ALfloat SourcesVel[NUM_SOURCES][3];


// Position of the listener.
ALfloat ListenerPos[] = { 0.0, 0.0, 0.0 };

// Velocity of the listener.
ALfloat ListenerVel[] = { 0.0, 0.0, 0.0 };

// Orientation of the listener. (first 3 elements are "at", second 3 are "up")
ALfloat ListenerOri[] = { 0.0, 0.0, -1.0, 0.0, 1.0, 0.0 };

ALboolean LoadALData()
{
    // Variables to load into.

    ALenum format;
    ALsizei size;
    ALvoid* data;
    ALsizei freq;
    ALboolean loop;

    // Load wav data into buffers.

    alGenBuffers(NUM_BUFFERS, Buffers);

    if (alGetError() != AL_NO_ERROR)
        return AL_FALSE;

    alutLoadWAVFile((ALbyte*)"background.wav", &format, &data, &size, &freq, &loop);
    alBufferData(Buffers[BATTLE], format, data, size, freq);
    alutUnloadWAV(format, data, size, freq);

    alutLoadWAVFile((ALbyte*)"laser.wav", &format, &data, &size, &freq, &loop);
    alBufferData(Buffers[GUN1], format, data, size, freq);
    alutUnloadWAV(format, data, size, freq);

    alutLoadWAVFile((ALbyte*)"2.wav", &format, &data, &size, &freq, &loop);
    alBufferData(Buffers[GUN2], format, data, size, freq);
    alutUnloadWAV(format, data, size, freq);

    // Bind buffers into audio sources.

    alGenSources(NUM_SOURCES, Sources);

    if (alGetError() != AL_NO_ERROR)
        return AL_FALSE;

    alSourcei (Sources[BATTLE], AL_BUFFER,   Buffers[BATTLE]  );
    alSourcef (Sources[BATTLE], AL_PITCH,    1.0              );
    alSourcef (Sources[BATTLE], AL_GAIN,     1.0              );
    alSourcefv(Sources[BATTLE], AL_POSITION, SourcesPos[BATTLE]);
    alSourcefv(Sources[BATTLE], AL_VELOCITY, SourcesVel[BATTLE]);
    alSourcei (Sources[BATTLE], AL_LOOPING,  AL_TRUE          );

    alSourcei (Sources[GUN1], AL_BUFFER,   Buffers[GUN1]  );
    alSourcef (Sources[GUN1], AL_PITCH,    1.0            );
    alSourcef (Sources[GUN1], AL_GAIN,     1.0            );
    alSourcefv(Sources[GUN1], AL_POSITION, SourcesPos[GUN1]);
    alSourcefv(Sources[GUN1], AL_VELOCITY, SourcesVel[GUN1]);
    alSourcei (Sources[GUN1], AL_LOOPING,  AL_FALSE       );

    alSourcei (Sources[GUN2], AL_BUFFER,   Buffers[GUN2]  );
    alSourcef (Sources[GUN2], AL_PITCH,    1.0            );
    alSourcef (Sources[GUN2], AL_GAIN,     1.0            );
    alSourcefv(Sources[GUN2], AL_POSITION, SourcesPos[GUN2]);
    alSourcefv(Sources[GUN2], AL_VELOCITY, SourcesVel[GUN2]);
    alSourcei (Sources[GUN2], AL_LOOPING,  AL_FALSE       );

    // Do another error check and return.

    if( alGetError() != AL_NO_ERROR)
        return AL_FALSE;

    return AL_TRUE;
}


void SetListenerValues()
{
    alListenerfv(AL_POSITION,    ListenerPos);
    alListenerfv(AL_VELOCITY,    ListenerVel);
    alListenerfv(AL_ORIENTATION, ListenerOri);
}

void KillALData()
{
    alDeleteBuffers(NUM_BUFFERS, &Buffers[0]);
    alDeleteSources(NUM_SOURCES, &Sources[0]);
    alutExit();
}

int main(int argc, char **argv) {

    // Initialize OpenAL and clear the error bit.
    alutInit(NULL, 0);
    alGetError();

    // Load the wav data.
    if (LoadALData() == AL_FALSE)
        return 0;

    SetListenerValues();

    // Setup an exit procedure.
    atexit(KillALData);

    // Begin the battle sample to play.
    alSourcePlay(Sources[BATTLE]);

    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    int w = glutGet(GLUT_SCREEN_WIDTH);
    int h = glutGet(GLUT_SCREEN_HEIGHT);
    int windowWidth = w * 2 / 3;
    int windowHeight = h * 2 / 3;
    srand (static_cast <unsigned> (time(0)));
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition((w - windowWidth) / 2, (h - windowHeight) / 2);

    glutCreateWindow("Arachnophobia");  // Setup the window
    initRendering();

    // Register callbacks
    //glutMotionFunc(dragwithmouse);
    glutMotionFunc(dragwithmouse);
    glutDisplayFunc(drawScene);
    glutIdleFunc(drawScene);
    glutKeyboardFunc(handleKeypress1);
    glutSpecialFunc(handleKeypress2);
    glutMouseFunc(handleMouseclick);
    glutReshapeFunc(handleResize);
    glutTimerFunc(10, update, 0);
    glutMainLoop();
    return 0;
}


void printscore(float x, float y, string String)
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, windowWidth, 0,windowHeight, -1.0f, 1.0f);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glPushAttrib(GL_DEPTH_TEST);
    glDisable(GL_DEPTH_TEST);
    glRasterPos2f(x,y);
    for (size_t i=0; i<String.size(); i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, String[i]);
    }
    glPopAttrib();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// Function to draw objects on the screen
void drawScene() {
    if(paused==0 )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glPushMatrix();
        if(gameover==1)
        {
            sprintf(score,"Game Over");
            printscore(.0f,.0f,string(score));
            for(size_t i=0;i<spider.size();i++)
            {
                if(spider[i].y<-1.7f)
                    sub+=5;
            }
            sprintf(score,"FinalScore: %d",finalscore-sub);
            printscore(.0f,.1f,string(score));
            sub=0;
            glPopMatrix();
            glutSwapBuffers();
            return;
        }
        // Draw Box
        glTranslatef(0.0f, 0.0f, -5.0f);
        glColor3f(0.0f, 1.0f, 1.0f);
        drawBox(box_len);

        //Draw line
        glPushMatrix();
        glTranslatef(0.0f,-box_len*2/5,0.0f);
        glColor3f(0.0f,1.0f,1.0f);
        drawLine(box_len);
        glPopMatrix();

        //Draw rectangle
        glPushMatrix();
        glTranslatef(g_x,g_y,0.0f);
        glColor3f(0.0f,1.0f,0.0f);
        drawRectangle(0.4f);
        glPopMatrix();

        glPushMatrix();
        glTranslatef(r_x,r_y,0.0f);
        glColor3f(1.0f,0.0f,0.0f);
        drawRectangle(0.4f);
        glPopMatrix();

        if(cur==b||togcan==1||rotcan==1)
        drawArrow(b_x,'b');
        else if(cur==r||togred==1)
        drawArrow(r_x,'r');
        else if(cur==g||toggreen==1)
        drawArrow(g_x,'g');
        //Draw Canon
        glPushMatrix();
        glTranslatef(b_x,b_y,0.0f);
        glRotatef(b_theta-90.0f, 0.0f, 0.0f, 1.0f);
        glColor3f(0.0f,0.0f,1.0f);
        draw_canon();
        glPopMatrix();

        // Draw laser
        for(size_t i=0;i<laser.size();i++)
            drawLaser(laser[i].x1,laser[i].y1,laser[i].x2,laser[i].y2,laser[i].theta);

        time(&curr_t);
        if(difftime(curr_t,prevs_t)>=ti)
        {
            maxs+=0.02f;
            mins+=0.01f;
            prevs_t=curr_t;
            ti=ti/2.0;
        }
        if(difftime(curr_t,prev_t)>=ti*3.0/100.0f)
        {
            spiders s;
            s.x=(-1.8f + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(3.6f))));
            s.y=2.0f;
            s.c=rand()%3;
            s.s=(mins + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxs-mins))));
            prev_t=curr_t;
            spider.push_back(s);
        }
        for(size_t i=0;i<spider.size();i++)
            drawSpider(spider[i].x,spider[i].y,spider[i].c);
        for(size_t i=0;i<spider.size();i++)
        {
            if(spider[i].y<-1.7f)
                sub+=5;
        }
        sprintf(score,"Score: %d",finalscore-sub);
        printscore(.3f,.9f,string(score));
        sub=0;
        glPopMatrix();
        glutSwapBuffers();
    }
}

double distance(double x1, double y1, double x2, double y2)
{
    double x = x1 - x2;
    double y = y1 - y2;
    double dist;

    dist = pow(x,2)+pow(y,2);           //calculating distance by euclidean formula
    dist = sqrt(dist);                  //sqrt is function in math.h

    return dist;
}
void update(int value) {
    if(paused==0 && gameover==0)
    {
        for(size_t i=0;i<laser.size();i++)
        {
            if(laser[i].y2>=box_len/2 || laser[i].y2<=-box_len/2 || laser[i].x2>=box_len/2 || laser[i].x2<=-box_len/2)
            {
                if(laser[i].x2>=box_len/2 && laser[i].ref==0)
                {
                    beams l;
                    l.x1=l.x2=box_len/2;
                    l.y1=l.y2=laser[i].y2;
                    l.theta=laser[i].theta+90.0f;
                    l.x2+=0.005f*cos(DEG2RAD(l.theta));
                    l.y2+=0.005f*sin(DEG2RAD(l.theta));
                    l.ref=0;
                    laser[i].ref=1;
                    laser.push_back(l);
                }
                if(laser[i].x2<=-box_len/2 && laser[i].ref==0)
                {
                    beams l;
                    l.x1=l.x2=-box_len/2;
                    l.y1=l.y2=laser[i].y2;
                    l.theta=laser[i].theta-90.0f;
                    l.x2+=0.005f*cos(DEG2RAD(l.theta));
                    l.y2+=0.005f*sin(DEG2RAD(l.theta));
                    l.ref=0;
                    laser[i].ref=1;
                    laser.push_back(l);
                }
                laser[i].x1+=0.03f*cos(DEG2RAD(laser[i].theta));
                laser[i].y1+=0.03f*sin(DEG2RAD(laser[i].theta));
            }
            else
            {
                laser[i].x2+=0.03f*cos(DEG2RAD(laser[i].theta));
                laser[i].y2+=0.03f*sin(DEG2RAD(laser[i].theta));
                if(distance(laser[i].x1,laser[i].y1,laser[i].x2,laser[i].y2)>=0.5f)
                {
                    laser[i].x1+=0.03f*cos(DEG2RAD(laser[i].theta));
                    laser[i].y1+=0.03f*sin(DEG2RAD(laser[i].theta));
                }
            }
            if(laser[i].x1>box_len/2 || laser[i].x1<-box_len/2 || laser[i].y1>box_len/2 || laser[i].y1<-box_len/2)
                laser.erase(laser.begin()+i);
        }
        for(size_t i=0;i<spider.size();i++)
            if(spider[i].y>=-1.7f)
                spider[i].y-=spider[i].s;
        for(size_t i=0;i<laser.size();i++)
        {
            int flag=0;
            for(size_t j=0;j<spider.size();j++)
            {
                if(distance(spider[j].x,spider[j].y,laser[i].x2,laser[i].y2)<0.35f && spider[j].y>(-box_len/2+0.3f))
                {
                    flag=1;
                    if(spider[j].c==2)
                        finalscore++;
                    alSourcefv(Sources[2], AL_POSITION, SourcesPos[2]);

                    alSourcePlay(Sources[2]);
                    spider.erase(spider.begin()+j);
                }
            }
            if(flag)
                laser.erase(laser.begin()+i);
        }
        for(size_t i=0;i<spider.size();i++)
        {
            if(abs(b_x-spider[i].x)<=0.2f && spider[i].y<=b_y+0.6f && spider[i].y>-1.7f)
            {
                gameover=1;
                break;
		spider.erase(spider.begin()+i);
            }
            else if(abs(r_x-spider[i].x)<=0.2f && spider[i].y<=r_y+0.5f && spider[i].y>-1.7f)
            {
                if(spider[i].c==0)
                    finalscore++;
                else
                    finalscore--;
                spider.erase(spider.begin()+i);
            }
            else if(abs(g_x-spider[i].x)<=0.2f && spider[i].y<=g_y+0.5f && spider[i].y>-1.7f)
            {
                if(spider[i].c==1)
                    finalscore++;
                else
                    finalscore--;
                spider.erase(spider.begin()+i);
            }
        }
    }
    glutTimerFunc(10, update, 0);
}

void drawSpider(float x,float y,int c) {

    glPushMatrix();
    if(c==0)
        glColor3f(1.0f,0.0f,0.0f);
    else if(c==1)
        glColor3f(0.0f,1.0f,0.0f);
    else if(c==2)
        glColor3f(0.0f,0.0f,0.0f);
    glTranslatef(x,y,0.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_QUADS);
    glVertex3f(-0.055f,-0.08f,0.0f);
    glVertex3f(0.055f,-0.08f,0.0f);
    glVertex3f(0.055f,0.08f,0.0f);
    glVertex3f(-0.055f,0.08f,0.0f);
    glEnd();
    glPushMatrix();
    glTranslatef(0.0f,0.025f,0.0f);
    drawLine(0.3f);
    glPushMatrix();
    glTranslatef(0.15f,0.0f,0.0f);
    glRotatef(30.0f,0.0f,0.0f,1.0f);
    glTranslatef(0.05f,0.0f,0.0f);
    drawLine(0.1f);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(-0.15f,0.0f,0.0f);
    glRotatef(-30.0f,0.0f,0.0f,1.0f);
    glTranslatef(-0.05f,0.0f,0.0f);
    drawLine(0.1f);
    glPopMatrix();
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0.0f,-0.025f,0.0f);
    drawLine(0.3f);
    glPushMatrix();
    glTranslatef(0.15f,0.0f,0.0f);
    glRotatef(-30.0f,0.0f,0.0f,1.0f);
    glTranslatef(0.05f,0.0f,0.0f);
    drawLine(0.1f);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(-0.15f,0.0f,0.0f);
    glRotatef(30.0f,0.0f,0.0f,1.0f);
    glTranslatef(-0.05f,0.0f,0.0f);
    drawLine(0.1f);
    glPopMatrix();
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0.0f,0.08f,0.0f);
    glPushMatrix();
    glRotatef(30.0f,0.0f,0.0f,1.0f);
    glTranslatef(0.1f,0.0f,0.0f);
    drawLine(0.2f);
    glTranslatef(0.1f,0.0f,0.0f);
    glRotatef(60.0f,0.0f,0.0f,1.0f);
    glTranslatef(0.05f,0.00f,0.0f);
    drawLine(0.1f);
    glPopMatrix();
    glPushMatrix();
    glRotatef(-30.0f,0.0f,0.0f,1.0f);
    glTranslatef(-0.1f,0.0f,0.0f);
    drawLine(0.2f);
    glTranslatef(-0.1f,0.0f,0.0f);
    glRotatef(120.0f,0.0f,0.0f,1.0f);
    glTranslatef(0.05f,0.00f,0.0f);
    drawLine(0.1f);
    glPopMatrix();
    glBegin(GL_TRIANGLE_FAN);
    for(int i=0 ; i<360 ; i++) {
        glVertex2f(0.055f * cos(DEG2RAD(i)), 0.055f * sin(DEG2RAD(i)));
    }
    glEnd();
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0.0f,-0.1f,0.0f);
    glPushMatrix();
    glRotatef(-30.0f,0.0f,0.0f,1.0f);
    glTranslatef(0.1f,0.0f,0.0f);
    drawLine(0.2f);
    glTranslatef(0.1f,0.0f,0.0f);
    glRotatef(-60.0f,0.0f,0.0f,1.0f);
    glTranslatef(0.05f,0.00f,0.0f);
    drawLine(0.1f);
    glPopMatrix();
    glPushMatrix();
    glRotatef(30.0f,0.0f,0.0f,1.0f);
    glTranslatef(-0.1f,0.0f,0.0f);
    drawLine(0.2f);
    glTranslatef(-0.1f,0.0f,0.0f);
    glRotatef(-120.0f,0.0f,0.0f,1.0f);
    glTranslatef(0.05f,0.00f,0.0f);
    drawLine(0.1f);
    glPopMatrix();
    glBegin(GL_TRIANGLE_FAN);
    for(int i=0 ; i<360 ; i++) {
        glVertex2f(0.055f * cos(DEG2RAD(i)), 0.055f * sin(DEG2RAD(i)));
    }
    glEnd();
    glPopMatrix();

    glPopMatrix();

}

void drawLaser(float x1,float y1,float x2,float y2,float theta){
    glPushMatrix();
    //glTranslatef(x, y, 0.0f);
    // glRotatef(theta+90, 0.0f, 0.0f, 1.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    // drawLine(0.5f);
    //drawBall(ball_rad);
    glBegin(GL_LINES);
    glVertex2f(x1,y1);
    glVertex2f(x2,y2);
    glEnd();
    glPopMatrix();
}

void drawBox(float len) {

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_QUADS);
    glVertex2f(-len/2 , -len / 2);
    glVertex2f(len/2 , -len / 2);
    glVertex2f(len/2 , len / 2);
    glVertex2f(-len/2, len / 2);
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void drawLine(float len){
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glVertex2f(-len/2,0.0f);
    glVertex2f(len/2,0.0f);
    glEnd();
}
void drawverticalLine(float len){
    glBegin(GL_LINES);
    glVertex2f(0.0f,-len/2);
    glVertex2f(0.0f,len/2);
    glEnd();
}

void drawRectangle(float len){
    glPushMatrix();
    glTranslatef(0.0f,-len/2,0.0f);
    glBegin(GL_TRIANGLE_FAN);
    for(int i=180 ; i<360 ; i++) {
        glVertex2f(len/2 * cos(DEG2RAD(i)), len/4 * sin(DEG2RAD(i)));
    }
    //glPopMatrix();
    glEnd();
    glTranslatef(0.0f,len/2,0.0f);
    glBegin(GL_QUADS);
    glVertex2f(-len/2,-len/2);
    glVertex2f(len/2,-len/2);
    glVertex2f(len/2,len/2);
    glVertex2f(-len/2,len/2);
    glEnd();
    glTranslatef(0.0f,len/2,0.0f);
    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_TRIANGLE_FAN);
    for(int i=0 ; i<360 ; i++) {
        glVertex2f(len/2 * cos(DEG2RAD(i)), len/4 * sin(DEG2RAD(i)));
    }
    glEnd();
    glPopMatrix();
}

void draw_canon()
{
    float len=0.3f;
    glBegin(GL_QUADS);
    glVertex2f(-len/2,-len/2);
    glVertex2f(len/2,-len/2);
    glVertex2f(len/2,len/2);
    glVertex2f(-len/2,len/2);
    glPushMatrix();
    glTranslatef(0.0f,0.2f,0.0f);
    float x=0.05f;
    glVertex2f(-x/2,0.0f);
    glVertex2f(x/2,0.0f);
    glVertex2f(x/2,0.4f);
    glVertex2f(-x/2,0.4f);
    glPopMatrix();
    glEnd();
    len=0.4f;
    /*glPushMatrix();
    glTranslatef(-len/2,0.0f,0.0f);
    x=0.5f;
    glBegin(GL_QUADS);
    //glVertex2f(-len/2,-len/2);
    glVertex2f(0.0f,-x/2);
    glVertex2f(0.05f,-x/2);
    glVertex2f(0.05f,x/2);
    glVertex2f(0.0f,x/2);
    glPopMatrix();
    glEnd();*/
    glLineWidth(5.0f);
    glBegin(GL_LINES);
    glVertex2f(-len/2,0.0f);
    glVertex2f(len/2,0.0f);
    glLineWidth(3.0f);
    glEnd();
    glLineWidth(5.0f);
    glBegin(GL_LINES);
    glVertex2f(0.2f,-len/2);
    glVertex2f(0.2f,len/2);
    glLineWidth(3.0f);
    glEnd();
    glLineWidth(5.0f);
    glBegin(GL_LINES);
    glVertex2f(-0.2f,-len/2);
    glVertex2f(-0.2f,len/2);
    glLineWidth(3.0f);
    glEnd();
}


void drawBall(float rad) {

    glBegin(GL_TRIANGLE_FAN);
    for(int i=0 ; i<360 ; i++) {
        glVertex2f(rad * cos(DEG2RAD(i)), rad * sin(DEG2RAD(i)));
    }
    glEnd();
}

void drawTriangle() {

    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(-2.0f, -2.0f, 0.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, -1.0f, 0.0f);
    glEnd();
}

// Initializing some openGL 3D rendering options
void initRendering() {

    glEnable(GL_DEPTH_TEST);        // Enable objects to be drawn ahead/behind one another
    glEnable(GL_COLOR_MATERIAL);    // Enable coloring
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);   // Setting a background color
}

// Function called when the window is resized
void handleResize(int w, int h) {

    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)w / (float)h, 0.1f, 200.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void handleKeypress1(unsigned char key, int x, int y) {

    if (key == 27) {
        exit(0);
    }// escape key is pressed
    if(gameover==1)
        return;
    else if(key==80 || key==112)
    {
        if(paused==0)
            paused=1;
        else
            paused=0;
    }
    if(paused==1)
        return;
    else if(key==98 )
    {
        cur=b;
    }
    else if(key==103)
        cur=g;
    else if(key==114)
        cur=r;
    else if(key==32)
    {
        time(&bcurr_t);
        if(difftime(bcurr_t,bprev_t)>=1.0)
        {
            alSourcefv(Sources[1], AL_POSITION, SourcesPos[1]);

            alSourcePlay(Sources[1]);
            beams l;
            l.x1=l.x2=b_x;
            l.y1=l.y2=-box_len/2+0.3f;
            l.theta=b_theta;
            l.ref=0;
            laser.push_back(l);
            bprev_t=bcurr_t;
        }
    }
}

void handleKeypress2(int key, int x, int y) {

    if(paused==1 && gameover==1)
        return;
    if (key == GLUT_KEY_LEFT)
    {
        if(cur==r)
        {
            r_x-=0.1;
            int flag=0;
            if(r_x-0.1 < -box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,spider[i].y,r_x,r_y)<0.4f && spider[i].x<=r_x)
                    flag=1;
            }
            if(flag==1)
                r_x+=0.1;
        }
        else if(cur==b)
        {
            b_x-=0.1;
            int flag=0;
            if(b_x+0.1>box_len/2 || b_x-0.1<-box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,spider[i].y,b_x,b_y)<0.4f && spider[i].x<=b_x)
                    flag=1;
            }
            if(flag==1)
                b_x+=0.1;
        }
        else if(cur==g)
        {
            int flag=0;
            g_x-=0.1;
            if(g_x+0.1>box_len/2 || g_x-0.1<-box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,spider[i].y,g_x,g_y)<0.4f && spider[i].x<=g_x)
                    flag=1;
            }
            if(flag==1)
                g_x+=0.1;
        }
    }
    if (key == GLUT_KEY_RIGHT)
    {
        if(cur==r)
        {
            int flag=0;
            r_x+=0.1;
            if(r_x+0.1>box_len/2 || r_x-0.1<-box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,spider[i].y,r_x,r_y)<0.4f && spider[i].x>=r_x)
                    flag=1;
            }
            if(flag==1)
                r_x-=0.1;
        }
        else if(cur==b)
        {
            int flag=0;
            b_x+=0.1;
            if(b_x+0.1>box_len/2 || b_x-0.1<-box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,spider[i].y,b_x,b_y)<0.4f && spider[i].x>=b_x)
                    flag=1;
            }
            if(flag==1)
                b_x-=0.1;
        }
        else if(cur==g)
        {
            int flag=0;
            g_x+=0.1;
            if(g_x+0.1>box_len/2 || g_x-0.1<-box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,spider[i].y,g_x,g_y)<0.4f && spider[i].x>=g_x)
                    flag=1;
            }
            if(flag==1)
                g_x-=0.1;
        }
    }
    if(key == GLUT_KEY_UP && b_theta<150.0f)
        b_theta += 15;
    if(key == GLUT_KEY_DOWN && b_theta>30.0f)
        b_theta -= 15;
}

void handleMouseclick(int button, int state, int x, int y) {
    if(paused==1 && gameover==1)
        return;
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );

    winX = (float)x;
    winY = (float)viewport[3] - (float)y;

    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );

    gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);


    mousex=posX*50.45;
    mousey=posY*50.45;


    if (state == GLUT_DOWN)
    {
        if (button == GLUT_LEFT_BUTTON)
        {
            if(distance(mousex,mousey,b_x,b_y)<0.2f)
            {
                togcan=1;
                toggreen=0;
                togred=0;
                rotcan=0;

            }
            else if(distance(mousex,mousey,g_x,g_y)<0.2f)
            {
                togcan=0;
                toggreen=1;
                togred=0;
                rotcan=0;
            }
            else if(distance(mousex,mousey,r_x,r_y)<0.2f)
            {
                togcan=0;
                toggreen=0;
                togred=1;
                rotcan=0;
            }
            else
            {
                togcan=0;
                toggreen=0;
                togred=0;
                rotcan=0;
            }
        }
        if(button == GLUT_RIGHT_BUTTON)
        {
            togcan=0;
            toggreen=0;
            togred=0;
            if(distance(mousex,mousey,b_x,b_y)<0.4f)
            {
                rotcan=1;
            }
            else
                rotcan=0;
        }
    }
    glutPostRedisplay();
}

void dragwithmouse(int x,int y)
{
    if(paused==1 && gameover==1)
        return;
    float move;
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );

    winX = (float)x;
    winY = (float)viewport[3] - (float)y;

    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );

    gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);


    mousex=posX*50.45;
    mousey=posY*50.45;
    if(togred)
    {
        move=(mousex-r_x);
        if(move>0 && distance(mousex,mousey,r_x,r_y)<0.4f)
        {
            r_x+=move;
            int flag=0;
            if(r_x+0.1f>box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,0,r_x,0)<0.4f && spider[i].x>=r_x && spider[i].y<=-1.7f)
                {
                    flag=1;
                    break;
                }
            }
            if(flag==1)
                r_x-=move;
        }
        else if(move<0 && distance(mousex,mousey,r_x,r_y)<0.4f)
        {
            r_x+=move;
            int flag=0;
            if(r_x-0.1<-box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,spider[i].y,r_x,r_y)<0.4f && spider[i].x<=r_x&& spider[i].y<=-1.7f)
                    flag=1;
            }
            if(flag==1)
                r_x-=move;
        }
    }
    else if(toggreen)
    {
        move=(mousex-g_x);
        if(move>0 && distance(mousex,mousey,g_x,g_y)<0.4f)
        {
            g_x+=move;
            int flag=0;
            if(g_x+0.1>box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,spider[i].y,g_x,g_y)<0.4f && spider[i].x>=g_x&& spider[i].y<=-1.7f)
                    flag=1;
            }
            if(flag==1)
                g_x-=move;
        }
        else if(move<0 && distance(mousex,mousey,g_x,g_y)<0.4f)
        {
            g_x+=move;
            int flag=0;
            if(g_x-0.1<-box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,spider[i].y,g_x,g_y)<0.4f && spider[i].x<=g_x&& spider[i].y<=-1.7f)
                    flag=1;
            }
            if(flag==1)
                g_x-=(move);
        }
    }
    else if(togcan)
    {
        int x=1;
        move=(mousex-b_x);
        if(move>0 && distance(mousex,mousey,b_x,b_y)<0.4f)
        {
            b_x+=(move);
            int flag=0;
            if(b_x+0.1>box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,spider[i].y,b_x,b_y)<0.4f && spider[i].x>=b_x&& spider[i].y<=-1.7f)
                    flag=1;
            }
            if(flag==1)
                b_x-=(move);
        }
        else if(move<0 && distance(mousex,mousey,b_x,b_y)<0.4f)
        {
            b_x+=(move);
            int flag=0;
            if(b_x-0.1<-box_len/2 )
                flag=1;
            for(size_t i=0;i<spider.size();i++)
            {
                if(distance(spider[i].x,spider[i].y,b_x,b_y)<0.4f && spider[i].x<=b_x&& spider[i].y<=-1.7f)
                    flag=1;
            }
            if(flag==1)
                b_x-=(move);
        }
    }
    if(rotcan && distance(mousex,mousey,b_x,b_y)<0.8)
    {
        float hy = distance(b_x,b_y,mousex,mousey);;
        float base = mousex-b_x;
        float c1 = base/hy;
        float ang = acos(c1) * 180/(3.142);
        if(ang>b_theta)
        {
            if(b_theta + 1.875<150.0f)
                b_theta += 1.875;
        }
        else if(ang<b_theta)
        {

            if(b_theta - 1.875>30.0f)
                b_theta -=1.875;
        }
    }

}
void drawArrow(float x,char color) {

    glPushMatrix();
    glTranslatef(x,-4.1f,-5.0f);
    if(color=='g')
        glColor3f(0.0,1.0,0.0);
    else if(color=='r')
        glColor3f(1.0,0.0,0.0);
    else
        glColor3f(0.0f,0.0,1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_TRIANGLES);
    glVertex3f(x,0.2f,0.0f);
    glVertex3f(x-0.25f,0.0f,0.0f);
    glVertex3f(x+0.25f,0.0f,0.0f);
    glEnd();
    glBegin(GL_QUADS);
    glVertex3f(x-0.15f,0.0f,0.0f);
    glVertex3f(x-0.15f,-0.35f,0.0f);
    glVertex3f(x+0.15f,-0.35f,0.0f);
    glVertex3f(x+0.15f,0.0f,0.0f);
    glEnd();
    glPopMatrix();

}

