#include <GL/glut.h>

#include "KinectDisplay.h"

using namespace cv;
XnBool g_bDrawBackground = TRUE;
XnBool g_bDrawPixels = TRUE;
XnBool g_bDrawSkeleton = TRUE;
XnBool g_bPrintID = TRUE;
XnBool g_bPrintState = TRUE;

struct JointsDataFC//joints data for comparision
{
    XnSkeletonJointPosition  jointPPrev[15];
    XnUserID id;
    int JointsEqual;
};

vector<JointsDataFC> JointVec1;

#define MAX_DEPTH 10000
float g_pDepthHist[MAX_DEPTH];
cv::Size sizepic1(640,480);
cv::Mat rgb(sizepic1,CV_8UC3);// = latest_rgb_;

cv::Mat get_Joint_position()
{
    Mat temp;
    rgb.copyTo(temp);
    rgb.setTo(cv::Scalar(0,0,0));
    return temp;
}

unsigned int getClosestPowerOfTwo(unsigned int n)
{
    unsigned int m = 2;
    while(m < n) m<<=1;

    return m;
}
GLuint initTexture(void** buf, int& width, int& height)
{
    GLuint texID = 0;
    glGenTextures(1,&texID);

    width = getClosestPowerOfTwo(width);
    height = getClosestPowerOfTwo(height);
    *buf = new unsigned char[width*height*4];
    glBindTexture(GL_TEXTURE_2D,texID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    return texID;
}

GLfloat texcoords[8];
void DrawRectangle(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
    GLfloat verts[8] = {	topLeftX, topLeftY,
                            topLeftX, bottomRightY,
                            bottomRightX, bottomRightY,
                            bottomRightX, topLeftY
                       };
    glVertexPointer(2, GL_FLOAT, 0, verts);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

    //TODO: Maybe glFinish needed here instead - if there's some bad graphics crap
    glFlush();
}
void DrawTexture(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, 0, texcoords);

    DrawRectangle(topLeftX, topLeftY, bottomRightX, bottomRightY);

    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

XnFloat Colors[][3] =
{
    {0,1,1},
    {0,0,1},
    {0,1,0},
    {1,1,0},
    {1,0,0},
    {1,.5,0},
    {.5,1,0},
    {0,.5,1},
    {.5,0,1},
    {1,1,.5},
    {1,1,1}
};
XnUInt32 nColors = 10;

void glPrintString(void *font, char *str)
{
    int i,l = strlen(str);

    for(i=0; i<l; i++)
    {
        glutBitmapCharacter(font,*str++);
    }
}


void kinect_display_drawDepthMapGL(const xn::DepthMetaData& dmd,
                                   const xn::SceneMetaData& smd)
{
    static bool bInitialized = false;
    static GLuint depthTexID;
    static unsigned char* pDepthTexBuf;
    static int texWidth, texHeight;

    float topLeftX;
    float topLeftY;
    float bottomRightY;
    float bottomRightX;
    float texXpos;
    float texYpos;

    if(!bInitialized)
    {

        texWidth =  getClosestPowerOfTwo(dmd.XRes());
        texHeight = getClosestPowerOfTwo(dmd.YRes());

        printf("Initializing depth texture: width = %d, height = %d\n", texWidth, texHeight);
        depthTexID = initTexture((void**)&pDepthTexBuf,texWidth, texHeight) ;

        printf("Initialized depth texture: width = %d, height = %d\n", texWidth, texHeight);
        bInitialized = true;

        topLeftX = dmd.XRes();
        topLeftY = 0;
        bottomRightY = dmd.YRes();
        bottomRightX = 0;
        texXpos =(float)dmd.XRes()/texWidth;
        texYpos  =(float)dmd.YRes()/texHeight;

        memset(texcoords, 0, 8*sizeof(float));
        texcoords[0] = texXpos, texcoords[1] = texYpos, texcoords[2] = texXpos, texcoords[7] = texYpos;

    }
    unsigned int nValue = 0;
    unsigned int nHistValue = 0;
    unsigned int nIndex = 0;
    unsigned int nX = 0;
    unsigned int nY = 0;
    unsigned int nNumberOfPoints = 0;
    XnUInt16 g_nXRes = dmd.XRes();
    XnUInt16 g_nYRes = dmd.YRes();

    unsigned char* pDestImage = pDepthTexBuf;

    const XnDepthPixel* pDepth = dmd.Data();
    const XnLabel* pLabels = smd.Data();

    // Calculate the accumulative histogram
    memset(g_pDepthHist, 0, MAX_DEPTH*sizeof(float));
    for (nY=0; nY<g_nYRes; nY++)
    {
        for (nX=0; nX<g_nXRes; nX++)
        {
            nValue = *pDepth;

            if (nValue != 0)
            {
                g_pDepthHist[nValue]++;
                nNumberOfPoints++;
            }

            pDepth++;
        }
    }

    for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
    {
        g_pDepthHist[nIndex] += g_pDepthHist[nIndex-1];
    }
    if (nNumberOfPoints)
    {
        for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
        {
            g_pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (g_pDepthHist[nIndex] / nNumberOfPoints)));
        }
    }

    pDepth = dmd.Data();
    if (g_bDrawPixels)
    {
        XnUInt32 nIndex = 0;
        // Prepare the texture map
        for (nY=0; nY<g_nYRes; nY++)
        {
            for (nX=0; nX < g_nXRes; nX++, nIndex++)
            {

                pDestImage[0] = 0;
                pDestImage[1] = 0;
                pDestImage[2] = 0;
                if (g_bDrawBackground || *pLabels != 0)
                {
                    nValue = *pDepth;
                    XnLabel label = *pLabels;
                    XnUInt32 nColorID = label % nColors;
                    if (label == 0)
                    {
                        nColorID = nColors;
                    }
                    if (nValue != 0)
                    {
                        nHistValue = g_pDepthHist[nValue];

                        pDestImage[0] = nHistValue * Colors[nColorID][0];
                        pDestImage[1] = nHistValue * Colors[nColorID][1];
                        pDestImage[2] = nHistValue * Colors[nColorID][2];
                    }
                }

                pDepth++;
                pLabels++;
                pDestImage+=3;
            }

            pDestImage += (texWidth - g_nXRes) *3;
        }
    }
    else
    {
        xnOSMemSet(pDepthTexBuf, 0, 3*2*g_nXRes*g_nYRes);
    }

    glBindTexture(GL_TEXTURE_2D, depthTexID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, pDepthTexBuf);

    // Display the OpenGL texture map
    glColor4f(0.75,0.75,0.75,1);

    glEnable(GL_TEXTURE_2D);
    DrawTexture(dmd.XRes(),dmd.YRes(),0,0);
    glDisable(GL_TEXTURE_2D);
}


void DrawLimb(xn::UserGenerator& userGenerator,
              xn::DepthGenerator& depthGenerator,
              XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
{
    if (!userGenerator.GetSkeletonCap().IsTracking(player))
    {
        printf("not tracked!\n");
        return;
    }
    XnSkeletonJointPosition joint1, joint2;
    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

    if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
    {
        return;
    }

    XnPoint3D pt[2];
    pt[0] = joint1.position;
    pt[1] = joint2.position;
    depthGenerator.ConvertRealWorldToProjective(2, pt, pt);
    glVertex3i(pt[0].X, pt[0].Y, 0);
    glVertex3i(pt[1].X, pt[1].Y, 0);
    Point center( pt[0].X, pt[0].Y );
    ellipse( rgb, center, Size( 5, 5), 0, 0, 360, Scalar( 0, 0, 255 ), 2, 8, 0);
    Point center1( pt[1].X, pt[1].Y );
    ellipse( rgb, center1, Size( 5, 5), 0, 0, 360, Scalar( 0, 0, 255 ), 2, 8, 0);
    line(rgb,center,center1,Scalar( 255, 255, 0),2,8,0);
}

void kinect_display_drawSkeletonGL(xn::UserGenerator& userGenerator,
                                   xn::DepthGenerator& depthGenerator)
{
    char strLabel[50] = "";
    XnUserID aUsers[15];
    XnUInt16 nUsers = 15;
    userGenerator.GetUsers(aUsers, nUsers);
    for (int i = 0; i < nUsers; ++i)
    {
        if (g_bPrintID)
        {
            XnPoint3D com;
            userGenerator.GetCoM(aUsers[i], com);
            depthGenerator.ConvertRealWorldToProjective(1, &com, &com);

            xnOSMemSet(strLabel, 0, sizeof(strLabel));
            if (!g_bPrintState)
            {
                // Tracking
                sprintf(strLabel, "%d", aUsers[i]);
            }
            else if (userGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
            {
                // Tracking
                sprintf(strLabel, "%d - Tracking", aUsers[i]);
            }
            else if (userGenerator.GetSkeletonCap().IsCalibrating(aUsers[i]))
            {
                // Calibrating
                sprintf(strLabel, "%d - Calibrating...", aUsers[i]);
            }
            else
            {
                // Nothing
                sprintf(strLabel, "%d - Looking for pose", aUsers[i]);
            }


            glColor4f(1-Colors[i%nColors][0], 1-Colors[i%nColors][1], 1-Colors[i%nColors][2], 1);

            glRasterPos2i(com.X, com.Y);
            glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
        }

        if (g_bDrawSkeleton && userGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
        {
            glBegin(GL_LINES);
            glColor4f(1-Colors[aUsers[i]%nColors][0], 1-Colors[aUsers[i]%nColors][1], 1-Colors[aUsers[i]%nColors][2], 1);

            XnSkeletonJointPosition jointP[15];
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_HEAD, jointP[0]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_NECK, jointP[1]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_TORSO, jointP[2]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_SHOULDER, jointP[3]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_ELBOW, jointP[4]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_HAND, jointP[5]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_SHOULDER, jointP[6]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_ELBOW, jointP[7]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_HAND, jointP[8]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_HIP, jointP[9]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_KNEE, jointP[10]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_FOOT, jointP[11]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_HIP, jointP[12]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_KNEE, jointP[13]);
            userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_FOOT, jointP[14]);
            int jointCompare = 0;
            bool isIDInVec = false;
            std::vector<JointsDataFC>::iterator itTemp;
            for (std::vector<JointsDataFC>::iterator it = JointVec1.begin(); it != JointVec1.end(); ++it)
            {
                if(it->id == aUsers[i])
                {
                    itTemp = it;
                    isIDInVec = true;
                    for(int q = 0; q < 15;++q)
                    {
                        if(abs(jointP[q].position.X - it->jointPPrev[q].position.X)<0.5 && abs(jointP[q].position.Y - it->jointPPrev[q].position.Y)<0.5 && abs(jointP[q].position.Z - it->jointPPrev[q].position.Z)<0.5)
                        {
                            ++jointCompare;
                        }
                        it->jointPPrev[q].position = jointP[q].position;
                    }
                }
            }
            if(!isIDInVec)
            {
                JointsDataFC temp;
                temp.id = aUsers[i];
                for(int q = 0; q < 15;++q)
                {
                    temp.jointPPrev[q].position = jointP[q].position;
                }
                temp.JointsEqual = 0;
                JointVec1.push_back(temp);
                itTemp = JointVec1.end();
            }
            if(jointCompare == 15)
            {
                itTemp->JointsEqual++;
            }
            else
            {
                itTemp->JointsEqual=0;
            }
            if(itTemp->JointsEqual<5)
            {
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_HEAD, XN_SKEL_NECK);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_TORSO, XN_SKEL_LEFT_HIP);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT);
                DrawLimb(userGenerator, depthGenerator, aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP);
            }
        }
        bool CheckID = false;
        for (std::vector<JointsDataFC>::iterator it = JointVec1.begin() ; it != JointVec1.end(); ++it)
        {
            for (int i = 0; i < nUsers; ++i)
            {
                if(it->id == aUsers[i])
                    CheckID = true;
            }
            if(!CheckID)
            {
                JointVec1.erase(it);
                break;
            }
        }
        glEnd();
    }
}

