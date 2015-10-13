#pragma once
#include "ofMain.h"

static const float GRAVITY_A = 9.8 * 1000;//[mm/(s*s)]
static const float AIR_FRICTION = 0.005;//[N/(mm/s)]

struct ofxKyonyuPoint
{
	ofVec3f mPosition;//[mm]
	ofVec3f mVelocity;//[mm/s]
	ofVec3f mForce;//[mm/(s*s)]
    ofVec2f mTexCoord;//uv
	float mMass;//[kg];
    bool mIsPinned;
    bool mIsDragging;
	
	ofxKyonyuPoint()
	:mPosition()
	,mVelocity()
	,mForce()
	,mMass(6.0/1000)
	,mIsPinned(false)
	,mIsDragging(false)
	{
	}
    void updateForce()
    {
		mForce+=mVelocity*(AIR_FRICTION*-1);
    }
    void updatePosition(float inDt);
    
    //オイラー法で暴走したので、ルンゲクッタ法という積分法を使う
    //http://www6.ocn.ne.jp/~simuphys/runge-kutta.html
    inline void rungeKutta(ofVec3f inA,ofVec3f inV,ofVec3f inX,float inDt,ofVec3f& outK,ofVec3f& outL)
    {
        ofVec3f x1 = inV*inDt;
        ofVec3f v1 = inA*inDt;
        ofVec3f x2 = (inV+(v1*0.5) )*inDt;
        ofVec3f v2 = inA*(inDt * 0.5);//あってる？　Aを求めなおす必要がある？
        ofVec3f x3 = (inV+v2*0.5)*inDt;
        ofVec3f v3 = inA*(inDt * 0.5);//あってる？　Aを求めなおす必要がある？
        ofVec3f x4 = (inV+v3)*inDt;
        ofVec3f v4 = inA*inDt;//あってる？　Aを求めなおす必要がある？
        outK=(x1+(x2*2)+(x3*2)+x4)/6;
        outL=(v1+(v2*2)+(v3*2)+v4)/6;
    }
};
