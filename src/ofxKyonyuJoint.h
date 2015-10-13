#pragma once

#include "ofxKyonyuPoint.h"
struct ofxKyonyuJoint
{
	ofxKyonyuPoint* mPoint;
	ofxKyonyuPoint* mTarget;
    float mSpring;//[N/mm]
    float mDamper;//[N/(mm/s)]
    float mNaturalLength;
	ofxKyonyuJoint(ofxKyonyuPoint* inPoint=NULL,ofxKyonyuPoint* inTarget=NULL)
	:mPoint(inPoint)
	,mTarget(inTarget)
	,mSpring(10.0f)
	,mDamper(0.01f)
	,mNaturalLength(0.0f)
	{
		resetNaturalLength();
	}
	
    void resetNaturalLength()
    {
        if(!(mPoint && mTarget))
        {
            return;
        }
        mNaturalLength = mPoint->mPosition.distance(mTarget->mPosition);
    }
    void updateForce()
    {
        if(!(mPoint && mTarget))
        {
            return;
        }
        //バネの力
		ofVec3f dx=(mTarget->mPosition)-(mPoint->mPosition);
		ofVec3f nx=dx.getNormalized();//単位ベクトル
		
        ofVec3f springForce = nx*((dx.length() - mNaturalLength) * mSpring);
        
        //ダンパの力
        ofVec3f dv = (mTarget->mVelocity)-(mPoint->mVelocity);
        ofVec3f damperForce = dv*mDamper;
        
        //合力
        ofVec3f totalForce = springForce+damperForce;
		mPoint->mForce+=totalForce;
        //逆の力をかける
		mTarget->mForce+=totalForce*-1;
        
    }
	
};