
#include "ofxKyonyuPoint.h"

void ofxKyonyuPoint::updatePosition(float inDt)
{
    if (mIsDragging || mIsPinned)
    {
        mVelocity=ofVec3f();
    }
    else
    {
        ofVec3f a = mForce/mMass;
        a.y -= GRAVITY_A;
        if (0)
        {
            mVelocity+=a*inDt;
            mPosition+=mVelocity*inDt;
        }
        else
        {
            ofVec3f k;
            ofVec3f l;
            rungeKutta(a, mVelocity, mPosition,inDt, k, l);
            mPosition+=k;
            mVelocity+=l;
        }
    }
    mForce=ofVec3f();
}
