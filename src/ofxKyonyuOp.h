#pragma once

#include"ofxKyonyuMesh.h"
#include"ofxKyonyuJoint.h"
#include<cmath>//TODO: MoveTo ofxKyonyuOp.cpp

class ofxKyonyuOp:public ofxKyonyuMesh
{
public:
	static const int COLS=12;
	static const int ROWS=6;
    static const float RADIUS;// = 90;//[mm]
    static const float WARP_LENGTH;// = 100;//[mm]
private:
	std::vector<ofxKyonyuJoint> mJointList;
	std::vector<ofxKyonyuPoint> mPointList;
	ofMatrix4x4 mPinnedMatrix;
    std::vector<std::pair<ofVec3f,float> > mTouchingList;
    bool mIsTouched;
    
	inline size_t getOpCount()
	{
        return COLS * (ROWS - 1) + 1+1;//周囲＋極＋中心
	}
    inline size_t getOpIndex(int inY,int inX)
    {
        if (inY >= ROWS-1)
        {
            return inY * COLS;
        }
        else
        {
            return inY * COLS + (COLS + inX) % COLS;
        }
    }
    void putPointAndJoint();
	inline void transfer()
	{
		mVertexList.resize(mPointList.size());
        mTexCoordList.resize(mPointList.size());
		for(size_t i=0;i<mPointList.size();++i)
		{
			mVertexList[i]=mPointList[i].mPosition;
			mTexCoordList[i]=mPointList[i].mTexCoord;
		}
	}
public:
	ofxKyonyuOp()
	:ofxKyonyuMesh()
	,mJointList()
	,mPointList()
	,mPinnedMatrix()
    ,mIsTouched(false)
	{
		putPointAndJoint();
//		for(size_t i=0;i<m_PointList.size();++i)
//		{
//			m_PointList[i]+=inTranslate;
//		}
	}
    void update(float inDt);
	void setPinnedMatrix(ofMatrix4x4 inPinnedMatrix);
    std::pair<ofVec3f,ofVec3f> getBound()const;
    void touch(const ofVec3f& inPosition,float inRadius);
    void addTouching(const ofVec3f& inPosition,float inRadius);
    void clearTouching()
    {
        mTouchingList.clear();
        mIsTouched=false;
    }
    //valid after update
    bool isTouched()const
    {
        return mIsTouched;
    }
	
};



