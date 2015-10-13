#pragma once
#include"ofxKyonyuOp.h"
class ofxKyonyuPairOp
{
    static const int OP_QTY=2;
    ofxKyonyuOp mOpList[OP_QTY];
    ofVec3f mOpOffsetSide;
    ofVec3f mOpOffsetFront;
    ofMatrix4x4 mOpRot;
    ofTexture mUnknownTexture;
    ofTexture mTextures[OP_QTY];
    bool mHasTexture;
public:
    ofxKyonyuPairOp(const ofTexture& inUnknownTexture=ofTexture())
    :mOpOffsetSide(75,0,0)
    ,mOpOffsetFront(0,100,50)
    ,mOpRot(ofMatrix4x4::newRotationMatrix(0/*PI*/, 0, 1, 0))
    ,mUnknownTexture(inUnknownTexture)
    ,mHasTexture(false)
    {
        clearTextures();
    }
    ~ofxKyonyuPairOp()
    {
        
    }
    std::pair<ofVec3f,ofVec3f> getBound(size_t inIndex)const;
    void update(float inDt);
	void setPinnedMatrix(ofMatrix4x4 inPinnedMatrix);
    void draw();
    void setTextures(ofTexture inTextures[])
    {
        for(int i=0;i<OP_QTY;++i){
            mTextures[i]=inTextures[i];
        }
        mHasTexture=true;
    }
    void clearTextures(){
        for(int i=0;i<OP_QTY;++i){
            mTextures[i]=mUnknownTexture;
        }
        mHasTexture=false;
    }
    void addTouching(const ofVec3f& inPosition,float inRadius);
    void clearTouching();
    
    //valid after update
    bool isTouched()const;
    bool hasTexture()const;
    
private:
    ofVec3f getOffset(size_t inIndex)const;
};