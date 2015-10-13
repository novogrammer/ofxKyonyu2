
#include "ofxKyonyuPairOp.h"

void ofxKyonyuPairOp::update(float inDt)
{
    for(int i=0;i<OP_QTY;++i){
        mOpList[i].update(inDt);
    }
}

ofVec3f ofxKyonyuPairOp::getOffset(size_t inIndex)const
{
    if(inIndex==0)
    {
        return mOpOffsetFront+mOpOffsetSide*-1;
    }
    else
    {
        return mOpOffsetFront+mOpOffsetSide*+1;
    }
}

std::pair<ofVec3f,ofVec3f> ofxKyonyuPairOp::getBound(size_t inIndex)const
{
    std::pair<ofVec3f,ofVec3f> bound=mOpList[inIndex].getBound();
    return std::make_pair(bound.first+getOffset(inIndex),bound.second+getOffset(inIndex));
}


void ofxKyonyuPairOp::setPinnedMatrix(ofMatrix4x4 inPinnedMatrix)
{
    for(int i=0;i<OP_QTY;++i){
        mOpList[i].setPinnedMatrix(ofMatrix4x4::newTranslationMatrix(getOffset(i))*mOpRot*inPinnedMatrix);
    }
}
void ofxKyonyuPairOp::draw()
{
    bool enabled=ofGetUsingNormalizedTexCoords();
    ofEnableNormalizedTexCoords();
    
    for(int i=0;i<OP_QTY;++i){
        mTextures[i].bind();
        mOpList[i].draw();
        mTextures[i].unbind();
        
    }
    
    if(!enabled){
        ofDisableNormalizedTexCoords();
    }
    
}

void ofxKyonyuPairOp::addTouching(const ofVec3f& inPosition,float inRadius)
{
    for(int i=0;i<OP_QTY;++i){
        mOpList[i].addTouching(inPosition,inRadius);
    }
}
void ofxKyonyuPairOp::clearTouching()
{
    for(int i=0;i<OP_QTY;++i){
        mOpList[i].clearTouching();
    }
}
bool ofxKyonyuPairOp::isTouched()const
{
    bool isTouched=false;
    for(int i=0;i<OP_QTY;++i){
        isTouched=isTouched||mOpList[i].isTouched();
    }
    return isTouched;
}

bool ofxKyonyuPairOp::hasTexture()const{
    return mHasTexture;
}



