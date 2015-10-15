#include "ofApp.h"


void ofApp::setupNI2(){
    mDevice.setup();
    if(mColorStream.setup(mDevice)){
        mColorStream.setSize(BUFFER_WIDTH, BUFFER_HEIGHT);
        mColorStream.setFps(FPS);
        mColorStream.start();
        cout << "mColorStream is OK" << endl;
    }
    
    if(mDepthStream.setup(mDevice)){
        mDepthStream.setSize(BUFFER_WIDTH, BUFFER_HEIGHT);
        mDepthStream.setFps(FPS);
        mDepthStream.start();
        float fovy=ofRadToDeg(mDepthStream.get().getVerticalFieldOfView());
        mCamera.setupPerspective(false,fovy,100,10000);//[mm]
        mCamera.setAspectRatio(ofGetWidth()/(float)ofGetHeight());
        cout << "mDepthStream is OK" << endl;
    }
    if (mUserTracker.setup(mDevice)){
        mUserTracker.setSkeletonSmoothingFactor(0.3f);
        ofAddListener(mUserTracker.newUser, this, &ofApp::onNewUser);
        ofAddListener(mUserTracker.lostUser, this, &ofApp::onLostUser);
        
        
        cout << "mUserTracker is OK" << endl;
    }
    
}
void ofApp::updateOp(){
    for(UserAndPairOfOpMap::iterator it=mUserAndPairOfOpMap.begin();it!=mUserAndPairOfOpMap.end();++it){
        it->second.mPairOfOp->clearTouching();
    }
    vector<nite::JointType> jointsForTouch;
    jointsForTouch.push_back(nite::JOINT_LEFT_ELBOW);
    jointsForTouch.push_back(nite::JOINT_RIGHT_ELBOW);
    jointsForTouch.push_back(nite::JOINT_LEFT_HAND);
    jointsForTouch.push_back(nite::JOINT_RIGHT_HAND);
    
    for(UserAndPairOfOpMap::iterator toucherIterator=mUserAndPairOfOpMap.begin();toucherIterator!=mUserAndPairOfOpMap.end();++toucherIterator){
        ofxNiTE2::User::Ref toucherUser=toucherIterator->second.mUser;
        for(int i=0;i<jointsForTouch.size();++i){
            nite::JointType jointForTouch=jointsForTouch[i];
            const nite::SkeletonJoint& joint=toucherUser->getJoint(jointForTouch).get();
            bool isConfidence=joint.getPositionConfidence()>CONFIDENCE_FACTOR;
            if(isConfidence){
                ofVec3f p=toPosition(joint);
                for(UserAndPairOfOpMap::iterator toucheeIterator=mUserAndPairOfOpMap.begin();toucheeIterator!=mUserAndPairOfOpMap.end();++toucheeIterator){
                    ofxKyonyuPairOpPtr op=toucheeIterator->second.mPairOfOp;
                    op->addTouching(p, KANSETSU_RADIUS);
                }
            }
        }
    }
    
    for(UserAndPairOfOpMap::iterator it=mUserAndPairOfOpMap.begin();it!=mUserAndPairOfOpMap.end();++it){
        ofxKyonyuPairOpPtr op=it->second.mPairOfOp;
        ofxNiTE2::User::Ref user=it->second.mUser;
        const nite::SkeletonJoint& joint=user->getJoint(nite::JOINT_TORSO).get();
        bool isConfidence=joint.getOrientationConfidence()>CONFIDENCE_FACTOR && joint.getPositionConfidence()>CONFIDENCE_FACTOR;
        if(isConfidence){
            ofMatrix4x4 m=toMatrix(joint);
            op->setPinnedMatrix(m);
        }
        op->update(INV_FPS);
        if(op->isTouched()){
            //TODO playSound
        }
    }
    
}


void ofApp::drawPointCloud(){
    mCamera.begin(ofRectangle(0,0,ofGetWidth(),ofGetHeight()));
    ofPushMatrix();
    if(mIsMirror){
        glScalef(-1, 1, 1);
    }
    
    ofPushStyle();
    ofEnablePointSprites();
    ofEnableDepthTest();
    ofDisableLighting();
    const openni::VideoStream& rawDepthStream=mDepthStream.get();
    const openni::VideoStream& rawColorStream=mColorStream.get();
    const ofShortPixels& depthPixels=mDepthStream.getPixelsRef();
    const ofPixels& colorPixels=mColorStream.getPixelsRef();
    
    mPointCloudShader.begin();
    float size=ofGetHeight()/BUFFER_HEIGHT;
    float sizePerZ=-tan(ofDegToRad(mCamera.getFov())/2)*2*size*mPointSizeFactor;
    mPointCloudShader.setUniform1f("SizePerZ", sizePerZ);
    
    mPointCloudMesh.clear();
    const unsigned short *depthPixelsBuffer=depthPixels.getPixels();
    const unsigned char *colorPixelsBuffer=colorPixels.getPixels();
    float fovy=mCamera.getFov();
    float fovx=fovy*mCamera.getAspectRatio();
    mTanY=tan(ofDegToRad(fovy*0.5));
    mTanX=tan(ofDegToRad(fovx*0.5));
    ofVec3f pos;
    ofFloatColor col;
    for(int y=0;y<BUFFER_HEIGHT;++y){
        for(int x=0;x<BUFFER_WIDTH;++x){
            openni::DepthPixel depth=depthPixelsBuffer[depthPixels.getPixelIndex(x, y)];
            const short DEPTH_MAX=5000;//[mm]
            if(depth>DEPTH_MAX || depth==0)
            {
                depth=DEPTH_MAX;
            }
            //depth=1000;
            int colX=0;
            int colY=0;
            colX=x;
            colY=y;
            int colorIndex=colorPixels.getPixelIndex(x, y);
            const float inv255=1.0/255;
            col.r=colorPixelsBuffer[colorIndex+0]*inv255;
            col.g=colorPixelsBuffer[colorIndex+1]*inv255;
            col.b=colorPixelsBuffer[colorIndex+2]*inv255;
            pos=toWorld(x, y, depth);
            //pos.x*=-1;
            mPointCloudMesh.addVertex(pos);
            mPointCloudMesh.addColor(col);
        }
    }
    mPointCloudMesh.drawVertices();
    
    mPointCloudShader.end();
    ofDisablePointSprites();
    ofPopStyle();
    ofPopMatrix();
    mCamera.end();
    
}

void ofApp::drawUsers(){
    mCamera.begin(ofRectangle(0,0,ofGetWidth(),ofGetHeight()));
    ofPushMatrix();
    if(mIsMirror){
        glScalef(-1, 1, 1);
    }
    ofPushStyle();
    ofSetColor(255);
    ofEnableDepthTest();
    ofEnableLighting();
    mLight.enable();
    mMaterial.begin();
    for(UserAndPairOfOpMap::iterator it=mUserAndPairOfOpMap.begin();it!=mUserAndPairOfOpMap.end();++it){
        ofxNiTE2::User::Ref user=it->second.mUser;
        ofxKyonyuPairOpPtr op=it->second.mPairOfOp;
        if(!op->hasTexture()){
            bool isValid=true;
            const int OP_QTY=2;
            const nite::SkeletonJoint& torsoJoint=user->getJoint(nite::JOINT_TORSO).get();
            bool isConfidence=torsoJoint.getOrientationConfidence()>CONFIDENCE_FACTOR_FOR_CAPTURE && torsoJoint.getPositionConfidence()>CONFIDENCE_FACTOR_FOR_CAPTURE;
            if(isConfidence){
                ofMatrix4x4 torsoMatrix=toMatrix(torsoJoint);
                
                vector<ofTexture> textures(OP_QTY,ofTexture());
                for(int i=0;i<OP_QTY;++i){
                    
                    ofVec3f to=torsoMatrix.getTranslation();
                    to=toPosition(torsoJoint);
                    std::pair<ofVec3f,ofVec3f> bound=op->getBound(i);
                    ofVec2f pos1=toDepth(to+bound.first);
                    ofVec2f pos2=toDepth(to+bound.second);
                    //cout << "1 " << pos1.x << " " << pos1.y << endl;
                    //cout << "2 " << pos2.x << " " << pos2.y << endl;
                    ofPixels pixels;
                    pixels.allocate(OP_TEXTURE_WIDTH, OP_TEXTURE_HEIGHT,OF_PIXELS_RGB );
                    for(int y=0;y<OP_TEXTURE_HEIGHT;++y){
                        for(int x=0;x<OP_TEXTURE_WIDTH;++x){
                            ofVec2f pos=ofVec2f(
                                ofMap(x, 0, OP_TEXTURE_WIDTH-1, pos2.x, pos1.x),
                                ofMap(y, 0, OP_TEXTURE_HEIGHT-1, pos2.y, pos1.y)
                            );
                            //pos.x=BUFFER_WIDTH-1-pos.x;
                            //pos.y=BUFFER_HEIGHT-1-pos.y;
                            ofColor col(0xff,0xff,0xff);
                            if(0<=pos.x&&pos.x<BUFFER_WIDTH&&
                               0<=pos.y&&pos.y<BUFFER_HEIGHT
                            ){
                                col=mColorImage.getColor((int)pos.x,(int)pos.y);
                            }
                            pixels.setColor(x, y, col);
                        }
                    }
                    
                    textures[i].loadData(pixels);
                }
                op->setTextures(&textures[0]);
            }else{
                //次のフレームでもう一度試す
            }
        }
        op->draw();
        
    }
#if 0
    ofSpherePrimitive sphere;
    sphere.set(100, 10);
    sphere.setPosition(0, 0, 1500);
    sphere.draw();
#endif
    mMaterial.end();
    mLight.disable();
    ofDisableLighting();
    ofPopStyle();
    ofPopMatrix();
    mCamera.end();
}
ofVec3f ofApp::toPosition(const nite::SkeletonJoint& joint){
    nite::Point3f position=joint.getPosition();
    return ofVec3f(position.x,position.y,-position.z);//DX to GL
}


ofMatrix4x4 ofApp::toMatrix(const nite::SkeletonJoint& joint){
    ofMatrix4x4 m;
    
    nite::Quaternion nq=joint.getOrientation();
    ofQuaternion q(-nq.x,-nq.y,nq.z,nq.w);//DX to GL
    
    m.rotate(q);
    
    nite::Point3f position=joint.getPosition();
    m.translate(position.x, position.y, -position.z);//DX to GL
    return m;
}


void ofApp::setup(){
    ofSetFrameRate(FPS);
    ofSetVerticalSync(false);
    ofDisableAntiAliasing();
    ofBackground(0);
    mCanDisplayDebug=true;
    mRotY=0;
    mPointCloudShader.load("shaders/pointcloud");
    mPointCloudMesh.enableColors();
    mPointSizeFactor=POINT_SIZE_FACTOR;
    mIsMirror=true;

    mLight.setPosition(ofVec3f(0,-10000,-10000));
    mLight.setDiffuseColor(ofFloatColor(0.9,0.9,0.9,1.0));
    mLight.setAmbientColor(ofFloatColor(0.5,0.5,0.5,1.0));
    mUnknownImage.loadImage("textures/unknown.png");
    mMaterial.setDiffuseColor(ofFloatColor(0.8,0.8,0.8,1.0));
    mMaterial.setAmbientColor(ofFloatColor(0.8,0.8,0.8,1.0));
    
    
    setupNI2();
    

}
void ofApp::exit(){
    //Device::exit has a problem around stream.
    //So,Needs Stream::exit before Device::exit.
    mUserTracker.exit();
    mDepthStream.exit();
    mColorStream.exit();
    mDevice.exit();

}

void ofApp::update(){
    mDevice.update();
    ofVec3f lookAt=ofVec3f(0,0,-1500);
    mCamera.setPosition(lookAt-ofMatrix4x4::newRotationMatrix(mRotY, ofVec3f(0,1,0))*lookAt);
    mCamera.lookAt(lookAt);
    
    updateOp();
    
    

}

void ofApp::draw(){
    const ofShortPixels& depthPixels=mDepthStream.getPixelsRef();
    if(0==depthPixels.size()){
        return;
    }
    const ofShortPixels& depthPixelsForDebug=mDepthStream.getPixelsRef(1000,4000);
    mDepthImage.setFromPixels(depthPixelsForDebug);
    const ofPixels& colorPixels=mColorStream.getPixelsRef();
    if(0==colorPixels.size()){
        return;
    }
    mColorImage.setFromPixels(colorPixels);
    
    if(mCanDisplayDebug){
        ofPushStyle();
        // draw depth
        ofSetColor(255);
        mDepthImage.draw(0, 0,BUFFER_WIDTH/2,BUFFER_HEIGHT/2);
        // draw color
        ofSetColor(255);
        mColorImage.draw(BUFFER_WIDTH/2, 0,BUFFER_WIDTH/2,BUFFER_HEIGHT/2);
        
        //debug string
        ofSetColor(255,0,255);
        
        const float STRING_HEIGHT=15;
        float x=10;
        float y=200;
        //draw fps
        string fps=ofToString(ofGetFrameRate(), 2);
        ofDrawBitmapString("FPS            :"+fps, x, y);
        y+=STRING_HEIGHT;
        
        //draw mPointSizeFactor
        string pointSizeFactor=ofToString(mPointSizeFactor,2);
        ofDrawBitmapString("PointSizeFactor:"+pointSizeFactor, x, y);
        y+=STRING_HEIGHT;
        
        //draw mIsMirror
        string isMirror=ofToString(mIsMirror);
        ofDrawBitmapString("IsMirror       :"+isMirror, x, y);
        y+=STRING_HEIGHT;
        
        y+=STRING_HEIGHT;
        
        ofDrawBitmapString("KEY ASSIGN", x, y);
        y+=STRING_HEIGHT;
        ofDrawBitmapString("1:Toggle Debug", x, y);
        y+=STRING_HEIGHT;
        ofDrawBitmapString("2:Toggle Mirror Mode", x, y);
        y+=STRING_HEIGHT;
        ofDrawBitmapString("9:Clear Textures", x, y);
        y+=STRING_HEIGHT;
        ofDrawBitmapString("0:Reset Settings", x, y);
        y+=STRING_HEIGHT;
        ofDrawBitmapString("up/down:Change PointSizeFactor", x, y);
        y+=STRING_HEIGHT;
        
        ofSetColor(255);
        
        // draw in 2D
        ofPushView();
        ofCamera overlayCamera=mUserTracker.getOverlayCamera();
        overlayCamera.begin(ofRectangle(0, 0, mDepthImage.getWidth(), mDepthImage.getHeight()));
        ofDrawAxis(100);
        mUserTracker.draw();
        overlayCamera.end();
        ofPopView();
        ofPopStyle();
    }
    
    
    drawPointCloud();
    
    drawUsers();
    
    if(mCanDisplayDebug){
        ofPushStyle();
        // draw in 3D
        mCamera.begin(ofRectangle(0,0,ofGetWidth(),ofGetHeight()));
        ofPushMatrix();
        if(mIsMirror){
            glScalef(-1, 1, 1);
        }

        //ofDrawAxis(100);
        mUserTracker.draw();
        
        // draw box
        ofNoFill();
        ofSetColor(255, 0, 255);
        for (int i = 0; i < mUserTracker.getNumUser(); i++)
        {
            ofxNiTE2::User::Ref user = mUserTracker.getUser(i);
            const ofxNiTE2::Joint &joint = user->getJoint(nite::JOINT_HEAD);
            
            joint.transformGL();
            ofBox(300);
            joint.restoreTransformGL();
        }
        ofPopMatrix();
        mCamera.end();
        ofPopStyle();
    }
    

}

void ofApp::keyPressed(int key){
    switch(key){
        case '1':
            mCanDisplayDebug=!mCanDisplayDebug;
            break;
        case '2':
            //flip mirror
            mIsMirror=!mIsMirror;
            break;
        case '9':
            //clear op textures
            for(UserAndPairOfOpMap::iterator it=mUserAndPairOfOpMap.begin();it!=mUserAndPairOfOpMap.end();++it){
                ofxKyonyuPairOpPtr op=it->second.mPairOfOp;
                op->clearTextures();
            }
            
            break;
        case '0':
            mPointSizeFactor=POINT_SIZE_FACTOR;
            mRotY=0;
            break;
        case OF_KEY_UP:
            mPointSizeFactor+=0.01;
            break;
        case OF_KEY_DOWN:
            mPointSizeFactor-=0.01;
            break;
    }
}

void ofApp::keyReleased(int key){

}

void ofApp::mouseMoved(int x, int y ){

}

void ofApp::mouseDragged(int x, int y, int button){
    mRotY=((x/(float)ofGetWidth())-0.5)*2*180*-1;

}


void ofApp::mousePressed(int x, int y, int button){

}

void ofApp::mouseReleased(int x, int y, int button){

}

void ofApp::windowResized(int w, int h){

}

void ofApp::gotMessage(ofMessage msg){

}

void ofApp::dragEvent(ofDragInfo dragInfo){

}


void ofApp::onNewUser(ofxNiTE2::User::Ref& user){
    UserAndPairOfOp userAndPairOfOp;
    userAndPairOfOp.mUser=user;
    userAndPairOfOp.mPairOfOp=ofxKyonyuPairOpPtr(new ofxKyonyuPairOp(mUnknownImage.getTextureReference()));
    mUserAndPairOfOpMap.insert(UserAndPairOfOpMap::value_type(user->getId(),userAndPairOfOp));
}
void ofApp::onLostUser(ofxNiTE2::User::Ref& user){
    mUserAndPairOfOpMap.erase(user->getId());
}

