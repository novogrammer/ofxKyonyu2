#pragma once

#include "ofMain.h"

#include "ofxNI2.h"
#include "ofxNiTE2.h"
#include "ofxKyonyuPairOp.h"
#include "ofSoundPlayer.h"

#define FPS 30
#define INV_FPS 1.0/FPS
#define BUFFER_WIDTH 320
#define BUFFER_HEIGHT 240
//#define BUFFER_WIDTH 640
//#define BUFFER_HEIGHT 480

#define CONFIDENCE_FACTOR 0.3
#define CONFIDENCE_FACTOR_FOR_CAPTURE 0.6
#define KANSETSU_RADIUS 150
#define CAPTURE_ERROR_RANGE_DEG 5
#define OP_TEXTURE_WIDTH 256
#define OP_TEXTURE_HEIGHT 256
#define POINT_SIZE_FACTOR 1.5

typedef ofPtr<ofxKyonyuPairOp> ofxKyonyuPairOpPtr;
struct UserAndPairOfOp{
    ofxNiTE2::User::Ref mUser;
    ofxKyonyuPairOpPtr mPairOfOp;
};
typedef map<nite::UserId,UserAndPairOfOp> UserAndPairOfOpMap;

class ofApp : public ofBaseApp{
    
    ofxNI2::Device mDevice;
    ofxNiTE2::UserTracker mUserTracker;
    ofImage mDepthImage;
    ofImage mColorImage;
    ofxNI2::ColorStream mColorStream;
    ofxNI2::DepthStream mDepthStream;
    ofCamera mCamera;
    
    ofShader mPointCloudShader;
    ofMesh mPointCloudMesh;
    ofLight mLight;
    ofMaterial mMaterial;
    ofImage mUnknownImage;
    
    UserAndPairOfOpMap mUserAndPairOfOpMap;
    
    float mPointSizeFactor;
    bool mIsMirror;
    
    bool mCanDisplayDebug;
    float mRotY;
    
    float mTanX;
    float mTanY;
    


private:
    void setupNI2();
    void updateOp();
    void drawPointCloud();
    void drawUsers();
    ofVec3f toPosition(const nite::SkeletonJoint& joint);
    ofMatrix4x4 toMatrix(const nite::SkeletonJoint& joint);
    ofVec3f toWorld(int x,int y,short depth){
        return ofVec3f (
            ((x/(float)BUFFER_WIDTH)-0.5)*2*depth*mTanX,
            ((y/(float)BUFFER_HEIGHT)-0.5)*2*depth*mTanY*-1,
            -depth
        );
    }
    ofVec2f toDepth(const ofVec3f& pos){
        return ofVec2f(
           ((pos.x/(mTanX*pos.z*2*-1)   )+0.5)*BUFFER_WIDTH,
           ((pos.y/(mTanY*pos.z*2*-1)*-1)+0.5)*BUFFER_HEIGHT
        );
    }
public:
    void setup();
    void exit();
    void update();
    void draw();

    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    void onNewUser(ofxNiTE2::User::Ref& user);
    void onLostUser(ofxNiTE2::User::Ref& user);
    
		
};