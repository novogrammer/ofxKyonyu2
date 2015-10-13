#include "ofxKyonyuOp.h"

const float ofxKyonyuOp::RADIUS = 90;//[mm]
const float ofxKyonyuOp::WARP_LENGTH = 100;//[mm]


void ofxKyonyuOp::update(float inDt)
{
    static const int times = 20; 
    volatile size_t jointCount=mJointList.size();
    volatile size_t pointCount=mPointList.size();
    for (int j = 0; j < times;++j)
    {
        //touch対応
        for(size_t i=0;i<mTouchingList.size();++i)
        {
            touch(mTouchingList[i].first, mTouchingList[i].second);
            
        }
        //フェーズを二つに分ける。力更新と位置更新
        // update force
        for(size_t i=0;i<jointCount;++i) {
            mJointList[i].updateForce();
        }
        for(size_t i=0;i<pointCount;++i) {
            mPointList[i].updateForce();
        }
        // update position
        for(size_t i=0;i<pointCount;++i) {
            mPointList[i].updatePosition(inDt/times);
        }
        
    }
    
    //drawLine();
    transfer();
}


void ofxKyonyuOp::putPointAndJoint()
{
    //Xを経度
    //Yを緯度
    float diffRotX = DEG_TO_RAD*360.0 / COLS;
    float diffRotY = DEG_TO_RAD*90.0 / (ROWS-1);
    
    mPointList.resize(0);
    mJointList.resize(0);
    mIndexList.resize(0);
    
    mPointList.resize(getOpCount(),ofxKyonyuPoint());
    mVertexList.resize(getOpCount());
    mIndexList.reserve(getOpCount()*3);//だいたい予約
    
    
    //ポイントとジョイントを一気に配置したほうが楽なので
    for(int y=0;y<ROWS;++y)
    {
        for(int x=0;x<COLS;++x)
        {
            ofxKyonyuPoint& point=mPointList[getOpIndex(y,x)];
            //point.name = String(y) + "-" + String(x) + " ";//デバッグ用
            //var noize:Number = Math.random()*10;//0から1
            //よこ
            //var tmp:MyVector3D = new MyVector3D(0,Math.cos(diffRotX*x),Math.sin(diffRotX*x));
            //point._position = new MyVector3D(Math.sin(diffRotY * y) * RADIUS, Math.cos(diffRotY*y)*tmp.y*RADIUS, Math.cos(diffRotY*y)*tmp.z*RADIUS);
            ofVec3f tmp = ofVec3f(sinf(diffRotX*x),cosf(diffRotX*x),0);
            point.mPosition = ofVec3f(
                                         cos(diffRotY * y) * tmp.x * RADIUS,
                                         cos(diffRotY * y) * tmp.y * RADIUS,
                                         sin(diffRotY * y) * RADIUS
                                         );
            if (y == ROWS - 1)
            {
                point.mPosition.z *= 1.1;
            }
            
            
            //面
            if (y > 0)
            {
                mIndexList.push_back(getOpIndex(y - 1,x - 1));
                mIndexList.push_back(getOpIndex(y - 1,x + 0));
                mIndexList.push_back(getOpIndex(y + 0,x - 1));
                
                if (y < ROWS - 1)
                {
                    mIndexList.push_back(getOpIndex(y - 1, x + 0));
                    mIndexList.push_back(getOpIndex(y + 0, x + 0));
                    mIndexList.push_back(getOpIndex(y + 0, x - 1));
                }
                
            }
            
            if(y>0)
            {
                ofxKyonyuPoint& pointUp = mPointList[getOpIndex(y - 1, x)];
                mJointList.push_back(ofxKyonyuJoint(&point, &pointUp));//„Åü„Å¶
                
                if (y < ROWS - 1)
                {
                    ofxKyonyuPoint& pointNaname1 = mPointList[getOpIndex(y - 1, x - 1)];
                    ofxKyonyuJoint jointNaname1 = ofxKyonyuJoint(&point, &pointNaname1);
                    jointNaname1.mSpring *= 0.75;
                    mJointList.push_back(jointNaname1);//ななめ1（せん断抵抗）
                    
                    ofxKyonyuPoint& pointNaname2 = mPointList[getOpIndex(y - 1, x + 1)];
                    ofxKyonyuJoint jointNaname2 = ofxKyonyuJoint(&point, &pointNaname2);
                    jointNaname2.mSpring *= 0.75;
                    mJointList.push_back(jointNaname2);//ななめ2（せん断抵抗）
                }
            }
            if (y < ROWS - 1)
            {
                ofxKyonyuPoint& pointLeft = mPointList[getOpIndex(y, x - 1)];
                mJointList.push_back(ofxKyonyuJoint(&point,&pointLeft));//„Çà„Åì
            }
            if(y>1)
            {
                ofxKyonyuPoint& pointUp2 = mPointList[getOpIndex(y - 2, x)];
                ofxKyonyuJoint jointUp2 = ofxKyonyuJoint(&point, &pointUp2);
                if (y < ROWS -1)
                {
                    jointUp2.mSpring *= 1.0;
                }
                else
                {//極部分
                    jointUp2.mSpring *= 2.0;
                    jointUp2.mDamper *= 2.0;
                }
                mJointList.push_back(jointUp2);//たて　ひとつ飛ばし（角度抵抗）
            }
            if (y < ROWS - 1)
            {
                ofxKyonyuPoint& pointLeft2 = mPointList[getOpIndex(y,x - 2)];
                ofxKyonyuJoint jointLeft2 = ofxKyonyuJoint(&point, &pointLeft2);
                jointLeft2.mSpring *= 1.0;
                mJointList.push_back(jointLeft2);//よこ　ひとつ飛ばし（角度抵抗）
                
                //            var pointDiagonal:Point = _points[getOpIndex(y,x+_cols/2)];
                //            var jointDiagonal:Joint = new Joint(point, pointDiagonal);
                //            jointDiagonal.SPRING *= 0.5;
                //            _joints.push(jointDiagonal);//よこ　対角線
            }
            
        }
        //極部分はバネが集中するので重くする
        ofxKyonyuPoint& pointPole = mPointList[mPointList.size() - 2];
        pointPole.mMass *= 1.5;
        
        //中心部分を設定
        ofxKyonyuPoint& pointCenter = mPointList[mPointList.size() - 1];
        pointCenter.mPosition = ofVec3f(0,0,RADIUS/4*-1);
        pointCenter.mIsPinned = true;
        for (int volIndex = 0; volIndex < mPointList.size() -1;++volIndex)
        {//圧力
            ofxKyonyuJoint jointVol = ofxKyonyuJoint(&pointCenter, &mPointList[volIndex]);
            if (volIndex < mPointList.size() - 2)
            {
                jointVol.mSpring *= 0.5;
            }
            else
            {//極部分
                jointVol.mSpring *= 2.0;
            }
            mJointList.push_back(jointVol);
        }
        
        for(size_t i=0;i<mJointList.size();++i)
        {
            ofxKyonyuJoint &joint=mJointList[i];
            joint.resetNaturalLength();
        }
        
        
        //根元を固定します。
        for (int i = 0; i < COLS;++i)
        {
            mPointList[i].mIsPinned = true;
            mPointList[i+COLS].mIsPinned = true;
        }
    }
    
    //テクスチャ座標
    for(size_t i=0;i<mPointList.size();++i)
    {
        mPointList[i].mTexCoord.x=1-(mPointList[i].mPosition.x/(RADIUS*2)+0.5);
        mPointList[i].mTexCoord.y=1-(mPointList[i].mPosition.y/(RADIUS*2)+0.5);
    }
}



void ofxKyonyuOp::setPinnedMatrix(ofMatrix4x4 inPinnedMatrix)
{
    ofMatrix4x4 diff= ofMatrix4x4::getInverseOf(mPinnedMatrix)*inPinnedMatrix;
    mPinnedMatrix=inPinnedMatrix;
    
    if(diff.getTranslation().length()>WARP_LENGTH)
    {
        putPointAndJoint();//リセット
        for(size_t i=0;i<mPointList.size();++i)
        {
            ofxKyonyuPoint& p=mPointList[i];
            p.mPosition=p.mPosition*mPinnedMatrix;//今の姿勢へ
        }
    }
    else
    {
		
        for(size_t i=0;i<mPointList.size();++i)
        {
            ofxKyonyuPoint& p=mPointList[i];
            if((p.mIsPinned || p.mIsDragging))
            {
                p.mPosition=p.mPosition*diff;
            }
        }
    }
}

std::pair<ofVec3f,ofVec3f> ofxKyonyuOp::getBound()const
{
    return std::make_pair(ofVec3f(-RADIUS,-RADIUS,0),ofVec3f(+RADIUS,+RADIUS,0));
}

void ofxKyonyuOp::touch(const ofVec3f& inPosition,float inRadius)
{
    float closest=1000;
    for(size_t i=0;i<mPointList.size();++i)
    {
        ofxKyonyuPoint& p=mPointList[i];
        if(!(p.mIsPinned || p.mIsDragging))
        {
            closest=MAX(closest,(p.mPosition-inPosition).length());
            if((p.mPosition-inPosition).length()<inRadius)
            {
#if 0
                p.mPosition=(p.m_Position-inPosition).getNormalized()*inRadius+inPosition;
                p.mVelocity=ofxVec3f();//タッチは速度なし
#else
                ofVec3f f=(p.mPosition-inPosition).getNormalized()*(inRadius-(p.mPosition-inPosition).length())*10;
                static const float maxForce=250;
                if(f.length()>maxForce)
                {
                    f=f.getNormalized()*maxForce;
                }
                p.mForce+=f;
#endif
                mIsTouched=true;
            }
        }
    }
}

void ofxKyonyuOp::addTouching(const ofVec3f& inPosition,float inRadius)
{
    mTouchingList.push_back(std::make_pair(inPosition,inRadius));
}



