#pragma once

#include <vector>
#include "ofVec3f.h"

class ofxKyonyuMesh
{
protected:
	std::vector<int> mIndexList;
	std::vector<ofVec3f> mVertexList;
	std::vector<ofVec3f> mNormalList;
    std::vector<ofVec2f> mTexCoordList;
public:
	ofxKyonyuMesh():mIndexList(),mVertexList()
	{
	}
	void draw()
	{
        mNormalList.resize(mVertexList.size(),ofVec3f());
        mTexCoordList.resize(mVertexList.size(),ofVec2f());//念のため
        
		for(size_t i=0;i<mIndexList.size()/3;++i)
		{
            ofVec3f a=mVertexList[mIndexList[i*3+1]]-mVertexList[mIndexList[i*3+0]];
            ofVec3f b=mVertexList[mIndexList[i*3+2]]-mVertexList[mIndexList[i*3+0]];
            ofVec3f n=(a.crossed(b)).normalized();
            for(size_t j=0;j<3;++j)
            {
                mNormalList[mIndexList[i*3+j]]+=n;
            }
        }
        
		for(size_t i=0;i<mNormalList.size();++i)
        {
            mNormalList[i].normalize();
        }
        
		glPolygonMode(GL_FRONT, GL_FILL);
		glBegin(GL_TRIANGLES);
		for(size_t i=0;i<mIndexList.size()/3;++i)
		{
            glTexCoord2fv(&mTexCoordList[mIndexList[i*3+0]][0]);
            glNormal3fv(&mNormalList[mIndexList[i*3+0]][0]);
			glVertex3fv(&mVertexList[mIndexList[i*3+0]][0]);
            glTexCoord2fv(&mTexCoordList[mIndexList[i*3+1]][0]);
            glNormal3fv(&mNormalList[mIndexList[i*3+1]][0]);
			glVertex3fv(&mVertexList[mIndexList[i*3+1]][0]);
            glTexCoord2fv(&mTexCoordList[mIndexList[i*3+2]][0]);
            glNormal3fv(&mNormalList[mIndexList[i*3+2]][0]);
			glVertex3fv(&mVertexList[mIndexList[i*3+2]][0]);
		}
		glEnd();
	}
};
