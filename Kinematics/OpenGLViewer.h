//////////////////////////////////////////////////////////////////////////
// OpenGLViewer.h -- Header file for OpenGl Viewer for Skeleton
//
// Liming Zhao
// 11/02/2007

#pragma once

#include "Animation.h"
#include <GL/glu.h>
#include <GL/glut.h>

enum{eFKSelect, eIKSelect};

class OpenGLViewer
{
public:
	OpenGLViewer(void);
	~OpenGLViewer(void);

	void SetSkeleton(Skeleton* pSkeleton) { m_pSkeleton = pSkeleton; }
	void SetMode(const unsigned int& mode) { m_mode = mode; }
	void SetDrawCoord(bool bDraw) { m_bDrawCoord = bDraw; }
	
	Skeleton* GetSkeleton(void) { return m_pSkeleton; }
	const unsigned int& GetMode(void) const { return m_mode; }
	bool IsDrawCoord(void) const { return m_bDrawCoord; }

	bool IsGlPick(void) const { return m_bGlPick; }
	void SetGlPick(bool bPick) { m_bGlPick = bPick; } 

	void Display(void);
	
private:
	void DrawBodyParts( Joint* currentJoint);
	void SetMaterial (float* colorVec);
	void DrawJoint(const Transform& globalTransform, const unsigned int& id);
	void DrawLimb(const vec3& startPosition, const vec3& endPosition, const unsigned int& id);
	void GLApplyRotation(const mat3& rotation);

	Skeleton* m_pSkeleton;
	unsigned int m_mode;
	bool m_bDrawCoord;
	bool m_bGlPick;
	
};
