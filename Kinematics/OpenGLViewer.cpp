//////////////////////////////////////////////////////////////////////////
// OpenGLViewer.h -- Header file for OpenGl Viewer for Skeleton
//
// Liming Zhao
// 11/02/2007

#include "StdAfx.h"	// Needed only for MFC

#include "OpenGLViewer.h"

float DarkBlue[] = {0.0510, 0.1255, 0.4627, 1.0};
float DarkYellow[] = {0.9922, 0.8392, 0.1725, 1.0};
float LightRed[] = {1.0, 0.5, 0.5, 1.0};
float LightGreen[] = {0.0, 1.0, 0.25, 1.0};
float Specular[] = {1.0, 1.0, 1.0, 1.0};
const float jointRadius = 2.0f;
const float limbScale = 2.0f;
const float coordLength = 4.0f;


OpenGLViewer::OpenGLViewer(void)
{
	m_mode = eFKSelect;
	m_bDrawCoord = true;
	m_pSkeleton = NULL;
	m_bGlPick = false;
}

OpenGLViewer::~OpenGLViewer(void)
{
}

void OpenGLViewer::Display(void)
{
	if (m_pSkeleton && m_pSkeleton->GetJointCount() > 0)
	{
		glPushMatrix();
		unsigned int id = m_pSkeleton->GetRootJoint()->GetID();
		if (m_pSkeleton->GetSelectedJoint() == id)
		{
			SetMaterial(LightRed);
		}else
			SetMaterial(DarkYellow);
		Joint* pJoint = m_pSkeleton->GetRootJoint();
		DrawJoint(pJoint->GetGlobalTransform(), id);

		unsigned int totalChildren = pJoint->GetChildCount();
		for (unsigned int i = 0; i < totalChildren; i++)
			DrawBodyParts(pJoint->GetChildAt(i));

		glPopMatrix();

		if (m_mode == eIKSelect && m_pSkeleton->GetSelectedJoint() != -1)
		{
			const vec3& pos = m_pSkeleton->GetGoalPosition();
			glPushMatrix();
			SetMaterial(LightGreen);
			glTranslatef(pos[0], pos[1], pos[2]);	
			glutSolidSphere(jointRadius, 24, 12);
			glPopMatrix();
		}
	}
}

void OpenGLViewer::DrawBodyParts( Joint* currentJoint)
{
	// Draw joint
	unsigned int id = currentJoint->GetID();
	if (m_pSkeleton->GetSelectedJoint() == id)
	{
		SetMaterial(LightRed);
	}else
		SetMaterial(DarkYellow);
	DrawJoint(currentJoint->GetGlobalTransform(), id);
	// Draw limb	
	Joint* pParent = currentJoint->GetParent();
	if (m_mode == eFKSelect)
		id = pParent->GetID();
	SetMaterial(DarkBlue);
	DrawLimb(pParent->GetGlobalTranslation(), currentJoint->GetGlobalTranslation(), id);

	unsigned int totalChildren = currentJoint->GetChildCount();
	for (unsigned int i = 0; i < totalChildren; i++)
		DrawBodyParts(currentJoint->GetChildAt(i));
}

void OpenGLViewer::SetMaterial(float* colorVec)
{
	glMaterialfv(GL_FRONT, GL_AMBIENT, colorVec);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, colorVec);
	glMaterialfv(GL_FRONT, GL_SPECULAR, Specular);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0f);
}

void OpenGLViewer::GLApplyRotation(const mat3& rotation)
{
	float mat[16];
	mat[0] = rotation[0][0];	mat[4] = rotation[0][1];	mat[8] = rotation[0][2];	mat[12] = 0.0f;
	mat[1] = rotation[1][0];	mat[5] = rotation[1][1];	mat[9] = rotation[1][2];	mat[13] = 0.0f;
	mat[2] = rotation[2][0];	mat[6] = rotation[2][1];	mat[10] = rotation[2][2];	mat[14] = 0.0f;
	mat[3] = 0.0f;				mat[7] = 0.0f;				mat[11] = 0.0f;				mat[15] = 1.0f;
	glMultMatrixf(mat);
}

void OpenGLViewer::DrawJoint(const Transform& globalTransform, const unsigned int& id)
{
	const vec3& globalPosition = globalTransform.m_translation;
	const mat3& globalRotation = globalTransform.m_rotation;

	glPushMatrix();
		if (m_bGlPick)
		{
			glPushName(id);
		}
		
		glTranslatef(globalPosition[0], globalPosition[1], globalPosition[2]);	
		glutSolidSphere(jointRadius, 24, 12);

		if (m_bDrawCoord && !m_bGlPick)
		{
			GLApplyRotation(globalRotation);
			glDisable(GL_LIGHTING);
			glBegin(GL_LINES);
			glColor3f(1.0f,0.0f,0.0f);
			glVertex3f(0.0f,0.0f,0.0f);
			glVertex3f(coordLength,0.0f,0.0f);

			glColor3f(0.0f,1.0f,0.0f);
			glVertex3f(0.0f,0.0f,0.0f);
			glVertex3f(0.0f,coordLength,0.0f);

			glColor3f(0.0f,0.0f,1.0f);
			glVertex3f(0.0f,0.0f,0.0f);
			glVertex3f(0.0f,0.0f,coordLength);
			glEnd();
			glEnable(GL_LIGHTING);
		}

		if (m_bGlPick)
		{
			glPopName();
		}
	glPopMatrix();
}

void OpenGLViewer::DrawLimb(const vec3& startPosition, const vec3& endPosition, const unsigned int& id)
{
	// Determine rotation axis
	vec3 direction = endPosition - startPosition;
	float l = direction.Length();
	direction /= l;
	vec3 direction2 = Prod(direction, direction);
	float dx = direction2[VY] + direction2[VZ];
	float dy = direction2[VX] + direction2[VZ];
	float dz = direction2[VX] + direction2[VY];
	unsigned int eAxis;
	vec3 axis;
	float angle;
	if (dx >= dy && dx >= dz)
	{
		eAxis = VX;
		axis = axisX.Cross(direction);
		angle = acos(direction * axisX) * Rad2Deg;
	}
	else if (dy >= dx && dy >= dz)
	{
		eAxis = VY;
		axis = axisY.Cross(direction);
		angle = acos(direction * axisY) * Rad2Deg;
	}
	else
	{
		eAxis = VZ;
		axis = axisZ.Cross(direction);
		angle = acos(direction * axisZ) * Rad2Deg;
	}
	glPushMatrix();

		if (m_bGlPick)
		{
			glPushName(id);
		}

		vec3 center = (endPosition + startPosition) / 2.0f;
		glTranslatef(center[0], center[1], center[2]);
		glRotatef(angle, axis[0], axis[1], axis[2]);
		switch(eAxis)
		{
		case VX:
			glScalef(l / 2.0f, 2.0f, 2.0f);
			break;
		case VY:
			glScalef(2.0f, l / 2.0f, 2.0f);
		    break;
		case VZ:
			glScalef(2.0f, 2.0f, l / 2.0f);
		    break;
		}
		glutSolidSphere (1.0f, 24, 12);
		glScalef(1.0f, 1.0f, 1.0f);

		if (m_bGlPick)
		{
			glPopName();
		}
	glPopMatrix();
}
