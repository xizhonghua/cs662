// Camera.cpp: implementation of the Camera class.
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h" // Only needed for MFC

#include "Camera.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
const float floorSize = 400.0f;
const float gridSize = 20.0f;
const float lightGrid[] = {0.4f, 0.4f, 0.4f, 1.0f};
const float darkGrid[] = {0.8f, 0.8f, 0.8f, 1.0f};
const float specularGrid[] = {1.0f, 1.0f, 1.0f, 1.0f};

Camera::Camera()
{
	Reset();
}

Camera::~Camera()
{

}

void Camera::Reset()
{
	// Default positions
	m_eye = vec3(0.0f, 80.0f, 200.0f);
	m_center = vec3(0.0f, 0.0f, 0.0f);
	// Default fovy, nearP and farP
	m_fovy = 60.0f;
	m_nearP = 0.1f;
	m_farP = 1000.0f;

	// Compute upVec
	m_frontVec = m_center - m_eye;
	m_dist = m_frontVec.Length();
	m_frontVec.Normalize();
	m_rightVec = vec3(1.0f, 0.0f, 0.0f);
	m_upVec = m_rightVec.Cross(m_frontVec);
	m_upVec.Normalize();

	// Update Projection Matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	Perspective();
}

void Camera::SetCamera(const vec3& eye, const vec3& center)
{
	m_eye = eye;
	m_center = center;
	m_frontVec = m_center - m_eye;
	m_dist = m_frontVec.Length();
	m_frontVec.Normalize();
	m_upVec = vec3(0.0f, 1.0f, 0.0f);
	m_rightVec = m_frontVec.Cross(m_upVec);
	m_rightVec.Normalize();
	m_upVec = m_rightVec.Cross(m_frontVec);
	m_upVec.Normalize(); 
}

const vec3& Camera::GetEye(void) const
{
	return m_eye;
}

const vec3& Camera::GetCenter(void) const
{
	return m_center;
}

const vec3& Camera::GetFrontVector(void) const
{
	return m_frontVec;
}

const vec3& Camera::GetRightVector(void) const
{
	return m_rightVec;
}

const vec3& Camera::GetUpVector(void) const
{
	return m_upVec;
}

void Camera::UpdateAll(void)
{
	// Know eye, center and upVec to compute frontVec, rightVec and distance.
	m_frontVec = m_center - m_eye;
	m_dist = m_frontVec.Length();
	m_frontVec.Normalize();

	m_rightVec = m_frontVec.Cross(m_upVec);
	m_rightVec.Normalize();
}

void Camera::LookAt()
{
	gluLookAt(m_eye[0], m_eye[1], m_eye[2],					// eye position
			  m_center[0], m_center[1], m_center[2],		// center where eye is looking at
			  m_upVec[0], m_upVec[1], m_upVec[2]);			// up vector of camera system
}

void Camera::MoveSWD(float dx, float dy) //move side way(left right up and down)
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	if (viewport[3] == 0){
		viewport[3] = 1;
	}
	float ty = tan(m_fovy / 180.0 * M_PI);
	float tx = ty * float(viewport[2]) / float(viewport[3]);

	float ly = viewport[3] / ty;
	float lx = viewport[2] / tx;

	dx = dx / lx * m_dist;
	dy = dy / ly * m_dist;

	vec3 tmp = m_rightVec * dx + m_upVec * dy;
	m_eye = m_eye + tmp;
	m_center = m_center + tmp;

	UpdateAll();
}

void Camera::RotateCenter(float dx, float dy)
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	vec3 tmp;
	mat3 rotMat;
	float unitx = M_PI / viewport[2] * 2.0f;
	float unity = M_PI / viewport[3] * 2.0f;

	if (unitx > 0.6f)
		unitx = 0.6f;
	if (unity > 0.6f)
		unity = 0.6f;

	dx = unitx * dx;
	dy = unity * dy;

	//compute the rotation matrix for dx and dy
	rotMat = mat3::Rotation3DRad(axisY, dx) * mat3::Rotation3DRad(m_rightVec, dy);
	//compute the new eye location
	tmp = m_eye - m_center;
	tmp = rotMat * tmp;
	m_eye = m_center + tmp;
	//compute the new upVec
	m_upVec = rotMat * m_upVec;

	UpdateAll();
}

void Camera::Zoom(float dx)
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	float total = viewport[3];
	if (dx > 0){//zoom in
		if (m_dist < 0.1)
			return;
		dx = dx / total * m_dist;
		if (dx > m_dist)
			return;
	}
	else{//zoom out
		dx = dx / total * m_dist;
	}

	m_eye = m_eye + m_frontVec * dx;

	UpdateAll();
}

void Camera::DrawFloor() const
{
	bool row, col;
	row = col = false;
	for (int i = -floorSize; i <= floorSize; i+= gridSize){
		for (int j = -floorSize; j <= floorSize; j+= gridSize){
			if (col)
			{
				glColor3fv(lightGrid);
				
				/*glMaterialfv(GL_FRONT, GL_AMBIENT, lightGrid);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, lightGrid);
				glMaterialfv(GL_FRONT, GL_SPECULAR, specularGrid);
				glMaterialf(GL_FRONT, GL_SHININESS, 50.0f);*/
			}
			else
			{
				glColor3f(0.8, 0.8, 0.8);

				/*glMaterialfv(GL_FRONT, GL_AMBIENT, darkGrid);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, darkGrid);
				glMaterialfv(GL_FRONT, GL_SPECULAR, specularGrid);
				glMaterialf(GL_FRONT, GL_SHININESS, 50.0f);*/
			}

			glBegin(GL_QUADS);
			glVertex3f(float(i) - gridSize / 2.0, 0 , float(j) - gridSize / 2.0);
			glVertex3f(float(i) - gridSize / 2.0, 0 , float(j) + gridSize / 2.0);
			glVertex3f(float(i) + gridSize / 2.0, 0 , float(j) + gridSize / 2.0);
			glVertex3f(float(i) + gridSize / 2.0, 0 , float(j) - gridSize / 2.0);
			glEnd();
			col = !col;
		}
		row = !row;
		col = row;
	}
}

void Camera::Perspective( void )
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	if (viewport[3] == 0){
		viewport[3] = 1;
	}
	gluPerspective(60.0f,float(viewport[2]+0.0f) / float(viewport[3]),m_nearP,m_farP);
}