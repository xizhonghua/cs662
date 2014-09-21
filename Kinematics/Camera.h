// Camera.h: interface for the Camera class.
// Author: Liming Zhao
//////////////////////////////////////////////////////////////////////

#pragma once

#include <windows.h>
#include <GL/Glu.h>
#include <math.h>
#include "Transformation.h"

class Camera
{
public:
	Camera(void);
	~Camera(void);

	const vec3& GetEye(void) const;
	const vec3& GetCenter(void) const;
	const vec3& GetFrontVector(void) const;
	const vec3& GetRightVector(void) const;
	const vec3& GetUpVector(void) const;
	void SetCamera(const vec3& eye, const vec3& center);
	void Zoom(float dx);
	void RotateCenter(float dx, float dy);
	void MoveSWD(float dx, float dy);
	void LookAt();
	void Reset();
	void DrawFloor() const;
	void Perspective(void);
	
private:
	void UpdateAll(void);
	double m_fovy, m_nearP, m_farP;
	vec3 m_eye, m_center;		
	vec3 m_upVec, m_frontVec, m_rightVec;
	double m_dist;
};