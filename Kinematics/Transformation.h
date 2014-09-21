//////////////////////////////////////////////////////////////////////////
// Transformation.h -- Header file for useful Classes about 3D transformations
//
// Liming Zhao
// 11/02/2007

/****************************************************************
*																*
* C++ Vector and Matrix Algebra routines						*
* Author: Jean-Francois DOUE									*
* Version 3.1 --- October 1993									*
*																*
****************************************************************/
//
//	From "Graphics Gems IV / Edited by Paul S. Heckbert
//	Academic Press, 1994, ISBN 0-12-336156-9
//	"You are free to use and modify this code in any way 
//	you like." (p. xv)
//
//	Modified by J. Nagle, March 1997
//	-	All functions are inline.
//	-	All functions are const-correct.
//	-	All checking is via the standard "assert" macro.
//	-	Stream I/O is disabled for portability, but can be
//		re-enabled by defining ALGEBRA3IOSTREAMS.

#pragma once

#include <iostream>
#include <assert.h>
#include <cmath>

using namespace std;

enum {VX, VY, VZ, VW};		// axes
enum {PA, PB, PC, PD};		// planes
enum {RED, GREEN, BLUE};	// colors
enum {KA, KD, KS, ES};		// phong coefficients

//////////////////////////////////////////////////////////////////////////
//PI
//
#ifndef M_PI
const float M_PI = 3.14159265358979323846f;		// per CRC handbook, 14th. ed.
#endif
const float M_PI_2 = (M_PI/2.0f);				// PI/2
const float M2_PI = (M_PI*2.0f);				// PI*2
const float Rad2Deg = (180.0f / M_PI);			// Rad to Degree
const float Deg2Rad = (M_PI / 180.0f);			// Degree to Rad

#ifndef EPSILON
#define EPSILON 0.001
#endif

// this line defines a new type: pointer to a function which returns a
// float and takes as argument a float
typedef float (*V_FCT_PTR)(float);

// min-max macros
#define MIN(A,B) ((A) < (B) ? (A) : (B))
#define MAX(A,B) ((A) > (B) ? (A) : (B))

// error handling macro
#define ALGEBRA_ERROR(E) { assert(false); }

class vec2;
class vec3;
class mat3;
class Quaternion;
class Transform;

/****************************************************************
*																*
*				2D Vector										*
*																*
****************************************************************/

class vec2
{
protected:

	float n[2];

public:

	// Constructors
	vec2();
	vec2(const float x, const float y);
	vec2(const vec2& v);				// copy constructor

	// Assignment operators
	vec2& operator	= ( const vec2& v );	// assignment of a vec2
	vec2& operator += ( const vec2& v );	// incrementation by a vec2
	vec2& operator -= ( const vec2& v );	// decrementation by a vec2
	vec2& operator *= ( const float d );	// multiplication by a constant
	vec2& operator /= ( const float d );	// division by a constant
	float& operator [] ( int i);			// indexing
	float vec2::operator [] ( int i) const;// read-only indexing

	// Special functions
	float Length() const;			// length of a vec2
	float SqrLength() const;		// squared length of a vec2
	vec2& Normalize() ;				// normalize a vec2 in place

	// friends
	friend vec2 operator- (const vec2& v);					// -v1
	friend vec2 operator+ (const vec2& a, const vec2& b);	// v1 + v2
	friend vec2 operator- (const vec2& a, const vec2& b);	// v1 - v2
	friend vec2 operator* (const vec2& a, const float d);	// v1 * 3.0
	friend vec2 operator* (const float d, const vec2& a);	// 3.0 * v1
	friend float operator* (const vec2& a, const vec2& b);	// dot product
	friend vec2 operator/ (const vec2& a, const float d);	// v1 / 3.0
	friend vec3 operator^ (const vec2& a, const vec2& b);	// cross product
	friend int operator== (const vec2& a, const vec2& b);	// v1 == v2 ?
	friend int operator!= (const vec2& a, const vec2& b);	// v1 != v2 ?
	friend vec2 Prod(const vec2& a, const vec2& b);			// term by term *
	friend float Dot(const vec2& a, const vec2& b);			// dot product
};

/****************************************************************
*																*
*				3D Vector										*
*																*
****************************************************************/

class vec3
{
protected:

	float n[3];

public:

	// Constructors
	vec3();
	vec3(const float x, const float y, const float z);
	vec3(const vec3& v);					// copy constructor

	// Assignment operators
	vec3& operator	= ( const vec3& v );		// assignment of a vec3
	vec3& operator += ( const vec3& v );		// incrementation by a vec3
	vec3& operator -= ( const vec3& v );		// decrementation by a vec3
	vec3& operator *= ( const float d );		// multiplication by a constant
	vec3& operator /= ( const float d );		// division by a constant
	float& operator [] ( int i);				// indexing
	float operator[] (int i) const;				// read-only indexing

	// special functions
	float Length() const;				// length of a vec3
	float SqrLength() const;			// squared length of a vec3
	vec3& Normalize();					// normalize a vec3 in place
	vec3 Cross(vec3 &v) const;			// cross product

	// friends
	friend vec3 operator - (const vec3& v);					// -v1
	friend vec3 operator + (const vec3& a, const vec3& b);	// v1 + v2
	friend vec3 operator - (const vec3& a, const vec3& b);	// v1 - v2
	friend vec3 operator * (const vec3& a, const float d);	// v1 * 3.0
	friend vec3 operator * (const float d, const vec3& a);	// 3.0 * v1
	friend float operator * (const vec3& a, const vec3& b); // dot product
	friend vec3 operator / (const vec3& a, const float d);	// v1 / 3.0
	friend vec3 operator ^ (const vec3& a, const vec3& b);	// cross product
	friend int operator == (const vec3& a, const vec3& b);	// v1 == v2 ?
	friend int operator != (const vec3& a, const vec3& b);	// v1 != v2 ?
	friend vec3 Prod(const vec3& a, const vec3& b);			// term by term *
	friend float Dot(const vec3& a, const vec3& b);			// dot product

	// necessary friend declarations
	friend class mat3;
	friend vec3 operator * (const mat3& a, const vec3& v);
	friend mat3 operator * (const mat3& a, const mat3& b);	// matrix 3 product
};

const vec3 axisX(1.0f, 0.0f, 0.0f);
const vec3 axisY(0.0f, 1.0f, 0.0f);
const vec3 axisZ(0.0f, 0.0f, 1.0f);
const vec3 vec3Zero(0.0f, 0.0f, 0.0f);

/****************************************************************
*																*
*			   3x3 Matrix										*
*																*
****************************************************************/

class mat3
{
protected:

	vec3 v[3];

public:

	// Constructors
	mat3();
	mat3(const vec3& v0, const vec3& v1, const vec3& v2);
	mat3(const mat3& m);

	// Static functions
	static mat3 Identity();
	static mat3 Rotation3DRad(const vec3& axis, const float angleRad);
	static mat3 Rotation3DRad(const int Axis, const float angleRad);

	// Rotation operations, matrix must be orthonomal
	bool ToEulerAnglesZXY(vec3& anglesRad) const;
	mat3 FromEulerAnglesZXY(const vec3& anglesRad);

	// Conversion with Quaternion
	Quaternion ToQuaternion() const;
	void FromQuaternion(const Quaternion& q);
	
	// Assignment operators
	mat3& operator	= ( const mat3& m );		// assignment of a mat3
	mat3& operator += ( const mat3& m );		// incrementation by a mat3
	mat3& operator -= ( const mat3& m );		// decrementation by a mat3
	mat3& operator *= ( const float d );		// multiplication by a constant
	mat3& operator /= ( const float d );		// division by a constant
	vec3& operator [] ( int i);					// indexing
	const vec3& operator [] ( int i) const;		// read-only indexing

	// special functions
	mat3 Transpose() const;								// transpose
	
	// friends
	friend mat3 operator - (const mat3& a);						// -m1
	friend mat3 operator + (const mat3& a, const mat3& b);		// m1 + m2
	friend mat3 operator - (const mat3& a, const mat3& b);		// m1 - m2
	friend mat3 operator * (const mat3& a, const mat3& b);		// m1 * m2
	friend mat3 operator * (const mat3& a, const float d);		// m1 * 3.0
	friend mat3 operator * (const float d, const mat3& a);		// 3.0 * m1
	friend mat3 operator / (const mat3& a, const float d);		// m1 / 3.0
	friend int operator == (const mat3& a, const mat3& b);		// m1 == m2 ?
	friend int operator != (const mat3& a, const mat3& b);		// m1 != m2 ?

	// necessary friend declarations
	friend vec3 operator * (const mat3& a, const vec3& v);		// linear transform
	friend mat3 operator * (const mat3& a, const mat3& b);		// matrix 3 product
};

/****************************************************************
*																*
*			Quaternion											*
*																*
****************************************************************/


class Quaternion
{
protected:

	float n[4];

	// Used by Slerp
	static float CounterWarp(float t, float fCos);
	static float ISqrt_approx_in_neighborhood(float s);

	// Internal indexing
	float& operator[](int i);
	float operator[](int i) const;
public:

	// Constructors
	Quaternion();
	Quaternion(const float w, const float x, const float y, const float z);
	Quaternion(const Quaternion& q);

	// Static functions
	static float Dot(const Quaternion& q0, const Quaternion& q1);
	static Quaternion Exp(const Quaternion& q);
	static Quaternion Log(const Quaternion& q);
	static Quaternion UnitInverse(const Quaternion& q);
	static Quaternion Slerp(float t, const Quaternion& q0, const Quaternion& q1);
	static Quaternion Intermediate (const Quaternion& q0, const Quaternion& q1, const Quaternion& q2);
	static Quaternion Squad(float t, const Quaternion& q0, const Quaternion& a, const Quaternion& b, const Quaternion& q1);

	// Conversion functions
	void ToAxisAngle (vec3& axis, float& angleRad) const;
	void FromAxisAngle (const vec3& axis, float angleRad);
	mat3 ToRotation () const;
	void FromRotation (const mat3& rot);

	// Assignment operators
	Quaternion& operator = (const Quaternion& q);	// assignment of a quaternion
	Quaternion& operator += (const Quaternion& q);	// summation with a quaternion
	Quaternion& operator -= (const Quaternion& q);	// subtraction with a quaternion
	Quaternion& operator *= (const Quaternion& q);	// multiplication by a quaternion
	Quaternion& operator *= (const float d);		// multiplication by a scalar
	Quaternion& operator /= (const float d);		// division by a scalar

	// Indexing
	float& W();
	float W() const;
	float& X();
	float X() const;
	float& Y();
	float Y() const;
	float& Z();
	float Z() const;

	// Friends
	friend Quaternion operator - (const Quaternion& q);							// -q
	friend Quaternion operator + (const Quaternion& q0, Quaternion& q1);		// q0 + q1
	friend Quaternion operator - (const Quaternion& q0, const Quaternion& q1);	// q0 - q1
	friend Quaternion operator * (const Quaternion& q, const float d);			// q * 3.0
	friend Quaternion operator * (const float d, const Quaternion& q);			// 3.0 * v
	friend Quaternion operator * (const Quaternion& q0, const Quaternion& q1);	// q0 * q1
	friend Quaternion operator / (const Quaternion& q, const float d);			// q / 3.0
	friend bool operator == (const Quaternion& q0, const Quaternion& q1);		// q0 == q1 ?
	friend bool operator != (const Quaternion& q0, const Quaternion& q1);		// q0 != q1 ?

	// Special functions
	float Length() const;
	float SqrLength() const;
	Quaternion& Normalize();
	Quaternion& FastNormalize();
	Quaternion Inverse() const;
	void Zero();

	friend mat3;
};

/****************************************************************
*																*
*			Transform											*
*																*
****************************************************************/

class Transform
{
public:
	Transform();
	Transform(const vec3& translation, const mat3& rotation);
	Transform(const vec3& translation);
	Transform(const mat3& rotation);
	Transform(const Transform& transform);
	~Transform();

	Transform Inverse() const;
	Transform& operator = (const Transform& source); // assignment
	void Identity();

	friend Transform operator * (const Transform& t1, const Transform& t2);
	static Transform Lerp(const float fPerc, const Transform& t0, const Transform& t1);

public:
	vec3 m_translation;
	mat3 m_rotation;
};

// Some functions for handling computation on angles
inline void ClampAngle(float& angle)
{
	while (angle > M_PI)
	{
		angle -= M2_PI;
	}
	while (angle < -M_PI)
	{
		angle += M2_PI;
	}
}

inline void ClampAngleDeg(float& angle)
{
	while (angle > 180.0f)
	{
		angle -= 360.0f;
	}
	while (angle < -180.0f)
	{
		angle += 360.0f;
	}
}

inline float AngleDiff(float angle0, float angle1)
{
	float value = angle0 - angle1;
	ClampAngle(value);
	return value;
}

inline float Lerp(float v0, float v1, float fPerc)
{
	return v0 + fPerc * (v1 - v0);
}
