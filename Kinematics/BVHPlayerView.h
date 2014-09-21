// BVHPlayerView.h : interface of the CBVHPlayerView class
//
/////////////////////////////////////////////////////////////////////////////

#if !defined(AFX_BVHPLAYERVIEW_H__2022A6A9_B024_4364_A670_00021505A51B__INCLUDED_)
#define AFX_BVHPLAYERVIEW_H__2022A6A9_B024_4364_A670_00021505A51B__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Camera.h"
#include "Hierarchy.h"

#include "Animation.h"
#include "OpenGLViewer.h"

class CBVHPlayerView : public CView
{
protected: // create from serialization only
	CBVHPlayerView();
	DECLARE_DYNCREATE(CBVHPlayerView)

// Attributes
public:
	Player m_player;
	OpenGLViewer m_viewer;
	HGLRC m_hRC;
	HDC m_hDC;
	Camera m_camera;
	CHierarchy* m_hierarchy;
	int	m_lastMouseX, m_lastMouseY;	
// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CBVHPlayerView)
	public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	protected:
	//}}AFX_VIRTUAL

// Implementation
public:
	Skeleton* GetSkeleton(void);
	Player* GetPlayer(void);
	Motion* GetCurrentMotion(void);
	//Compute the position in 3D given a point on 2D screen
	void ComputePos(int x, int y);
	//Find out the joint that is selected
	int ProcessHits(GLint hits, GLuint buffer[]);
	//Pick the joint when user selects it
	void PickObject(int x, int y);
	//Switch among the animations and update necessary informations
	void SwitchAnimation(const unsigned int&);
	//Get the pointer to the view class
	static CBVHPlayerView* GetView();
	virtual ~CBVHPlayerView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	//{{AFX_MSG(CBVHPlayerView)
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnFileOpen();
	afx_msg void OnPlaybwd();
	afx_msg void OnPlayfwd();
	afx_msg void OnStop();
	afx_msg void OnStepfwd();
	afx_msg void OnStepbwd();
	afx_msg void OnCoord();
	afx_msg void OnUpdateCoord(CCmdUI* pCmdUI);
	afx_msg void OnBvh1();
	afx_msg void OnUpdateBvh1(CCmdUI* pCmdUI);
	afx_msg void OnBvh2();
	afx_msg void OnUpdateBvh2(CCmdUI* pCmdUI);
	afx_msg void OnTrans();
	afx_msg void OnUpdateTrans(CCmdUI* pCmdUI);
	afx_msg void OnIk();
	afx_msg void OnUpdateIk(CCmdUI* pCmdUI);
	afx_msg void OnIKStyleJacobianCCD();
	afx_msg void OnIKStyleJacobianPseudoInverse();
	afx_msg void OnIKStyleJacobianTranspose();
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
	afx_msg void OnEditInterp();
	afx_msg void OnFileSave();
	afx_msg void OnApplyframe();
	afx_msg void OnEditSkeleton();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};
/*
#ifndef _DEBUG  // debug version in BVHPlayerView.cpp
inline CBVHPlayerDoc* CBVHPlayerView::GetDocument()
   { return (CBVHPlayerDoc*)m_pDocument; }
#endif
*/
/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_BVHPLAYERVIEW_H__2022A6A9_B024_4364_A670_00021505A51B__INCLUDED_)
