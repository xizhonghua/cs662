#if !defined(AFX_JOINTDLG_H__C6D92D8F_DC20_4DC1_B6F6_5DD2AA2EC97F__INCLUDED_)
#define AFX_JOINTDLG_H__C6D92D8F_DC20_4DC1_B6F6_5DD2AA2EC97F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// JointDlg.h : header file
//
#include "drawwind.h"
/////////////////////////////////////////////////////////////////////////////
// CJointDlg dialog

class CJointDlg : public CDialog
{
// Construction
public:
	void DrawCurve(DrawWindow& dw);
	CJointDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CJointDlg)
	enum { IDD = IDD_CURVE };
	CScrollBar	m_Scroller;
	CStatic	m_canvas;
	//}}AFX_DATA
	int range;
	int x,y;

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CJointDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CJointDlg)
	afx_msg void OnPaint();
	virtual BOOL OnInitDialog();
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_JOINTDLG_H__C6D92D8F_DC20_4DC1_B6F6_5DD2AA2EC97F__INCLUDED_)
