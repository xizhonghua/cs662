#if !defined(AFX_MYTOOLBAR_H__E498B744_8BFA_4A03_A8F2_27DE08DE4D67__INCLUDED_)
#define AFX_MYTOOLBAR_H__E498B744_8BFA_4A03_A8F2_27DE08DE4D67__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// MyToolBar.h : header file
//
/////////////////////////////////////////////////////////////////////////////
// CMyToolBar window

enum PlayMode {PFW, PBW, STP, SFW, SBW};

class CMyToolBar : public CToolBar
{
// Construction
public:
	CMyToolBar();

// Attributes
public:
	CEdit m_FrameInfo;
	CSliderCtrl m_FrameSlider;
	PlayMode mode;
	int deltaFrame;
// Operations
public:
	
// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CMyToolBar)
	//}}AFX_VIRTUAL

// Implementation
public:
	void UpdateInfo();
	void Stop();
	void Stepbwd();
	void Stepfwd();
	void Playfwd();
	bool CreateEdit(int nToolBarBtnID, int nComboID);
	bool CreateSlider(int nToolBarBtnID, int nSliderID);
	void Playbwd();
	void SetPos(int frame);
	void FrameInfo(int frame);
	void SetSliderRange(int maxV);
	virtual ~CMyToolBar();

	// Generated message map functions
	
protected:
	//{{AFX_MSG(CMyToolBar)
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnTimer(UINT nIDEvent);
	//}}AFX_MSG

	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_MYTOOLBAR_H__E498B744_8BFA_4A03_A8F2_27DE08DE4D67__INCLUDED_)
