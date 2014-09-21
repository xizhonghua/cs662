#if !defined(AFX_INTERPOLATION_H__D2EF31DA_05D5_434A_8975_887A7C2BA305__INCLUDED_)
#define AFX_INTERPOLATION_H__D2EF31DA_05D5_434A_8975_887A7C2BA305__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Interpolation.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CInterpolation dialog
class CInterpolation : public CDialog
{
// Construction
public:
	void SetFrames(const unsigned int& frameCount0, const unsigned int& frameCount1,
				 unsigned int* startFrame, unsigned int* endFrame, unsigned int* interpFrame, unsigned int* type);
	CInterpolation(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	//{{AFX_DATA(CInterpolation)
	enum { IDD = IDD_INTERP };
	UINT	m_endFrame;
	UINT	m_interpFrame;
	UINT	m_startFrame;
	//}}AFX_DATA
	CComboBox* m_ComboBox;
	unsigned int* m_pType;
	unsigned int* m_pStartFrame;
	unsigned int* m_pEndFrame;
	unsigned int* m_pInterpFrame;
	unsigned int m_frameCount0, m_frameCount1;

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CInterpolation)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CInterpolation)
	virtual BOOL OnInitDialog();
	virtual void OnOK();
	afx_msg void OnSelchangeComboType();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_INTERPOLATION_H__D2EF31DA_05D5_434A_8975_887A7C2BA305__INCLUDED_)
