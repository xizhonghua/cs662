#if !defined(AFX_HIERARCHY_H__A0078417_DA46_4DEC_96B3_6219FB8E4704__INCLUDED_)
#define AFX_HIERARCHY_H__A0078417_DA46_4DEC_96B3_6219FB8E4704__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Hierarchy.h : header file
//
#include "Resource.h"
#include "Animation.h"

/////////////////////////////////////////////////////////////////////////////
// CHierarchy dialog

class CHierarchy : public CDialog
{
// Construction
public:
	//pass in necessary data for viewing and modifying the skeleton hierarchy
	void GetData(Skeleton* pSkeleton, CHierarchy** temp);
	//build hierarchy tree give the bvh skeleton
	void BuildTree(Joint* pJoint, HTREEITEM item);
	//update skeleton information
	void UpdateInfo(Skeleton* pSkeleton);	

	CHierarchy(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CHierarchy)
	enum { IDD = IDD_HIERARCHY };
	CTreeCtrl	m_Tree;
	CString	m_JointName;
	float	m_Rx;
	float	m_Ry;
	float	m_Rz;
	float	m_Tx;
	float	m_Ty;
	float	m_Tz;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CHierarchy)
	public:
	virtual BOOL DestroyWindow();
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	virtual void PostNcDestroy();
	//}}AFX_VIRTUAL

// Implementation
protected:
	CHierarchy** m_pSelf;
	Skeleton* m_pSkeleton;
	int id;
	// Generated message map functions
	//{{AFX_MSG(CHierarchy)
	//select a new joint if it is double clicked
	afx_msg void OnDblclkTree1(NMHDR* pNMHDR, LRESULT* pResult);
	afx_msg void OnChangeRx();
	afx_msg void OnChangeRy();
	afx_msg void OnChangeRz();
	afx_msg void OnChangeTx();
	afx_msg void OnChangeTy();
	afx_msg void OnChangeTz();
	virtual BOOL OnInitDialog();
	virtual void OnOK();
	virtual void OnCancel();
	afx_msg void OnApply();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_HIERARCHY_H__A0078417_DA46_4DEC_96B3_6219FB8E4704__INCLUDED_)
