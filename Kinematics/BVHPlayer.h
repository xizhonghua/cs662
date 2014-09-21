// BVHPlayer.h : main header file for the BVHPLAYER application
//

#if !defined(AFX_BVHPLAYER_H__46E1C7D4_76EE_4E0C_9821_5262379875A3__INCLUDED_)
#define AFX_BVHPLAYER_H__46E1C7D4_76EE_4E0C_9821_5262379875A3__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"       // main symbols

/////////////////////////////////////////////////////////////////////////////
// CBVHPlayerApp:
// See BVHPlayer.cpp for the implementation of this class
//

class CBVHPlayerApp : public CWinApp
{
public:
	CBVHPlayerApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CBVHPlayerApp)
	public:
	virtual BOOL InitInstance();
	//}}AFX_VIRTUAL

// Implementation
	//{{AFX_MSG(CBVHPlayerApp)
	afx_msg void OnAppAbout();
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_BVHPLAYER_H__46E1C7D4_76EE_4E0C_9821_5262379875A3__INCLUDED_)
