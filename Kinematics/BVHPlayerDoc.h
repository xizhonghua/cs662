// BVHPlayerDoc.h : interface of the CBVHPlayerDoc class
//
/////////////////////////////////////////////////////////////////////////////

#if !defined(AFX_BVHPLAYERDOC_H__42F3644C_9712_409D_9F8F_621FD504A8A2__INCLUDED_)
#define AFX_BVHPLAYERDOC_H__42F3644C_9712_409D_9F8F_621FD504A8A2__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CBVHPlayerDoc : public CDocument
{
protected: // create from serialization only
	CBVHPlayerDoc();
	DECLARE_DYNCREATE(CBVHPlayerDoc)

// Attributes
public:

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CBVHPlayerDoc)
	public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CBVHPlayerDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	//{{AFX_MSG(CBVHPlayerDoc)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_BVHPLAYERDOC_H__42F3644C_9712_409D_9F8F_621FD504A8A2__INCLUDED_)
