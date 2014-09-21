// BVHPlayerDoc.cpp : implementation of the CBVHPlayerDoc class
//

#include "stdafx.h"
#include "BVHPlayer.h"

#include "BVHPlayerDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CBVHPlayerDoc

IMPLEMENT_DYNCREATE(CBVHPlayerDoc, CDocument)

BEGIN_MESSAGE_MAP(CBVHPlayerDoc, CDocument)
	//{{AFX_MSG_MAP(CBVHPlayerDoc)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CBVHPlayerDoc construction/destruction

CBVHPlayerDoc::CBVHPlayerDoc()
{
	// TODO: add one-time construction code here

}

CBVHPlayerDoc::~CBVHPlayerDoc()
{
}

BOOL CBVHPlayerDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)

	return TRUE;
}



/////////////////////////////////////////////////////////////////////////////
// CBVHPlayerDoc serialization

void CBVHPlayerDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}

/////////////////////////////////////////////////////////////////////////////
// CBVHPlayerDoc diagnostics

#ifdef _DEBUG
void CBVHPlayerDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CBVHPlayerDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CBVHPlayerDoc commands
