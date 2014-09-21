// Hierarchy.cpp : implementation file
//

#include "stdafx.h"
#include "BVHPlayer.h"
#include "Hierarchy.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#include "BVHPlayerView.h"

/////////////////////////////////////////////////////////////////////////////
// CHierarchy dialog

void CHierarchy::GetData(Skeleton *pSkeleton, CHierarchy **pSelf)
{
	//pass in necessary information
	//me is a pointer to the pointer of current skeleton
	//On exit, (*me) will be set to NULL, so that system knows that the hierarchy window is destroyed
	//current is a pointer to the skeleton of current bvh motion
	//you need to utilize current to access necessary informations and perform necessary modifications
	m_pSelf = pSelf;
	m_pSkeleton = pSkeleton;
}

CHierarchy::CHierarchy(CWnd* pParent /*=NULL*/)
	: CDialog(CHierarchy::IDD , pParent)
//				IDD_HIERARCHY, pParent)
{
	//{{AFX_DATA_INIT(CHierarchy)
	m_JointName = _T("");
	m_Rx = 0.0f;
	m_Ry = 0.0f;
	m_Rz = 0.0f;
	m_Tx = 0.0f;
	m_Ty = 0.0f;
	m_Tz = 0.0f;
	//}}AFX_DATA_INIT
}


void CHierarchy::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CHierarchy)
	DDX_Control(pDX, IDC_TREE1, m_Tree);
	DDX_Text(pDX, IDC_JOINTNAME, m_JointName);
	DDX_Text(pDX, IDC_RX, m_Rx);
	DDX_Text(pDX, IDC_RY, m_Ry);
	DDX_Text(pDX, IDC_RZ, m_Rz);
	DDX_Text(pDX, IDC_TX, m_Tx);
	DDX_Text(pDX, IDC_TY, m_Ty);
	DDX_Text(pDX, IDC_TZ, m_Tz);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CHierarchy, CDialog)
	//{{AFX_MSG_MAP(CHierarchy)
	ON_NOTIFY(NM_DBLCLK, IDC_TREE1, OnDblclkTree1)
	ON_EN_CHANGE(IDC_RX, OnChangeRx)
	ON_EN_CHANGE(IDC_RY, OnChangeRy)
	ON_EN_CHANGE(IDC_RZ, OnChangeRz)
	ON_EN_CHANGE(IDC_TX, OnChangeTx)
	ON_EN_CHANGE(IDC_TY, OnChangeTy)
	ON_EN_CHANGE(IDC_TZ, OnChangeTz)
	ON_WM_CLOSE()
	ON_BN_CLICKED(ID_APPLY, OnApply)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CHierarchy message handlers

void CHierarchy::OnChangeRx() 
{
	// TODO: If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.
	
	// TODO: Add your control notification handler code here
	UpdateData();
	ClampAngleDeg(m_Rx);
	UpdateData(FALSE);
	Joint* pJoint = m_pSkeleton->GetJointByID(id);
	vec3 angles(m_Rx, m_Ry, m_Rz);
	angles *= Deg2Rad;
	mat3 rot;
	rot.FromEulerAnglesZXY(angles);
	pJoint->SetLocalRotation(rot);
	pJoint->UpdateTransformation(true);
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	pView->OnDraw(NULL);
}

void CHierarchy::OnChangeRy() 
{
	// TODO: If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.
	
	// TODO: Add your control notification handler code here
	UpdateData();
	ClampAngleDeg(m_Ry);
	UpdateData(FALSE);
	Joint* pJoint = m_pSkeleton->GetJointByID(id);
	vec3 angles(m_Rx, m_Ry, m_Rz);
	angles *= Deg2Rad;
	mat3 rot;
	rot.FromEulerAnglesZXY(angles);
	pJoint->SetLocalRotation(rot);
	pJoint->UpdateTransformation(true);
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	pView->OnDraw(NULL);
}

void CHierarchy::OnChangeRz() 
{
	// TODO: If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.
	
	// TODO: Add your control notification handler code here
	UpdateData();
	ClampAngleDeg(m_Rz);
	UpdateData(FALSE);
	Joint* pJoint = m_pSkeleton->GetJointByID(id);
	vec3 angles(m_Rx, m_Ry, m_Rz);
	angles *= Deg2Rad;
	mat3 rot;
	rot.FromEulerAnglesZXY(angles);
	pJoint->SetLocalRotation(rot);
	pJoint->UpdateTransformation(true);
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	pView->OnDraw(NULL);
}

void CHierarchy::OnChangeTx() 
{
	// TODO: If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.
	
	// TODO: Add your control notification handler code here
	UpdateData();
	Joint* pJoint = m_pSkeleton->GetJointByID(id);
	vec3 trans = pJoint->GetLocalTranslation();
	trans[VX] = m_Tx;
	pJoint->SetLocalTranslation(trans);
	pJoint->UpdateTransformation(true);
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	pView->OnDraw(NULL);
}

void CHierarchy::OnChangeTy() 
{
	// TODO: If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.
	
	// TODO: Add your control notification handler code here
	UpdateData();
	Joint* pJoint = m_pSkeleton->GetJointByID(id);
	vec3 trans = pJoint->GetLocalTranslation();
	trans[VY] = m_Ty;
	pJoint->SetLocalTranslation(trans);
	pJoint->UpdateTransformation(true);
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	pView->OnDraw(NULL);

}

void CHierarchy::OnChangeTz() 
{
	// TODO: If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.
	
	// TODO: Add your control notification handler code here
	UpdateData();
	Joint* pJoint = m_pSkeleton->GetJointByID(id);
	vec3 trans = pJoint->GetLocalTranslation();
	trans[VZ] = m_Tz;
	pJoint->SetLocalTranslation(trans);
	pJoint->UpdateTransformation(true);
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	pView->OnDraw(NULL);

}

void CHierarchy::PostNcDestroy() 
{
	// TODO: Add your specialized code here and/or call the base class
	CDialog::PostNcDestroy();
	delete this;
}

BOOL CHierarchy::OnInitDialog() 
{
	CDialog::OnInitDialog();
	m_Tree.DeleteAllItems();
	BuildTree(m_pSkeleton->GetRootJoint(), NULL);
	// TODO: Add extra initialization here
	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CHierarchy::OnOK() 
{
	// TODO: Add extra validation here	
	CDialog::OnOK();
	DestroyWindow();
}

BOOL CHierarchy::DestroyWindow() 
{
	// TODO: Add your specialized code here and/or call the base class
	(*m_pSelf) = NULL;
	m_pSkeleton->SetSelectedJoint(-1);
	return CDialog::DestroyWindow();
}

void CHierarchy::OnCancel() 
{
	// TODO: Add extra cleanup here
	CDialog::OnCancel();
	DestroyWindow();
}

void CHierarchy::OnApply() 
{
	// TODO: Add your control notification handler code here
	UpdateData();	
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	pView->GetPlayer()->SaveFrame();
}

/*
 *	You should read all the functions above to get some understanding of this class
 *  You need to code the following functions
 */


void CHierarchy::BuildTree(Joint *pJoint, HTREEITEM item)
{
	//Build the hierarchy tree recursively	
	HTREEITEM newItem;
	if (pJoint->GetName() == "Site"){
		newItem = m_Tree.InsertItem((pJoint->GetName()).c_str(), 0 , 0, item);
		m_Tree.SetItemData(newItem, (DWORD)pJoint->GetID());
		m_Tree.EnsureVisible(newItem);
		return;
	}else{
		if (pJoint->GetID() == 0){
			newItem = m_Tree.InsertItem((pJoint->GetName()).c_str(), 0 , 0);
		}else{
			newItem = m_Tree.InsertItem((pJoint->GetName()).c_str(), 0 , 0, item);
		}		
		m_Tree.SetItemData(newItem, (DWORD)pJoint->GetID());
		m_Tree.EnsureVisible(newItem);

		unsigned int totalChildren = pJoint->GetChildCount();
		for (unsigned int i = 0; i < totalChildren; i++)
		{
			BuildTree(pJoint->GetChildAt(i), newItem);
		}
	}	
}

void CHierarchy::UpdateInfo(Skeleton *pSkeleton)
{
	if (pSkeleton->GetSelectedJoint() < 0){
		//If no joint is selected, select the root by default
		pSkeleton->SetSelectedJoint(0);	
	}
	//Setup the information table:
	//Joint name, joint local translation and local rotation values
	//Only for the root joint, you are allowed to modify the translations
	id = pSkeleton->GetSelectedJoint();
	Joint* pJoint = pSkeleton->GetJointByID(id);
	m_JointName = (pJoint->GetName()).c_str();
	vec3 v;
	pJoint->GetLocalRotation().ToEulerAnglesZXY(v);
	v *= Rad2Deg;
	m_Rx = v[VX];
	m_Ry = v[VY];
	m_Rz = v[VZ];
	CEdit* box;
	if (id == 0){
		box = (CEdit*)GetDlgItem(IDC_TX);
		box->SetReadOnly(FALSE);
		box = (CEdit*)GetDlgItem(IDC_TY);
		box->SetReadOnly(FALSE);
		box = (CEdit*)GetDlgItem(IDC_TZ);
		box->SetReadOnly(FALSE);
	}else{		
		box = (CEdit*)GetDlgItem(IDC_TX);
		box->SetReadOnly(TRUE);
		box = (CEdit*)GetDlgItem(IDC_TY);
		box->SetReadOnly(TRUE);
		box = (CEdit*)GetDlgItem(IDC_TZ);
		box->SetReadOnly(TRUE);		
	}
	v = pJoint->GetLocalTranslation();
	m_Tx = v[VX];
	m_Ty = v[VY];
	m_Tz = v[VZ];
	UpdateData(FALSE);
}

void CHierarchy::OnDblclkTree1(NMHDR* pNMHDR, LRESULT* pResult) 
{
	// TODO: Add your control notification handler code here
	// Handle event of double click on a tree node
	// Display information of that selected joint
	HTREEITEM m_selected = m_Tree.GetSelectedItem();
	string s = m_Tree.GetItemText(m_selected);
	id = (int)m_Tree.GetItemData(m_selected);
	if (id != -1){
		m_JointName = s.c_str();
		CEdit* box;
		if (id == 0){
			box = (CEdit*)GetDlgItem(IDC_TX);
			box->SetReadOnly(FALSE);
			box = (CEdit*)GetDlgItem(IDC_TY);
			box->SetReadOnly(FALSE);
			box = (CEdit*)GetDlgItem(IDC_TZ);
			box->SetReadOnly(FALSE);
		}else{
			box = (CEdit*)GetDlgItem(IDC_TX);
			box->SetReadOnly(TRUE);
			box = (CEdit*)GetDlgItem(IDC_TY);
			box->SetReadOnly(TRUE);
			box = (CEdit*)GetDlgItem(IDC_TZ);
			box->SetReadOnly(TRUE);
		}
		Joint* pJoint = m_pSkeleton->GetJointByID(id);
		vec3 v;
		v = pJoint->GetLocalTranslation();
		m_Tx = v[VX];
		m_Ty = v[VY];
		m_Tz = v[VZ];
		pJoint->GetLocalRotation().ToEulerAnglesZXY(v);
		v *= Rad2Deg;
		m_Rx = v[VX];
		m_Ry = v[VY];
		m_Rz = v[VZ];
	}
	UpdateData(FALSE);
	//Update selected joint information of current skeleton
	//So that the view highlights the newly selected joint
	m_pSkeleton->SetSelectedJoint(id);
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	pView->OnDraw(NULL);
	*pResult = 1;
}