// Interpolation.cpp : implementation file
//

#include "stdafx.h"
#include "BVHPlayer.h"
#include "Interpolation.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CInterpolation dialog


CInterpolation::CInterpolation(CWnd* pParent /*=NULL*/)
	: CDialog(CInterpolation::IDD, pParent)
{
	//{{AFX_DATA_INIT(CInterpolation)
	m_endFrame = 0;
	m_interpFrame = 0;
	m_startFrame = 0;
	//}}AFX_DATA_INIT
}


void CInterpolation::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CInterpolation)
	DDX_Text(pDX, IDC_EFRAME, m_endFrame);
	DDX_Text(pDX, IDC_IFRAME, m_interpFrame);
	DDX_Text(pDX, IDC_SFRAME, m_startFrame);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CInterpolation, CDialog)
	//{{AFX_MSG_MAP(CInterpolation)
	ON_CBN_SELCHANGE(IDC_COMBO_TYPE, OnSelchangeComboType)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CInterpolation message handlers

BOOL CInterpolation::OnInitDialog() 
{
	CDialog::OnInitDialog();
	
	// TODO: Add extra initialization here
	m_ComboBox = (CComboBox*)GetDlgItem(IDC_COMBO_TYPE);
	m_ComboBox->AddString("Cubic");
	m_ComboBox->AddString("SLERP");
	m_ComboBox->AddString("SQUAD");
	m_ComboBox->SetCurSel(0);
	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CInterpolation::SetFrames(const unsigned int& frameCount0, const unsigned int& frameCount1,
							 unsigned int* startFrame, unsigned int* endFrame, unsigned int* interpFrame, unsigned int* type)
{
	m_frameCount0 = frameCount0;
	m_frameCount1 = frameCount1;
	m_pStartFrame = startFrame;
	m_pEndFrame = endFrame;
	m_pInterpFrame = interpFrame;
	m_pType = type;
	*m_pType = 0;

}

void CInterpolation::OnOK() 
{
	// TODO: Add extra validation here
	UpdateData();
	char buffer[128];
	if (m_startFrame < 1 || m_startFrame > m_frameCount0 - 2){
		sprintf_s(buffer, 128, "Start frame must be between 1 and %d", m_frameCount0 - 2);
		AfxMessageBox(buffer);
		return;
	}
	if (m_endFrame < 1 || m_endFrame > m_frameCount1 - 2){
		sprintf_s(buffer, 128, "Start frame must be between 1 and %d", m_frameCount1 - 2);
		AfxMessageBox(buffer);
		return;
	}
	*m_pStartFrame = m_startFrame;
	*m_pEndFrame = m_endFrame;
	*m_pInterpFrame = m_interpFrame;
	CDialog::OnOK();
}

void CInterpolation::OnSelchangeComboType() 
{
	// TODO: Add your control notification handler code here
	*m_pType = m_ComboBox->GetCurSel();
}
