// MyToolBar.cpp : implementation file
//

#include "stdafx.h"
#include "MyToolBar.h"
#include "BVHPlayerView.h"
#include "MainFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define ANIMATIONTIMER 1

/////////////////////////////////////////////////////////////////////////////
// CMyToolBar

CMyToolBar::CMyToolBar()
{
	mode = STP;
	deltaFrame = 0;
}

CMyToolBar::~CMyToolBar()
{
}

bool CMyToolBar::CreateEdit(int nToolBarBtnID,int nEditID)
{
    // only allow a single call to this method since we can only
    // handle a single combo box.
    if ( ::IsWindow(m_FrameInfo.m_hWnd) )
		return false;
    int nIdx = CommandToIndex(nToolBarBtnID);
    ASSERT( nIdx >= 0 );
    SetButtonInfo(nIdx, nEditID, TBBS_SEPARATOR, 120);
    CRect rect;
    GetItemRect(nIdx, &rect);
	rect.top += 2;
	rect.bottom += 2;
	if (!m_FrameInfo.Create(ES_READONLY | ES_RIGHT, rect, this, nEditID))
    {
        TRACE0("Failed to create combobox in ToolBar\n");
        return false;
    }
    m_FrameInfo.ShowWindow( SW_SHOWNORMAL );
    return true;
}

bool CMyToolBar::CreateSlider(int nToolBarBtnID,int nSliderID)
{
	// only allow a single call to this method since we can only
    // handle a single combo box.
    if ( ::IsWindow(m_FrameSlider.m_hWnd) )
		return false;
    int nIdx = CommandToIndex(nToolBarBtnID);
    ASSERT( nIdx >= 0 );
    SetButtonInfo(nIdx, nSliderID, TBBS_SEPARATOR, 200);
    CRect rect;
    GetItemRect(nIdx, &rect);
	rect.bottom = 20;
	if (!m_FrameSlider.Create(TBS_HORZ, rect, this, nSliderID))
    {
        TRACE0("Failed to create slider box in ToolBar\n");
        return false;
    }
    m_FrameSlider.ShowWindow( SW_SHOWNORMAL );
	return true;
}

BEGIN_MESSAGE_MAP(CMyToolBar, CToolBar)
	//{{AFX_MSG_MAP(CMyToolBar)
	ON_WM_HSCROLL()
	ON_WM_TIMER()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CMyToolBar message handlers

void CMyToolBar::SetSliderRange(int maxV)
{
	m_FrameSlider.SetRange(0, maxV);
}

void CMyToolBar::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar) 
{
	// TODO: Add your message handler code here and/or call default
	int frame = m_FrameSlider.GetPos();
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	Motion* pMotion = pView->GetCurrentMotion();
	if (pMotion == NULL || pMotion->GetFrameCount() == 0)
		return;	
	int frameCount = int(pMotion->GetFrameCount());
	if (frame >= frameCount){
		frame = frameCount - 1;
	}
	FrameInfo(frame);
	pMotion->SetCurrentFrameCount(unsigned int(frame));
	CToolBar::OnHScroll(nSBCode, nPos, pScrollBar);
	pView->GetPlayer()->UpdateFrame();
	pView->OnDraw(NULL);
}

void CMyToolBar::FrameInfo(int frame)
{
	CString s;
	s.Format("%d / %d Frames", frame, m_FrameSlider.GetRangeMax());
	m_FrameInfo.SetWindowText(s);
}

void CMyToolBar::SetPos(int frame)
{
	m_FrameSlider.SetPos(frame);
}

void CMyToolBar::OnTimer(UINT nIDEvent) 
{
	// TODO: Add your message handler code here and/or call default
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	Motion* pMotion = pView->GetCurrentMotion();
	if (pMotion == NULL || pMotion->GetFrameCount() == 0)
	{
		CWnd::KillTimer(ANIMATIONTIMER);
		return;
	}
	if (nIDEvent == ANIMATIONTIMER && pMotion != NULL){		
		pMotion->Update(deltaFrame);
		int num = int(pMotion->GetCurrentFrameCount());
		FrameInfo(num);
		m_FrameSlider.SetPos(num);
		pView->GetPlayer()->UpdateFrame();
		pView->OnDraw(NULL);
	}

	CToolBar::OnTimer(nIDEvent);
}


void CMyToolBar::Playfwd() 
{
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	Motion* pMotion = pView->GetCurrentMotion();
	if (pMotion == NULL || pMotion->GetFrameCount() == 0)
		return;	
	if (mode != PFW){
		if (mode != PBW)
			CWnd::SetTimer(ANIMATIONTIMER, 33, 0);
		deltaFrame = 1;
		mode = PFW;
	}
}

void CMyToolBar::Playbwd()
{
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	Motion* pMotion = pView->GetCurrentMotion();
	if (pMotion == NULL || pMotion->GetFrameCount() == 0)
		return;	
	if (mode != PBW){
		if (mode != PFW)
			CWnd::SetTimer(ANIMATIONTIMER, 33, 0);
		deltaFrame = -1;
		mode = PBW;
	}

}

void CMyToolBar::Stepfwd()
{
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	Motion* pMotion = pView->GetCurrentMotion();
	if (pMotion == NULL || pMotion->GetFrameCount() == 0)
		return;	
	if (mode == PFW || mode == PBW)
		CWnd::KillTimer(ANIMATIONTIMER);
	deltaFrame = 0;
	mode = SFW;
	pMotion->Update(1);
	int num = int(pMotion->GetCurrentFrameCount());
	FrameInfo(num);
	m_FrameSlider.SetPos(num);
	pView->GetPlayer()->UpdateFrame();
	pView->OnDraw(NULL);
}

void CMyToolBar::Stepbwd()
{
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	Motion* pMotion = pView->GetCurrentMotion();
	if (pMotion == NULL || pMotion->GetFrameCount() == 0)
		return;	
	if (mode == PFW || mode == PBW)
		CWnd::KillTimer(ANIMATIONTIMER);
	deltaFrame = 0;
	mode = SBW;
	pMotion->Update(-1);
	int num = int(pMotion->GetCurrentFrameCount());
	FrameInfo(num);
	m_FrameSlider.SetPos(num);
	pView->GetPlayer()->UpdateFrame();
	pView->OnDraw(NULL);

}

void CMyToolBar::Stop()
{
	if (mode == PFW || mode == PBW)
		CWnd::KillTimer(ANIMATIONTIMER);
	deltaFrame = 0;
	mode = STP;
}

void CMyToolBar::UpdateInfo()
{
	CBVHPlayerView* pView = CBVHPlayerView::GetView();
	Motion* pMotion = pView->GetCurrentMotion();
	if (pMotion && pMotion->GetFrameCount() > 0){

		SetSliderRange(pMotion->GetFrameCount());
		SetPos(pMotion->GetCurrentFrameCount());
		FrameInfo(pMotion->GetCurrentFrameCount());
	}else{
		SetSliderRange(0);
		SetPos(0);
		FrameInfo(0);
		
	}	
}
