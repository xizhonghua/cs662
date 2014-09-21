// JointDlg.cpp : implementation file
//

#include "stdafx.h"
#include "BVHPlayer.h"
#include "JointDlg.h"
#include "drawwind.h"
#include <string>
#include <sstream>
using namespace std;

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CJointDlg dialog


CJointDlg::CJointDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CJointDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CJointDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}


void CJointDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CJointDlg)
	DDX_Control(pDX, IDC_VIEWSCROLLBAR, m_Scroller);
	DDX_Control(pDX, IDC_CURVEVIEW, m_canvas);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CJointDlg, CDialog)
	//{{AFX_MSG_MAP(CJointDlg)
	ON_WM_PAINT()
	ON_WM_HSCROLL()
	ON_WM_MOUSEMOVE()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CJointDlg message handlers

void CJointDlg::DrawCurve(DrawWindow &dw)
{
	dw.SetLineColor(255, 0, 0);
	dw.SetFillColor(0, 255, 255);
	int tmp = m_Scroller.GetScrollPos();
	int x1 = 100 - tmp;
	int y1 = 10;
	dw.Rectangle(x1,y1,x1+90,y1+60);
	ostringstream os;
	os << "scroll = "<<tmp<<"  x = "<<x1;
	
	// Set a small monospaced font, with white characters
	// on a red background.
	dw.SetFont(100,"Courier New");
	dw.SetLineColor(255,255,255);
	dw.SetFillColor(255,0,0);
	
	// Display the formatted text string.
	dw.TextOut(10,150,os.str());
	dw.SetFillColor(0, 255, 0);
	int posx = x - tmp;
	dw.Circle(posx, y, 10);
}

void CJointDlg::OnPaint() 
{
	if (IsIconic()){
		CPaintDC dc(this); // device context for painting
	}
	else{
		DrawWindow dw(m_canvas);
		DrawCurve(dw);
	}
	CDialog::OnPaint();
}

BOOL CJointDlg::OnInitDialog() 
{
	CDialog::OnInitDialog();
	CRect rect;
	m_canvas.GetWindowRect(rect);
	range = rect.Width();
	SCROLLINFO   info;
	info.cbSize = sizeof(SCROLLINFO);
	info.fMask = SIF_ALL; 
	info.nMax = range;
	info.nMin = 0;
	info.nPage = int(range / 10);
	info.nPos = 0;
	m_Scroller.SetScrollInfo(&info);
	x = y = 100;
	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CJointDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar) 
{
	// TODO: Add your message handler code here and/or call default
	int CurPos = m_Scroller.GetScrollPos();
	switch (nSBCode)
	{
	case SB_LEFT:      // Scroll to far left.
		CurPos = 0;
		break;
		
	case SB_RIGHT:      // Scroll to far right.
		CurPos = range;
		break;
		
	case SB_ENDSCROLL:   // End scroll.
		break;
		
	case SB_LINELEFT:      // Scroll left.
		if (CurPos > 0)
			CurPos--;
		break;
		
	case SB_LINERIGHT:   // Scroll right.
		if (CurPos < range)
			CurPos++;
		break;
		
	case SB_PAGELEFT:    // Scroll one page left.
		{
			// Get the page size. 
			SCROLLINFO   info;
			m_Scroller.GetScrollInfo(&info, SIF_ALL);
			
			if (CurPos > 0)
				CurPos = max(0, CurPos - (int) info.nPage);
		}
		break;
		
	case SB_PAGERIGHT:      // Scroll one page right
		{
			// Get the page size. 
			SCROLLINFO   info;
			m_Scroller.GetScrollInfo(&info, SIF_ALL);
			
			if (CurPos < range)
				CurPos = min(range, CurPos + (int) info.nPage);
		}
		break;
		
	case SB_THUMBPOSITION: // Scroll to absolute position. nPos is the position
		CurPos = nPos;      // of the scroll box at the end of the drag operation.
		break;
		
	case SB_THUMBTRACK:   // Drag scroll box to specified position. nPos is the
		CurPos = nPos;     // position that the scroll box has been dragged to.
		break;
	}
	
	// Set the new position of the thumb (scroll box).
	m_Scroller.SetScrollPos(CurPos);
	CDialog::OnHScroll(nSBCode, nPos, pScrollBar);
	Invalidate();
}

void CJointDlg::OnMouseMove(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	if(nFlags & MK_LBUTTON){
		x = point.x + m_Scroller.GetScrollPos();
		y = point.y;
		Invalidate();
	}
	CDialog::OnMouseMove(nFlags, point);
}
