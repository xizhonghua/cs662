// BVHPlayerView.cpp : implementation of the CBVHPlayerView class
//

#include "stdafx.h"
#include "BVHPlayer.h"

#include "BVHPlayerView.h"
#include "MyToolBar.h"
#include "MainFrm.h"
#include "Interpolation.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CBVHPlayerView

IMPLEMENT_DYNCREATE(CBVHPlayerView, CView)

BEGIN_MESSAGE_MAP(CBVHPlayerView, CView)
	//{{AFX_MSG_MAP(CBVHPlayerView)
	ON_WM_CREATE()
	ON_WM_SIZE()
	ON_WM_ERASEBKGND()
	ON_WM_MOUSEMOVE()
	ON_COMMAND(ID_FILE_OPEN, OnFileOpen)
	ON_COMMAND(ID_PLAYBWD, OnPlaybwd)
	ON_COMMAND(ID_PLAYFWD, OnPlayfwd)
	ON_COMMAND(ID_STOP, OnStop)
	ON_COMMAND(ID_STEPFWD, OnStepfwd)
	ON_COMMAND(ID_STEPBWD, OnStepbwd)
	ON_COMMAND(ID_COORD, OnCoord)
	ON_UPDATE_COMMAND_UI(ID_COORD, OnUpdateCoord)
	ON_COMMAND(ID_BVH1, OnBvh1)
	ON_UPDATE_COMMAND_UI(ID_BVH1, OnUpdateBvh1)
	ON_COMMAND(ID_BVH2, OnBvh2)
	ON_UPDATE_COMMAND_UI(ID_BVH2, OnUpdateBvh2)
	ON_COMMAND(ID_TRANS, OnTrans)
	ON_UPDATE_COMMAND_UI(ID_TRANS, OnUpdateTrans)
	ON_COMMAND(ID_IK, OnIk)
	ON_COMMAND(ID_IKSTYLE_CYCLICCOORDINATEDESCENT, OnIKStyleJacobianCCD)
	ON_COMMAND(ID_IKSTYLE_JACOBIANTRANSPOSE, OnIKStyleJacobianTranspose)
	ON_COMMAND(ID_IKSTYLE_JACOBIANPSEUDO, OnIKStyleJacobianPseudoInverse)
	ON_UPDATE_COMMAND_UI(ID_IK, OnUpdateIk)
	ON_WM_LBUTTONDBLCLK()
	ON_COMMAND(ID_EDIT_INTERP, OnEditInterp)
	ON_COMMAND(ID_FILE_SAVE, OnFileSave)
	ON_COMMAND(ID_APPLYFRAME, OnApplyframe)
	ON_COMMAND(ID_EDIT_SKELETON, OnEditSkeleton)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CBVHPlayerView construction/destruction

CBVHPlayerView::CBVHPlayerView()
{
	// TODO: add construction code here
	m_hierarchy = NULL;
	m_viewer.SetSkeleton(m_player.GetSkeleton());
}

CBVHPlayerView::~CBVHPlayerView()
{
}

BOOL CBVHPlayerView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

/////////////////////////////////////////////////////////////////////////////
// CBVHPlayerView drawing

void CBVHPlayerView::OnDraw(CDC* pDC)
{
//	CBVHPlayerDoc* pDoc = GetDocument();
//	ASSERT_VALID(pDoc);
	// TODO: add draw code for native data here
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// Clear Screen And Depth Buffer
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	m_camera.LookAt();
	glDisable(GL_LIGHTING);
	m_camera.DrawFloor();
	glEnable(GL_LIGHTING);
	m_viewer.Display();
	SwapBuffers(m_hDC);
}

/////////////////////////////////////////////////////////////////////////////
// CBVHPlayerView diagnostics

#ifdef _DEBUG
void CBVHPlayerView::AssertValid() const
{
	CView::AssertValid();
}

void CBVHPlayerView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

/*CBVHPlayerDoc* CBVHPlayerView::GetDocument() // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CBVHPlayerDoc)));
	return (CBVHPlayerDoc*)m_pDocument;
}*/
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CBVHPlayerView message handlers

int CBVHPlayerView::OnCreate(LPCREATESTRUCT lpCreateStruct) 
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;
	
	// TODO: Add your specialized creation code here
	int nPixelFormat;					// Pixel format index
	m_hDC = ::GetDC(m_hWnd);			// Get the Device context
	
	static PIXELFORMATDESCRIPTOR pfd = {
		sizeof(PIXELFORMATDESCRIPTOR),	// Size of this structure
			1,								// Version of this structure	
			PFD_DRAW_TO_WINDOW |			// Draw to Window (not to bitmap)
			PFD_SUPPORT_OPENGL |            // Support OpenGL clas in window
			PFD_DOUBLEBUFFER,			    // use double buffering
			PFD_TYPE_RGBA,					// RGBA Color mode
			32,								// Want 24bit color 
			0,0,0,0,0,0,					// Not used to select mode
			0,0,							// Not used to select mode
			0,0,0,0,0,						// Not used to select mode
			32,								// Size of depth buffer
			0,								// Not used to select mode
			0,								// Not used to select mode
			PFD_MAIN_PLANE,					// Draw in main plane
			0,								// Not used to select mode
			0,0,0 };						// Not used to select mode
		
		// Choose a pixel format that best matches that described in pfd
		nPixelFormat = ChoosePixelFormat(m_hDC, &pfd);
		
		// Set the pixel format for the device context
		SetPixelFormat(m_hDC, nPixelFormat, &pfd);
		
		// Create the rendering context
		m_hRC = wglCreateContext(m_hDC);
		
		// Make the rendering context current, perform initialization, then
		// deselect it
		wglMakeCurrent(m_hDC,m_hRC);

		glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);				// Background
		glClearDepth(1.0f);									// Depth Buffer Setup
		glEnable(GL_LIGHTING);								// Enable Lighting
		glEnable(GL_LIGHT0);								// First Light Source
		glEnable(GL_NORMALIZE);								// Normalize Surface Normal Vector
		glEnable(GL_BLEND);									// Blend Color								
		glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
//		glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
		glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
		
		//////////////////////////////////////////////////////////////////
		// Do any initialization of the rendering context here, such as
		// setting background colors, setting up lighting, or performing
		// preliminary calculations.
		float lightPosition[4] = {200.0f, 200.0f, 200.0f, 0.0};
		glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

		float lightColor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
		float ambientColor[4] = {0.3f, 0.3f, 0.3f, 1.0f};
		glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
		glLightfv(GL_LIGHT0, GL_SPECULAR, lightColor);	
		
		// erase both buffers:
		glClear(GL_COLOR_BUFFER_BIT);
		SwapBuffers(m_hDC);
		glClear(GL_COLOR_BUFFER_BIT);
		SwapBuffers(m_hDC);
	return 0;
}

void CBVHPlayerView::OnSize(UINT nType, int cx, int cy) 
{
	CView::OnSize(nType, cx, cy);
	
	// TODO: Add your message handler code here
	if (cy==0)										// Prevent A Divide By Zero By
	{
		cy=1;										// Making Height Equal One
	}
	
	glViewport(0,0,cx,cy);						// Reset The Current Viewport
	
	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix	
	// Calculate The Aspect Ratio Of The Window	
	m_camera.Perspective();
	
	glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
	glLoadIdentity();									// Reset The Modelview Matrix
}

BOOL CBVHPlayerView::OnEraseBkgnd(CDC* pDC) 
{
	// TODO: Add your message handler code here and/or call default
	return TRUE;
	//return CView::OnEraseBkgnd(pDC);
}

void CBVHPlayerView::OnMouseMove(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	int dx, dy;
	if ( nFlags & MK_CONTROL )
	{
		if( nFlags & MK_RBUTTON )
		{
			// Zoom camera
			if ( m_lastMouseY != -1 )
			{			
				m_camera.Zoom(((point.y - m_lastMouseY) + (point.x - m_lastMouseX)) / 2.0f);
				OnDraw( NULL );
			}
			m_lastMouseY = point.y;
			m_lastMouseX = point.x;
		}
		else if ( nFlags & MK_MBUTTON )
		{
			// Move camera
			if ( m_lastMouseX != -1 )
			{
				dx = (point.x - m_lastMouseX);
				dy = (point.y - m_lastMouseY);
				
				m_camera.MoveSWD(-dx, dy);
				
				OnDraw( NULL );
			}
			m_lastMouseX = point.x;
			m_lastMouseY = point.y;
		}
		else
			if (nFlags & MK_LBUTTON) {
				// Rotate camera.
				if ( m_lastMouseX != -1 )
				{
					dy = point.y - m_lastMouseY;
					dx = point.x - m_lastMouseX;
					m_camera.RotateCenter(-dx, -dy);
					OnDraw( NULL );
				}
				m_lastMouseX = point.x;
				m_lastMouseY = point.y;
			}
			else{
				m_lastMouseX = -1;
				m_lastMouseY = -1;
			}
	}else{
		m_lastMouseX = -1;
		m_lastMouseY = -1;
		if (m_player.IsValid() && nFlags & MK_SHIFT && nFlags & MK_LBUTTON){
			if(m_player.GetSkeleton()->GetSelectedJoint() != -1 && m_viewer.GetMode() == eIKSelect){
				//Compute the goal position in 3D
				ComputePos(point.x, point.y);
				//Compute IK
				m_player.SolveIK();
				OnDraw(NULL);
			}
		}
	}
	CView::OnMouseMove(nFlags, point);
}

CBVHPlayerView* CBVHPlayerView::GetView()
{
	CMainFrame* pFrame = (CMainFrame*)(AfxGetApp()->m_pMainWnd);
	CBVHPlayerView* pView = (CBVHPlayerView*)(pFrame->GetActiveView());
	if (!pView)
		return NULL;
	return pView;
}

void CBVHPlayerView::OnFileOpen() 
{
	// TODO: Add your command handler code here
	if (m_player.GetCurrentMotionIndex() == Player::eMotion3){
		AfxMessageBox("Cannot load motion into translation result holder.");
		return;
	}
	
	char BASED_CODE szFilter[] = "Biovision (*.bvh)|*.bvh||";  // WILL INCLUDE Biovision Hierarchy BVH (*.bvh)|*.bvh|Acclaim File (*.asf)|*.asf|
	CFileDialog	*dialog;
	///////////////////////////////////////////////////////////////////////////////
	
	dialog = new CFileDialog(TRUE,"bvh",NULL, NULL,szFilter);
	bool status = false;
	if (dialog->DoModal() == IDOK)
	{
		CString filename = dialog->GetPathName();
		ifstream inFile(filename.GetBuffer(0));
		if (inFile.is_open())
		{
			status = m_player.LoadBVHFile(inFile);
		}
		inFile.close();				
	}
	if (status == true)
	{
		CMyToolBar* toolbar = ((CMainFrame*)::AfxGetMainWnd())->getToolBar();
		toolbar->UpdateInfo();
		m_player.UpdateFrame();
		OnDraw(NULL);
	}else
	{
		AfxMessageBox("Cannot load BVH file!");
	}	
}

void CBVHPlayerView::OnPlaybwd() 
{
	// TODO: Add your command handler code here
	CMyToolBar* toolbar = ((CMainFrame*)::AfxGetMainWnd())->getToolBar();
	toolbar->Playbwd();
}

void CBVHPlayerView::OnPlayfwd() 
{
	// TODO: Add your command handler code here
	CMyToolBar* toolbar = ((CMainFrame*)::AfxGetMainWnd())->getToolBar();
	toolbar->Playfwd();
}

void CBVHPlayerView::OnStop() 
{
	// TODO: Add your command handler code here
	CMyToolBar* toolbar = ((CMainFrame*)::AfxGetMainWnd())->getToolBar();
	toolbar->Stop();
}

void CBVHPlayerView::OnStepfwd() 
{
	// TODO: Add your command handler code here
	CMyToolBar* toolbar = ((CMainFrame*)::AfxGetMainWnd())->getToolBar();
	toolbar->Stepfwd();
}

void CBVHPlayerView::OnStepbwd() 
{
	// TODO: Add your command handler code here
	CMyToolBar* toolbar = ((CMainFrame*)::AfxGetMainWnd())->getToolBar();
	toolbar->Stepbwd();
}

void CBVHPlayerView::OnCoord() 
{
	// TODO: Add your command handler code here
	bool state = !(m_viewer.IsDrawCoord());
	m_viewer.SetDrawCoord(state);
	OnDraw(NULL);
}

void CBVHPlayerView::OnUpdateCoord(CCmdUI* pCmdUI) 
{
	// TODO: Add your command update UI handler code here
	pCmdUI->SetCheck(m_viewer.IsDrawCoord());
}

void CBVHPlayerView::SwitchAnimation(const unsigned int& index)
{
	CMyToolBar* toolbar = ((CMainFrame*)::AfxGetMainWnd())->getToolBar();
	m_player.SetCurrentMotion((Player::MotionType)index);
	if (m_hierarchy){
		if (!m_player.IsValid()){
			m_hierarchy->DestroyWindow();
			m_hierarchy = NULL;
		}
		else
			m_hierarchy->UpdateInfo(m_player.GetSkeleton());
	}
	toolbar->UpdateInfo();
}

void CBVHPlayerView::OnBvh1() 
{
	// TODO: Add your command handler code here
	SwitchAnimation(Player::eMotion1);
	m_player.UpdateFrame();
	OnDraw(NULL);
}

void CBVHPlayerView::OnUpdateBvh1(CCmdUI* pCmdUI) 
{
	// TODO: Add your command update UI handler code here
	pCmdUI->SetCheck(m_player.GetCurrentMotionIndex() == Player::eMotion1);
}

void CBVHPlayerView::OnBvh2() 
{
	// TODO: Add your command handler code here
	SwitchAnimation(Player::eMotion2);
	m_player.UpdateFrame();
	OnDraw(NULL);
}

void CBVHPlayerView::OnUpdateBvh2(CCmdUI* pCmdUI) 
{
	// TODO: Add your command update UI handler code here
	pCmdUI->SetCheck(m_player.GetCurrentMotionIndex() == Player::eMotion2);
}

void CBVHPlayerView::OnTrans() 
{
	// TODO: Add your command handler code here
	SwitchAnimation(Player::eMotion3);
	m_player.UpdateFrame();
	OnDraw(NULL);
}

void CBVHPlayerView::OnUpdateTrans(CCmdUI* pCmdUI) 
{
	// TODO: Add your command update UI handler code here
	pCmdUI->SetCheck(m_player.GetCurrentMotionIndex() == Player::eMotion3);
}

void CBVHPlayerView::OnIk() 
{
	// TODO: Add your command handler code here
	if (m_player.IsValid())
	{
		Skeleton* pSkeleton = m_player.GetSkeleton();
		unsigned int mode = m_viewer.GetMode();
		if (mode == eIKSelect)
		{
			m_viewer.SetMode(eFKSelect);
			pSkeleton->SetSelectedJoint(-1);
		}else
		{
			m_viewer.SetMode(eIKSelect);
			pSkeleton->SetSelectedJoint(-1);
		}
	}
	OnDraw(NULL);
}

namespace
{
	void SetIKMenuChecks( const Player::IKType& type )
	{
		const CWnd* pMain = AfxGetMainWnd();
		assert( pMain );
		CMenu* menu = pMain->GetMenu();
		assert( menu );

		// http://stackoverflow.com/questions/1210720/mfc-menu-item-checkbox-behavior
		menu->CheckMenuItem( ID_IKSTYLE_CYCLICCOORDINATEDESCENT, MF_UNCHECKED | MF_BYCOMMAND );
		menu->CheckMenuItem( ID_IKSTYLE_JACOBIANPSEUDO, MF_UNCHECKED | MF_BYCOMMAND );
		menu->CheckMenuItem( ID_IKSTYLE_JACOBIANTRANSPOSE, MF_UNCHECKED | MF_BYCOMMAND );

		int command_id = -1;
		switch( type )
		{
		case Player::eCCD:
			command_id = ID_IKSTYLE_CYCLICCOORDINATEDESCENT;
			break;
		case Player::eJacobianPinv:
			command_id = ID_IKSTYLE_JACOBIANPSEUDO;
			break;
		case Player::eJacobianT:
			command_id = ID_IKSTYLE_JACOBIANTRANSPOSE;
			break;
		default:;
		}
		if( command_id != -1 )
		{
			menu->CheckMenuItem( command_id, MF_CHECKED | MF_BYCOMMAND );
		}
	}
}

void CBVHPlayerView::OnIKStyleJacobianCCD()
{
	SetIKMenuChecks( Player::eCCD );
	m_player.SetIKType( Player::eCCD );
}
void CBVHPlayerView::OnIKStyleJacobianPseudoInverse()
{
	SetIKMenuChecks( Player::eJacobianPinv );
	m_player.SetIKType( Player::eJacobianPinv );
}
void CBVHPlayerView::OnIKStyleJacobianTranspose()
{
	SetIKMenuChecks( Player::eJacobianT );
	m_player.SetIKType( Player::eJacobianT );
}

void CBVHPlayerView::OnUpdateIk(CCmdUI* pCmdUI) 
{
	// TODO: Add your command update UI handler code here
	if (m_player.IsValid())
	{
		pCmdUI->SetCheck(m_viewer.GetMode() == eIKSelect);
	}else
		pCmdUI->SetCheck(false);
}



void CBVHPlayerView::PickObject(int x, int y)
{
	GLuint selectBuf[512];
	GLint hits;
	GLint viewport[4];
	
	int selection;
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glSelectBuffer(512, selectBuf);//Can't be in selection mode here
	glGetIntegerv(GL_VIEWPORT,viewport);
	glRenderMode(GL_SELECT);
	glInitNames() ;
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();		
		glLoadIdentity();			
		gluPickMatrix((GLdouble) x, (GLdouble) (viewport[3] - y), 5.0, 5.0, viewport);	
		m_camera.Perspective();		
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();		
		m_camera.LookAt();
		m_viewer.SetGlPick(true);		
		m_viewer.Display();		
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glFlush();
	
	hits = glRenderMode(GL_RENDER);
	
	selection = ProcessHits(hits, selectBuf);
	
	m_viewer.SetGlPick(false);
	
	m_player.GetSkeleton()->SetSelectedJoint(selection);
}

int CBVHPlayerView::ProcessHits(GLint hits, GLuint buffer[])
{
	GLuint names, *ptr, minZ, *ptrNames, numberOfNames, closestName;
	GLuint ii=0;
	
	ptr = (GLuint *)buffer;
	
	if (!hits)
		return -1;
	
	minZ = 0xffffffff;
	for (int i = 0; i < hits; i++)
	{
		names = *ptr;
		ptr++;
		if (*ptr < minZ) {
			numberOfNames = names;
			minZ = *ptr;
			ptrNames = ptr+2;
		}
		ptr += names+2;
	}
	closestName = *ptrNames;
	return int(closestName);
}

void CBVHPlayerView::ComputePos(int x, int y)
{
	Skeleton* pSkeleton = m_player.GetSkeleton();
	vec3 Pos = pSkeleton->GetGoalPosition();
	vec3 LPos;
	LPos = Pos - m_camera.GetEye();
	vec3 LCoord;	
	LCoord[2] = LPos * m_camera.GetFrontVector();
	
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	float talpha = tan(30.0f * Deg2Rad);
	float tbeta = talpha * float(viewport[2]) / float(viewport[3]);
	LCoord[0] = (x - float(viewport[2])/2.0f) / (float(viewport[2])/2.0f) * tbeta * LCoord[2];
	LCoord[1] = (float(viewport[3])/2.0f - y) / (float(viewport[3])/2.0f) * talpha * LCoord[2];
	
	LPos = m_camera.GetRightVector() * LCoord[0] + m_camera.GetUpVector() * LCoord[1] + m_camera.GetFrontVector() * LCoord[2];
	
	pSkeleton->SetGoalPosition(m_camera.GetEye() + LPos);
}

void CBVHPlayerView::OnFileSave() 
{
	// TODO: Add your command handler code here
	char BASED_CODE szFilter[] = "Biovision (*.bvh)|*.bvh||";  // WILL INCLUDE Biovision Hierarchy BVH (*.bvh)|*.bvh|Acclaim File (*.asf)|*.asf|
	CFileDialog	*dialog;
	///////////////////////////////////////////////////////////////////////////////
	
	dialog = new CFileDialog(FALSE,"bvh",NULL, NULL,szFilter);
	bool status = false;
	if (dialog->DoModal() == IDOK)
	{
		CString filename = dialog->GetPathName();
		ofstream outFile(filename.GetBuffer(0));
		if (outFile.is_open())
		{
			status = m_player.SaveBVHFile(outFile);
		}
		outFile.close();
	}
	if (status == false)
	{
		AfxMessageBox("Cannot save BVH file!");
	}
}

Skeleton* CBVHPlayerView::GetSkeleton()
{
	return m_player.GetSkeleton();
}

Player* CBVHPlayerView::GetPlayer()
{
	return &m_player;
}

Motion* CBVHPlayerView::GetCurrentMotion()
{
	return m_player.GetCurrentMotion();
}

void CBVHPlayerView::OnApplyframe() 
{
	// TODO: Add your command handler code here
	if (m_player.IsValid())
		m_player.SaveFrame();
}

void CBVHPlayerView::OnEditSkeleton() 
{
	// TODO: Add your command handler code here
	if (m_player.IsValid())
	{
		Skeleton* pSkeleton = m_player.GetSkeleton();
		if (m_hierarchy == NULL){
			m_hierarchy = new CHierarchy;
			m_hierarchy->GetData(pSkeleton, &m_hierarchy);
			m_hierarchy->Create(IDD_HIERARCHY, this);
			m_hierarchy->ShowWindow(SW_SHOW);
		}
		m_hierarchy->UpdateInfo(pSkeleton);
	}
	OnDraw(NULL);
}

void CBVHPlayerView::OnEditInterp() 
{
	// TODO: Add your command handler code here
	unsigned int frameCount0, frameCount1, startFrame, endFrame, interpFrame, type;
	if (!m_player.BlendingReady(frameCount0, frameCount1)){
		AfxMessageBox("Load two motions first.");
		return;
	}
	CInterpolation dialog;
	dialog.SetFrames(frameCount0, frameCount1, &startFrame, &endFrame, &interpFrame, &type);
	if (dialog.DoModal() == IDOK){
		m_player.Blend(startFrame, endFrame, interpFrame, (Player::BlendType)type);
		SwitchAnimation(Player::eMotion3);
	}	
}

void CBVHPlayerView::OnLButtonDblClk(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	if (m_player.IsValid())
	{
		PickObject(point.x, point.y);
		Skeleton* pSkeleton = m_player.GetSkeleton();
		if (pSkeleton->GetSelectedJoint() >= 0 && m_viewer.GetMode() == eFKSelect){
			if (m_hierarchy == NULL){
				m_hierarchy = new CHierarchy;
				m_hierarchy->GetData(pSkeleton, &m_hierarchy);
				m_hierarchy->Create(IDD_HIERARCHY, this);
				m_hierarchy->ShowWindow(SW_SHOW);
			}
			m_hierarchy->UpdateInfo(pSkeleton);
			OnDraw(NULL);
		}else
			OnDraw(NULL);
	}
	CView::OnLButtonDblClk(nFlags, point);
}
