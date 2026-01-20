// TraceObject.cpp : Defines the initialization routines for the DLL.
//

#include "stdafx.h"
#include "TraceObject.h"
#include "BufStruct.h"
#include "ImageProc.h"
//#include "TraceFeature.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//
//	Note!
//
//		If this DLL is dynamically linked against the MFC
//		DLLs, any functions exported from this DLL which
//		call into MFC must have the AFX_MANAGE_STATE macro
//		added at the very beginning of the function.
//
//		For example:
//
//		extern "C" BOOL PASCAL EXPORT ExportedFunction()
//		{
//			AFX_MANAGE_STATE(AfxGetStaticModuleState());
//			// normal function body here
//		}
//
//		It is very important that this macro appear in each
//		function, prior to any calls into MFC.  This means that
//		it must appear as the first statement within the 
//		function, even before any object variable declarations
//		as their constructors may generate calls into the MFC
//		DLL.
//
//		Please see MFC Technical Notes 33 and 58 for additional
//		details.
//

/////////////////////////////////////////////////////////////////////////////
// CTraceObjectApp

BEGIN_MESSAGE_MAP(CTraceObjectApp, CWinApp)
	//{{AFX_MSG_MAP(CTraceObjectApp)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CTraceObjectApp construction

CTraceObjectApp::CTraceObjectApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}

/////////////////////////////////////////////////////////////////////////////
// The one and only CTraceObjectApp object

CTraceObjectApp theApp;
#define GOOD_TRACE_DIST 40
#define MIDD_TRACE_DIST 80
//#define BAD_TRACE_DIST 120
#define MAX_DIST_TO_ORG 150

bool bLastPlugin = false;

DLL_INP void InitFeatureVector(FeatureVector* pThis);
DLL_INP bool UpdateVectorsFrom(FeatureVector* pFV, FeatureVector* aFV,int nOrgWeight);
DLL_INP aPOINT CompareFromImage(FeatureVector* pFV, aBYTE* pImageBits, int nLineW,int nH,
                             aRect rcSampleRC, aRect rcRange,int* nMinDist,FeatureVector* theMinFV);
DLL_INP int FV_Distance(FeatureVector* pFV,FeatureVector* aFV,int nFaceClrWeight,int nLevelWeight);
//DLL_INP bool CopyVectorsFrom4P(FeatureVector* pThis, FeatureVector* fv4p);

DLL_EXP void ON_PLUGIN_BELAST(bool bLast)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//Ä£¿é×´Ì¬ÇÐ»»
	bLastPlugin = bLast;
}

char sInfo[] = "ÈËÁ³¸ú×Ù-ÑÛ¾¦±Ç×Ó¸ú×Ù´¦Àí²å¼þ";

DLL_EXP LPCTSTR ON_PLUGININFO(void)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//Ä£¿é×´Ì¬ÇÐ»»
	return sInfo;
}

DLL_EXP void ON_INITPLUGIN(LPVOID lpParameter)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//Ä£¿é×´Ì¬ÇÐ»»
	//theApp.dlg.Create(IDD_PLUGIN_SETUP);
	//theApp.dlg.ShowWindow(SW_HIDE);
}

DLL_EXP int ON_PLUGINCTRL(int nMode,void* pParameter)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//Ä£¿é×´Ì¬ÇÐ»»
	int nRet = 0;
	switch(nMode)
	{
	case 0:
		{
			//theApp.dlg.ShowWindow(SW_SHOWNORMAL);
			//theApp.dlg.SetWindowPos(NULL,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE|SWP_FRAMECHANGED);
		}
		break;
	}
	return nRet;
}


/*******************************************************************/
//ÑÛ±Ç¸ú×Ù
/*******************************************************************/
DLL_EXP void ON_PLUGINRUN(int w,int h,BYTE* pYBits,BYTE* pUBits,BYTE* pVBits,BYTE* pBuffer)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//Ä£¿é×´Ì¬ÇÐ»»
	 //Çë±àÐ´ÏàÓ¦´¦Àí³ÌÐò

}
/*******************************************************************/

/*******************************************************************/
DLL_EXP void ON_PLUGINEXIT()
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//Ä£¿é×´Ì¬ÇÐ»»
	//theApp.dlg.DestroyWindow();
}

