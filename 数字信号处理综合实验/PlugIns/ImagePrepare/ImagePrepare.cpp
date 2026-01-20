// ImagePrepare.cpp : Defines the initialization routines for the DLL.

#include "stdafx.h"
#include "ImagePrepare.h"
//
#include "BufStruct.h"
#include "ImageProc.h"
#include "math.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

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
// CImagePrepareApp
BEGIN_MESSAGE_MAP(CImagePrepareApp, CWinApp)
	//{{AFX_MSG_MAP(CImagePrepareApp)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()
//
/////////////////////////////////////////////////////////////////////////////
// CImagePrepareApp construction
CImagePrepareApp::CImagePrepareApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}
/////////////////////////////////////////////////////////////////////////////
// The one and only CImagePrepareApp object
CImagePrepareApp theApp;

char sInfo[] = "人脸跟踪-摄像头视频流图片截取处理插件";
bool bLastPlugin = false;

DLL_EXP void ON_PLUGIN_BELAST(bool bLast)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//模块状态切换
	bLastPlugin = bLast;
}
//插件名称
DLL_EXP LPCTSTR ON_PLUGININFO(void)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//模块状态切换
	return sInfo;
}
//
DLL_EXP void ON_INITPLUGIN(LPVOID lpParameter)
{   
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//模块状态切换
	//theApp.dlg.Create(IDD_PLUGIN_SETUP);
	//theApp.dlg.ShowWindow(SW_HIDE);
}
DLL_EXP int ON_PLUGINCTRL(int nMode,void* pParameter)
{
//模块状态切换
	AFX_MANAGE_STATE(AfxGetStaticModuleState());
	int nRet = 0;
	return nRet;
}
/*******************************************************************/
//将图片复制到目标图片中的某一区域
/*******************************************************************/
DLL_EXP void CopyToRect(
	         aBYTE* ThisImage,//区域图片指针
			 aBYTE* anImage,//目标图片指针
			 int W,int H,//区域图片大小
		     int DestW,int DestH,//目标图片大小
			 int nvLeft,int nvTop,//区域位置：图片左上角在目标图片的坐标
			 bool bGray//灰度图片标记，true：灰度；false：彩色。
					  )
{
    aBYTE  * lpSrc = ThisImage;
    aBYTE  * lpDes = anImage;
    aBYTE  * lps, * lpd;
    int h;
	ASSERT(ThisImage && anImage);
    ASSERT( nvLeft+W<=DestW && nvTop+H<=DestH && nvLeft>=0 && nvTop>=0 );
	//Y
    for(h=0;h<H;h++){
        lpd = lpDes+(nvTop+h)*DestW+nvLeft;
        lps = lpSrc+(DWORD)h*W;
		memcpy(lpd,lps,W);
    }
	if( bGray )	return;
	//U
	lpSrc += W*H;
	lpDes += DestW*DestH;
    for(h=0;h<H;h++){
        lpd = lpDes+(nvTop+h)*(DestW/2)+nvLeft/2;
        lps = lpSrc+(DWORD)h*W/2;
		memcpy(lpd,lps,W/2);
    }
	//V
	lpSrc += W*H/2;
	lpDes += DestW*DestH/2;
    for(h=0;h<H;h++){
        lpd = lpDes+(nvTop+h)*(DestW/2)+nvLeft/2;
        lps = lpSrc+(DWORD)h*W/2;
		memcpy(lpd,lps,W/2);
    }
}
/****************************************************************************************/
/*                             摄像头视频流图片截取、重采样等处理                       */
/****************************************************************************************/
DLL_EXP void ON_PLUGINRUN(int w,int h,BYTE* pYBits,BYTE* pUBits,BYTE* pVBits,BYTE* pBuffer)
{
//pYBits 大小为w*h
//pUBits 和 pVBits 的大小为 w*h/2
//pBuffer 的大小为 w*h*6
//下面算法都基于一个假设，即w是16的倍数
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//模块状态切换
   
	//ShowDebugMessage("与printf函数用法相似,X=%d,Y=%d\n",10,5);

     //请编写相应处理程序

	if (((BUF_STRUCT*)pBuffer)->bNotInited) {
		((BUF_STRUCT*)pBuffer)->colorBmp = pBuffer + sizeof(BUF_STRUCT);
		((BUF_STRUCT*)pBuffer)->grayBmp = ((BUF_STRUCT*)pBuffer)->colorBmp;
		((BUF_STRUCT*)pBuffer)->clrBmp_1d8 = ((BUF_STRUCT*)pBuffer)->grayBmp + w*h*2;
		((BUF_STRUCT*)pBuffer)->grayBmp_1d16 = ((BUF_STRUCT*)pBuffer)->clrBmp_1d8 + w*h/4;
		((BUF_STRUCT*)pBuffer)->TempImage1d8 = ((BUF_STRUCT*)pBuffer)->grayBmp_1d16 + w*h/16;
		((BUF_STRUCT*)pBuffer)->lastImageQueue1d16m8 = ((BUF_STRUCT*)pBuffer)->TempImage1d8 + w*h/8;
		((BUF_STRUCT*)pBuffer)->pOtherVars = (OTHER_VARS*)(((BUF_STRUCT*)pBuffer)->lastImageQueue1d16m8 + w*h/2);
		((BUF_STRUCT*)pBuffer)->pOtherData = (aBYTE*)((BUF_STRUCT*)pBuffer)->pOtherVars + sizeof(OTHER_VARS);
		for (int i = 0; i <= 7; i++)
			((BUF_STRUCT*)pBuffer)->pImageQueue[i] = ((BUF_STRUCT*)pBuffer)->lastImageQueue1d16m8 + i*(w*h/16);

		((BUF_STRUCT*)pBuffer)->W = w;
		((BUF_STRUCT*)pBuffer)->H = h;
		((BUF_STRUCT*)pBuffer)->cur_allocSize = 0;
		((BUF_STRUCT*)pBuffer)->allocTimes = 0;
		((BUF_STRUCT*)pBuffer)->cur_maxallocsize = 0;
		
		//省略了一些暂时用不到的初始化

		((BUF_STRUCT*)pBuffer)->max_allocSize = w*h*17/16 - sizeof(BUF_STRUCT) - sizeof(OTHER_VARS);
		myHeapAllocInit((BUF_STRUCT*)pBuffer);
		((BUF_STRUCT*)pBuffer)->bNotInited = false;
	}
	
	((BUF_STRUCT*)pBuffer)->displayImage = pYBits;
	
	memcpy(((BUF_STRUCT*)pBuffer)->colorBmp, pYBits, w*h*2);
	
	ReSample(((BUF_STRUCT*)pBuffer)->colorBmp, w, h, w/2, h/4, false, false, ((BUF_STRUCT*)pBuffer)->clrBmp_1d8);
	ReSample(((BUF_STRUCT*)pBuffer)->grayBmp, w, h, w/4, h/4, false, true, ((BUF_STRUCT*)pBuffer)->grayBmp_1d16);

	//下面的步骤用于测试图像产生的结果是否正确。
	
		if( bLastPlugin )
		{
		
			//测试grayBmp_1d16:将grayBmp_1d16复制到显示图片的左上角
			CopyToRect(((BUF_STRUCT*)pBuffer)->grayBmp_1d16, pYBits, w/4, h/4, w, h, 0, 0, true);
			//测试colorBmp:将clrBmp_1d8复制到显示图片的右上角
			CopyToRect(((BUF_STRUCT*)pBuffer)->clrBmp_1d8,  pYBits, w/2, h/4, w, h, w/2, 0, false);
		}
	
}



DLL_EXP void ON_PLUGINEXIT()
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//模块状态切换
	//theApp.dlg.DestroyWindow();
}
