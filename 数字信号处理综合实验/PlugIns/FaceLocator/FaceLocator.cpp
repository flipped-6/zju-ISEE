// FaceLocator.cpp : Defines the initialization routines for the DLL.
#include "stdafx.h"
#include "FaceLocator.h"
#include "BufStruct.h"
#include "ImageProc.h"

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
// CFaceLocatorApp
BEGIN_MESSAGE_MAP(CFaceLocatorApp, CWinApp)
	//{{AFX_MSG_MAP(CFaceLocatorApp)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()
/////////////////////////////////////////////////////////////////////////////
// CFaceLocatorApp construction
CFaceLocatorApp::CFaceLocatorApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}
/////////////////////////////////////////////////////////////////////////////
// The one and only CFaceLocatorApp object
CFaceLocatorApp theApp;
char sInfo[] = "��������-���ڲ�ɫ��Ϣ�������ָ�����";
bool bLastPlugin = false;
DLL_EXP void ON_PLUGIN_BELAST(bool bLast)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//ģ��״̬�л�
	bLastPlugin = bLast;
}
DLL_EXP LPCTSTR ON_PLUGININFO(void)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//ģ��״̬�л�
	return sInfo;
}
DLL_EXP void ON_INITPLUGIN(LPVOID lpParameter)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//ģ��״̬�л�
	//theApp.dlg.Create(IDD_PLUGIN_SETUP);
	//theApp.dlg.ShowWindow(SW_HIDE);
}
DLL_EXP int ON_PLUGINCTRL(int nMode,void* pParameter)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());
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
/****************************************************************************************/
/*                             ��������붨λ                                           */
/****************************************************************************************/
DLL_EXP void erode(aBYTE *src, aBYTE *dst, int w, int h, int N) {
	aBYTE* tempImg1 = myHeapAlloc(w*h);
	memset(tempImg1, 0, w*h);

	int r = (N - 1) / 2;
	int i, j, k;

	for (i = r*w; i < (h - r)*w; i += w) {
		for (j = r; j < w - r; j++) {
			tempImg1[i + j] = 255;
			for (k = i - r*w; k <= i + r*w; k += w) {
				if (src[k + j] == 0) {
					tempImg1[i + j] = 0;
					break;
				}
			}
		}
	}
	for (i = r; i < w - r; i++) {
		for (j = r*w; j < (h - r)*w; j += w) {
			dst[i + j] = 255;
			for (k = i - r; k <= i + r; k++) {
				if (tempImg1[k + j] == 0) {
					dst[i + j] = 0;
					break;
				}
			}
		}
	}
	myHeapFree(tempImg1);
}

DLL_EXP void dilate(aBYTE *src, aBYTE *dst, int w, int h, int N) {
	int r = (N - 1) / 2;
	int i, j, k;
	aBYTE* tempImg1 = myHeapAlloc((w+2*r)*(h+2*r));
	aBYTE* tempImg2 = myHeapAlloc((w+2*r)*(h+2*r));
	memset(tempImg1, 0, (w+2*r)*(h+2*r));
	memset(tempImg2, 0, (w+2*r)*(h+2*r));

	for (i = 0; i < h*w; i += w) {
		for (j = 0; j < w; j++) {
			tempImg2[i + r*(w+2*r) + j + r] = src[i + j];
		}
	}
	for (i = 0; i < h*w; i += w) {
		for (j = 0; j < w; j++) {
			tempImg1[i + j] = 0;
			for (int k = i - r*w; k <= i + r*w; k += w) {
				if (tempImg2[k + j] != 0) {
					tempImg1[i + j] = 255;
					break;
				}
			}
		}
	}
	for (i = 0; i < w; i++) {
		for (j = 0; j < h*w; j += w) {
			dst[i + j] = 0;
			for (k = i - r; k <= i + r; k++) {
				if (tempImg1[k + j] != 0) {
					dst[i + j] = 255;
					break;
				}
			}
		}
	}
	myHeapFree(tempImg2);
	myHeapFree(tempImg1);
}

DLL_EXP void markArea(aBYTE* src, aBYTE* dst, int w, int h) {	
	int* tempImg = (int*)calloc(w*h, sizeof(int));
	int* LK = (int*)calloc(1024, sizeof(int));
	
	int i, j;
	for (i = 0; i < 1024; i++) {
		LK[i] = i;
	}

	//��ʱ���
	int index = 1, left = 0, up = 0, Lmin, Lmax;
	for (i = 0; i < h*w; i += w) {
		for (j = 0; j < w; j++) {
			if (src[i + j] == 0) continue;

			if (i == 0) {
				if (j == 0) {
					tempImg[i + j] = index++;
				} else {
					if (src[i + j - 1] == 255) {
						tempImg[i + j] = tempImg[i + j - 1];
					} else {
						tempImg[i + j] = index++;
					}
				}
			} else if (j == 0) {
				if (src[i + j - w] == 0) {
					tempImg[i + j] = index++;
				} else {
					tempImg[i + j] = tempImg[i + j - w];
				}
			} else {
				if (src[i + j - 1] == 0 && src[i + j - w] == 0) {
					tempImg[i + j] = index++;
				} else if (src[i + j - 1] == 255 && src[i + j - w] == 255) {
					tempImg[i + j] = tempImg[i + j - 1];
					if (tempImg[i + j - w] > tempImg[i + j - 1]) {
						Lmax = tempImg[i + j - w];
						Lmin = tempImg[i + j - 1];
					} else {
						Lmin = tempImg[i + j - w];
						Lmax = tempImg[i + j - 1];
					}
					if (Lmax == Lmin || (left == tempImg[i + j - 1] && up == tempImg[i + j - w])) continue;
					while (LK[Lmax] != Lmax) {
						Lmax = LK[Lmax];
					}
					while (LK[Lmin] != Lmin) {
						Lmin = LK[Lmin];
					}
					if (Lmax > Lmin) {
						LK[Lmax] = Lmin;
					} else {
						LK[Lmin] = Lmax;
					}	
					left = tempImg[i + j - 1];
					up = tempImg[i + j - w];
				} else {
					if (src[i + j - 1] == 255) {
						tempImg[i + j] = tempImg[i + j - 1];
					} else {
						tempImg[i + j] = tempImg[i + j - w];
					}
				}
			}
		}
	}
	
	//�����ȼ۱�
	int temp;
	for (i = 1; i < index; i++) {
		if (LK[i] != LK[LK[i]]) {
			temp = LK[i];
			while (LK[temp] != temp) {
				temp = LK[temp];
			}
			LK[i] = temp;
		}
	}
	
	//��ͨ�������±��
	int type = 1;
	for (i = 1; i < index; i++) {
		if (i == LK[i]) {
			LK[i] = type++;
		} else {
			LK[i] = LK[LK[i]];
		}
	}

	int* pixelSum = (int*)calloc(type, sizeof(int));	
	
	for (i = 0; i < h*w; i += w) {
		for (j = 0; j < w; j++) {
			if (tempImg[i + j] != 0) {
				if (LK[tempImg[i + j]] > 255) {
					ShowDebugMessage("error===========================\n");
				} else {
					//dst[i + j] = 1;
					dst[i + j] = LK[tempImg[i + j]];
					if (dst[i + j] >= type) {
						ShowDebugMessage("error==== %d, type==== %d\n", dst[i + j], type);
						return;
					}
					pixelSum[LK[tempImg[i + j]]]++;
				}
				
			}
		}
	}
	
	int maxIdx = 1;
	for (i = 1; i < type; i++) {
		if (pixelSum[i] > pixelSum[maxIdx]) {
			maxIdx = i;
		}
	}
	
	for (i = 0; i < h*w; i += w) {
		for (j = 0; j < w; j++) {
			if (dst[i + j] == maxIdx) {
				dst[i + j] = 255;
			} else {
				dst[i + j] = 0;
			}
		}
	}
	
	free(pixelSum);
	free(LK);
	free(tempImg);
}




DLL_EXP void ON_PLUGINRUN(int w,int h,BYTE* pYBits,BYTE* pUBits,BYTE* pVBits,BYTE* pBuffer)
{
//pYBits ��СΪw*h
//pUBits �� pVBits �Ĵ�СΪ w*h/2
//pBuffer �Ĵ�СΪ w*h*4
//�����㷨������һ�����裬��w��16�ı���

	AFX_MANAGE_STATE(AfxGetStaticModuleState());//ģ��״̬�л�
	
      //���д��Ӧ��������
	aBYTE* pU = (aBYTE*)(((BUF_STRUCT*)pBuffer)->clrBmp_1d8) + w*h/8;
	aBYTE* pV = pU + w*h/16;

	aBYTE* tempImg = myHeapAlloc(w*h/16);
	for (int i = 0; i < w*h/16; i++) {
		if (pU[i] >= 85 && pU[i] <= 126 && pV[i] >= 130 && pV[i] <= 165) {
			tempImg[i] = 255; 
		} else {
			tempImg[i] = 0;
		}
	}

	erode(tempImg, tempImg, w/4, h/4, 3);	
	dilate(tempImg, tempImg, w/4, h/4, 3);
	dilate(tempImg, tempImg, w/4, h/4, 7);
	erode(tempImg, tempImg, w/4, h/4, 7);

	markArea(tempImg, tempImg, w/4, w/4);
	
	// ShowDebugMessage("====================\n");
	
	aRect face;
	faceLoc(tempImg, &face, w/4, h/4, pBuffer);
	DrawRectangle(((BUF_STRUCT*)pBuffer)->displayImage, w, h, face, TYUV1(0, 191, 255), false);
	
	
	myHeapFree(tempImg);

}

DLL_EXP void ON_PLUGINEXIT()
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());//ģ��״̬�л�
	//theApp.dlg.DestroyWindow();
}

