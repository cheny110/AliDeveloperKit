#include "main.h"
#include "WindowsCB.h"
#include "GUI.h"

void menuWinCB(WM_MESSAGE *pMsg){

    switch (pMsg->MsgId)
    {
    case WM_PAINT:
        /* code */
        GUI_SetBkColor(GUI_RED);
        GUI_Clear();
        GUI_SetColor(GUI_BLACK);
        GUI_DispStringAt("Hello,world!!",10,10);
        break;
    
    default:
        WM_DefaultProc(pMsg);
        break;
    }
}