#define WIN32_LEAN_AND_MEAN
#define NOCRYPT
#define NOSERVICE
#define NOMCX
#define NOIME

#include <windows.h>
#include <windowsx.h>

#include <commctrl.h>
#include <dbt.h>
#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>

#include "SerialPort.h"
#include "main.h"

#define NELEMS(a) (sizeof(a) / sizeof((a)[0]))

static INT_PTR CALLBACK MainDlgProc(HWND, UINT, WPARAM, LPARAM);
static BOOL MainDlg_OnInitDialog(HWND hwnd, HWND hwndFocus, LPARAM lParam);
static void MainDlg_OnCommand(HWND hwnd, int id, HWND hwndCtl, UINT codeNotify);
static BOOL MainDlg_OnDeviceChange(HWND hwnd, UINT nEventType,
                                   DWORD_PTR dwData);
static LRESULT MainDlg_OnNotify(HWND hwnd, INT id, LPNMHDR pnm);
static void MainDlg_OnClose(HWND hwnd);

static HANDLE ghInstance;

int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
                      WCHAR *pszCmdLine, int nCmdShow)
{
    INITCOMMONCONTROLSEX icc;
    WNDCLASSEX wcx;

    ghInstance = hInstance;

    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_WIN95_CLASSES;
    InitCommonControlsEx(&icc);

    wcx.cbSize = sizeof(wcx);
    if (!GetClassInfoEx(NULL, MAKEINTRESOURCE(32770), &wcx))
        return 0;

    wcx.hInstance = hInstance;
    wcx.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDR_ICO_MAIN));
    wcx.lpszClassName = TEXT("SerialPortClass");
    if (!RegisterClassEx(&wcx))
        return 0;

    return (int)DialogBox(hInstance, MAKEINTRESOURCE(DLG_MAIN), NULL,
                          (DLGPROC)MainDlgProc);
}

static INT_PTR CALLBACK MainDlgProc(HWND hwnd, UINT uMsg, WPARAM wParam,
                                    LPARAM lParam)
{
    switch (uMsg) {
        HANDLE_MSG(hwnd, WM_INITDIALOG, MainDlg_OnInitDialog);
        HANDLE_MSG(hwnd, WM_COMMAND, MainDlg_OnCommand);
        HANDLE_MSG(hwnd, WM_DEVICECHANGE, MainDlg_OnDeviceChange);
        HANDLE_MSG(hwnd, WM_NOTIFY, MainDlg_OnNotify);
        HANDLE_MSG(hwnd, WM_CLOSE, MainDlg_OnClose);
    }
    return FALSE;
}

static void PortListAddPort(HWND hList, PTSTR szPort, PTSTR szStr, BOOL bSel)
{
    int start = 0;
    int last = ComboBox_GetCount(hList) - 1;
    while (start <= last) {
        int cur = (start + last) / 2;
        PTSTR str = (PTSTR)ComboBox_GetItemData(hList, cur);
        int val = _tcsicmp(str, szPort);
        if (val < 0) {
            start = cur + 1;
        } else if (val > 0) {
            last = cur - 1;
        } else {
            ComboBox_DeleteString(hList, cur);
            break;
        }
    }
    ComboBox_InsertString(hList, start, szStr);
    ComboBox_SetItemData(hList, start, szPort);
    if (bSel) {
        ComboBox_SetCurSel(hList, start);
    }
    return;
}

static void RefrashPortList(HWND hwnd, LONG serialId, LONG listId)
{
    HWND hCom = GetDlgItem(hwnd, serialId);
    HWND hList = GetDlgItem(hwnd, listId);

    // save current selected port name
    LPTSTR pszSelPortName = NULL;
    int sel = ComboBox_GetCurSel(hList);
    if (sel != CB_ERR) {
        LPTSTR pszPortName = (LPTSTR)ComboBox_GetItemData(hList, sel);
        if (pszPortName != NULL) {
            pszSelPortName =
                (LPTSTR)malloc((_tcslen(pszPortName) + 1) * sizeof(TCHAR));
            if (pszSelPortName != NULL) {
                _tcscpy(pszSelPortName, pszPortName);
            }
        }
    }

    DWORD dwCount;
    LPSPPORTEX *lpList = SerialPort_GetPortsEx(hCom, NULL, &dwCount);
    if (lpList == NULL) {
        if (pszSelPortName != NULL) {
            free(pszSelPortName);
        }
        return;
    }
    ComboBox_ResetContent(hList);
    BOOL notSetCurSel = (pszSelPortName == NULL) ? TRUE : FALSE;
    for (DWORD i = 0; i < dwCount; i++) {
        if (lpList[i]->bPresent) {
            if (notSetCurSel) {
                notSetCurSel = FALSE;
                PortListAddPort(hList, lpList[i]->pszPortName,
                                lpList[i]->pszFriendlyName, TRUE);
            } else {
                PortListAddPort(
                    hList, lpList[i]->pszPortName, lpList[i]->pszFriendlyName,
                    (pszSelPortName != NULL)
                        ? _tcsicmp(lpList[i]->pszPortName, pszSelPortName) == 0
                        : FALSE);
            }
        } else {
            PortListAddPort(
                hList, lpList[i]->pszPortName, lpList[i]->pszPortName,
                (pszSelPortName != NULL)
                    ? _tcsicmp(lpList[i]->pszPortName, pszSelPortName) == 0
                    : FALSE);
        }
    }
    if (notSetCurSel) {
        ComboBox_SetCurSel(hList, 0);
    }

    // clean
    if (pszSelPortName != NULL) {
        free(pszSelPortName);
    }
}

static BOOL MainDlg_OnInitDialog(HWND hwnd, HWND hwndFocus, LPARAM lParam)
{
    New_SerialPortControl(hwnd, ID_SERIAL);
    RefrashPortList(hwnd, ID_SERIAL, ID_PORTLIST);
    return TRUE;
}

static BOOL bConnected = FALSE;

static void Connect(HWND hwnd)
{
    if (bConnected) {
        return;
    }
    HWND hList = GetDlgItem(hwnd, ID_PORTLIST);
    int sel = ComboBox_GetCurSel(hList);
    if (sel == CB_ERR) {
        return;
    }
    SPCONFIG cfg = {
        .mask = SPCF_PORTNAME | SPCF_BAUDRATE | SPCF_FLOWCONT,
        .pszPortName = (LPTSTR)ComboBox_GetItemData(hList, sel),
        .dwBaudRate = 115200,
        .flowControl = NoFlowControl,
    };
    if (!SerialPort_Open(GetDlgItem(hwnd, ID_SERIAL), &cfg)) {
        MessageBox(hwnd, TEXT("Open serial post failed!"), NULL,
                   MB_OK | MB_ICONERROR);
        return;
    }
    bConnected = TRUE;
    ComboBox_Enable(hList, FALSE);
    Button_SetText(GetDlgItem(hwnd, ID_CONNECT), TEXT("&Disconnect"));
    Button_Enable(GetDlgItem(hwnd, ID_SEND), TRUE);
}

static void Disconnect(HWND hwnd)
{
    if (!bConnected) {
        return;
    }
    if (!SerialPort_Close(GetDlgItem(hwnd, ID_SERIAL))) {
        return;
    }
    bConnected = FALSE;
    ComboBox_Enable(GetDlgItem(hwnd, ID_PORTLIST), TRUE);
    Button_SetText(GetDlgItem(hwnd, ID_CONNECT), TEXT("&Connect"));
    Button_Enable(GetDlgItem(hwnd, ID_SEND), FALSE);
}

static void MainDlg_OnCommand(HWND hwnd, int id, HWND hwndCtl, UINT codeNotify)
{
    switch (id) {
    case ID_PORTLIST:
        switch (codeNotify) {
        case CBN_SELCHANGE:
            break;
        }
        break;
    case ID_CONNECT:
        if (bConnected) {
            Disconnect(hwnd);
        } else {
            Connect(hwnd);
        }
        break;
    case IDOK:
    case IDCANCEL:
        EndDialog(hwnd, TRUE);
        return;
    }
}

static BOOL MainDlg_OnDeviceChange(HWND hwnd, UINT nEventType, DWORD_PTR dwData)
{
    PDEV_BROADCAST_PORT p = (PDEV_BROADCAST_PORT)dwData;
    if (p == NULL || p->dbcp_devicetype != DBT_DEVTYP_PORT) {
        return FALSE;
    }
    switch (nEventType) {
    case DBT_DEVICEARRIVAL:
        RefrashPortList(hwnd, ID_SERIAL, ID_PORTLIST);
        break;
    case DBT_DEVICEREMOVECOMPLETE:
        if (bConnected) {
            HWND hList = GetDlgItem(hwnd, ID_PORTLIST);
            int sel = ComboBox_GetCurSel(hList);
            if (sel != CB_ERR) {
                LPTSTR pszPortName = (LPTSTR)ComboBox_GetItemData(hList, sel);
                if (_tcsicmp(pszPortName, p->dbcp_name) == 0) {
                    Disconnect(hwnd);
                }
            }
        }
        RefrashPortList(hwnd, ID_SERIAL, ID_PORTLIST);
        break;
    }
    return TRUE;
}

static LRESULT MainDlg_OnNotify(HWND hwnd, INT id, LPNMHDR pnm)
{
    switch (pnm->code) {
    case SPN_DATARECEIVED: {
        if (EV_RXFLAG == ((LPSPNMSERIAL)pnm)->dwCode) {
            return FALSE;
        }

        DWORD dwCount = 0;
        while (1) { // Wait till all data is sent.
            DWORD dw = SerialPort_BytesToRead(pnm->hwndFrom);
            if (dw == 0) {
                return FALSE;
            } else if (dw == dwCount) {
                break;
            } else {
                dwCount = dw;
            }
            Sleep(100); // Give the serial port time to catch up
        }

        // Crop the buffer to the data
        PBYTE pByte = malloc(dwCount);
        if (pByte != NULL) {
            if (SerialPort_ReadBytes(pnm->hwndFrom, pByte, dwCount)) {
                // @todo:
            }
            free(pByte);
        }
        return TRUE;
    } break;
    }
    return FALSE;
}

static void MainDlg_OnClose(HWND hwnd) { EndDialog(hwnd, 0); }
