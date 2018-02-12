#include "window_mod.h"
#include <Uxtheme.h>
#include <Dwmapi.h>


// Paint the title on the custom frame.
static HRESULT GlassEff(HWND hWnd, HDC hdc)
{


	MARGINS margins = { -1 };
	HRESULT hr = S_OK;

	// Extend the frame across the entire window.
	hr = DwmExtendFrameIntoClientArea(hWnd, &margins);
	if (SUCCEEDED(hr))
	{
		// ...
	}
	return hr;



}


LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}


LRESULT CALLBACK WndProcPanel(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
	case WM_NCPAINT:  // here I'm creating the border
	{
		RECT rect;
		GetWindowRect(hwnd, &rect);
		OffsetRect(&rect, -rect.left, -rect.top);
		HDC hdc = GetWindowDC(hwnd);

		HRGN g_hrgnButton = CreateRoundRectRgn(rect.left, rect.top, rect.right, rect.bottom, 40, 40);
		FrameRgn(hdc, g_hrgnButton, CreateSolidBrush(RGB(50, 50, 255)), 5, 5);

		ReleaseDC(hwnd, hdc);
		return 0;
	}
	case WM_NCCALCSIZE:  // here I think I'm setting the client area size.
	{
		RECT *rc = (RECT*)lParam;
		InflateRect(rc, -5, -5);
		return 0;
	}
	break;
	case WM_PAINT:  // Setting the client area background color
	{
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hwnd, &ps);

		RECT rect = ps.rcPaint;
		HRGN hrgn = CreateRoundRectRgn(rect.left, rect.top, rect.right, rect.bottom, 40, 40);
		SelectObject(hdc, GetStockObject(DC_BRUSH));
		SetDCBrushColor(hdc, RGB(0, 255, 0));
		PaintRgn(hdc, hrgn);

		EndPaint(hwnd, &ps);
	}
	}
	return DefWindowProc(hwnd, msg, wParam, lParam);
}