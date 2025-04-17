#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <shellapi.h>
#include <Shlwapi.h>

#include "Application.h"
#include "PhysicsSimulation.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/Dx12LibExt.lib")
#else
#pragma comment(lib, "../x64/Release/Dx12LibExt.lib")
#endif // _DEBUG

#pragma comment(lib, "D3DCompiler.lib")
#pragma comment(lib, "Shlwapi.lib")

#include <dxgidebug.h>
#include <iostream>

void ReportLiveObjects()
{
    IDXGIDebug1* dxgiDebug = nullptr;
    const auto hr = DXGIGetDebugInterface1(0, IID_PPV_ARGS(&dxgiDebug));
    if (FAILED(hr) || dxgiDebug == nullptr) {
        std::cerr << "Failed to get DXGI debug interface. HRESULT: " << hr << std::endl;
        return;
    }
    dxgiDebug->ReportLiveObjects(DXGI_DEBUG_ALL, DXGI_DEBUG_RLO_IGNORE_INTERNAL);
    dxgiDebug->Release();
}

int CALLBACK wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR lpCmdLine, int nCmdShow)
{
    int retCode = 0;

    WCHAR path[MAX_PATH];

    int argc = 0;
    LPWSTR* argv = CommandLineToArgvW(lpCmdLine, &argc);
    if (argv)
    {
        for (int i = 0; i < argc; ++i)
        {
            // -wd Specify the Working Directory.
            if (wcscmp(argv[i], L"-wd") == 0)
            {
                wcscpy_s(path, argv[++i]);
                SetCurrentDirectoryW(path);
            }
        }
        LocalFree(argv);
    }

    Application::Create(hInstance);
    {
        std::shared_ptr<PhysicsSimulation> demo = std::make_shared<PhysicsSimulation>(L"Learning DirectX 12 - Lesson 3", 1280, 720);
        retCode = Application::Get().Run(demo);
    }
    Application::Destroy();

    atexit(&ReportLiveObjects);

    return retCode;
}