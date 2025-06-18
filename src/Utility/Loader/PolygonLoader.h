#ifndef POLYGON_LOADER_HPP
#define POLYGON_LOADER_HPP

#include <Windows.h>
#include <atlbase.h>
#include <format>
#include <fstream>
#include <iostream>
#include <shlobj.h>
#include <shobjidl.h>
#include <string>
#include <strsafe.h>
#include <vector>

#include "../Shapes.hpp"

namespace tp::loader
{

// A helper to convert wide string to narrow string
inline std::string WStringToString(const std::wstring& wstr)
{
	if (wstr.empty())
		return std::string();
	int size_needed = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL);
	std::string strTo(size_needed, 0);
	WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), &strTo[0], size_needed, NULL, NULL);
	return strTo;
}

inline std::wstring StringToWString(const std::string& str)
{
	if (str.empty())
		return std::wstring();
	int size_needed = MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), NULL, 0);
	std::wstring wstrTo(size_needed, 0);
	MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), &wstrTo[0], size_needed);
	return wstrTo;
}

inline std::string OpenFileViaIFileOpenDialog()
{
	HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
	if (FAILED(hr))
	{
		std::cerr << std::format("Failed to initialize COM: 0x{:X}\n", hr);
		return "";
	}

	CComPtr<IFileOpenDialog> pFileOpen;

	hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_ALL,
		IID_IFileOpenDialog, reinterpret_cast<void**>(&pFileOpen));
	if (FAILED(hr))
	{
		std::cerr << std::format("Failed to create IFileOpenDialog: 0x{:X}\n", hr);
		CoUninitialize();
		return "";
	}

	COMDLG_FILTERSPEC rgSpec[] = {
		{ L"Text Files", L"*.txt" },
		{ L"All Files", L"*.*" },
	};
	hr = pFileOpen->SetFileTypes(ARRAYSIZE(rgSpec), rgSpec);
	if (FAILED(hr))
	{
		std::cerr << std::format("Failed to set file types: 0x{:X}\n", hr);
		CoUninitialize();
		return "";
	}

	hr = pFileOpen->SetDefaultExtension(L"txt");

	FILEOPENDIALOGOPTIONS fos;
	hr = pFileOpen->GetOptions(&fos);
	if (SUCCEEDED(hr))
	{
		hr = pFileOpen->SetOptions(fos | FOS_FORCEFILESYSTEM | FOS_PATHMUSTEXIST | FOS_FILEMUSTEXIST);
	}

	hr = pFileOpen->Show(NULL);

	std::string filePath = "";

	if (SUCCEEDED(hr))
	{
		CComPtr<IShellItem> pItem;
		hr = pFileOpen->GetResult(&pItem);
		if (SUCCEEDED(hr))
		{
			LPWSTR pszFilePath;
			hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);
			if (SUCCEEDED(hr))
			{
				filePath = WStringToString(pszFilePath);
				CoTaskMemFree(pszFilePath);
			}
		}
	}
	else if (hr == HRESULT_FROM_WIN32(ERROR_CANCELLED))
	{
		std::cout << "File selection cancelled.\n";
	}
	else
	{
		std::cerr << std::format("IFileOpenDialog show error: 0x{:X}\n", hr);
	}

	CoUninitialize();
	return filePath;
}

inline std::vector<shapes::Polygon> LoadPolygonsFromFile(const std::string& filename)
{
	using namespace tp::shapes;

	std::vector<Polygon> polygons;
	std::ifstream file(filename);

	if (!file.is_open())
	{
		std::cerr << "Error: Could not open file " << filename << std::endl;
		return polygons; // Возвращаем пустой вектор
	}

	int numPolygons;
	file >> numPolygons; // Считываем количество полигонов

	if (file.fail())
	{
		std::cerr << "Error: Failed to read number of polygons from " << filename << std::endl;
		file.close();
		return polygons;
	}

	for (int i = 0; i < numPolygons; ++i)
	{
		int numPoints;
		file >> numPoints; // Считываем количество точек для текущего полигона

		if (file.fail())
		{
			std::cerr << "Error: Failed to read number of points for polygon " << i << " from " << filename << std::endl;
			file.close();
			return polygons;
		}

		Polygon currentPolygon;
		for (int j = 0; j < numPoints; ++j)
		{
			float x, y;
			file >> x >> y; // Считываем координаты точки

			if (file.fail())
			{
				std::cerr << "Error: Failed to read point " << j << " for polygon " << i << " from " << filename << std::endl;
				file.close();
				return polygons;
			}

			currentPolygon.push_back({ x, y });
		}
		polygons.push_back(currentPolygon);
	}

	file.close();
	return polygons;
}
} // namespace tp::loader
#endif
