#ifndef _SHLOBJIDL_CORE_
#define _SHLOBJIDL_CORE_

#if __GNUC__ >= 3
#pragma GCC system_header
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <basetyps.h>
#include <wtypes.h>

#include <ocidl.h>

#include <shobjidl.h>
#include <shtypes.h>

const IID IID_IFileOpenDialog = {0xd57c7288,0xd4ad,0x4768,{0xbe,0x02,0x9d,0x96,0x95,0x32,0xd9,0x60}};
const IID IID_IFileSaveDialog = {0x84bccd23,0x5fde,0x4cdb,{0xae,0xa4,0xaf,0x64,0xb8,0x3d,0x78,0xab}};
const CLSID CLSID_FileOpenDialog = {0xDC1C5A9C,0xE88A,0x4dde,{0xA5,0xA1,0x60,0xF8,0x2A,0x20,0xAE,0xF7}};
const CLSID CLSID_FileSaveDialog = {0xC0B4E2F3,0xBA21,0x4773,{0x8D,0xBA,0x33,0x5E,0xC9,0x46,0xEB,0x8B}};

typedef enum SIATTRIBFLAGS
{
  SIATTRIBFLAGS_AND	= 0x1,
  SIATTRIBFLAGS_OR	= 0x2,
  SIATTRIBFLAGS_APPCOMPAT	= 0x3,
  SIATTRIBFLAGS_MASK	= 0x3,
  SIATTRIBFLAGS_ALLITEMS	= 0x4000
} SIATTRIBFLAGS;

typedef struct _COMDLG_FILTERSPEC {
  LPCWSTR pszName;
  LPCWSTR pszSpec;
} COMDLG_FILTERSPEC;

typedef enum _FILEOPENDIALOGOPTIONS
{
  FOS_OVERWRITEPROMPT	= 0x2,
  FOS_STRICTFILETYPES	= 0x4,
  FOS_NOCHANGEDIR	= 0x8,
  FOS_PICKFOLDERS	= 0x20,
  FOS_FORCEFILESYSTEM	= 0x40,
  FOS_ALLNONSTORAGEITEMS	= 0x80,
  FOS_NOVALIDATE	= 0x100,
  FOS_ALLOWMULTISELECT	= 0x200,
  FOS_PATHMUSTEXIST	= 0x800,
  FOS_FILEMUSTEXIST	= 0x1000,
  FOS_CREATEPROMPT	= 0x2000,
  FOS_SHAREAWARE	= 0x4000,
  FOS_NOREADONLYRETURN	= 0x8000,
  FOS_NOTESTFILECREATE	= 0x10000,
  FOS_HIDEMRUPLACES	= 0x20000,
  FOS_HIDEPINNEDPLACES	= 0x40000,
  FOS_NODEREFERENCELINKS	= 0x100000,
  FOS_OKBUTTONNEEDSINTERACTION	= 0x200000,
  FOS_DONTADDTORECENT	= 0x2000000,
  FOS_FORCESHOWHIDDEN	= 0x10000000,
  FOS_DEFAULTNOMINIMODE	= 0x20000000,
  FOS_FORCEPREVIEWPANEON	= 0x40000000,
  FOS_SUPPORTSTREAMABLEITEMS	= 0x80000000
};
typedef DWORD FILEOPENDIALOGOPTIONS;

typedef enum FDAP
{
  FDAP_BOTTOM	= 0,
  FDAP_TOP	= 1
} FDAP;

typedef enum FDE_OVERWRITE_RESPONSE
{
  FDEOR_DEFAULT	= 0,
  FDEOR_ACCEPT	= 1,
  FDEOR_REFUSE	= 2
} FDE_OVERWRITE_RESPONSE;

typedef enum FDE_SHAREVIOLATION_RESPONSE
{
  FDESVR_DEFAULT	= 0,
  FDESVR_ACCEPT	= 1,
  FDESVR_REFUSE	= 2
} FDE_SHAREVIOLATION_RESPONSE;


// forward declarations
struct IEnumShellItems;
struct IShellItemArray;
struct IFileDialog;
struct IFileDialogEvents;
struct IFileOpenDialog;
struct IShellItemFilter;
struct IFileSaveDialog;
struct IFileOperationProgressSink;


struct IEnumShellItems : public IUnknown
{
public:
  virtual HRESULT __stdcall Next(ULONG celt, IShellItem **rgelt, ULONG *pceltFetched) = 0;
  virtual HRESULT __stdcall Skip(ULONG celt) = 0;
  virtual HRESULT __stdcall Reset() = 0;
  virtual HRESULT __stdcall Clone(IEnumShellItems **ppenum) = 0;
};

struct IShellItemArray : public IUnknown
{
public:
  virtual HRESULT __stdcall BindToHandler(IBindCtx *pbc, REFGUID bhid, REFIID riid, void **ppvOut) = 0;
  virtual HRESULT __stdcall GetPropertyStore(GETPROPERTYSTOREFLAGS flags, REFIID riid, void **ppv) = 0;
  virtual HRESULT __stdcall GetPropertyDescriptionList(REFPROPERTYKEY keyType, REFIID riid, void **ppv) = 0;
  virtual HRESULT __stdcall GetAttributes(SIATTRIBFLAGS AttribFlags, SFGAOF sfgaoMask, SFGAOF *psfgaoAttribs) = 0;
  virtual HRESULT __stdcall GetCount(DWORD *pdwNumItems) = 0;
  virtual HRESULT __stdcall GetItemAt(DWORD dwIndex, IShellItem **ppsi) = 0;
  virtual HRESULT __stdcall EnumItems(IEnumShellItems **ppenumShellItems) = 0;
};

struct IShellItemFilter : public IUnknown
{
public:
  virtual HRESULT __stdcall IncludeItem(IShellItem *psi) = 0;
  virtual HRESULT __stdcall GetEnumFlagsForItem(IShellItem *psi, SHCONTF *pgrfFlags) = 0;
};

struct IFileDialog : public IModalWindow
{
public:
  virtual HRESULT __stdcall SetFileTypes(UINT cFileTypes, const COMDLG_FILTERSPEC *rgFilterSpec) = 0;
  virtual HRESULT __stdcall SetFileTypeIndex(UINT iFileType) = 0;
  virtual HRESULT __stdcall GetFileTypeIndex(UINT *piFileType) = 0;
  virtual HRESULT __stdcall Advise(IFileDialogEvents *pfde, DWORD *pdwCookie) = 0;
  virtual HRESULT __stdcall Unadvise(DWORD dwCookie) = 0;
  virtual HRESULT __stdcall SetOptions(FILEOPENDIALOGOPTIONS fos) = 0;
  virtual HRESULT __stdcall GetOptions(FILEOPENDIALOGOPTIONS *pfos) = 0;
  virtual HRESULT __stdcall SetDefaultFolder(IShellItem *psi) = 0;
  virtual HRESULT __stdcall SetFolder(IShellItem *psi) = 0;
  virtual HRESULT __stdcall GetFolder(IShellItem **ppsi) = 0;
  virtual HRESULT __stdcall GetCurrentSelection(IShellItem **ppsi) = 0;
  virtual HRESULT __stdcall SetFileName(LPCWSTR pszName) = 0;
  virtual HRESULT __stdcall GetFileName(LPWSTR *pszName) = 0;
  virtual HRESULT __stdcall SetTitle(LPCWSTR pszTitle) = 0;
  virtual HRESULT __stdcall SetOkButtonLabel(LPCWSTR pszText) = 0;
  virtual HRESULT __stdcall SetFileNameLabel(LPCWSTR pszLabel) = 0;
  virtual HRESULT __stdcall GetResult(IShellItem **ppsi) = 0;
  virtual HRESULT __stdcall AddPlace(IShellItem *psi, FDAP fdap) = 0;
  virtual HRESULT __stdcall SetDefaultExtension(LPCWSTR pszDefaultExtension) = 0;
  virtual HRESULT __stdcall Close(HRESULT hr) = 0;
  virtual HRESULT __stdcall SetClientGuid(REFGUID guid) = 0;
  virtual HRESULT __stdcall ClearClientData() = 0;
  virtual HRESULT __stdcall SetFilter(IShellItemFilter *pFilter) = 0;
};

struct IFileDialogEvents : public IUnknown
{
public:
  virtual HRESULT __stdcall OnFileOk(IFileDialog *pfd) = 0;
  virtual HRESULT __stdcall OnFolderChanging(IFileDialog *pfd, IShellItem *psiFolder) = 0;
  virtual HRESULT __stdcall OnFolderChange(IFileDialog *pfd) = 0;
  virtual HRESULT __stdcall OnSelectionChange(IFileDialog *pfd) = 0;
  virtual HRESULT __stdcall OnShareViolation(IFileDialog *pfd, IShellItem *psi, FDE_SHAREVIOLATION_RESPONSE *pResponse) = 0;
  virtual HRESULT __stdcall OnTypeChange(IFileDialog *pfd) = 0;
  virtual HRESULT __stdcall OnOverwrite(IFileDialog *pfd, IShellItem *psi, FDE_OVERWRITE_RESPONSE *pResponse) = 0;
};

struct IFileOpenDialog : public IFileDialog
{
public:
  virtual HRESULT __stdcall GetResults(IShellItemArray **ppenum) = 0;
  virtual HRESULT __stdcall GetSelectedItems(IShellItemArray **ppsai) = 0;
};

struct IFileOperationProgressSink : public IUnknown
{
public:
  virtual HRESULT __stdcall StartOperations() = 0;
  virtual HRESULT __stdcall FinishOperations(HRESULT hrResult) = 0;
  virtual HRESULT __stdcall PreRenameItem(DWORD dwFlags, IShellItem *psiItem, LPCWSTR pszNewName) = 0;
  virtual HRESULT __stdcall PostRenameItem(DWORD dwFlags, IShellItem *psiItem, LPCWSTR pszNewName,
    HRESULT hrRename, IShellItem *psiNewlyCreated) = 0;
  virtual HRESULT __stdcall PreMoveItem(DWORD dwFlags, IShellItem *psiItem, IShellItem *psiDestinationFolder,
    LPCWSTR pszNewName) = 0;
  virtual HRESULT __stdcall PostMoveItem(DWORD dwFlags, IShellItem *psiItem, IShellItem *psiDestinationFolder,
    LPCWSTR pszNewName, HRESULT hrMove, IShellItem *psiNewlyCreated) = 0;
  virtual HRESULT __stdcall PreCopyItem(DWORD dwFlags, IShellItem *psiItem, IShellItem *psiDestinationFolder,
    LPCWSTR pszNewName) = 0;
  virtual HRESULT __stdcall PostCopyItem(DWORD dwFlags, IShellItem *psiItem, IShellItem *psiDestinationFolder,
    LPCWSTR pszNewName, HRESULT hrCopy, IShellItem *psiNewlyCreated) = 0;
  virtual HRESULT __stdcall PreDeleteItem(DWORD dwFlags, IShellItem *psiItem) = 0;
  virtual HRESULT __stdcall PostDeleteItem(DWORD dwFlags, IShellItem *psiItem, HRESULT hrDelete, IShellItem *psiNewlyCreated) = 0;
  virtual HRESULT __stdcall PreNewItem(DWORD dwFlags, IShellItem *psiDestinationFolder, LPCWSTR pszNewName) = 0;
  virtual HRESULT __stdcall PostNewItem(DWORD dwFlags, IShellItem *psiDestinationFolder, LPCWSTR pszNewName,
    LPCWSTR pszTemplateName, DWORD dwFileAttributes, HRESULT hrNew, IShellItem *psiNewItem) = 0;
  virtual HRESULT __stdcall UpdateProgress(UINT iWorkTotal, UINT iWorkSoFar) = 0;
  virtual HRESULT __stdcall ResetTimer() = 0;
  virtual HRESULT __stdcall PauseTimer() = 0;
  virtual HRESULT __stdcall ResumeTimer() = 0;
};

struct IFileSaveDialog : public IFileDialog
{
public:
  virtual HRESULT __stdcall SetSaveAsItem(IShellItem *psi) = 0;
  virtual HRESULT __stdcall SetProperties(IPropertyStore *pStore) = 0;
  virtual HRESULT __stdcall SetCollectedProperties(IPropertyDescriptionList *pList, BOOL fAppendDefault) = 0;
  virtual HRESULT __stdcall GetProperties(IPropertyStore **ppStore) = 0;
  virtual HRESULT __stdcall ApplyProperties(IShellItem *psi, IPropertyStore *pStore, HWND hwnd, IFileOperationProgressSink *pSink) = 0;
};

#ifdef __cplusplus
}
#endif

#endif // _SHLOBJIDL_CORE_
