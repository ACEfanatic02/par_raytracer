@echo off
REM Builds a single C file with MSVC

set FPATH=..\%1

REM Setup MSVC env vars
pushd C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\
REM x86 build:
REM call vcvarsall.bat x86
REM x64 build:
call vcvarsall.bat x64
popd

if NOT exist build mkdir build
pushd build

REM For asm output add flag /Fa
set BUILD_OPTS=/O2 /Zi /nologo /EHsc

cl %BUILD_OPTS% %FPATH% /INCREMENTAL:NO

popd