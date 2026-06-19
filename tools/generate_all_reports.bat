@echo off
setlocal enabledelayedexpansion
REM tools\generate_all_reports.bat
REM
REM Drives the full per-study analysis pipeline across every case-study/*/
REM directory: run_analysis.py (MC + fault sweep + WCET + mu analysis, each
REM self-skipping when a study lacks the relevant data/hooks) then
REM generate_report.py (writes that study's own report.html). Finishes by
REM refreshing docs\case_study_status.md.
REM
REM Studies with no logs\ directory at all (the "Not started" / plan-only
REM stubs in docs\case_study_status.md) are skipped outright - there is
REM nothing for either tool to read. "Open placeholder" scaffolds (logs\
REM exists but only has placeholder OpenLoop data) are NOT filtered out
REM here - that distinction needs tools\case_study_tracker.py's literal
REM template-fingerprint check, which this script does not replicate.
REM
REM Usage (from anywhere):  tools\generate_all_reports.bat

cd /d "%~dp0\.."

set LOGFILE=tools\generate_all_reports.log

echo ============================================================ >> "%LOGFILE%"
echo  generate_all_reports.bat  -  started %date% %time% >> "%LOGFILE%"
echo ============================================================ >> "%LOGFILE%"
echo Logging to %LOGFILE%

set /a N_PROCESSED=0 >nul
set /a N_SKIPPED=0 >nul

for /d %%F in ("case-study\*") do (
    @echo off
    if exist "%%F\logs" (
        echo. >> "%LOGFILE%"
        echo ---- %%~nxF ---- >> "%LOGFILE%"
        echo [%%~nxF] running analysis...
        conda run -n soft_robotics -- python tools\run_analysis.py --study "%%~nxF" >> "%LOGFILE%" 2>&1
        conda run -n soft_robotics -- python tools\generate_report.py --study "%%~nxF" >> "%LOGFILE%" 2>&1
        set /a N_PROCESSED+=1
    ) else (
        echo ---- %%~nxF: SKIP - no logs\ directory ---- >> "%LOGFILE%"
        set /a N_SKIPPED+=1
    )
)

echo. >> "%LOGFILE%"
echo Refreshing docs\case_study_status.md ... >> "%LOGFILE%"
conda run -n soft_robotics -- python tools\case_study_tracker.py >> "%LOGFILE%" 2>&1

echo. >> "%LOGFILE%"
echo Done. %N_PROCESSED% studies processed, %N_SKIPPED% skipped (no logs dir). >> "%LOGFILE%"
echo Done. %N_PROCESSED% studies processed, %N_SKIPPED% skipped (no logs dir). See %LOGFILE%

endlocal
