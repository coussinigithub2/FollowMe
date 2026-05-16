@echo off
setlocal enabledelayedexpansion
title Convertisseur WAV Stereo + Volume

:: Configuration du chemin FFmpeg
set FFMPEG_PATH="C:\ffmpeg\bin\ffmpeg.exe"

echo ====================================================
echo    CONVERTISSEUR AUDIO STEREO (QUALITE X-PLANE)
echo ====================================================
echo.

if not exist "nouveau_stereo" mkdir "nouveau_stereo"

:: Demande du gain a l'utilisateur
set /p GAIN="Facteur de volume (ex: 1.0=normal, 2.0=double, 0.5=moitie) : "

echo.
echo Traitement en cours...

for %%f in (*.mp3) do (
    echo [OK] %%f
    %FFMPEG_PATH% -i "%%f" -ar 44100 -ac 2 -filter:a "volume=%GAIN%" -codec:a pcm_s16le -map_metadata -1 -y "nouveau_stereo\%%~nf.wav"
)

echo.
echo Termine ! Les fichiers sont dans le dossier 'nouveau_stereo'.
pause