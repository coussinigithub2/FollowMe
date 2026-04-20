@echo off
set FFMPEG_PATH="C:\ffmpeg\bin\ffmpeg.exe"

if not exist "nouveau" mkdir "nouveau"

echo Conversion : 8000Hz + Stereo + Normalisation + Amplification 50%%...
echo.

for %%f in (*.wav) do (
    echo Traitement de : %%f
    %FFMPEG_PATH% -i "%%f" -ar 8000 -ac 2 -codec:a pcm_s16le -af "loudnorm=I=-16.29:TP=-1.71:LRA=1,volume=1.5" -map_metadata -1 "nouveau\%%f"
)

echo.
echo Operation terminee. Les fichiers amplifies sont dans le dossier "nouveau".
pause