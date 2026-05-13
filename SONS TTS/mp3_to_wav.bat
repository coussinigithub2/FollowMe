@echo off
set FFMPEG_PATH="C:\ffmpeg\bin\ffmpeg.exe"

if not exist "nouveau" mkdir "nouveau"

echo Conversion WAV X-Plane avec compression audio...
echo.

for %%f in (*.mp3) do (

    echo Traitement : %%f

    %FFMPEG_PATH% -i "%%f" ^
    -ar 44100 ^
    -ac 1 ^
    -codec:a pcm_s16le ^
    -af "acompressor=threshold=-18dB:ratio=4:attack=20:release=250:makeup=5" ^
    -map_metadata -1 ^
    "nouveau\%%~nf.wav"
)

echo.
echo Termine.
pause