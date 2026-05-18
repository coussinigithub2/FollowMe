@echo off
setlocal

:: Dossier de sortie
set "output=JPEG"

:: Cree le dossier s'il n'existe pas
if not exist "%output%" mkdir "%output%"

:: Convertit tous les PNG du dossier courant
for %%f in (*.png) do (
    echo Conversion de %%f ...
    C:\ffmpeg\bin\ffmpeg.exe -y -i "%%f" -q:v 4 "%output%\%%~nf.jpg"
)

echo.
echo Toutes les images ont ete converties dans le dossier "%output%"
pause