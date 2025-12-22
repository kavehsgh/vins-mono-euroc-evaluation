@echo off
cd /d "%~dp0"
echo Starting VINS-Mono Container...
echo Ensure VcXsrv is running with "Disable access control" checked!
echo Ensure Docker Engine is running!

docker-compose up -d --build
docker exec -it vins_mono_container bash
