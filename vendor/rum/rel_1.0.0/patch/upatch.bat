@echo off

rem This path is based on the uTasker download name. This can be changed depending
rem on your custom folder name.

cd uTaskerV1.3.0_SAM7X


echo ############################################################################
echo ### Beginning patch of uTasker                                           ###
echo ############################################################################

patch -p1 -i ../utasker-patch

echo ############################################################################
echo ### Inspect output of above command for successful application of patch! ###
echo ############################################################################
pause