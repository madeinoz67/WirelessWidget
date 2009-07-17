            ATMEL Route Under MAC (RUM) with IPv6 & 6LoWPAN


This readme describes how to use the upatch.bat file to patch uTasker with
RUM and IPv6 & 6LoWPAN.


1) Download the latest WinAVR from www.sourceforge.net as this contains the
patch.exe program needed for the patch procedure.

2) With a proper license from uTasker, download and unzip the main OS (uTaskerV1.3.0_SAM7X.zip).

3) Download the uTasker SP4 (uTaskerV1.3.0_SAM7X_SP4.zip) and unzip. Copy the
SP4 content into the main uTasker folder over-writing all similar content.

4) Copy upatch.bat and utasker-patch files to the same level as the uTasker main
with SP4 combined folder. (eg. C:\project... should contain these two files and
the uTasker source folder).

5) Double click the .bat file from Windows Explorer to begin patch. A status window
will update the current progress of the patch. upatch.bat expects the path of
..\uTaskerV1.3.0_SAM7X to initiate the patch procedure. This can be modified within
the .bat file if needed.

6) When complete, uTasker will now be patched to operate RUM with IPv6 &6LoWPAN.

Simply copy the efsl, rum_src, and uip folders into the \Applications directory of
the uTasker source folder to begin development work.

See application note AVR2070 for more detailed information about this procedure.
Technical support can be sent to avr@atmel.com.