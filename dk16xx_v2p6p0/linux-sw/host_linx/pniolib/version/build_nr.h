#define SINEC_BUILD 3409
#define CD_MAJOR 07
#define CD_MINOR_PROD 00
#define CD_SP 00
#define CD_MINOR_FILE 00
#define CD_HOTFIX 00
#define CD_INKREMENT 01
#define SINEC_BUILD_MAJOR 34
#define SINEC_BUILD_MINOR 09

/***************************************************************
* ACHTUNG !                                                    *
* Unbedingt zu berücksichtigen bei einer Änderung von CD_MAJOR *
*                                                              *
* Einige DLLs haben z. Zt. falsche Versionsnummern             *
* z.B dpc1lib.dll : 6.58.1004.2458                             *
*                                                              *
* Vor der Umstellung muss :                                    *
* a) festgestellt werden, welche DLLs, .EXE betroffen sind     *
* b) bei diesen DLLs der Aufbau des Versionsstrings angepasst  *
*    werden -> Sollergebnis : M.m.sp.bbbb                      *
*   (M=CD_MAJOR, m=CD_MINOR, sp=Service-Pack-Nr., bbbb = Build)*
***************************************************************/
