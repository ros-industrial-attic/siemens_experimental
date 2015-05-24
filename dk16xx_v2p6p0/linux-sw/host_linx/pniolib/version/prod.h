#ifndef _SINEC_PROD_
#define _SINEC_PROD_

#include "siemens.h"

/* conversion of numbers into string: */
#define chSTR(x)  #x
#define chSTR2(x) chSTR(x)

#define eval(x) x

#define VER_COMPANYNAME_STR         SIN_COMPANY_NAME
#define VER_LEGALCOPYRIGHT_STR      SIN_LEGALCOPYRIGHT

#define VER_PRODUCTNAME_STR	        SIN_PRODUCT_STR
#define VER_PRODUCTVERSION_STR      "V" chSTR2(CD_MAJOR) "." chSTR2(CD_MINOR_PROD) "." chSTR2(CD_SP) "." chSTR2(CD_HOTFIX) "_" chSTR2(SINEC_BUILD_MAJOR) "." chSTR2(SINEC_BUILD_MINOR) ".00." chSTR2(CD_INKREMENT)
#define VER_PRODUCTVERSION          eval(CD_MAJOR) ## eval(CD_MINOR_PROD),eval(CD_SP) ## eval(CD_HOTFIX),SINEC_BUILD,CD_INKREMENT

#endif


