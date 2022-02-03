#warning "dependency injection working"
#pragma once
#include "sdkconfig.h"

#define _weak_alias(name, aliasname) \
    extern __typeof (name) aliasname __attribute__ ((weak, alias (#name)));

#if CONFIG_MB_UTEST
#define ut_static __attribute__((weak))
//_weak_alias(__wrap_xMBTCPPortMasterConnect, xMBTCPPortMasterConnect);

#define esp_netif_init __wrap_esp_netif_init
#define esp_wifi_set_ps __wrap_esp_wifi_set_ps
#define get_example_netif __wrap_get_example_netif
#define example_connect __wrap_example_connect
#define init_services __wrap_init_services

#else
#define ut_static static
#endif