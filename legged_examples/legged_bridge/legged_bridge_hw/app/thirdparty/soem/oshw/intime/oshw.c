/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

#include <sys/endian.h>
#include "oshw.h"

/**
 * Host to Network byte order (i.e. to big endian).
 *
 * Note that Ethercat uses little endian byte order, except for the Ethernet
 * header which is big endian as usual.
 */
uint16 oshw_htons (uint16 host)
{
   uint16 network = htons (host);
   return network;
}

/**
 * Network (i.e. big endian) to Host byte order.
 *
 * Note that Ethercat uses little endian byte order, except for the Ethernet
 * header which is big endian as usual.
 */
uint16 oshw_ntohs (uint16 network)
{
   uint16 host = ntohs (network);
   return host;
}

/* Create list over available network adapters.
 * @return First element in linked list of adapters
 */
ec_adaptert * oshw_find_adapters (void)
{
   return NULL;
}

/** Free memory allocated memory used by adapter collection.
 * @param[in] adapter = First element in linked list of adapters
 * EC_NOFRAME.
 */
void oshw_free_adapters (ec_adaptert * adapter)
{

}
