/*
# MBusino: M-Bus --> MQTT-Gateway

One sketch for all MBusino variants, see config.h for the variant selection.

https://github.com/Zeppelin500/MBusino/

## Licence
****************************************************
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see https://www.gnu.org/licenses/.
****************************************************
*/

#include "config.h"

#if defined(MBUSINO_ETH) && MBUSINO_SLAVES == 1
#include "src/variants/nano/main.h"
#elif defined(MBUSINO_ETH) && MBUSINO_SLAVES == 5
#include "src/variants/nano5s/main.h"
#elif defined(MBUSINO_SENSORS) && MBUSINO_SLAVES == 1
#include "src/variants/shield/main.h"
#elif defined(MBUSINO_SENSORS) && MBUSINO_SLAVES == 5
#include "src/variants/shield5s/main.h"
#endif
