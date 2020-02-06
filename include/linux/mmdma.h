/*
 * mmdma.h
 *
 * Copyright (c) 2009 by emtrion GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Author:      Markus Pietrek
 *
 **/

#ifndef _LINUX_MMDMA_H
#define _LINUX_MMDMA_H

#define MMDMA_MAGIC		'm'

#define MMDMA_GETPHYS		_IOR(MMDMA_MAGIC, 0, unsigned long*)

#endif
