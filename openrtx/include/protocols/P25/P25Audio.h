/*
*   Copyright (C) 2016 by Jonathan Naylor G4KLX
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#if !defined(P25Audio_H)
#define P25Audio_H

#include "AMBEFEC.h"

class CP25Audio
{
public:
    CP25Audio();
    ~CP25Audio();

    unsigned int process(unsigned char *data);

    void encode(unsigned char *data, const unsigned char *imbe, unsigned int n);

    void decode(const unsigned char *data, unsigned char *imbe, unsigned int n);

    /*
     * One IMBE channel frame (144 bits in 18 bytes: Golay and Hamming coded,
     * whitened, interleaved), as both P25 LDUs and YSF voice full rate (VW)
     * frames carry it, to and from the 88 IMBE bits (11 bytes).  Decoding
     * takes the bits as they are: correct them with
     * CAMBEFEC::regenerateIMBE() first.
     */
    static void decodeFrame(const unsigned char *frame, unsigned char *imbe);

    static void encodeFrame(const unsigned char *imbe, unsigned char *frame);

private:
    CAMBEFEC m_fec;
};

#endif
