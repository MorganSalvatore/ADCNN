/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     Mv.h
    \brief    motion vector class (header)
*/

#ifndef __MV__
#define __MV__

#include "CommonDef.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

enum MvPrecision
{
  MV_PRECISION_4PEL     = 0,      // 4-pel
  MV_PRECISION_INT      = 2,      // 1-pel, shift 2 bits from 4-pel
#if JVET_M0246_AFFINE_AMVR
  MV_PRECISION_HALF     = 3,      // 1/2-pel
#endif
  MV_PRECISION_QUARTER  = 4,      // 1/4-pel (the precision of regular MV difference signaling), shift 4 bits from 4-pel
  MV_PRECISION_INTERNAL = 6,      // 1/16-pel (the precision of internal MV), shift 6 bits from 4-pel
};

/// basic motion vector class
class Mv
{
private:
  static const MvPrecision m_amvrPrecision[3];

public:
  int   hor;     ///< horizontal component of motion vector
  int   ver;     ///< vertical component of motion vector

  // ------------------------------------------------------------------------------------------------------------------
  // constructors
  // ------------------------------------------------------------------------------------------------------------------

  Mv(                    ) : hor( 0    ), ver( 0    ) {}
  Mv( int iHor, int iVer ) : hor( iHor ), ver( iVer ) {}

  // ------------------------------------------------------------------------------------------------------------------
  // set
  // ------------------------------------------------------------------------------------------------------------------

  void  set       ( int iHor, int iVer)     { hor = iHor;  ver = iVer; }
  void  setHor    ( int i )                 { hor = i;                 }
  void  setVer    ( int i )                 { ver = i;                 }
  void  setZero   ()                        { hor = ver = 0;           }

  // ------------------------------------------------------------------------------------------------------------------
  // get
  // ------------------------------------------------------------------------------------------------------------------

  int   getHor    () const { return hor;          }
  int   getVer    () const { return ver;          }
  int   getAbsHor () const { return abs( hor );   }
  int   getAbsVer () const { return abs( ver );   }

  // ------------------------------------------------------------------------------------------------------------------
  // operations
  // ------------------------------------------------------------------------------------------------------------------

  const Mv& operator += (const Mv& _rcMv)
  {
    {
      Mv rcMv = _rcMv;

      hor += rcMv.hor;
      ver += rcMv.ver;
    }
    return  *this;
  }

  const Mv& operator-= (const Mv& _rcMv)
  {
    {
      Mv rcMv = _rcMv;

      hor -= rcMv.hor;
      ver -= rcMv.ver;
    }
    return  *this;
  }


  //! shift right with rounding
  void divideByPowerOf2 (const int i)
  {
#if ME_ENABLE_ROUNDING_OF_MVS
    const int offset = (i == 0) ? 0 : 1 << (i - 1);
    hor += offset;
    ver += offset;
#endif
    hor >>= i;
    ver >>= i;
  }

  const Mv& operator<<= (const int i)
  {
    hor <<= i;
    ver <<= i;
    return  *this;
  }

  const Mv& operator>>= ( const int i )
  {
    hor >>= i;
    ver >>= i;
    return  *this;
  }

  const Mv operator - ( const Mv& rcMv ) const
  {
    return Mv( hor - rcMv.hor, ver - rcMv.ver );
  }

  const Mv operator + ( const Mv& rcMv ) const
  {
    return Mv( hor + rcMv.hor, ver + rcMv.ver );
  }

  bool operator== ( const Mv& rcMv ) const
  {
    return ( hor == rcMv.hor && ver == rcMv.ver );
  }

  bool operator!= ( const Mv& rcMv ) const
  {
    return !( *this == rcMv );
  }

  const Mv scaleMv( int iScale ) const
  {
#if JVET_M0479_18BITS_MV_CLIP
    const int mvx = Clip3( -131072, 131071, (iScale * getHor() + 127 + (iScale * getHor() < 0)) >> 8 );
    const int mvy = Clip3( -131072, 131071, (iScale * getVer() + 127 + (iScale * getVer() < 0)) >> 8 );
#else
    const int mvx = Clip3( -32768, 32767, (iScale * getHor() + 127 + (iScale * getHor() < 0)) >> 8 );
    const int mvy = Clip3( -32768, 32767, (iScale * getVer() + 127 + (iScale * getVer() < 0)) >> 8 );
#endif
    return Mv( mvx, mvy );
  }

  void changePrecision(const MvPrecision& src, const MvPrecision& dst)
  {
    const int shift = (int)dst - (int)src;
    if (shift >= 0)
    {
      *this <<= shift;
    }
    else
    {
      const int rightShift = -shift;
      const int nOffset = 1 << (rightShift - 1);
      hor = hor >= 0 ? (hor + nOffset) >> rightShift : -((-hor + nOffset) >> rightShift);
      ver = ver >= 0 ? (ver + nOffset) >> rightShift : -((-ver + nOffset) >> rightShift);
    }
  }

  void changePrecisionAmvr(const int amvr, const MvPrecision& dst)
  {
    changePrecision(m_amvrPrecision[amvr], dst);
  }

  void roundToPrecision(const MvPrecision& src, const MvPrecision& dst)
  {
    changePrecision(src, dst);
    changePrecision(dst, src);
  }

  void roundToAmvrSignalPrecision(const MvPrecision& src, const int amvr)
  {
    roundToPrecision(src, m_amvrPrecision[amvr]);
  }

#if JVET_M0444_SMVD
  Mv getSymmvdMv(const Mv& curMvPred, const Mv& tarMvPred)
  {
    return Mv(tarMvPred.hor - hor + curMvPred.hor, tarMvPred.ver - ver + curMvPred.ver);
  }
#endif

#if JVET_M0145_AFFINE_MV_CLIP
  void clipToStorageBitDepth()
  {
    hor = Clip3( -(1 << 17), (1 << 17) - 1, hor );
    ver = Clip3( -(1 << 17), (1 << 17) - 1, ver );
  }
#endif
};// END CLASS DEFINITION MV

namespace std
{
  template <>
  struct hash<Mv> : public unary_function<Mv, uint64_t>
  {
    uint64_t operator()(const Mv& value) const
    {
      return (((uint64_t)value.hor << 32) + value.ver);
    }
  };
};
void clipMv ( Mv& rcMv, const struct Position& pos,
              const struct Size& size,
              const class SPS& sps );

void roundAffineMv( int& mvx, int& mvy, int nShift );

//! \}

#endif // __MV__
