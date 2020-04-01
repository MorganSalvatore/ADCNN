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

/** \file     BinaryDecisionTree.cpp
 *  \brief    defines a binary decision tree that can be used for automated optimal coding of multi-level decisions
 */

#include "BinaryDecisionTree.h"
#include "CommonDef.h"

#if !REMOVE_BIN_DECISION_TREE

#include <algorithm>

struct DecisionTreeBuilder
{
  DecisionTreeBuilder( unsigned _id, unsigned _depth = 0 ) : left( nullptr ), right( nullptr ), depth( _depth ), id( _id ) { }

  DecisionTreeBuilder* left;
  DecisionTreeBuilder* right;
  unsigned depth;

  unsigned id;
};

DecisionTreeBuilder* decision( unsigned id, unsigned id0, unsigned id1 )
{
  DecisionTreeBuilder* dtb = new DecisionTreeBuilder( id, 1 );
  dtb->left                = new DecisionTreeBuilder( id0 );
  dtb->right               = new DecisionTreeBuilder( id1 );

  return dtb;
}
DecisionTreeBuilder* decision( unsigned id, DecisionTreeBuilder* sub0, unsigned id1 )
{
  DecisionTreeBuilder* dtb = new DecisionTreeBuilder( id,  sub0->depth + 1 );
  dtb->left                = sub0;
  dtb->right               = new DecisionTreeBuilder( id1, sub0->depth );

  return dtb;
}
DecisionTreeBuilder* decision( unsigned id, DecisionTreeBuilder* sub0, DecisionTreeBuilder* sub1 )
{
  DecisionTreeBuilder* dtb = new DecisionTreeBuilder( id, std::max( sub0->depth, sub1->depth ) + 1 );
  dtb->left                = sub0;
  dtb->right               = sub1;

  return dtb;
}
DecisionTreeBuilder* decision( unsigned id, unsigned id0, DecisionTreeBuilder* sub1 )
{
  DecisionTreeBuilder* dtb = new DecisionTreeBuilder( id,  sub1->depth + 1 );
  dtb->left                = new DecisionTreeBuilder( id0, sub1->depth );
  dtb->right               = sub1;

  return dtb;
}

DecisionTreeTemplate::DecisionTreeTemplate( unsigned _depth )
{
  depth = _depth;

  unsigned maxIds = 2 * ( 1u << depth );

  memset( ids,     0xff, maxIds * sizeof( unsigned ) );
  memset( hasSub,  true, maxIds * sizeof( bool     ) );
  memset( mapping, 0xff, maxIds * sizeof( unsigned ) );
}

void compile( DecisionTreeTemplate& dtt, unsigned offset, unsigned depth, DecisionTreeBuilder* dtb )
{
  dtt.ids[offset] = dtb->id;

  if( dtb->left || dtb->right )
  {
    dtt.hasSub[offset] = true;

    if( dtb->right )
    {
      compile( dtt, offset + 1, depth - 1, dtb->right );
    }

    if( dtb->left )
    {
      compile( dtt, offset + ( 1u << depth ), depth - 1, dtb->left );
    }
  }
  else
  {
    dtt.hasSub[offset] = false;
  }

  delete dtb;
}

DecisionTreeTemplate compile( DecisionTreeBuilder *dtb )
{
  DecisionTreeTemplate dtt( dtb->depth );

  VTMCHECK( dtt.depth > MAX_DEPTH_DECISION_TREE, "Maximum allowed decision tree depth exceeded" );

  compile( dtt, 0, dtt.depth, dtb );

  unsigned maxIds = 2 * ( 1 << dtt.depth );

  for( unsigned i = 0; i < maxIds; i++ )
  {
    if( dtt.ids[i] < maxIds )
    {
      dtt.mapping[dtt.ids[i]] = i;
    }
  }

  return dtt;
}

DecisionTree::DecisionTree( const DecisionTreeTemplate& _dtt ) : dtt( _dtt )
{
  unsigned maxIds = 2 * ( 1u << dtt.depth );

  memset( isAvail, true, maxIds * sizeof( bool     ) );
  memset( ctxId,   0,    maxIds * sizeof( unsigned ) );
}

void DecisionTree::setAvail( unsigned id, bool _isAvail )
{
  CHECKD( dtt.hasSub[dtt.mapping[id]], "Trying to set availability of a not-endnode element" );

  isAvail[dtt.mapping[id]] = _isAvail;
}

void DecisionTree::setCtxId( unsigned id, unsigned _ctxId )
{
  CHECKD( !dtt.hasSub[dtt.mapping[id]], "Trying to set a context of a endnode element" );

  // value 0 means coding as EP bin
  ctxId[dtt.mapping[id]] = _ctxId + 1;
}

void DecisionTree::reduce( unsigned offset /*= 0*/, int depth /*= -1 */ )
{
  if( depth < 0 ) depth = dtt.depth;

  bool avail = false;

  if( !dtt.hasSub[offset] ) return;

  CHECKD( depth == 0, "Inconsistent tree" );

  reduce( offset + 1, depth - 1 );
  avail |= isAvail[offset + 1];

  reduce( offset + ( 1u << depth ), depth - 1 );
  avail |= isAvail[offset + ( 1u << depth )];

  isAvail[offset] = avail;
}

#endif
