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

/** \file     BinaryDecisionTree.h
 *  \brief    declares a binary decision tree that can be used for automated optimal coding of multi-level cascaded decisions
 */

#ifndef __BINARYDECISIONTREE__
#define __BINARYDECISIONTREE__

#include "CommonDef.h"

#if !REMOVE_BIN_DECISION_TREE

#define MAX_DEPTH_DECISION_TREE   5
#define MAX_NODES_DECISION_TREE ( 2 * ( 1 << MAX_DEPTH_DECISION_TREE ) )

// decision tree template describes the trees and contains mappings between node ids and their in-tree positions
struct DecisionTreeTemplate
{
  DecisionTreeTemplate( unsigned _depth );

  unsigned depth;
  unsigned ids    [MAX_NODES_DECISION_TREE]; // maps the in-tree position to node-id
  bool     hasSub [MAX_NODES_DECISION_TREE]; // stores, for each in-tree position, the information if the node is an end-node ('false'), or a decision-node ('true')
  unsigned mapping[MAX_NODES_DECISION_TREE]; // maps the node-ids to the in-tree positions
};

// decision tree contains sparsity information (node availability) and the context ids needed to encode a decision
// the tree topology described by the decision tree template 'dtt' is used for the tree structure
struct DecisionTree
{
  DecisionTree( const DecisionTreeTemplate& _dtt );

  const DecisionTreeTemplate &dtt;
  bool      isAvail[MAX_NODES_DECISION_TREE];
  unsigned  ctxId  [MAX_NODES_DECISION_TREE];

  // if an end-node is not available, some coding bins can be skipped - availability of the end-nodes can be set using this function
  void setAvail( unsigned id, bool _isAvail );
  // for decision nodes, the context for decision encoding can be set here. don't set any context to code as a EP bin
  void setCtxId( unsigned id, unsigned _ctxId );
  // propagate the end-nodes availability across decision nodes (call with default parameters for correct results)
  void reduce  ( unsigned offset = 0, int depth = -1 );
};

struct DecisionTreeBuilder;

// use this function to easily create decision trees from the output of decision(...) functions (allows for nice human-readable form)
DecisionTreeTemplate compile( DecisionTreeBuilder *dtb );

// parameter naming
// id:  name of the node
// id0: name of a direct outcome for the '0'-case
// id1: name of a direct outcome for the '1'-case

// a simple decision node with two direct outcomes
DecisionTreeBuilder* decision( unsigned id, unsigned id0, unsigned id1 );
// a decision node with decision sub-tree on the left side (the '0'-case) and a direct outcome on the right (the '1'-case)
DecisionTreeBuilder* decision( unsigned id, DecisionTreeBuilder* sub0, unsigned id1 );
// a decision node with direct outcome on the left side and a decision sub-tree on the right
DecisionTreeBuilder* decision( unsigned id, DecisionTreeBuilder* sub0, DecisionTreeBuilder* sub1 );
// a decision with two decision sub-trees
DecisionTreeBuilder* decision( unsigned id, unsigned id0, DecisionTreeBuilder* sub1 );

#endif

#endif
