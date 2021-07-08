//*****************************************************************************
//
//  state.pb.h
//! @file
//!
//! @brief Auto-generated (see below).
//!
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.5.1 of the AmbiqSuite Development Package.
//
//*****************************************************************************
/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.1 at Fri Nov 09 16:58:28 2018. */

#ifndef PB_STATE_PB_H_INCLUDED
#define PB_STATE_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _GetState
{
    uint32_t feature;
/* @@protoc_insertion_point(struct:GetState) */
} GetState;

typedef struct _State
{
    uint32_t feature;
    pb_size_t which_value;
    union
    {
        bool boolean;
        uint32_t integer;
    } value;
/* @@protoc_insertion_point(struct:State) */
} State;

typedef struct _SetState
{
    State state;
/* @@protoc_insertion_point(struct:SetState) */
} SetState;

typedef struct _SynchronizeState
{
    State state;
/* @@protoc_insertion_point(struct:SynchronizeState) */
} SynchronizeState;

/* Default values for struct fields */

/* Initializer values for message structs */
#define State_init_default                       {0, 0, {0}}
#define GetState_init_default                    {0}
#define SetState_init_default                    {State_init_default}
#define SynchronizeState_init_default            {State_init_default}
#define State_init_zero                          {0, 0, {0}}
#define GetState_init_zero                       {0}
#define SetState_init_zero                       {State_init_zero}
#define SynchronizeState_init_zero               {State_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define GetState_feature_tag                     1
#define State_boolean_tag                        2
#define State_integer_tag                        3
#define State_feature_tag                        1
#define SetState_state_tag                       1
#define SynchronizeState_state_tag               1

/* Struct field encoding specification for nanopb */
extern const pb_field_t State_fields[4];
extern const pb_field_t GetState_fields[2];
extern const pb_field_t SetState_fields[2];
extern const pb_field_t SynchronizeState_fields[2];

/* Maximum encoded size of messages (where known) */
#define State_size                               12
#define GetState_size                            6
#define SetState_size                            14
#define SynchronizeState_size                    14

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define STATE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
