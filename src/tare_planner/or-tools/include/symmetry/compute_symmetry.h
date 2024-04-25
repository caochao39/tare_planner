/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/*                  This file is part of the program and library             */
/*         SCIP --- Solving Constraint Integer Programs                      */
/*                                                                           */
/*  Copyright (c) 2002-2023 Zuse Institute Berlin (ZIB)                      */
/*                                                                           */
/*  Licensed under the Apache License, Version 2.0 (the "License");          */
/*  you may not use this file except in compliance with the License.         */
/*  You may obtain a copy of the License at                                  */
/*                                                                           */
/*      http://www.apache.org/licenses/LICENSE-2.0                           */
/*                                                                           */
/*  Unless required by applicable law or agreed to in writing, software      */
/*  distributed under the License is distributed on an "AS IS" BASIS,        */
/*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*  See the License for the specific language governing permissions and      */
/*  limitations under the License.                                           */
/*                                                                           */
/*  You should have received a copy of the Apache-2.0 license                */
/*  along with SCIP; see the file LICENSE. If not visit scipopt.org.         */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/**@file   compute_symmetry.h
 * @brief  interface for symmetry computations
 * @author Marc Pfetsch
 */

/*---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8----+----9----+----0----+----1----+----2*/

#ifndef __SCIP_COMPUTE_SYMMETRY_H_
#define __SCIP_COMPUTE_SYMMETRY_H_

#include "scip/scip.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "symmetry/struct_symmetry.h"

/** return whether symmetry can be computed */
SCIP_EXPORT
SCIP_Bool SYMcanComputeSymmetry(void);

/** return name of external program used to compute generators */
SCIP_EXPORT
const char* SYMsymmetryGetName(void);

/** return description of external program used to compute generators */
SCIP_EXPORT
const char* SYMsymmetryGetDesc(void);

/** return name of additional external program used for computing symmetries */
SCIP_EXPORT
const char* SYMsymmetryGetAddName(void);

/** return description of additional external program used to compute symmetries */
SCIP_EXPORT
const char* SYMsymmetryGetAddDesc(void);

/** compute generators of symmetry group */
SCIP_EXPORT
SCIP_RETCODE SYMcomputeSymmetryGenerators(
   SCIP*                 scip,               /**< SCIP pointer */
   int                   maxgenerators,      /**< maximal number of generators constructed (= 0 if unlimited) */
   SYM_MATRIXDATA*       matrixdata,         /**< data for MIP matrix */
   SYM_EXPRDATA*         exprdata,           /**< data for nonlinear constraints */
   int*                  nperms,             /**< pointer to store number of permutations */
   int*                  nmaxperms,          /**< pointer to store maximal number of permutations (needed for freeing storage) */
   int***                perms,              /**< pointer to store permutation generators as (nperms x npermvars) matrix */
   SCIP_Real*            log10groupsize,     /**< pointer to store log10 of size of group */
   SCIP_Real*            symcodetime         /**< pointer to store the time for symmetry code */
   );

#ifdef __cplusplus
}
#endif

#endif
