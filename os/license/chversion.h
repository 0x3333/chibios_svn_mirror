/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    chversion.h
 * @brief   Version Module macros and structures.
 *
 * @addtogroup chibios_version
 * @details This module contains information about the ChibiOS release, it
 *          is common to all subsystems.
 * @{
 */

#ifndef CHVERSION_H
#define CHVERSION_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @brief   ChibiOS product identification macro.
 */
#define __CHIBIOS__

/**
 * @brief   Stable release flag.
 */
#define CH_VERSION_STABLE       1

/**
 * @name    ChibiOS version identification
 * @{
 */
/**
 * @brief   ChibiOS version string.
 */
<<<<<<< HEAD
#define CH_VERSION              "20.3.0"
=======
#define CH_VERSION              "2012.1.0"
>>>>>>> upstream/master

/**
 * @brief   ChibiOS version release year.
 */
<<<<<<< HEAD
#define CH_VERSION_YEAR         20
=======
#define CH_VERSION_YEAR         12
>>>>>>> upstream/master

/**
 * @brief   ChibiOS version release month.
 */
<<<<<<< HEAD
#define CH_VERSION_MONTH        3
=======
#define CH_VERSION_MONTH        1
>>>>>>> upstream/master

/**
 * @brief   ChibiOS version patch number.
 */
#define CH_VERSION_PATCH        0

/**
 * @brief   ChibiOS version nickname.
 */
#define CH_VERSION_NICKNAME     "Erchie"
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/**
 * @brief   Current version date in numeric form (yyyymm).
 */
#define CH_VERSION_DATE                                                     \
  (((CH_VERSION_YEAR + 2000) * 100) + CH_VERSION_MONTH)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* CHVERSION_H */

/** @} */
