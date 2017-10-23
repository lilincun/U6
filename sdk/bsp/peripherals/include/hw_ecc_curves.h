/**
 * \addtogroup BSP
 * \{
 * \addtogroup DEVICES
 * \{
 * \addtogroup ECC_CURVES
 * \{
 * \brief ECC curves domain parameters
 *
 * \note All curve parameters are provided in big endian form except from Curve25519. 
 *       The ECC engine operates using little endian approach so all parameters apart 
 *       from the ones of Curve25519 must be stored in the ECC RAM reversed.
 */

/**
 *****************************************************************************************
 *
 * @file hw_ecc_curves.h
 *
 * @brief ECC Engine curves parameters.
 *
 * Copyright (C) 2016. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor. All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 *****************************************************************************************
 */

#ifndef HW_ECC_CURVES_H_
#define HW_ECC_CURVES_H_

#if dg_configUSE_HW_ECC

#include "hw_ecc.h"

/**
 * \addtogroup PRIME_FIELD_CURVES
 * \{
 * \brief Prime field curves
 */

/**
 * \brief Curve P-192 (secp192r1, prime192v1) field size
 */
extern const uint8_t hw_ecc_p192_q[32];

/**
 * \brief Curve P-192 (secp192r1, prime192v1) order of generator point
 */
extern const uint8_t hw_ecc_p192_n[32];

/**
 * \brief Curve P-192 (secp192r1, prime192v1) generator point x coordinate
 */
extern const uint8_t hw_ecc_p192_Gx[32];

/**
 * \brief Curve P-192 (secp192r1, prime192v1) generator point y coordinate
 */
extern const uint8_t hw_ecc_p192_Gy[32];

/**
 * \brief Curve P-192 (secp192r1, prime192v1) coefficient a
 */
extern const uint8_t hw_ecc_p192_a[32];

/**
 * \brief Curve P-192 (secp192r1, prime192v1) coefficient b
 */
extern const uint8_t hw_ecc_p192_b[32];

/**
 * \brief Load Curve P-192 (secp192r1, prime192v1) parameters in ECC memory
 *
 * \param [in] base_addr The base address of the ECC engine data RAM (must be 1KiB aligned)
 */
static inline void hw_ecc_p192_load_params(uint8_t *base_addr)
{
        hw_ecc_write256_r(0, hw_ecc_p192_q, base_addr);
        hw_ecc_write256_r(1, hw_ecc_p192_n, base_addr);
        hw_ecc_write256_r(2, hw_ecc_p192_Gx, base_addr);
        hw_ecc_write256_r(3, hw_ecc_p192_Gy, base_addr);
        hw_ecc_write256_r(4, hw_ecc_p192_a, base_addr);
        hw_ecc_write256_r(5, hw_ecc_p192_b, base_addr);
}

/**
 * \brief Curve P-224 (secp224r1) field size
 */
extern const uint8_t hw_ecc_p224_q[32];

/**
 * \brief Curve P-224 (secp224r1) order of generator point
 */
extern const uint8_t hw_ecc_p224_n[32];

/**
 * \brief Curve P-224 (secp224r1) generator point x coordinate
 */
extern const uint8_t hw_ecc_p224_Gx[32];

/**
 * \brief Curve P-224 (secp224r1) generator point y coordinate
 */
extern const uint8_t hw_ecc_p224_Gy[32];

/**
 * \brief Curve P-224 (secp224r1) coefficient a
 */
extern const uint8_t hw_ecc_p224_a[32];

/**
 * \brief Curve P-224 (secp224r1) coefficient b
 */
extern const uint8_t hw_ecc_p224_b[32];

/**
 * \brief Load Curve P-224 (secp224r1) parameters in ECC memory
 *
 * \param [in] base_addr The base address of the ECC engine data RAM (must be 1KiB aligned)
 */
static inline void hw_ecc_p224_load_params(uint8_t *base_addr)
{
        hw_ecc_write256_r(0, hw_ecc_p224_q, base_addr);
        hw_ecc_write256_r(1, hw_ecc_p224_n, base_addr);
        hw_ecc_write256_r(2, hw_ecc_p224_Gx, base_addr);
        hw_ecc_write256_r(3, hw_ecc_p224_Gy, base_addr);
        hw_ecc_write256_r(4, hw_ecc_p224_a, base_addr);
        hw_ecc_write256_r(5, hw_ecc_p224_b, base_addr);
}

/**
 * \brief Curve P-256 (secp256r1) field size
 */
extern const uint8_t hw_ecc_p256_q[32];

/**
 * \brief Curve P-256 (secp256r1) order of generator point
 */
extern const uint8_t hw_ecc_p256_n[32];

/**
 * \brief Curve P-256 (secp256r1) generator point x coordinate
 */
extern const uint8_t hw_ecc_p256_Gx[32];

/**
 * \brief Curve P-256 (secp256r1) generator point y coordinate
 */
extern const uint8_t hw_ecc_p256_Gy[32];

/**
 * \brief Curve P-256 (secp256r1) coefficient a
 */
extern const uint8_t hw_ecc_p256_a[32];

/**
 * \brief Curve P-256 (secp256r1) coefficient b
 */
extern const uint8_t hw_ecc_p256_b[32];

/**
 * \brief Load Curve P-256 (secp256r1) parameters in ECC memory
 *
 * \param [in] base_addr The base address of the ECC engine data RAM (must be 1KiB aligned)
 */
static inline void hw_ecc_p256_load_params(uint8_t *base_addr)
{
        hw_ecc_write256_r(0, hw_ecc_p256_q, base_addr);
        hw_ecc_write256_r(1, hw_ecc_p256_n, base_addr);
        hw_ecc_write256_r(2, hw_ecc_p256_Gx, base_addr);
        hw_ecc_write256_r(3, hw_ecc_p256_Gy, base_addr);
        hw_ecc_write256_r(4, hw_ecc_p256_a, base_addr);
        hw_ecc_write256_r(5, hw_ecc_p256_b, base_addr);
}

/**
 * \brief Curve 25519 field size
 */
extern const uint8_t hw_ecc_curve25519_p[32];

/**
 * \brief Curve 25519 coefficient (a - 2)/4
 */
extern const uint8_t hw_ecc_curve25519_a24[32];

/**
 * \brief Curve 25519 generator point
 */
extern const uint8_t hw_ecc_curve25519_G[32];

/**
 * \}
 */

/**
 * \addtogroup BINARY_FIELD_CURVES
 * \{
 * \brief Binary field curves
 */

/**
 * \brief Curve B-233 field size
 */
extern const uint8_t hw_ecc_b233_q[32];

/**
 * \brief Curve B-233 order of generator point
 */
extern const uint8_t hw_ecc_b233_n[32];

/**
 * \brief Curve B-233 generator point x coordinate
 */
extern const uint8_t hw_ecc_b233_Gx[32];

/**
 * \brief Curve B-233 generator point y coordinate
 */
extern const uint8_t hw_ecc_b233_Gy[32];

/**
 * \brief Curve B-233 coefficient a
 */
extern const uint8_t hw_ecc_b233_a[32];

/**
 * \brief Curve B-233 coefficient b
 */
extern const uint8_t hw_ecc_b233_b[32];

/**
 * \brief Load Curve B-233 parameters in ECC memory
 *
 * \param [in] base_addr The base address of the ECC engine data RAM (must be 1KiB aligned)
 */
static inline void hw_ecc_b233_load_params(uint8_t *base_addr)
{
        hw_ecc_write256_r(0, hw_ecc_b233_q, base_addr);
        hw_ecc_write256_r(1, hw_ecc_b233_n, base_addr);
        hw_ecc_write256_r(2, hw_ecc_b233_Gx, base_addr);
        hw_ecc_write256_r(3, hw_ecc_b233_Gy, base_addr);
        hw_ecc_write256_r(4, hw_ecc_b233_a, base_addr);
        hw_ecc_write256_r(5, hw_ecc_b233_b, base_addr);
}

/**
 * \}
 */

#endif /* dg_configUSE_HW_ECC */

#endif /* HW_ECC_CURVES_H_ */

/**
 * \}
 * \}
 * \}
 */

