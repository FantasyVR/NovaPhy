#pragma once

namespace novaphy {

/**
 * @brief Scalar floating-point type used across NovaPhy.
 * @details NovaPhy currently assumes single-precision floating point (`float`)
 *          for all simulation math to balance performance and memory use.
 * @warning Mixing `double` with engine APIs may cause implicit conversions and
 *          inconsistent numerical behavior.
 */
using Scalar = float;

}  // namespace novaphy
