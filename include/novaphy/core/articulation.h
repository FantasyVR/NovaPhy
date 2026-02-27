#pragma once

#include <vector>

#include "novaphy/core/body.h"
#include "novaphy/core/joint.h"
#include "novaphy/math/math_types.h"
#include "novaphy/math/spatial.h"

namespace novaphy {

/**
 * @brief Tree-structured articulated rigid-body model.
 *
 * @details Stores per-link joint definitions and inertial properties for use
 * by Featherstone algorithms. Link order must satisfy parent-before-child.
 */
struct Articulation {
    std::vector<Joint> joints;           /**< Joint list; joint[i] connects link i to parent. */
    std::vector<RigidBody> bodies;       /**< Rigid-body inertial properties per link. */
    std::vector<SpatialMatrix> I_body;   /**< Spatial inertia matrix per link in body frame. */

    /**
     * @brief Get total number of links.
     *
     * @return Number of articulation links.
     */
    int num_links() const { return static_cast<int>(joints.size()); }

    /**
     * @brief Get total generalized-position dimension.
     *
     * @return Sum of `num_q()` over all joints.
     */
    int total_q() const {
        int n = 0;
        for (const auto& j : joints) n += j.num_q();
        return n;
    }

    /**
     * @brief Get total generalized-velocity dimension.
     *
     * @return Sum of `num_qd()` over all joints.
     */
    int total_qd() const {
        int n = 0;
        for (const auto& j : joints) n += j.num_qd();
        return n;
    }

    /**
     * @brief Compute position-block start index for a link.
     *
     * @param [in] link Link index.
     * @return Start offset in global generalized-position vector q.
     */
    int q_start(int link) const {
        int offset = 0;
        for (int i = 0; i < link; ++i) offset += joints[i].num_q();
        return offset;
    }

    /**
     * @brief Compute velocity-block start index for a link.
     *
     * @param [in] link Link index.
     * @return Start offset in global generalized-velocity vector qd.
     */
    int qd_start(int link) const {
        int offset = 0;
        for (int i = 0; i < link; ++i) offset += joints[i].num_qd();
        return offset;
    }

    /**
     * @brief Build per-link spatial inertia matrices from rigid-body properties.
     *
     * @return void
     */
    void build_spatial_inertias() {
        I_body.resize(num_links());
        for (int i = 0; i < num_links(); ++i) {
            I_body[i] = spatial_inertia_matrix(
                bodies[i].mass, bodies[i].com, bodies[i].inertia);
        }
    }
};

}  // namespace novaphy
