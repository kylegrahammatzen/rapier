pub(crate) use contact_constraint_element::*;
pub(crate) use contact_constraints_set::{
    ContactConstraintsSet, joint_data_num_constraints, joint_num_constraints,
};
pub(crate) use contact_with_coulomb_friction::*;
pub(crate) use generic_contact_constraint::*;
pub(crate) use generic_contact_constraint_element::*;

#[cfg(feature = "dim3")]
pub(crate) use contact_with_twist_friction::*;

mod contact_constraint_element;
mod contact_constraints_set;
mod contact_with_coulomb_friction;
mod generic_contact_constraint;
mod generic_contact_constraint_element;

#[cfg(feature = "dim3")]
mod contact_with_twist_friction;

#[cfg(feature = "dim3")]
use crate::utils::ScalarType;
#[cfg(feature = "dim3")]
use crate::{math::DIM, utils::OrthonormalBasis};

#[inline]
#[cfg(feature = "dim3")]
pub(crate) fn compute_tangent_contact_directions<N: ScalarType>(
    force_dir1: &N::Vector,
    linvel1: &N::Vector,
    linvel2: &N::Vector,
) -> [N::Vector; DIM - 1] {
    use crate::utils::CrossProduct;
    use OrthonormalBasis;

    let _ = (linvel1, linvel2);

    // A DETERMINISTIC basis from the (frozen while recycled) contact normal.
    // It must NOT follow the relative velocity: near quiescence that direction
    // is pure noise, and re-applying warm-started friction in a randomly spun basis (a tangential
    // kick every step) pumps the friction-only mode of large stacks; the exact 2×2 tangent solve
    // makes the in-plane orientation accuracy-neutral.
    let tangent1 = force_dir1.orthonormal_vector();
    let bitangent1 = force_dir1.gcross(tangent1);

    [tangent1, bitangent1]
}

/// The linear part of the tangent effective-mass coupling `t1·(im ⊙ t2)`, with `im` the
/// per-axis inverse mass of both bodies.
///
/// The tangents are orthonormal, so this only depends on how `im` differs between axes: it is
/// exactly zero unless a translation is locked, and with one locked (e.g. a body kept in a
/// plane) it keeps the 2×2 friction matrix from losing the coupling between the tangents.
#[inline]
#[cfg(feature = "dim3")]
pub(crate) fn tangent_linear_coupling(
    im: <crate::math::SimdReal as ScalarType>::Vector,
    tangents: &[<crate::math::SimdReal as ScalarType>::Vector; DIM - 1],
) -> crate::math::SimdReal {
    let [t1, t2] = tangents;
    (im.y - im.x) * t1.y * t2.y + (im.z - im.x) * t1.z * t2.z
}
