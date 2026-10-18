//! filters: safe wrappers over the shape filter glue (src/c/box2d_glue_filters.c). Every shape id must be a live shape.
use crate::Shape;

extern "C" {
    pub fn b2Glue_SetShapeFilter(shape: Shape, category_bits: u64, mask_bits: u64, group_index: i32);
    pub fn b2Glue_GetShapeFilter(shape: Shape, out2: *mut u64, out_group: *mut i32);
}

/// Collision filter of a shape, as `b2Filter`. Two shapes collide when each one's `category_bits` overlap the
/// other's `mask_bits`, unless they share a nonzero `group_index` (positive forces collision, negative forbids it).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Filter {
    pub category_bits: u64,
    pub mask_bits: u64,
    pub group_index: i32,
}

/// Box2D's default: category bit 0, collides with every category, group 0.
impl Default for Filter {
    fn default() -> Self {
        Filter { category_bits: 1, mask_bits: u64::MAX, group_index: 0 }
    }
}

pub fn set_filter(shape: Shape, filter: Filter) {
    unsafe { b2Glue_SetShapeFilter(shape, filter.category_bits, filter.mask_bits, filter.group_index) }
}

pub fn filter(shape: Shape) -> Filter {
    let mut bits = [0u64; 2];
    let mut group = 0i32;
    unsafe { b2Glue_GetShapeFilter(shape, bits.as_mut_ptr(), &mut group) };
    Filter { category_bits: bits[0], mask_bits: bits[1], group_index: group }
}
