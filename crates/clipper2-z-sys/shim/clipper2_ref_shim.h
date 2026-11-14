// Test-only reference exports over the Clipper2 the engine pins, in both builds:
// `cz2_ref_z_*` (USINGZ) and `cz2_ref_xy_*` (no Z). See clipper2_ref_shim.cpp and build.rs.
#ifndef CLIPPER2_REF_SHIM_H
#define CLIPPER2_REF_SHIM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// The solution of one `Clipper64::Execute`.
// `coords`: (x, y, z) triples of every path, z = 0 in the build without Z.
// `path_lens`: `num_closed` closed paths, then `num_open` open paths. With `use_tree`, the
// closed paths are the nodes of the PolyTree64 in depth-first pre-order, `tree_parent` the
// index of each node's parent (-1 for a child of the root) and `tree_is_hole` its IsHole().
typedef struct {
    int64_t *coords;
    int32_t *path_lens;
    int32_t *tree_parent;
    int32_t *tree_is_hole;
    int32_t  num_closed;
    int32_t  num_open;
    int32_t  total_points;
    int32_t  succeeded;
    uint64_t cb_calls;
    uint64_t cb_hash;
} Cz2RefResult;

#define CZ2_REF_DECLARE(prefix)                                                                             \
    Cz2RefResult prefix##execute(const int64_t *subj_xyz, const int32_t *subj_lens, int32_t subj_num,       \
                                 const int64_t *open_xyz, const int32_t *open_lens, int32_t open_num,       \
                                 const int64_t *clip_xyz, const int32_t *clip_lens, int32_t clip_num,       \
                                 int32_t clip_type, int32_t fill_rule, int32_t preserve_collinear,          \
                                 int32_t reverse_solution, int32_t use_tree, int32_t add_order,             \
                                 int32_t z_callback, int64_t default_z);                                    \
    void prefix##free(Cz2RefResult r);                                                                      \
    int32_t prefix##point_in_polygon(int64_t x, int64_t y, const int64_t *xyz, int32_t n);                  \
    double prefix##area(const int64_t *xyz, int32_t n);                                                     \
    void prefix##core(const int64_t *p, int64_t *out_i, double *out_d, int32_t *out_b);                     \
    int32_t prefix##from_pin(void);                                                                         \
    const char *prefix##version(void);

CZ2_REF_DECLARE(cz2_ref_z_)
CZ2_REF_DECLARE(cz2_ref_xy_)

#ifdef __cplusplus
}
#endif

#endif // CLIPPER2_REF_SHIM_H
