//! OBJ file format support.
//!
//! This is a thin compatibility shim over the faithful 1:1 port of
//! BambuStudio's `src/libslic3r/Format/OBJ.cpp`, which lives at
//! [`crate::format::obj`] (mirroring the C++ `Format/` subdirectory layout).
//!
//! The real loading/storing logic — `objparse`/`mtlparse`, material/colour
//! extraction, quad triangulation and the `volume() < 0 -> flip_triangles()`
//! orientation fix — is implemented there. This module only exposes the
//! ergonomic `path -> Model` / `path -> Vec<TriangleMesh>` entry points used by
//! `crate::model` and other callers, forwarding to the faithful port.
//!
//! - `src/libslic3r/Format/OBJ.cpp`

use crate::format::obj::{self, ObjInfo};
use crate::model::Model;
use crate::triangle_mesh::TriangleMesh;
use crate::Result;
use std::path::Path;

/// Load an OBJ file into a `Model`.
///
/// Faithful entry point: forwards to [`crate::format::obj::load_obj_to_model`],
/// the 1:1 port of `bool load_obj(const char *path, Model *model, ...)`.
/// Format/OBJ.cpp:247-264
pub fn load_obj<P: AsRef<Path>>(path: P) -> Result<Model> {
    // Format/OBJ.cpp:247 — ObjInfo / message are produced by the faithful loader.
    let mut obj_info = ObjInfo::new();
    let mut message = String::new();
    // Format/OBJ.cpp:251 — object_name == nullptr -> derive name from the path.
    // Format/OBJ.cpp:251 — gamma_correct defaults to false (OBJ.hpp:76).
    obj::load_obj_to_model(path.as_ref(), &mut obj_info, &mut message, None, false)
}

/// Load meshes from an OBJ file (legacy API): extract the mesh from each model object.
/// Format/OBJ.cpp:247-264
pub fn load_obj_meshes<P: AsRef<Path>>(path: P) -> Result<Vec<TriangleMesh>> {
    // Format/OBJ.cpp:251
    let model = load_obj(path)?;
    // Format/OBJ.cpp:253-261
    Ok(model.objects.into_iter().map(|o| o.mesh).collect())
}

/// Save a `Model` to an OBJ file.
///
/// Forwards to [`crate::format::obj::store_obj_model`], the port of
/// `bool store_obj(const char *path, Model *model)`.
/// Format/OBJ.cpp:279-283
pub fn save_obj<P: AsRef<Path>>(path: P, model: &Model) -> Result<()> {
    // Format/OBJ.cpp:281-282
    obj::store_obj_model(path.as_ref(), model)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_obj_missing_file() {
        // A non-existent path must fail to parse (objparse returns false).
        let result = load_obj("/nonexistent/path/to/file.obj");
        assert!(result.is_err());
    }

    #[test]
    fn test_save_obj_empty_model() {
        let model = Model::new();
        let tmp = std::env::temp_dir().join("slic3r_obj_shim_test.obj");
        let result = save_obj(&tmp, &model);
        assert!(result.is_ok());
        let _ = std::fs::remove_file(&tmp);
    }
}
