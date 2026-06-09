//! OBJ file loading and storing.
//!
//! C++ Reference:
//! - Format/OBJ.hpp
//! - Format/OBJ.cpp
//!
//! Higher-level OBJ I/O that uses `objparser` to build `TriangleMesh` / `Model`.

use crate::color::ColorRGBA;
use crate::format::objparser::{self, MtlData, ObjData, ObjUseMtl, OBJ_VERTEX_LENGTH};
use crate::geometry::Point3F;
use crate::model::{Model, ModelObject};
use crate::normal_utils::{indexed_triangle_set, StlTriangleVertexIndices, StlVertex};
use crate::triangle_mesh::{its_flip_triangles, its_volume, Triangle, TriangleMesh};
use crate::{Error, Result};

use log::error;
use std::collections::HashMap;
use std::path::Path;

#[cfg(target_os = "windows")]
const DIR_SEPARATOR: char = '\\';
#[cfg(not(target_os = "windows"))]
const DIR_SEPARATOR: char = '/';

// ---------------------------------------------------------------------------
// Types  (OBJ.hpp)
// ---------------------------------------------------------------------------

/// RGBA colour as four f32 components.
pub type RGBA = [f32; 4];

/// Per-triangle colour binding for 3MF-style colour data.
/// OBJ.hpp:13-16
#[derive(Debug, Clone)]
pub struct TriangleColor {
    pub pid: i32,
    pub indices: [i32; 3],
}

impl TriangleColor {
    pub fn new() -> Self {
        Self {
            pid: -1,
            indices: [-1, -1, -1],
        }
    }
}

/// Per-volume colour binding.
/// OBJ.hpp:19-23
#[derive(Debug, Clone)]
pub struct VolumeColorInfo {
    pub pid: i32,
    pub pindex: i32,
    pub triangle_colors: Vec<TriangleColor>,
}

impl VolumeColorInfo {
    pub fn new() -> Self {
        Self {
            pid: -1,
            pindex: -1,
            triangle_colors: Vec::new(),
        }
    }
}

/// Information extracted during OBJ loading (colours, materials, UVs, etc.).
/// OBJ.hpp:26-44
#[derive(Debug, Clone)]
pub struct ObjInfo {
    pub vertex_colors: Vec<RGBA>,
    pub face_colors: Vec<RGBA>,
    pub mtl_colors: Vec<RGBA>,
    pub mtl_color_names: Vec<String>,
    pub usemtls: Vec<ObjUseMtl>,
    pub first_time_using_makerlab: bool,
    pub is_single_mtl: bool,
    pub lost_material_name: String,
    pub uvs: Vec<[[f32; 2]; 3]>,
    pub obj_directory: String,
    pub pngs: HashMap<String, bool>,
    pub uv_map_pngs: HashMap<i32, String>,
    pub has_uv_png: bool,
    pub ml_region: String,
    pub ml_name: String,
    pub ml_id: String,
}

impl ObjInfo {
    pub fn new() -> Self {
        Self {
            vertex_colors: Vec::new(),
            face_colors: Vec::new(),
            mtl_colors: Vec::new(),
            mtl_color_names: Vec::new(),
            usemtls: Vec::new(),
            first_time_using_makerlab: false,
            is_single_mtl: false,
            lost_material_name: String::new(),
            uvs: Vec::new(),
            obj_directory: String::new(),
            pngs: HashMap::new(),
            uv_map_pngs: HashMap::new(),
            has_uv_png: false,
            ml_region: String::new(),
            ml_name: String::new(),
            ml_id: String::new(),
        }
    }
}

/// Selector for how an OBJ import dialog should be treated.
/// OBJ.hpp:66-72
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormatType {
    Obj,
    Standard3mf,
}

/// Data exchanged with a colour-assignment dialog.
/// OBJ.hpp:45-73
#[derive(Debug, Clone)]
pub struct ObjDialogInOut {
    pub input_colors: Vec<RGBA>,
    pub usemtls: Vec<ObjUseMtl>,
    pub is_single_color: bool,
    pub filament_ids: Vec<u8>,
    pub first_extruder_id: u8,
    pub deal_vertex_color: bool,
    pub volume_colors: HashMap<i32, VolumeColorInfo>,
    pub color_group_map: HashMap<i32, Vec<RGBA>>,
    pub mtl_colors: Vec<RGBA>,
    pub mtl_color_names: Vec<String>,
    pub first_time_using_makerlab: bool,
    pub ml_region: String,
    pub ml_name: String,
    pub ml_id: String,
    pub lost_material_name: String,
    pub input_type: FormatType,
    pub exist_color_error: bool,
    pub exist_texture_error: bool,
}

impl ObjDialogInOut {
    pub fn new() -> Self {
        Self {
            input_colors: Vec::new(),
            usemtls: Vec::new(),
            is_single_color: false,
            filament_ids: Vec::new(),
            first_extruder_id: 1,
            deal_vertex_color: false,
            volume_colors: HashMap::new(),
            color_group_map: HashMap::new(),
            mtl_colors: Vec::new(),
            mtl_color_names: Vec::new(),
            first_time_using_makerlab: false,
            ml_region: String::new(),
            ml_name: String::new(),
            ml_id: String::new(),
            lost_material_name: String::new(),
            input_type: FormatType::Obj,
            exist_color_error: false,
            exist_texture_error: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Loading  (OBJ.cpp:25-245)
// ---------------------------------------------------------------------------

/// Load an OBJ file into a `TriangleMesh`, filling `obj_info` with material/colour data.
/// OBJ.cpp:25-245
pub fn load_obj(
    path: &Path,
    obj_info: &mut ObjInfo,
    message: &mut String,
    do_gamma_correct: bool,
) -> Result<TriangleMesh> {
    let mut data = ObjData::new();
    let mut mtl_data = MtlData::new();

    if !objparser::objparse(path, &mut data) {
        error!("load_obj: failed to parse {:?}", path);
        *message = "load_obj: failed to parse".to_string();
        return Err(Error::IO("load_obj: failed to parse".into()));
    }

    obj_info.ml_region = data.ml_region.clone();
    obj_info.ml_name = data.ml_name.clone();
    obj_info.ml_id = data.ml_id.clone();

    let mut exist_mtl = false;
    if !data.mtllibs.is_empty() {
        for mtl_name in &data.mtllibs {
            if mtl_name.is_empty() {
                continue;
            }
            exist_mtl = true;

            let mut mtl_name_cleaned = mtl_name.clone();
            if mtl_name_cleaned.starts_with("./") {
                mtl_name_cleaned = mtl_name_cleaned[2..].to_string();
            }

            // Try as absolute path first, then relative to OBJ directory
            let mtl_abs = Path::new(&mtl_name_cleaned);
            let mtl_path = if mtl_abs.exists() {
                mtl_abs.to_path_buf()
            } else {
                let parent = path.parent().unwrap_or(Path::new(""));
                parent.join(&mtl_name_cleaned)
            };

            if mtl_path.exists() {
                if !objparser::mtlparse(&mtl_path, &mut mtl_data) {
                    error!("load_obj:load_mtl: failed to parse {:?}", mtl_path);
                    *message = "load mtl in obj: failed to parse".to_string();
                    return Err(Error::IO("load mtl in obj: failed to parse".into()));
                }
            } else {
                error!("load_obj: failed to load mtl_path: {:?}", mtl_path);
            }
        }
    }

    // Count faces and verify they are triangles or quads.
    let mut num_faces: usize = 0;
    let mut num_quads: usize = 0;
    {
        let mut i = 0;
        while i < data.vertices.len() {
            let mut j = i;
            while j < data.vertices.len() && data.vertices[j].coord_idx != -1 {
                j += 1;
            }
            let num_face_vertices = j - i;
            if num_face_vertices > 0 {
                if num_face_vertices > 4 {
                    error!(
                        "load_obj: failed to parse {:?}. Polygons with >4 vertices.",
                        path
                    );
                    *message = "The file contains polygons with more than 4 vertices.".to_string();
                    return Err(Error::Mesh(message.clone()));
                } else if num_face_vertices < 3 {
                    error!(
                        "load_obj: failed to parse {:?}. Polygons with <3 vertices.",
                        path
                    );
                    *message = "The file contains polygons with less than 2 vertices.".to_string();
                    return Err(Error::Mesh(message.clone()));
                }
                if num_face_vertices == 4 {
                    num_quads += 1;
                }
                num_faces += 1;
                i = j;
            }
            i += 1;
        }
    }

    // Build indexed triangle set.
    let num_vertices = data.coordinates.len() / OBJ_VERTEX_LENGTH;
    let mut vertices: Vec<Point3F> = Vec::with_capacity(num_vertices);
    let mut indices: Vec<Triangle> = Vec::with_capacity(num_faces + num_quads);

    if exist_mtl {
        obj_info.is_single_mtl = data.usemtls.len() == 1 && mtl_data.new_mtl_unmap.len() == 1;
        obj_info.face_colors.reserve(num_faces + num_quads);
        obj_info.usemtls = data.usemtls.clone();
    }

    for i in 0..num_vertices {
        let j = i * OBJ_VERTEX_LENGTH;
        vertices.push(Point3F::new(
            data.coordinates[j] as f64,
            data.coordinates[j + 1] as f64,
            data.coordinates[j + 2] as f64,
        ));
        // OBJ.cpp:118
        if data.has_vertex_color {
            // OBJ.cpp:119
            let mut color: RGBA = [
                data.coordinates[j + 3],
                data.coordinates[j + 4],
                data.coordinates[j + 5],
                data.coordinates[j + 6],
            ];
            // OBJ.cpp:120-122 — ColorRGBA::gamma_correct(color) corrects ALL four channels.
            if do_gamma_correct {
                ColorRGBA::gamma_correct_rgba(&mut color);
            }
            // OBJ.cpp:123-125 — clamp all four channels into [0, 1].
            for c in color.iter_mut() {
                *c = c.clamp(0.0, 1.0);
            }
            // OBJ.cpp:126
            obj_info.vertex_colors.push(color);
        }
    }

    let mut local_indices = [0i32; 4];
    let mut local_uvs = [0i32; 4];

    let mut vi = 0;
    while vi < data.vertices.len() {
        if data.vertices[vi].coord_idx == -1 {
            vi += 1;
            continue;
        }
        let mut cnt = 0usize;
        while vi < data.vertices.len() {
            let vertex = data.vertices[vi];
            vi += 1;
            if vertex.coord_idx == -1 {
                break;
            }
            if cnt >= 4 {
                continue;
            }
            if vertex.coord_idx < 0 || vertex.coord_idx >= vertices.len() as i32 {
                error!("load_obj: invalid vertex index in {:?}", path);
                *message = "The file contains invalid vertex index.".to_string();
                return Err(Error::Mesh(message.clone()));
            }
            local_indices[cnt] = vertex.coord_idx;
            local_uvs[cnt] = vertex.texture_coord_idx;
            cnt += 1;
        }
        if cnt >= 3 {
            // First triangle
            indices.push(Triangle::new(
                local_indices[0] as u32,
                local_indices[1] as u32,
                local_indices[2] as u32,
            ));
            let current_face_index = indices.len() as i32 - 1;

            // OBJ.cpp:155-170 — get_face_color lambda.
            let get_face_color = |mtl_name: &str, fc: &mut RGBA| -> bool {
                // OBJ.cpp:156
                if let Some(mtl) = mtl_data.new_mtl_unmap.get(mtl_name) {
                    // OBJ.cpp:157 — 0.1 is light ambient
                    for n in 0..3 {
                        // OBJ.cpp:158
                        let object_ka = if mtl.ka[n] > 0.01 && mtl.ka[n] < 0.99 {
                            // OBJ.cpp:160
                            mtl.ka[n] * 0.1
                        } else {
                            0.0
                        };
                        // OBJ.cpp:162
                        let value = object_ka + mtl.kd[n];
                        // OBJ.cpp:163
                        let temp = if do_gamma_correct {
                            ColorRGBA::gamma_correct_value(value)
                        } else {
                            value
                        };
                        // OBJ.cpp:164
                        fc[n] = temp.clamp(0.0, 1.0);
                    }
                    // OBJ.cpp:166 — alpha
                    fc[3] = if do_gamma_correct {
                        ColorRGBA::gamma_correct_value(mtl.tr)
                    } else {
                        mtl.tr
                    };
                    // OBJ.cpp:167
                    true
                } else {
                    // OBJ.cpp:169
                    false
                }
            };

            let set_face_color = |face_index: i32,
                                  mtl_name: &str,
                                  obj_info: &mut ObjInfo,
                                  data: &ObjData,
                                  mtl_data: &MtlData| {
                // OBJ.cpp:172
                if mtl_data.new_mtl_unmap.contains_key(mtl_name) {
                    // OBJ.cpp:173-174 — RGBA face_color; get_face_color(mtl_name, face_color);
                    let mut face_color = [0.0f32; 4];
                    let mtl = &mtl_data.new_mtl_unmap[mtl_name];
                    for n in 0..3 {
                        let object_ka = if mtl.ka[n] > 0.01 && mtl.ka[n] < 0.99 {
                            mtl.ka[n] * 0.1
                        } else {
                            0.0
                        };
                        let value = object_ka + mtl.kd[n];
                        let temp = if do_gamma_correct {
                            ColorRGBA::gamma_correct_value(value)
                        } else {
                            value
                        };
                        face_color[n] = temp.clamp(0.0, 1.0);
                    }
                    face_color[3] = if do_gamma_correct {
                        ColorRGBA::gamma_correct_value(mtl.tr)
                    } else {
                        mtl.tr
                    };

                    // OBJ.cpp:175-180
                    if !mtl.map_kd.is_empty() {
                        // OBJ.cpp:176
                        let png_name = mtl.map_kd.clone();
                        // OBJ.cpp:177
                        obj_info.has_uv_png = true;
                        // OBJ.cpp:178
                        obj_info.pngs.entry(png_name.clone()).or_insert(false);
                        // OBJ.cpp:179
                        obj_info.uv_map_pngs.insert(face_index, png_name);
                    }
                    // OBJ.cpp:181 — if (data.textureCoordinates.size() > 0)
                    if !data.texture_coordinates.is_empty() {
                        let tc = &data.texture_coordinates;
                        // OBJ.cpp:182-185
                        let uv0 = [
                            tc[local_uvs[0] as usize * 2],
                            tc[local_uvs[0] as usize * 2 + 1],
                        ];
                        let uv1 = [
                            tc[local_uvs[1] as usize * 2],
                            tc[local_uvs[1] as usize * 2 + 1],
                        ];
                        let uv2 = [
                            tc[local_uvs[2] as usize * 2],
                            tc[local_uvs[2] as usize * 2 + 1],
                        ];
                        // OBJ.cpp:186
                        obj_info.uvs.push([uv0, uv1, uv2]);
                    }
                    // OBJ.cpp:188
                    obj_info.face_colors.push(face_color);
                }
                // OBJ.cpp:190-194
                else if obj_info.lost_material_name.is_empty() {
                    obj_info.lost_material_name = mtl_name.to_string();
                }
            };

            let set_face_color_by_mtl =
                |face_index: i32, obj_info: &mut ObjInfo, data: &ObjData, mtl_data: &MtlData| {
                    if data.usemtls.len() == 1 {
                        set_face_color(
                            face_index,
                            &data.usemtls[0].name.clone(),
                            obj_info,
                            data,
                            mtl_data,
                        );
                    } else {
                        for k in 0..data.usemtls.len() {
                            let m = &data.usemtls[k];
                            if face_index >= m.face_start && face_index <= m.face_end {
                                set_face_color(
                                    face_index,
                                    &data.usemtls[k].name.clone(),
                                    obj_info,
                                    data,
                                    mtl_data,
                                );
                                break;
                            }
                        }
                    }
                };

            if exist_mtl {
                if obj_info.mtl_colors.is_empty() {
                    if mtl_data.first_time_using_makerlab {
                        obj_info.first_time_using_makerlab = true;
                    }
                    obj_info.mtl_colors.reserve(mtl_data.mtl_orders.len());
                    obj_info.mtl_color_names.reserve(mtl_data.mtl_orders.len());
                    for order_name in &mtl_data.mtl_orders {
                        let mut fc = [0.0f32; 4];
                        if get_face_color(order_name, &mut fc) {
                            obj_info.mtl_colors.push(fc);
                            obj_info.mtl_color_names.push(order_name.clone());
                        }
                    }
                }
                set_face_color_by_mtl(current_face_index, obj_info, &data, &mtl_data);
            }

            // Second triangle for quads
            if cnt == 4 {
                indices.push(Triangle::new(
                    local_indices[0] as u32,
                    local_indices[2] as u32,
                    local_indices[3] as u32,
                ));
                let quad_face_index = indices.len() as i32 - 1;
                if exist_mtl {
                    set_face_color_by_mtl(quad_face_index, obj_info, &data, &mtl_data);
                }
            }
        }
    }

    // OBJ.cpp:236 — *meshptr = TriangleMesh(std::move(its));
    let mut mesh = TriangleMesh::from_parts(vertices, indices);
    // OBJ.cpp:237
    if mesh.is_empty() {
        // OBJ.cpp:238
        error!(
            "load_obj: This OBJ file couldn't be read because it's empty. {:?}",
            path
        );
        // OBJ.cpp:239
        *message = "This OBJ file couldn't be read because it's empty.".to_string();
        // OBJ.cpp:240
        return Err(Error::Mesh(message.clone()));
    }
    // OBJ.cpp:242-243 — if (meshptr->volume() < 0) meshptr->flip_triangles();
    //
    // `TriangleMesh::volume()` is `its_volume(its)`, a *signed* volume computed on
    // the single-precision `indexed_triangle_set`. Reconstruct that `its` from the
    // mesh (the f64 `Point3F` coordinates are exact widenings of the original f32
    // OBJ values, so narrowing back recovers the same f32 bits the C++ used).
    {
        let mut its = indexed_triangle_set::default();
        its.vertices = mesh
            .vertices()
            .iter()
            .map(|v| StlVertex::new(v.x as f32, v.y as f32, v.z as f32))
            .collect();
        its.indices = mesh
            .indices()
            .iter()
            .map(|t| {
                StlTriangleVertexIndices::new(
                    t.indices[0] as i32,
                    t.indices[1] as i32,
                    t.indices[2] as i32,
                )
            })
            .collect();
        if its_volume(&its) < 0.0 {
            // TriangleMesh::flip_triangles() -> its_flip_triangles(its): swap face(1)/face(2).
            its_flip_triangles(&mut its);
            for (t, f) in mesh.indices_mut().iter_mut().zip(its.indices.iter()) {
                t.indices[0] = f[0] as u32;
                t.indices[1] = f[1] as u32;
                t.indices[2] = f[2] as u32;
            }
        }
    }
    // OBJ.cpp:244
    Ok(mesh)
}

/// Load an OBJ file into a `Model`.
/// OBJ.cpp:247-264
pub fn load_obj_to_model(
    path: &Path,
    obj_info: &mut ObjInfo,
    message: &mut String,
    object_name: Option<&str>,
    do_gamma_correct: bool,
) -> Result<Model> {
    let mesh = load_obj(path, obj_info, message, do_gamma_correct)?;

    let name = match object_name {
        Some(n) => n.to_string(),
        None => {
            let path_str = path.to_string_lossy();
            match path_str.rfind(DIR_SEPARATOR) {
                Some(pos) => path_str[pos + 1..].to_string(),
                None => path_str.to_string(),
            }
        }
    };

    let obj = ModelObject::new(name, mesh);
    let mut model = Model::new();
    model.add_object(obj);
    Ok(model)
}

/// Store a `TriangleMesh` to an OBJ file.
/// OBJ.cpp:266-271
pub fn store_obj(path: &Path, mesh: &TriangleMesh) -> Result<()> {
    use std::io::Write;
    let mut file = std::fs::File::create(path)
        .map_err(|e| Error::IO(format!("Failed to create OBJ file: {}", e)))?;

    for v in mesh.vertices() {
        writeln!(file, "v {} {} {}", v.x(), v.y(), v.z())
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
    }
    for tri in mesh.indices() {
        // OBJ uses 1-based indices
        writeln!(
            file,
            "f {} {} {}",
            tri.indices[0] + 1,
            tri.indices[1] + 1,
            tri.indices[2] + 1
        )
        .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
    }
    Ok(())
}

/// Store a `ModelObject` to an OBJ file.
/// OBJ.cpp:273-277
pub fn store_obj_object(path: &Path, model_object: &ModelObject) -> Result<()> {
    // OBJ.cpp:275 — TriangleMesh mesh = model_object->mesh();
    let mesh = model_object.mesh.clone();
    // OBJ.cpp:276 — return store_obj(path, &mesh);
    store_obj(path, &mesh)
}

/// Store a `Model` to an OBJ file (merges all object meshes).
/// OBJ.cpp:279-283
pub fn store_obj_model(path: &Path, model: &Model) -> Result<()> {
    // Merge all object meshes into one
    let mut all_vertices: Vec<Point3F> = Vec::new();
    let mut all_indices: Vec<Triangle> = Vec::new();
    for obj in &model.objects {
        let offset = all_vertices.len() as u32;
        all_vertices.extend_from_slice(obj.mesh.vertices());
        for tri in obj.mesh.indices() {
            all_indices.push(Triangle::new(
                tri.indices[0] + offset,
                tri.indices[1] + offset,
                tri.indices[2] + offset,
            ));
        }
    }
    let merged = TriangleMesh::from_parts(all_vertices, all_indices);
    store_obj(path, &merged)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_obj_info_defaults() {
        let info = ObjInfo::new();
        assert!(!info.has_uv_png);
        assert!(info.vertex_colors.is_empty());
        assert!(!info.is_single_mtl);
    }

    #[test]
    fn test_triangle_color_default() {
        let tc = TriangleColor::new();
        assert_eq!(tc.pid, -1);
        assert_eq!(tc.indices, [-1, -1, -1]);
    }
}
