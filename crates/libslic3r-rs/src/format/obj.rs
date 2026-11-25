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
/// OBJ.cpp:25-245 with `out_mtl = nullptr`.
pub fn load_obj(
    path: &Path,
    obj_info: &mut ObjInfo,
    message: &mut String,
    do_gamma_correct: bool,
) -> Result<TriangleMesh> {
    load_obj_with_mtl(path, obj_info, message, do_gamma_correct, None)
}

/// OBJ.cpp:25-245 `load_obj(path, TriangleMesh*, ObjInfo&, message, gamma_correct, ObjParser::MtlData *out_mtl)`:
/// on success the parsed materials are copied to `out_mtl` (OBJ.cpp:244-245).
pub fn load_obj_with_mtl(
    path: &Path,
    obj_info: &mut ObjInfo,
    message: &mut String,
    do_gamma_correct: bool,
    out_mtl: Option<&mut MtlData>,
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

            // OBJ.cpp:59-63: an absolute name is resolved as it is, a relative one against the OBJ's
            // directory, both case-insensitively (empty when nothing exists).
            let requested_mtl_path = Path::new(&mtl_name_cleaned);
            let obj_dir = path.parent().unwrap_or(Path::new(""));
            let mtl_path = if requested_mtl_path.is_absolute() {
                crate::format::resource_path_utils::resolve_existing_path_case_insensitive(requested_mtl_path, "load_obj: mtllib")
            } else {
                crate::format::resource_path_utils::resolve_existing_relative_path_case_insensitive(
                    obj_dir,
                    requested_mtl_path,
                    "load_obj: mtllib",
                )
            };

            if !mtl_path.as_os_str().is_empty() {
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

    // OBJ.cpp:78-93 — count the generated triangles and verify that all faces are valid polygons
    // (any polygon of 3 or more vertices, triangulated as a fan below).
    let mut num_triangles: usize = 0;
    {
        let mut i = 0;
        while i < data.vertices.len() {
            // Find the end of face.
            let mut j = i;
            while j < data.vertices.len() && data.vertices[j].coord_idx != -1 {
                j += 1;
            }
            let num_face_vertices = j - i;
            if num_face_vertices > 0 {
                if num_face_vertices < 3 {
                    error!("failed to parse {:?}. The file contains polygons with less than 3 vertices.", path);
                    *message = "The file contains polygons with less than 3 vertices.".to_string();
                    return Err(Error::Mesh(message.clone()));
                }
                num_triangles += num_face_vertices - 2;
                i = j;
            }
            i += 1;
        }
    }

    // Build indexed triangle set.
    let num_vertices = data.coordinates.len() / OBJ_VERTEX_LENGTH;
    let mut vertices: Vec<Point3F> = Vec::with_capacity(num_vertices);
    let mut indices: Vec<Triangle> = Vec::with_capacity(num_triangles);

    // OBJ.cpp:98-103
    if exist_mtl {
        obj_info.is_single_mtl = data.usemtls.len() == 1 && mtl_data.new_mtl_unmap.len() == 1;
        obj_info.face_colors.reserve(num_triangles);
        obj_info.uvs.reserve(num_triangles);
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

    // OBJ.cpp:119-150
    let mut vi = 0;
    while vi < data.vertices.len() {
        if data.vertices[vi].coord_idx == -1 {
            vi += 1;
            continue;
        }
        let mut local_indices: Vec<i32> = Vec::new();
        let mut local_uvs: Vec<i32> = Vec::new();
        while vi < data.vertices.len() {
            let vertex = data.vertices[vi];
            vi += 1;
            if vertex.coord_idx == -1 {
                break;
            }
            // OBJ.cpp:129-133
            if vertex.coord_idx < 0 || vertex.coord_idx >= vertices.len() as i32 {
                error!("failed to parse {:?}. The file contains invalid vertex index.", path);
                *message = "The file contains invalid vertex index.".to_string();
                return Err(Error::Mesh(message.clone()));
            }
            // OBJ.cpp:134-144 — reject UV indices outside the parsed texture coordinate table up front
            // (-1 is "no texture coordinate"; with an empty table the UV references are inert).
            if !data.texture_coordinates.is_empty()
                && vertex.texture_coord_idx != -1
                && (vertex.texture_coord_idx < 0
                    || (vertex.texture_coord_idx as usize) * 2 + 1 >= data.texture_coordinates.len())
            {
                error!("failed to parse {:?}. The file contains invalid texture coordinate index.", path);
                *message = "The file contains invalid texture coordinate index.".to_string();
                return Err(Error::Mesh(message.clone()));
            }
            // OBJ.cpp:145-146
            local_indices.push(vertex.coord_idx);
            local_uvs.push(vertex.texture_coord_idx);
        }
        // OBJ.cpp:148
        if !local_indices.is_empty() {
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
                                  tri_uvs: [i32; 3],
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
                        // OBJ.cpp:176-183 — get_uv: (0, 0) outside the table
                        let get_uv = |uv_idx: i32| -> [f32; 2] {
                            if uv_idx < 0 || (uv_idx as usize) * 2 + 1 >= tc.len() {
                                return [0.0, 0.0];
                            }
                            [tc[uv_idx as usize * 2], tc[uv_idx as usize * 2 + 1]]
                        };
                        let uv0 = get_uv(tri_uvs[0]);
                        let uv1 = get_uv(tri_uvs[1]);
                        let uv2 = get_uv(tri_uvs[2]);
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
                |face_index: i32, tri_uvs: [i32; 3], obj_info: &mut ObjInfo, data: &ObjData, mtl_data: &MtlData| {
                    if data.usemtls.len() == 1 {
                        set_face_color(
                            face_index,
                            &data.usemtls[0].name.clone(),
                            tri_uvs,
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
                                    tri_uvs,
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
            }

            // OBJ.cpp:222-229 — a fan over the polygon
            let mut tri = 0usize;
            while tri + 2 < local_indices.len() {
                indices.push(Triangle::new(
                    local_indices[0] as u32,
                    local_indices[tri + 1] as u32,
                    local_indices[tri + 2] as u32,
                ));
                let face_index = indices.len() as i32 - 1;
                if exist_mtl {
                    let tri_uvs = [local_uvs[0], local_uvs[tri + 1], local_uvs[tri + 2]];
                    set_face_color_by_mtl(face_index, tri_uvs, obj_info, &data, &mtl_data);
                }
                tri += 1;
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
    // OBJ.cpp:244-245 — if (out_mtl) *out_mtl = mtl_data;
    if let Some(out_mtl) = out_mtl {
        *out_mtl = mtl_data;
    }
    // OBJ.cpp:246
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
    load_obj_to_model_with_mtl(path, obj_info, message, object_name, do_gamma_correct, None)
}

/// OBJ.cpp:248-264 `load_obj(path, Model*, ObjInfo&, message, object_name, gamma_correct, MtlData *out_mtl)`.
pub fn load_obj_to_model_with_mtl(
    path: &Path,
    obj_info: &mut ObjInfo,
    message: &mut String,
    object_name: Option<&str>,
    do_gamma_correct: bool,
    out_mtl: Option<&mut MtlData>,
) -> Result<Model> {
    let mesh = load_obj_with_mtl(path, obj_info, message, do_gamma_correct, out_mtl)?;

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

    // OBJ.cpp:262 `model->add_object(object_name.c_str(), path, std::move(mesh));` (Model.cpp:603-622: one
    // model part, the object's input file, the default extruder).
    let mut model = Model::new();
    model.add_object_with_mesh(&name, &path.to_string_lossy(), crate::model::ModelMesh::from_triangle_mesh(&mesh));
    Ok(model)
}

/// OBJ.cpp:287-420 `obj_to_textured_mesh(obj_info, its, mtl_data, obj_directory, out)`: the textured mesh of an
/// OBJ whose materials map a texture (`map_Kd`): vertices and faces of `its`, per-face UVs with V flipped, the
/// materials' colours and textures (the files' raw bytes, case-insensitively resolved against `obj_directory`)
/// and per-face material ids from the `usemtl` ranges. False when nothing is textured or no texture loads.
pub fn obj_to_textured_mesh(
    obj_info: &ObjInfo,
    its: &indexed_triangle_set,
    mtl_data: &MtlData,
    obj_directory: &str,
    out: &mut crate::texture_painting::TexturedMesh,
) -> bool {
    use crate::texture_painting::TextureImage;
    use std::collections::BTreeMap;
    // OBJ.cpp:294-295
    if its.vertices.is_empty() || its.indices.is_empty() || !obj_info.has_uv_png {
        return false;
    }

    let nv = its.vertices.len();
    let nf = its.indices.len();

    // 1. Copy vertices (OBJ.cpp:300-303)
    out.vertices = (0..nv).map(|i| [its.vertices[i][0], its.vertices[i][1], its.vertices[i][2]]).collect();

    // 2. Copy face indices (OBJ.cpp:305-308)
    out.indices = (0..nf).map(|i| [its.indices[i][0], its.indices[i][1], its.indices[i][2]]).collect();

    // 3. Build per-face UV (uv_coords + uv_indices), V flipped (OBJ.cpp:310-330)
    if !obj_info.uvs.is_empty() {
        let uv_face_count = obj_info.uvs.len();
        out.uv_coords.resize(uv_face_count * 3, [0.0, 0.0]);
        out.uv_indices.resize(nf, [0, 0, 0]);
        for fi in 0..nf {
            if fi < uv_face_count {
                let base = (fi * 3) as i32;
                let b = base as usize;
                for k in 0..3 {
                    out.uv_coords[b + k] = [obj_info.uvs[fi][k][0], 1.0f32 - obj_info.uvs[fi][k][1]];
                }
                out.uv_indices[fi] = [base, base + 1, base + 2];
            } else {
                out.uv_indices[fi] = [0, 0, 0];
            }
        }
    }

    // 4. Build material list and load textures from disk (OBJ.cpp:332-397)
    // Map: material name -> material index (std::map: the last of equal names wins)
    let mut mtl_name_to_idx: BTreeMap<String, i32> = BTreeMap::new();
    for (i, name) in mtl_data.mtl_orders.iter().enumerate() {
        mtl_name_to_idx.insert(name.clone(), i as i32);
    }

    let num_materials = mtl_data.mtl_orders.len();
    out.material_colors.resize(num_materials, [1.0, 1.0, 1.0, 1.0]);
    out.material_texture_map.resize(num_materials, -1);

    // Map: texture filename -> index in out.textures
    let mut png_to_tex_idx: BTreeMap<String, i32> = BTreeMap::new();

    for mi in 0..num_materials {
        let name = &mtl_data.mtl_orders[mi];
        let mtl = match mtl_data.new_mtl_unmap.get(name) {
            Some(m) => m,
            None => continue,
        };

        // Material color from Kd (OBJ.cpp:352-353)
        out.material_colors[mi] = [mtl.kd[0], mtl.kd[1], mtl.kd[2], mtl.tr];

        // Texture from map_Kd (OBJ.cpp:355-363)
        if mtl.map_kd.is_empty() {
            continue;
        }

        if let Some(&idx) = png_to_tex_idx.get(&mtl.map_kd) {
            out.material_texture_map[mi] = idx;
            continue;
        }

        // Resolve texture file path (OBJ.cpp:365-375).
        let requested_tex_path = Path::new(&mtl.map_kd);
        let tex_path = if requested_tex_path.is_absolute() {
            crate::format::resource_path_utils::resolve_existing_path_case_insensitive(
                requested_tex_path,
                "obj_to_textured_mesh: map_Kd",
            )
        } else {
            crate::format::resource_path_utils::resolve_existing_relative_path_case_insensitive(
                Path::new(obj_directory),
                requested_tex_path,
                "obj_to_textured_mesh: map_Kd",
            )
        };

        if tex_path.as_os_str().is_empty() {
            log::warn!("texture not found: {:?}", requested_tex_path);
            continue;
        }

        // Read raw file bytes (OBJ.cpp:377-392)
        let data = match std::fs::read(&tex_path) {
            Ok(d) => d,
            Err(_) => continue,
        };
        if data.is_empty() {
            continue;
        }

        let ti = TextureImage { width: -1, height: -1, channels: 0, data };

        let new_idx = out.textures.len() as i32;
        out.textures.push(ti);
        png_to_tex_idx.insert(mtl.map_kd.clone(), new_idx);
        out.material_texture_map[mi] = new_idx;
    }

    // 5. Build per-face material_ids from usemtls ranges (OBJ.cpp:399-414)
    out.material_ids.resize(nf, -1);
    if !obj_info.usemtls.is_empty() {
        for fi in 0..nf {
            let face_idx = fi as i32;
            for um in &obj_info.usemtls {
                if face_idx >= um.face_start && face_idx <= um.face_end {
                    if let Some(&idx) = mtl_name_to_idx.get(&um.name) {
                        out.material_ids[fi] = idx;
                    }
                    break;
                }
            }
        }
    }

    // OBJ.cpp:416-419
    if out.textures.is_empty() {
        log::warn!("no textures loaded");
        return false;
    }

    log::info!("{} faces, {} textures, {} materials", nf, out.textures.len(), num_materials);
    true
}

/// Store a `TriangleMesh` to an OBJ file.
/// OBJ.cpp:266-271 — store_obj(path, TriangleMesh*) -> mesh->WriteOBJFile(path).
///
/// `TriangleMesh::WriteOBJFile` (TriangleMesh.cpp:240-243) delegates to
/// `its_write_obj` (admesh/shared.cpp:193-208), which writes the mesh's
/// `indexed_triangle_set` as `v %f %f %f` / `f %d %d %d` (1-based) lines under a
/// C-numeric locale. The `%f` conversion emits 6 fractional digits, so format
/// vertices as `{:.6}` to match byte-for-byte; index lines are 1-based ints.
pub fn store_obj(path: &Path, mesh: &TriangleMesh) -> Result<()> {
    use std::io::Write;
    let mut file = std::fs::File::create(path)
        .map_err(|e| Error::IO(format!("Failed to create OBJ file: {}", e)))?;

    // admesh/shared.cpp:202-203 — fprintf(fp, "v %f %f %f\n", ...).
    for v in mesh.vertices() {
        writeln!(file, "v {:.6} {:.6} {:.6}", v.x(), v.y(), v.z())
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
    }
    // admesh/shared.cpp:204-205 — fprintf(fp, "f %d %d %d\n", v0+1, v1+1, v2+1).
    for tri in mesh.indices() {
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
    // DIVERGENCE: C++ ModelObject::mesh() (Model.cpp:1476-1486) builds raw_mesh()
    // (the merge of all is_model_part() volumes) and re-merges it once per
    // ModelInstance after applying that instance's transform. The Rust crate has
    // not yet ported ModelInstance transforms / multi-volume raw_mesh(), so this
    // uses the single `mesh` field directly. Porting ModelObject::mesh() faithfully
    // is a cross-cutting Model-stack change, out of this file's scope.
    let mesh = model_object.mesh.clone();
    // OBJ.cpp:276 — return store_obj(path, &mesh);
    store_obj(path, &mesh)
}

/// Store a `Model` to an OBJ file (merges all object meshes).
/// OBJ.cpp:279-283
pub fn store_obj_model(path: &Path, model: &Model) -> Result<()> {
    // OBJ.cpp:281 — TriangleMesh mesh = model->mesh();
    // DIVERGENCE: C++ Model::mesh() (Model.cpp:733-739) merges each object's
    // ModelObject::mesh() (instance-transformed, multi-volume — see store_obj_object).
    // The Rust crate has not ported that path, so this merges each object's single
    // `mesh` field with a vertex-index offset. Out of this file's scope (Model stack).
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
