//! Text-shape (3D text) loading.
//!
//! C++ Reference:
//! - Shape/TextShape.hpp
//! - Shape/TextShape.cpp
//!
//! The C++ implementation is built entirely on OpenCascade (OCCT): font
//! discovery (`Font_FontMgr`), glyph-to-BRep conversion (`Font_BRepFont`,
//! `Font_BRepTextBuilder`), prism extrusion (`BRepPrimAPI_MakePrism`) and
//! tessellation (`BRepMesh_IncrementalMesh`). OCCT is a large native C++ CAD
//! kernel and is **not** wasm-safe; there is no OCCT binding crate available
//! in this workspace (the sibling `format/step.rs` port follows the same
//! pattern). This module therefore faithfully ports every portable piece --
//! the `TextResult` data structure, the font-suffix filter list, the
//! `g_occt_fonts_maps` cache, the control flow of `init_occt_fonts` /
//! `load_text_shape` and the helper routines `TextToBRep` / `Prism` /
//! `MakeMesh` -- while the OCCT-dependent geometry operations are gated behind
//! a runtime "OCCT unavailable" path that mirrors what the C++ produces when
//! the optional OCCT dependency is not compiled in (no fonts; no mesh).
//!
//! BLOCKED symbols (require a native OCCT backend, not wasm-safe, no binding
//! crate present): the OCCT calls inside `init_occt_fonts`, `text_to_brep`,
//! `prism`, `make_mesh`. These are ported as control flow with the native
//! kernel calls documented inline and short-circuited.

use crate::triangle_mesh::TriangleMesh;
use crate::utils::{decode_path, resources_dir};

use std::collections::BTreeMap;
use std::sync::Mutex;

// TextShape.cpp:33  namespace Slic3r {

// TextShape.cpp:35  static std::map<std::string, std::string> g_occt_fonts_maps; //map<font_name, font_path>
// A std::map is ordered by key; we mirror that with BTreeMap so that
// iteration order (used to build `stdFontNames` in order) matches C++.
static G_OCCT_FONTS_MAPS: Mutex<BTreeMap<String, String>> = Mutex::new(BTreeMap::new());

// TextShape.cpp:37-39
// static const std::vector<Standard_CString> fonts_suffix{ "Bold", ... };
static FONTS_SUFFIX: &[&str] = &[
    "Bold",
    "Medium",
    "Heavy",
    "Italic",
    "Oblique",
    "Inclined",
    "Light",
    "Thin",
    "Semibold",
    "ExtraBold",
    "ExtraBold",
    "Semilight",
    "SemiLight",
    "ExtraLight",
    "Extralight",
    "Ultralight",
    "Condensed",
    "Ultra",
    "Extra",
    "Expanded",
    "Extended",
    "1",
    "2",
    "3",
    "4",
    "5",
    "6",
    "7",
    "8",
    "9",
    "Al Tarikh",
];

// TextShape.hpp:7-11
// struct TextResult { TriangleMesh text_mesh; double text_width; };
#[derive(Debug, Clone, Default)]
pub struct TextResult {
    pub text_mesh: TriangleMesh,
    pub text_width: f64,
}

impl TextResult {
    pub fn new() -> Self {
        Self::default()
    }
}

// TextShape.cpp:41  std::map<std::string, std::string> get_occt_fonts_maps()
pub fn get_occt_fonts_maps() -> BTreeMap<String, String> {
    // TextShape.cpp:43  return g_occt_fonts_maps;
    G_OCCT_FONTS_MAPS.lock().unwrap().clone()
}

// TextShape.cpp:46  std::vector<std::string> init_occt_fonts()
pub fn init_occt_fonts() -> Vec<String> {
    // TextShape.cpp:48  std::vector<std::string> stdFontNames;
    let mut std_font_names: Vec<String> = Vec::new();

    // TextShape.cpp:50-51
    // Handle(Font_FontMgr) aFontMgr = Font_FontMgr::GetInstance();
    // aFontMgr->InitFontDataBase();
    //
    // TextShape.cpp:53-55
    // TColStd_SequenceOfHAsciiString availFontNames;
    // aFontMgr->GetAvailableFontsNames(availFontNames);
    // stdFontNames.reserve(availFontNames.Size());
    //
    // BLOCKED: Font_FontMgr (OCCT) enumerates system-installed fonts via the
    // native font database. There is no wasm-safe binding. With OCCT
    // unavailable, `availFontNames` is empty and the suffix-filter loop below
    // contributes nothing.
    let avail_font_names: Vec<String> = Vec::new();
    std_font_names.reserve(avail_font_names.len());

    // TextShape.cpp:57  g_occt_fonts_maps.clear();
    let mut occt_fonts_maps = G_OCCT_FONTS_MAPS.lock().unwrap();
    occt_fonts_maps.clear();

    // TextShape.cpp:59  BOOST_LOG_TRIVIAL(info) << "init_occt_fonts start";
    log::info!("init_occt_fonts start");

    // TextShape.cpp:60-64
    // #ifdef __APPLE__
    //     //from resource
    //     stdFontNames.push_back("HarmonyOS Sans SC");
    //     g_occt_fonts_maps.insert(std::make_pair("HarmoneyOS Sans SC",
    //         Slic3r::resources_dir() + "/fonts/" + "HarmonyOS_Sans_SC_Regular.ttf"));
    // #endif
    #[cfg(target_os = "macos")]
    {
        std_font_names.push("HarmonyOS Sans SC".to_string());
        let font_path = resources_dir()
            .join("fonts")
            .join("HarmonyOS_Sans_SC_Regular.ttf")
            .to_string_lossy()
            .into_owned();
        // NOTE: the C++ key here is the misspelled "HarmoneyOS Sans SC" (sic),
        // distinct from the "HarmonyOS Sans SC" pushed into stdFontNames.
        occt_fonts_maps.insert("HarmoneyOS Sans SC".to_string(), font_path);
    }
    #[cfg(not(target_os = "macos"))]
    {
        // resources_dir / decode_path are referenced only on the platform
        // branches below; keep them used so the imports are not dead.
        let _ = (resources_dir as fn() -> std::path::PathBuf, decode_path as fn(&str) -> String);
    }

    // TextShape.cpp:65-91  for (auto afn : availFontNames) { ... }
    //
    // The body filters out hidden fonts (macOS leading '.'), emoji fonts, and
    // styled-variant fonts whose name ends with one of `fonts_suffix`, then
    // resolves the regular-aspect font path through OCCT's
    // Font_SystemFont::FontPath and records ttf/otf/ttc files in
    // g_occt_fonts_maps.
    //
    // BLOCKED on OCCT: Font_SystemFont / Font_FontMgr::GetFont give the
    // per-font filesystem path. With OCCT unavailable `availFontNames` is
    // empty so this loop body never executes; we keep the full filtering logic
    // here for fidelity.
    for afn in &avail_font_names {
        // TextShape.cpp:66-69
        // #ifdef __APPLE__
        //     if (afn->String().StartsWith("."))
        //         continue;
        // #endif
        #[cfg(target_os = "macos")]
        {
            if afn.starts_with('.') {
                continue;
            }
        }
        // TextShape.cpp:70-71
        // if (afn->Search("Emoji") != -1 || afn->Search("emoji") != -1)
        //     continue;
        if afn.contains("Emoji") || afn.contains("emoji") {
            continue;
        }
        // TextShape.cpp:72  bool repeat = false;
        let mut repeat = false;
        // TextShape.cpp:73-78  for (size_t i = 0; i < fonts_suffix.size(); i++) {
        //     if (afn->SearchFromEnd(fonts_suffix[i]) != -1) { repeat = true; break; }
        // }
        for suffix in FONTS_SUFFIX {
            // SearchFromEnd searches for the substring anywhere (returning the
            // last occurrence); matching C++ this is a substring containment
            // test, not an ends-with test.
            if afn.contains(suffix) {
                repeat = true;
                break;
            }
        }
        // TextShape.cpp:79-80  if (repeat) continue;
        if repeat {
            continue;
        }

        // TextShape.cpp:82-90
        // Handle(Font_SystemFont) sys_font = aFontMgr->GetFont(afn->ToCString());
        // TCollection_AsciiString font_path =
        //     sys_font->FontPath(Font_FontAspect::Font_FontAspect_Regular);
        // if (!font_path.IsEmpty() && font_path.SearchFromEnd(".") != -1) {
        //     auto file_type = font_path.SubString(font_path.SearchFromEnd(".") + 1,
        //                                          font_path.Length());
        //     file_type.LowerCase();
        //     if (file_type == "ttf" || file_type == "otf" || file_type == "ttc") {
        //         g_occt_fonts_maps.insert(std::make_pair(afn->ToCString(),
        //             decode_path(font_path.ToCString())));
        //     }
        // }
        let font_path = occt_get_regular_font_path(afn);
        if let Some(font_path) = font_path {
            if !font_path.is_empty() {
                if let Some(dot) = font_path.rfind('.') {
                    let file_type = font_path[dot + 1..].to_lowercase();
                    if file_type == "ttf" || file_type == "otf" || file_type == "ttc" {
                        occt_fonts_maps.insert(afn.clone(), decode_path(&font_path));
                    }
                }
            }
        }
    }
    // TextShape.cpp:92  BOOST_LOG_TRIVIAL(info) << "init_occt_fonts end";
    log::info!("init_occt_fonts end");
    // TextShape.cpp:93-96  // in order
    // for (auto occt_font : g_occt_fonts_maps) {
    //     stdFontNames.push_back(occt_font.first);
    // }
    for occt_font in occt_fonts_maps.iter() {
        std_font_names.push(occt_font.0.clone());
    }
    // TextShape.cpp:97  return stdFontNames;
    std_font_names
}

/// OCCT-backed lookup of a font's regular-aspect filesystem path.
///
/// BLOCKED: needs `Font_FontMgr::GetFont` + `Font_SystemFont::FontPath`
/// (native OCCT, not wasm-safe). Returns `None` when OCCT is unavailable.
fn occt_get_regular_font_path(_font_name: &str) -> Option<String> {
    None
}

/// A faithfully-ported BRep text shape. With OCCT available this would wrap a
/// `TopoDS_Shape`; here it is an opaque marker because the geometry kernel is
/// not present.
#[derive(Debug, Default, Clone)]
struct TopoShape {
    is_null: bool,
}

// TextShape.cpp:100
// static bool TextToBRep(const char* text, const char* font, const float theTextHeight,
//     Font_FontAspect& theFontAspect, TopoDS_Shape& theShape, double& text_width)
fn text_to_brep(
    _text: &str,
    _font: &str,
    _the_text_height: f32,
    _the_font_aspect: FontFontAspect,
    _the_shape: &mut TopoShape,
    _text_width: &mut f64,
) -> bool {
    // TextShape.cpp:102-119  local OCCT objects (anArgIt, aName, aText, aFont,
    // aFontName, aTextHeight, aFontAspect, anIsCompositeCurve, aPenAx3,
    // aNormal, aDirection, aPenLoc, aHJustification, aVJustification,
    // aStrictLevel).
    //
    // TextShape.cpp:121-123
    // aFont.SetCompositeCurveMode(anIsCompositeCurve);
    // if (!aFont.FindAndInit(aFontName.ToCString(), aFontAspect, aTextHeight, aStrictLevel))
    //     return false;
    //
    // TextShape.cpp:125  aPenAx3 = gp_Ax3(aPenLoc, aNormal, aDirection);
    //
    // TextShape.cpp:127-131
    // Handle(Font_TextFormatter) aFormatter = new Font_TextFormatter();
    // aFormatter->Reset();
    // aFormatter->SetupAlignment(aHJustification, aVJustification);
    // aFormatter->Append(aText, *aFont.FTFont());
    // aFormatter->Format();
    //
    // TextShape.cpp:133-141  // get the text width
    // text_width = 0;
    // NCollection_String coll_str = aText;
    // for (NCollection_Utf8Iter anIter = coll_str.Iterator(); *anIter != 0;) {
    //     const Standard_Utf32Char aCharThis = *anIter;
    //     const Standard_Utf32Char aCharNext = *++anIter;
    //     double width = aFont.AdvanceX(aCharThis, aCharNext);
    //     text_width += width;
    // }
    //
    // TextShape.cpp:143-145
    // Font_BRepTextBuilder aBuilder;
    // theShape = aBuilder.Perform(aFont, aFormatter, aPenAx3);
    // return true;
    //
    // BLOCKED on OCCT (Font_BRepFont, Font_TextFormatter, Font_BRepTextBuilder,
    // FT_Face advance metrics). Without the native kernel we cannot build text
    // geometry; mirror `aFont.FindAndInit(...)` failing -> return false.
    false
}

// TextShape.cpp:148
// static bool Prism(const TopoDS_Shape& theBase, const float thickness, TopoDS_Shape& theSolid)
fn prism(the_base: &TopoShape, _thickness: f32, the_solid: &mut TopoShape) -> bool {
    // TextShape.cpp:150  if (theBase.IsNull()) return false;
    if the_base.is_null {
        return false;
    }

    // TextShape.cpp:152-153
    // gp_Vec V(0.f, 0.f, thickness);
    // BRepPrimAPI_MakePrism* Prism = new BRepPrimAPI_MakePrism(theBase, V, Standard_False);
    //
    // TextShape.cpp:155-156
    // theSolid = Prism->Shape();
    // return true;
    //
    // BLOCKED on OCCT (BRepPrimAPI_MakePrism extrudes the planar text faces
    // into a solid). Unreachable here because `text_to_brep` returns false
    // before any non-null base can be produced.
    *the_solid = TopoShape { is_null: true };
    true
}

// TextShape.cpp:159  static void MakeMesh(TopoDS_Shape& theSolid, TriangleMesh& theMesh)
fn make_mesh(_the_solid: &mut TopoShape, _the_mesh: &mut TriangleMesh) {
    // TextShape.cpp:161-162
    // const double STEP_TRANS_CHORD_ERROR = 0.005;
    // const double STEP_TRANS_ANGLE_RES = 1;
    let _step_trans_chord_error: f64 = 0.005;
    let _step_trans_angle_res: f64 = 1.0;

    // TextShape.cpp:164  BRepMesh_IncrementalMesh mesh(theSolid, ..., false, ..., true);
    //
    // TextShape.cpp:165-174  count nodes/triangles over all TopAbs_FACE.
    // TextShape.cpp:176-180  build an stl_file with number_of_facets and allocate.
    // TextShape.cpp:182-237  copy nodes (transformed by face location) and
    //   triangles (reversing winding for TopAbs_REVERSED faces, offsetting node
    //   indices per face), compute & normalize the per-facet normal.
    // TextShape.cpp:239  theMesh.from_stl(stl);
    //
    // BLOCKED on OCCT (BRepMesh_IncrementalMesh tessellation,
    // BRep_Tool::Triangulation, Poly_Triangulation). The `theSolid` here is
    // always null (see `prism`), so there are no faces to mesh and the
    // resulting TriangleMesh stays empty -- matching what C++ produces when no
    // text geometry could be built.
}

// TextShape.cpp:242
// void load_text_shape(const char* text, const char* font, const float text_height,
//     const float thickness, bool is_bold, bool is_italic, TextResult& text_result)
pub fn load_text_shape(
    text: &str,
    font: &str,
    text_height: f32,
    thickness: f32,
    is_bold: bool,
    is_italic: bool,
    text_result: &mut TextResult,
) {
    // TextShape.cpp:244-245  if (thickness <= 0) return;
    if thickness <= 0.0 {
        return;
    }

    // TextShape.cpp:247-249
    // Handle(Font_FontMgr) aFontMgr = Font_FontMgr::GetInstance();
    // if (aFontMgr->GetAvailableFonts().IsEmpty())
    //     aFontMgr->InitFontDataBase();
    //
    // BLOCKED on OCCT (Font_FontMgr font-database init). No-op when OCCT is
    // unavailable.

    // TextShape.cpp:251-252
    // TopoDS_Shape aTextBase;
    // Font_FontAspect aFontAspect = Font_FontAspect_UNDEFINED;
    let mut a_text_base = TopoShape { is_null: true };
    let a_font_aspect: FontFontAspect;
    // TextShape.cpp:253-260
    // if (is_bold && is_italic)      aFontAspect = Font_FontAspect_BoldItalic;
    // else if (is_bold)             aFontAspect = Font_FontAspect_Bold;
    // else if (is_italic)           aFontAspect = Font_FontAspect_Italic;
    // else                          aFontAspect = Font_FontAspect_Regular;
    if is_bold && is_italic {
        a_font_aspect = FontFontAspect::BoldItalic;
    } else if is_bold {
        a_font_aspect = FontFontAspect::Bold;
    } else if is_italic {
        a_font_aspect = FontFontAspect::Italic;
    } else {
        a_font_aspect = FontFontAspect::Regular;
    }

    // TextShape.cpp:262-263
    // if (!TextToBRep(text, font, text_height, aFontAspect, aTextBase, text_result.text_width))
    //     return;
    if !text_to_brep(
        text,
        font,
        text_height,
        a_font_aspect,
        &mut a_text_base,
        &mut text_result.text_width,
    ) {
        return;
    }

    // TextShape.cpp:265-267
    // TopoDS_Shape aTextShape;
    // if (!Prism(aTextBase, thickness, aTextShape))
    //     return;
    let mut a_text_shape = TopoShape::default();
    if !prism(&a_text_base, thickness, &mut a_text_shape) {
        return;
    }

    // TextShape.cpp:269  MakeMesh(aTextShape, text_result.text_mesh);
    make_mesh(&mut a_text_shape, &mut text_result.text_mesh);
}

// OCCT Font_FontAspect enumeration (used by load_text_shape / text_to_brep).
// Mirrors the subset of Font_FontAspect values referenced by TextShape.cpp.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FontFontAspect {
    #[allow(dead_code)]
    Undefined,
    Regular,
    Bold,
    Italic,
    BoldItalic,
}

// TextShape.cpp:272  }; // namespace Slic3r
