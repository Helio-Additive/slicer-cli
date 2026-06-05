//! 1:1 port of `TextConfiguration.hpp` (BambuStudio libslic3r).
//!
//! This is a header-only C++ translation unit: it contains only the data
//! structures `FontProp`, `EmbossStyle`, `TextConfiguration` and the
//! `EmbossStyles` type alias. There is no corresponding `.cpp`.
//!
//! C++ source: src/libslic3r/TextConfiguration.hpp

// TextConfiguration.hpp:4   #include <vector>
// TextConfiguration.hpp:5   #include <string>
// TextConfiguration.hpp:6   #include <optional>
// TextConfiguration.hpp:7   #include <cereal/cereal.hpp>
// TextConfiguration.hpp:8   #include <cereal/types/optional.hpp>
// TextConfiguration.hpp:9   #include <cereal/types/string.hpp>
// TextConfiguration.hpp:10  #include <cereal/archives/binary.hpp>
// TextConfiguration.hpp:11  #include "Point.hpp" // Transform3d
use serde::{Deserialize, Serialize};

// libslic3r.h:287  template <typename Number>
// libslic3r.h:288  constexpr inline bool is_approx(Number value, Number test_value, Number precision = EPSILON)
// libslic3r.h:290  { return std::fabs(double(value) - double(test_value)) < double(precision); }
//
// `FontProp::operator==` relies on the scalar overload with the default
// precision `EPSILON = 1e-4` (libslic3r.h). Reproduced here verbatim.
use crate::libslic3r::EPSILON;

#[inline]
fn is_approx(value: f64, test_value: f64) -> bool {
    (value - test_value).abs() < EPSILON
}

// TextConfiguration.hpp:13  namespace Slic3r {

/// User modifiable property of text style
/// NOTE: OnEdit fix serializations: EmbossStylesSerializable, TextConfigurationSerialization
///
/// TextConfiguration.hpp:18  struct FontProp
#[derive(Debug, Clone)]
pub struct FontProp {
    // TextConfiguration.hpp:21  define extra space between letters, negative mean closer letter
    // TextConfiguration.hpp:22  When not set value is zero and is not stored
    /// [in font point]
    pub char_gap: Option<i32>, // [in font point]

    // TextConfiguration.hpp:25  define extra space between lines, negative mean closer lines
    // TextConfiguration.hpp:26  When not set value is zero and is not stored
    /// [in font point]
    pub line_gap: Option<i32>, // [in font point]

    // TextConfiguration.hpp:29  positive value mean wider character shape
    // TextConfiguration.hpp:30  negative value mean tiner character shape
    // TextConfiguration.hpp:31  When not set value is zero and is not stored
    /// [in mm]
    pub boldness: Option<f32>, // [in mm]

    // TextConfiguration.hpp:34  positive value mean italic of character (CW)
    // TextConfiguration.hpp:35  negative value mean CCW skew (unItalic)
    // TextConfiguration.hpp:36  When not set value is zero and is not stored
    /// [ration x:y]
    pub skew: Option<f32>, // [ration x:y]

    // TextConfiguration.hpp:39  Parameter for True Type Font collections
    // TextConfiguration.hpp:40  Select index of font in collection
    pub collection_number: Option<u32>,

    // TextConfiguration.hpp:42  Distiguish projection per glyph
    pub per_glyph: bool,

    // TextConfiguration.hpp:50  change pivot of text
    // TextConfiguration.hpp:51  When not set, center is used and is not stored
    pub align: Align,

    //////
    // TextConfiguration.hpp:54  Duplicit data to wxFontDescriptor
    // TextConfiguration.hpp:55  used for store/load .3mf file
    //////

    // TextConfiguration.hpp:58  Height of text line (letters)
    // TextConfiguration.hpp:59  duplicit to wxFont::PointSize
    /// [in mm]
    pub size_in_mm: f32, // [in mm]

    // TextConfiguration.hpp:62  Additional data about font to be able to find substitution,
    // TextConfiguration.hpp:63  when same font is not installed
    pub family: Option<String>,
    pub face_name: Option<String>,
    pub style: Option<String>,
    pub weight: Option<String>,
}

/// NOTE: way of serialize to 3mf force that zero must be default value
///
/// TextConfiguration.hpp:46  enum class HorizontalAlign { left = 0, center, right };
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HorizontalAlign {
    Left = 0,
    Center,
    Right,
}

/// TextConfiguration.hpp:47  enum class VerticalAlign { top = 0, center, bottom };
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum VerticalAlign {
    Top = 0,
    Center,
    Bottom,
}

/// TextConfiguration.hpp:48  using Align = std::pair<HorizontalAlign, VerticalAlign>;
pub type Align = (HorizontalAlign, VerticalAlign);

impl FontProp {
    /// Only constructor with restricted values
    ///
    /// `line_height`: Y size of text [in mm]
    /// `depth`: Z size of text [in mm]
    ///
    /// TextConfiguration.hpp:74  FontProp(float line_height = 10.f) : size_in_mm(line_height), per_glyph(false) {}
    pub fn new(line_height: f32) -> Self {
        Self {
            char_gap: None,
            line_gap: None,
            boldness: None,
            skew: None,
            collection_number: None,
            per_glyph: false,
            // TextConfiguration.hpp:51  Align align = Align(HorizontalAlign::center, VerticalAlign::center);
            align: (HorizontalAlign::Center, VerticalAlign::Center),
            size_in_mm: line_height,
            family: None,
            face_name: None,
            style: None,
            weight: None,
        }
    }
}

// TextConfiguration.hpp:74  FontProp(float line_height = 10.f)
impl Default for FontProp {
    fn default() -> Self {
        Self::new(10.0)
    }
}

// TextConfiguration.hpp:77  bool operator==(const FontProp& other) const
impl PartialEq for FontProp {
    fn eq(&self, other: &Self) -> bool {
        // TextConfiguration.hpp:78  auto case0 = is_approx(boldness.value_or(0), other.boldness.value_or(0));
        let case0 = is_approx(
            self.boldness.unwrap_or(0.0) as f64,
            other.boldness.unwrap_or(0.0) as f64,
        );
        // TextConfiguration.hpp:79  auto case1 = is_approx(skew.value_or(0), other.skew.value_or(0));
        let case1 = is_approx(
            self.skew.unwrap_or(0.0) as f64,
            other.skew.unwrap_or(0.0) as f64,
        );
        // TextConfiguration.hpp:80  auto case2 = line_gap.value_or(0) == other.line_gap.value_or(0);
        let case2 = self.line_gap.unwrap_or(0) == other.line_gap.unwrap_or(0);
        // TextConfiguration.hpp:81  auto case3 = char_gap.value_or(0) == other.char_gap.value_or(0);
        let case3 = self.char_gap.unwrap_or(0) == other.char_gap.unwrap_or(0);
        // TextConfiguration.hpp:82  return per_glyph == other.per_glyph &&
        // TextConfiguration.hpp:83        align == other.align && is_approx(size_in_mm, other.size_in_mm)
        // TextConfiguration.hpp:84        && case0 && case1 && case2  &&case3;
        self.per_glyph == other.per_glyph
            && self.align == other.align
            && is_approx(self.size_in_mm as f64, other.size_in_mm as f64)
            && case0
            && case1
            && case2
            && case3
    }
}

// undo / redo stack recovery
//
// TextConfiguration.hpp:88  template<class Archive> void save(Archive &ar) const
// TextConfiguration.hpp:97  template<class Archive> void load(Archive &ar)
//
// In C++ this is a hand-written cereal split serialize: it writes
//   ar(size_in_mm, per_glyph, align.first, align.second);
//   cereal::save(ar, char_gap); ... line_gap; boldness; skew; collection_number;
// (and the matching load). The family/face_name/style/weight optionals are
// intentionally NOT serialized by the undo/redo recovery path. We mirror that
// field ordering and selection with serde so the on-wire shape matches.
#[derive(Serialize, Deserialize)]
struct FontPropSerde {
    size_in_mm: f32,
    per_glyph: bool,
    align_first: HorizontalAlign,
    align_second: VerticalAlign,
    char_gap: Option<i32>,
    line_gap: Option<i32>,
    boldness: Option<f32>,
    skew: Option<f32>,
    collection_number: Option<u32>,
}

impl Serialize for FontProp {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        // TextConfiguration.hpp:90  ar(size_in_mm, per_glyph, align.first, align.second);
        // TextConfiguration.hpp:91-95  cereal::save(ar, char_gap/line_gap/boldness/skew/collection_number);
        let s = FontPropSerde {
            size_in_mm: self.size_in_mm,
            per_glyph: self.per_glyph,
            align_first: self.align.0,
            align_second: self.align.1,
            char_gap: self.char_gap,
            line_gap: self.line_gap,
            boldness: self.boldness,
            skew: self.skew,
            collection_number: self.collection_number,
        };
        s.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for FontProp {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        // TextConfiguration.hpp:99  ar(size_in_mm, per_glyph, align.first, align.second);
        // TextConfiguration.hpp:100-104  cereal::load(ar, char_gap/line_gap/boldness/skew/collection_number);
        let s = FontPropSerde::deserialize(deserializer)?;
        Ok(FontProp {
            char_gap: s.char_gap,
            line_gap: s.line_gap,
            boldness: s.boldness,
            skew: s.skew,
            collection_number: s.collection_number,
            per_glyph: s.per_glyph,
            align: (s.align_first, s.align_second),
            size_in_mm: s.size_in_mm,
            // not part of the undo/redo recovery archive; default to None
            family: None,
            face_name: None,
            style: None,
            weight: None,
        })
    }
}

/// Style of embossed text
/// (Path + Type) must define how to open font for using on different OS
/// NOTE: OnEdit fix serializations: EmbossStylesSerializable, TextConfigurationSerialization
///
/// TextConfiguration.hpp:113  struct EmbossStyle
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmbossStyle {
    // TextConfiguration.hpp:116  Human readable name of style it is shown in GUI
    pub name: String,

    // TextConfiguration.hpp:119  Define how to open font
    // TextConfiguration.hpp:120  Meaning depend on type
    pub path: String,

    // TextConfiguration.hpp:124  Define what is stored in path
    pub r#type: EmbossStyleType,

    // TextConfiguration.hpp:127  User modification of font style
    pub prop: FontProp,
}

// when name is empty than Font item was loaded from .3mf file
// and potentionaly it is not reproducable
// define data stored in path
// when wx change way of storing add new descriptor Type
//
// TextConfiguration.hpp:122  enum class Type;
// TextConfiguration.hpp:133  enum class Type {
//
// Named `EmbossStyleType` in Rust (Rust has no nested type scoping like C++'s
// `EmbossStyle::Type`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EmbossStyleType {
    // TextConfiguration.hpp:134  undefined = 0,
    Undefined = 0,

    // TextConfiguration.hpp:136  wx font descriptors are platform dependent
    // TextConfiguration.hpp:137  path is font descriptor generated by wxWidgets
    // TextConfiguration.hpp:138  wx_win_font_descr, // on Windows
    WxWinFontDescr, // on Windows
    // TextConfiguration.hpp:139  wx_lin_font_descr, // on Linux
    WxLinFontDescr, // on Linux
    // TextConfiguration.hpp:140  wx_mac_font_descr, // on Max OS
    WxMacFontDescr, // on Max OS

    // TextConfiguration.hpp:142  TrueTypeFont file loacation on computer
    // TextConfiguration.hpp:143  for privacy: only filename is stored into .3mf
    // TextConfiguration.hpp:144  file_path
    FilePath,
}

impl Default for EmbossStyleType {
    // TextConfiguration.hpp:124  Type type { Type::undefined };
    fn default() -> Self {
        EmbossStyleType::Undefined
    }
}

// TextConfiguration.hpp:147  bool operator==(const EmbossStyle &other) const
impl PartialEq for EmbossStyle {
    fn eq(&self, other: &Self) -> bool {
        // TextConfiguration.hpp:149  auto case0 = prop == other.prop;
        let case0 = self.prop == other.prop;
        // TextConfiguration.hpp:150  return type == other.type && case0 && name == other.name;
        self.r#type == other.r#type && case0 && self.name == other.name
    }
}

// TextConfiguration.hpp:154  template<class Archive> void serialize(Archive &ar){ ar(name, path, type, prop); }
// (handled by #[derive(Serialize, Deserialize)] over the field order
//  name, path, type, prop)

// TextConfiguration.hpp:157  Emboss style name inside vector is unique
// TextConfiguration.hpp:158  It is not map beacuse items has own order (view inside of slect)
// TextConfiguration.hpp:159  It is stored into AppConfig by EmbossStylesSerializable
// TextConfiguration.hpp:160  using EmbossStyles = std::vector<EmbossStyle>;
pub type EmbossStyles = Vec<EmbossStyle>;

/// Define how to create 'Text volume'
/// It is stored into .3mf by TextConfigurationSerialization
/// It is part of ModelVolume optional data
///
/// TextConfiguration.hpp:167  struct TextConfiguration
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TextConfiguration {
    // TextConfiguration.hpp:170  Style of embossed text
    pub style: EmbossStyle,

    // TextConfiguration.hpp:173  Embossed text value
    pub text: String,
}

impl Default for TextConfiguration {
    // TextConfiguration.hpp:173  std::string text = "None";
    fn default() -> Self {
        Self {
            style: EmbossStyle {
                name: String::new(),
                path: String::new(),
                r#type: EmbossStyleType::default(),
                prop: FontProp::default(),
            },
            text: "None".to_string(),
        }
    }
}

// TextConfiguration.hpp:176  template<class Archive> void serialize(Archive &ar) { ar(style, text); }
// (handled by #[derive(Serialize, Deserialize)] over field order style, text)

// TextConfiguration.hpp:179  } // namespace Slic3r
