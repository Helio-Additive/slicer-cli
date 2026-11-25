//! Port of `libslic3r/Desensitize.hpp` and `Desensitize.cpp`.
//!
//! Masks sensitive values before they are written to logs. Masking happens only in builds
//! released to the public; this crate is one (`BBL_RELEASE_TO_PUBLIC` is 1 in the build of
//! `slicer_cli`), so `S_ENABLE_CROSS_TALK` is true.
//!
//! The C++ works on `std::string`, a string of bytes: a mask that keeps "the last 3 characters"
//! keeps three bytes and can cut a UTF-8 sequence in two. The functions ending in `_bytes` are
//! the ports and return those bytes. The functions without the suffix take and return `str`,
//! for the callers that hold Rust strings: they are the `_bytes` result with every cut sequence
//! replaced by U+FFFD, which is also what `json_log` writes for such a value (nlohmann's
//! `error_handler_t::replace`).
//!
//! `json_log` prints as `nlohmann::json::dump` does; the part of nlohmann's serializer that takes
//! is at the end of this file.
//!
//! C++ reference: `references/BambuStudio/src/libslic3r/Desensitize.{hpp,cpp}`,
//! `references/BambuStudio/src/nlohmann/detail/output/serializer.hpp`,
//! `references/BambuStudio/src/nlohmann/detail/conversions/to_chars.hpp`.

use serde_json::Value;

use crate::log_keys::log_key;

/// Desensitize.cpp:8-12 — `BBL_RELEASE_TO_PUBLIC` is 1 in the build this crate follows.
const S_ENABLE_CROSS_TALK: bool = true;

/// `std::string::npos`
const NPOS: usize = usize::MAX;

/// `std::string::find(needle, pos)` on bytes.
fn find(haystack: &[u8], needle: &[u8], pos: usize) -> usize {
    if pos > haystack.len() {
        return NPOS;
    }
    if needle.is_empty() {
        return pos;
    }
    if needle.len() > haystack.len() {
        return NPOS;
    }
    (pos..=haystack.len() - needle.len()).find(|i| &haystack[*i..*i + needle.len()] == needle).unwrap_or(NPOS)
}

/// The `str` form of a masked value: a UTF-8 sequence the mask cut becomes U+FFFD.
fn lossy(bytes: Vec<u8>) -> String {
    match String::from_utf8(bytes) {
        Ok(text) => text,
        Err(e) => String::from_utf8_lossy(e.as_bytes()).into_owned(),
    }
}

/// Desensitize.cpp:17-28
///
/// Mask a device id / serial number, keeping the first and last 3 characters.
pub fn dev_id_bytes(str: &[u8]) -> Vec<u8> {
    if !S_ENABLE_CROSS_TALK {
        return str.to_vec();
    }
    if str.len() > 6 {
        let mut cs_devid = str[..3].to_vec();
        cs_devid.resize(3 + str.len() - 6, b'*');
        cs_devid.extend_from_slice(&str[str.len() - 3..]);
        return cs_devid;
    }

    str.to_vec()
}

/// Desensitize.cpp:31-49
///
/// Mask the first two segments of an IPv4 address.
pub fn dev_ip_bytes(str: &[u8]) -> Vec<u8> {
    if !S_ENABLE_CROSS_TALK {
        return str.to_vec();
    }

    let mut format_ip = str.to_vec();
    let mut pos_st: usize = 0;

    for _ in 0..2 {
        // `pos_st + 1` wraps to 0 only for npos, which pos_st never is.
        let pos_en = find(&format_ip, b".", pos_st + 1);
        if pos_en == NPOS {
            return str.to_vec();
        }
        // `format_ip.replace(pos_st, pos_en - pos_st, "***")`. pos_en was found in the string as it
        // was before this replacement, and the next search starts from it all the same.
        format_ip.splice(pos_st..pos_en, *b"***");
        pos_st = pos_en + 1;
    }

    format_ip
}

/// Desensitize.cpp:52-76
///
/// Mask a device name, keeping at most the last 3 characters.
pub fn dev_name_bytes(str: &[u8]) -> Vec<u8> {
    if !S_ENABLE_CROSS_TALK {
        return str.to_vec();
    }

    let get_name = |name: &[u8], count: usize| -> Vec<u8> {
        let mut cs_dev_name = vec![b'*'; name.len() - count];
        cs_dev_name.extend_from_slice(&name[name.len() - count..]);
        cs_dev_name
    };

    if str.len() > 3 {
        return get_name(str, 3);
    } else if str.len() > 2 {
        return get_name(str, 2);
    } else if str.len() > 1 {
        return get_name(str, 1);
    }

    vec![b'*'; str.len()]
}

/// Desensitize.cpp:79-92
///
/// Mask the device id part of "<dev_id>_<suffix>"; returns "****" when there is no suffix.
fn channel_name(str: &[u8]) -> Vec<u8> {
    if !S_ENABLE_CROSS_TALK {
        return str.to_vec();
    }

    // `int pos = str.find("_")`, compared with npos and with `str.size() - 1` as a size_t again.
    let pos = find(str, b"_", 0);
    if pos != NPOS && pos < str.len().wrapping_sub(1) {
        let mut cs_channel_name = dev_id_bytes(&str[..pos]);
        cs_channel_name.extend_from_slice(&str[pos..]);
        return cs_channel_name;
    }

    b"****".to_vec()
}

// Desensitize.cpp:95-103
// Field names in device and cloud json that have no log_key of their own.
const K_IP: &str = "ip";
const K_USER_ID: &str = "user_id";
const K_PASSWD: &str = "passwd";
const K_CHANNEL_NAME: &str = "channel_name";
const K_MODEL_ID_CAMEL: &str = "modelId";

// Written instead of a json that could not be masked, so that no partly masked dump reaches the log.
const K_MASKING_FAILED: &str = "<json not logged: masking failed>";

/// Desensitize.cpp:105-109
///
/// Dumps without throwing: invalid UTF-8 in strings is replaced rather than raising type_error.
/// (A `serde_json::Value` holds no invalid UTF-8, so nothing is left to replace here.)
fn dump_for_log(json: &Value, indent: i32) -> String {
    let mut out = String::new();
    if indent >= 0 {
        nlohmann_dump(json, true, indent as usize, 0, &mut out);
    } else {
        nlohmann_dump(json, false, 0, 0, &mut out);
    }
    out
}

/// What the body of the `try` block of `json_log` does to one element of the traversal stack
/// (Desensitize.cpp:120-216). `Err` is the `nlohmann::json::exception` that `iter.key()` throws
/// for an element that is not an object.
fn mask_element<'a>(json: &'a mut Value, to_traverse_jsons: &mut Vec<&'a mut Value>, indent: i32) -> Result<(), ()> {
    let object = match json {
        Value::Object(object) => object,
        // `begin() == end()`: the loop body does not run.
        Value::Null => return Ok(()),
        Value::Array(array) if array.is_empty() => return Ok(()),
        // An array with elements, or a boolean, a number or a string: there is a first element,
        // and `iter.key()` throws invalid_iterator for it.
        _ => return Err(()),
    };

    for (key_str, value) in object.iter_mut() {
        if let Value::String(text) = value {
            let text: &[u8] = text.as_bytes();
            // Registered fields first, by exact name; the rules below catch other spellings by substring.
            let masked: Option<Vec<u8>> = if let Some(by_key) = mask_by_key_bytes(key_str, text) {
                Some(by_key)
            } else if key_str.contains(log_key::DEV_ID) || key_str.contains(log_key::SN) {
                Some(dev_id_bytes(text))
            } else if key_str.contains(log_key::DEV_NAME) {
                Some(dev_name_bytes(text))
            }
            // "ip" and "user_id" are matched whole: as substrings they would also hit ipcam/description and tag_uid/tray_uuid
            else if key_str.contains(log_key::DEV_IP) || key_str == K_IP {
                Some(dev_ip_bytes(text))
            } else if key_str == K_USER_ID {
                Some(user_id_bytes(text))
            } else if key_str.contains(log_key::PASSWORD) || key_str.contains(K_PASSWD) {
                Some(b"******".to_vec())
            } else if key_str.contains(log_key::ACCESS_CODE) {
                Some(b"******".to_vec())
            } else if key_str.contains(log_key::URL) {
                Some(b"******".to_vec())
            } else if key_str.contains(K_CHANNEL_NAME) {
                Some(channel_name(text))
            } else if key_str.contains(log_key::TTCODE_ENC) {
                Some(b"******".to_vec())
            } else if key_str.contains(log_key::AUTHKEY) {
                Some(b"******".to_vec())
            } else if key_str == log_key::UID {
                Some(user_id_bytes(text))
            } else if key_str.contains(log_key::REGION) {
                Some(b"******".to_vec())
            } else if key_str.contains(log_key::TOKEN) {
                Some(b"******".to_vec())
            } else if key_str.contains(K_MODEL_ID_CAMEL) || key_str.contains(log_key::MODEL_ID) {
                Some(b"******".to_vec())
            } else {
                None
            };
            if let Some(masked) = masked {
                // The C++ stores the bytes and the dump replaces what is not UTF-8.
                *value = Value::String(lossy(masked));
            }
        } else if value.is_number() {
            if key_str == log_key::UID || key_str == K_USER_ID {
                // `iter.value().dump()`
                let mut dumped = String::new();
                nlohmann_dump(value, false, 0, 0, &mut dumped);
                *value = Value::String(lossy(user_id_bytes(dumped.as_bytes())));
            } else if key_str == K_IP {
                // the printer reports its IP as a number; there is no dotted form to keep part of
                *value = Value::String("******".to_string());
            }
        } else if value.is_object() {
            to_traverse_jsons.push(value);
        } else if let Value::Array(array) = value {
            for array_item in array.iter_mut() {
                if array_item.is_object() || array_item.is_array() {
                    to_traverse_jsons.push(array_item);
                } else if let Value::String(string_val) = array_item {
                    // `try { ... } catch (...) { continue; }`: a string that does not parse stays.
                    if !string_val.is_empty() {
                        if let Some(sub_json) = nlohmann_parse(string_val) {
                            *array_item = Value::String(json_log(&sub_json, indent));
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

/// Desensitize.cpp:112-236
///
/// Dump a json with sensitive fields masked, recursing into nested objects and arrays.
///
/// Never fails: a json that cannot be processed gives a placeholder instead of a partly masked
/// dump. `indent` is as in nlohmann::json::dump; -1 gives a single line (the C++ default argument
/// is 1).
///
/// FIDELITY-NOTE: the json is a `serde_json::Value` where the C++ takes a `nlohmann::json`. What a
/// `Value` cannot hold is not reproduced: a string with invalid UTF-8 (the C++ replaces it on
/// dump), a float that is not finite (the C++ dumps `null`), binary values.
pub fn json_log(json: &Value, indent: i32) -> String {
    if !S_ENABLE_CROSS_TALK {
        return dump_for_log(json, indent); // Return the original JSON string if cross-talk is disabled
    }

    let mut copied_json: Value = json.clone();

    {
        let mut to_traverse_jsons: Vec<&mut Value> = vec![&mut copied_json];
        while let Some(json) = to_traverse_jsons.pop() {
            if mask_element(json, &mut to_traverse_jsons, indent).is_err() {
                // Handle JSON parsing exceptions if necessary
                crate::boost_log_trivial!(Info, "Error processing JSON for cross-talk: cannot use key() for non-object iterators");
                return K_MASKING_FAILED.to_string();
            }
        }
    }

    dump_for_log(&copied_json, indent)
}

/// Desensitize.cpp:239-249
///
/// Mask a user id, keeping the last 2 characters.
pub fn user_id_bytes(uid: &[u8]) -> Vec<u8> {
    if !S_ENABLE_CROSS_TALK {
        return uid.to_vec();
    }
    if uid.len() > 2 {
        let mut cs_uid = vec![b'*'; uid.len() - 2];
        cs_uid.extend_from_slice(&uid[uid.len() - 2..]);
        return cs_uid;
    }

    b"******".to_vec()
}

/// Desensitize.cpp:252-256
///
/// Mask the whole value, for credentials such as tokens and access codes.
pub fn mask_all_bytes(str: &[u8]) -> Vec<u8> {
    if !S_ENABLE_CROSS_TALK {
        return str.to_vec();
    }
    b"******".to_vec()
}

/// Desensitize.cpp:259 — `using MaskFn = std::string (*)(const std::string&);`
type MaskFn = fn(&[u8]) -> Vec<u8>;

/// Desensitize.cpp:261-277
///
/// Registered sensitive fields: field name -> mask function. Used for KV keys and, by exact name,
/// for json fields.
const K_MASKS: [(&str, MaskFn); 13] = [
    (log_key::DEV_ID, dev_id_bytes),
    (log_key::SN, dev_id_bytes),
    (log_key::DEV_IP, dev_ip_bytes),
    (log_key::DEV_NAME, dev_name_bytes),
    (log_key::UID, user_id_bytes),
    (log_key::ACCESS_CODE, mask_all_bytes),
    (log_key::TOKEN, mask_all_bytes),
    (log_key::PASSWORD, mask_all_bytes),
    (log_key::AUTHKEY, mask_all_bytes),
    (log_key::TTCODE_ENC, mask_all_bytes),
    (log_key::URL, mask_all_bytes),
    (log_key::REGION, mask_all_bytes),
    (log_key::MODEL_ID, mask_all_bytes),
];

/// Desensitize.cpp:280-286
///
/// Mask a value by its field name, using the table of registered sensitive fields. The name must
/// match a registered field exactly, such as a log_key constant. Returns the masked value, or
/// `None` (nullopt) if the field is not registered.
pub fn mask_by_key_bytes(key: &str, value: &[u8]) -> Option<Vec<u8>> {
    for (name, mask) in &K_MASKS {
        if *name == key {
            return Some(mask(value));
        }
    }
    None
}

/// `Desensitize::dev_id` for a Rust string; see the module comment.
pub fn dev_id(str: &str) -> String {
    lossy(dev_id_bytes(str.as_bytes()))
}

/// `Desensitize::dev_ip` for a Rust string; see the module comment.
pub fn dev_ip(str: &str) -> String {
    lossy(dev_ip_bytes(str.as_bytes()))
}

/// `Desensitize::dev_name` for a Rust string; see the module comment.
pub fn dev_name(str: &str) -> String {
    lossy(dev_name_bytes(str.as_bytes()))
}

/// `Desensitize::user_id` for a Rust string; see the module comment.
pub fn user_id(uid: &str) -> String {
    lossy(user_id_bytes(uid.as_bytes()))
}

/// `Desensitize::mask_all` for a Rust string.
pub fn mask_all(str: &str) -> String {
    lossy(mask_all_bytes(str.as_bytes()))
}

/// `Desensitize::mask_by_key` for a Rust string; see the module comment.
pub fn mask_by_key(key: &str, value: &str) -> Option<String> {
    mask_by_key_bytes(key, value.as_bytes()).map(lossy)
}

// ---------------------------------------------------------------------------------------------
// nlohmann::json, as far as json_log uses it: `json::parse(std::string)` and `json::dump`.
// ---------------------------------------------------------------------------------------------

/// `nlohmann::json::parse(string_val)`; `None` where it throws.
///
/// Read by the port of nlohmann's parser (config.rs, gate NLOHMANN_PARSE_EXACT). With the gate at
/// 0, FIDELITY-NOTE: read by `serde_json`. Known differences from nlohmann's parser: serde_json
/// refuses a document nested deeper than 128 levels, and without its `float_roundtrip` feature
/// (not enabled in this workspace) a number with more than 15 significant digits, or with a
/// decimal exponent beyond 22, can come out one unit in the last place away from the correctly
/// rounded double nlohmann's `strtod` gives.
pub fn nlohmann_parse(text: &str) -> Option<Value> {
    // R954 NLOHMANN_PARSE_EXACT (faithful): the port of nlohmann 3.10.4's lexer and parser in
    // config.rs, strict as `json::parse(std::string)` is (parser.hpp:85-99), whose floats are the
    // correctly rounded `strtod` of the pin (lexer.hpp scan_number); serde_json below reads a
    // 17-digit double one ulp off.
    if crate::faithful_gate("NLOHMANN_PARSE_EXACT") {
        return crate::config::nlohmann_json_parse_strict(text.as_bytes()).ok();
    }
    // nlohmann skips a byte order mark, and reads the token `-0` as the integer 0.
    let bytes = text.as_bytes();
    let bytes = bytes.strip_prefix(&[0xEF, 0xBB, 0xBF]).unwrap_or(bytes);
    let bytes = crate::support_recommended_params::json_minus_zero_as_integer(bytes);
    serde_json::from_slice::<Value>(&bytes).ok()
}

/// serializer.hpp:94-369 — `serializer::dump(val, pretty_print, ensure_ascii = false,
/// indent_step, current_indent)`.
pub(crate) fn nlohmann_dump(val: &Value, pretty_print: bool, indent_step: usize, current_indent: usize, o: &mut String) {
    match val {
        Value::Object(object) => {
            if object.is_empty() {
                o.push_str("{}");
                return;
            }
            // nlohmann's object is a std::map: the keys in byte order.
            let mut members: Vec<(&String, &Value)> = object.iter().collect();
            members.sort_by(|a, b| a.0.as_bytes().cmp(b.0.as_bytes()));

            if pretty_print {
                o.push_str("{\n");

                // variable to hold indentation for recursive calls
                let new_indent = current_indent + indent_step;

                for (i, (key, value)) in members.iter().enumerate() {
                    if i > 0 {
                        o.push_str(",\n");
                    }
                    push_indent(o, new_indent);
                    o.push('"');
                    dump_escaped(key, o);
                    o.push_str("\": ");
                    nlohmann_dump(value, true, indent_step, new_indent, o);
                }

                o.push('\n');
                push_indent(o, current_indent);
                o.push('}');
            } else {
                o.push('{');

                for (i, (key, value)) in members.iter().enumerate() {
                    if i > 0 {
                        o.push(',');
                    }
                    o.push('"');
                    dump_escaped(key, o);
                    o.push_str("\":");
                    nlohmann_dump(value, false, indent_step, current_indent, o);
                }

                o.push('}');
            }
        }
        Value::Array(array) => {
            if array.is_empty() {
                o.push_str("[]");
                return;
            }

            if pretty_print {
                o.push_str("[\n");

                // variable to hold indentation for recursive calls
                let new_indent = current_indent + indent_step;

                for (i, value) in array.iter().enumerate() {
                    if i > 0 {
                        o.push_str(",\n");
                    }
                    push_indent(o, new_indent);
                    nlohmann_dump(value, true, indent_step, new_indent, o);
                }

                o.push('\n');
                push_indent(o, current_indent);
                o.push(']');
            } else {
                o.push('[');

                for (i, value) in array.iter().enumerate() {
                    if i > 0 {
                        o.push(',');
                    }
                    nlohmann_dump(value, false, indent_step, current_indent, o);
                }

                o.push(']');
            }
        }
        Value::String(text) => {
            o.push('"');
            dump_escaped(text, o);
            o.push('"');
        }
        Value::Bool(b) => o.push_str(if *b { "true" } else { "false" }),
        Value::Number(number) => {
            // dump_integer for number_integer / number_unsigned, dump_float for number_float.
            if let Some(v) = number.as_u64() {
                o.push_str(&v.to_string());
            } else if let Some(v) = number.as_i64() {
                o.push_str(&v.to_string());
            } else {
                dump_float(number.as_f64().unwrap_or(0.0), o);
            }
        }
        Value::Null => o.push_str("null"),
    }
}

/// `indent_string.c_str(), new_indent`: that many times the indent character, a space.
fn push_indent(o: &mut String, count: usize) {
    for _ in 0..count {
        o.push(' ');
    }
}

/// serializer.hpp:372-628 — `dump_escaped(s, ensure_ascii = false)` for a valid UTF-8 string:
/// the short escapes, `\u00xx` for the other control characters, everything else as it is.
fn dump_escaped(s: &str, o: &mut String) {
    for c in s.chars() {
        match c {
            '\u{08}' => o.push_str("\\b"),
            '\u{09}' => o.push_str("\\t"),
            '\u{0A}' => o.push_str("\\n"),
            '\u{0C}' => o.push_str("\\f"),
            '\u{0D}' => o.push_str("\\r"),
            '\u{22}' => o.push_str("\\\""),
            '\u{5C}' => o.push_str("\\\\"),
            c if (c as u32) <= 0x1F => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
}

/// serializer.hpp:768-795 with to_chars.hpp:1066-1100 — `dump_float(x)` for a double: `to_chars`.
fn dump_float(mut value: f64, o: &mut String) {
    // NaN / inf
    if !value.is_finite() {
        o.push_str("null");
        return;
    }

    // Use signbit(value) instead of (value < 0) since signbit works for -0.
    if value.is_sign_negative() {
        value = -value;
        o.push('-');
    }

    if value == 0.0 {
        // +-0
        o.push_str("0.0");
        return;
    }

    // Compute v = buffer * 10^decimal_exponent.
    // The decimal digits are stored in the buffer, which needs to be interpreted
    // as an unsigned decimal integer.
    // len is the length of the buffer, i.e. the number of decimal digits.
    let mut buf: Vec<u8> = Vec::with_capacity(24);
    let mut decimal_exponent: i32 = 0;
    grisu2(&mut buf, &mut decimal_exponent, value);

    // Format the buffer like printf("%.*g", prec, value)
    const K_MIN_EXP: i32 = -4;
    // Use digits10 here to increase compatibility with version 2.
    const K_MAX_EXP: i32 = 15;

    format_buffer(&buf, decimal_exponent, K_MIN_EXP, K_MAX_EXP, o);
}

/// to_chars.hpp:47-125 — `struct diyfp`: f * 2^e
#[derive(Clone, Copy)]
struct Diyfp {
    f: u64,
    e: i32,
}

impl Diyfp {
    /// to_chars.hpp:60-66 — returns x - y; x.e == y.e and x.f >= y.f
    fn sub(x: Diyfp, y: Diyfp) -> Diyfp {
        Diyfp { f: x.f.wrapping_sub(y.f), e: x.e }
    }

    /// to_chars.hpp:72-135 — returns x * y; only the upper 64 bits, rounded, ties up
    fn mul(x: Diyfp, y: Diyfp) -> Diyfp {
        let u_lo = x.f & 0xFFFF_FFFF;
        let u_hi = x.f >> 32;
        let v_lo = y.f & 0xFFFF_FFFF;
        let v_hi = y.f >> 32;

        let p0 = u_lo * v_lo;
        let p1 = u_lo * v_hi;
        let p2 = u_hi * v_lo;
        let p3 = u_hi * v_hi;

        let p0_hi = p0 >> 32;
        let p1_lo = p1 & 0xFFFF_FFFF;
        let p1_hi = p1 >> 32;
        let p2_lo = p2 & 0xFFFF_FFFF;
        let p2_hi = p2 >> 32;

        let mut q = p0_hi + p1_lo + p2_lo;

        // The full product might now be computed as
        //
        // p_hi = p3 + p2_hi + p1_hi + (Q >> 32)
        // p_lo = p0_lo + (Q << 32)
        //
        // But in this particular case here, the full p_lo is not required.
        // Effectively we only need to add the highest bit in p_lo to p_hi (and
        // Q_hi + 1 does not overflow).

        q += 1u64 << (64 - 32 - 1); // round, ties up

        let h = p3 + p2_hi + p1_hi + (q >> 32);

        Diyfp { f: h, e: x.e + y.e + 64 }
    }

    /// to_chars.hpp:141-153 — normalize x such that the significand is >= 2^(q-1); x.f != 0
    fn normalize(mut x: Diyfp) -> Diyfp {
        while (x.f >> 63) == 0 {
            x.f <<= 1;
            x.e -= 1;
        }
        x
    }

    /// to_chars.hpp:159-167 — normalize x such that the result has the exponent E
    fn normalize_to(x: Diyfp, target_exponent: i32) -> Diyfp {
        let delta = x.e - target_exponent;
        Diyfp { f: x.f << delta, e: target_exponent }
    }
}

/// to_chars.hpp:170-175
struct Boundaries {
    w: Diyfp,
    minus: Diyfp,
    plus: Diyfp,
}

/// to_chars.hpp:183-248 — the normalized diyfp of a finite, positive double and its boundaries.
fn compute_boundaries(value: f64) -> Boundaries {
    // Convert the IEEE representation into a diyfp.
    //
    // If v is denormal:
    //      value = 0.F * 2^(1 - bias) = (          F) * 2^(1 - bias - (p-1))
    // If v is normalized:
    //      value = 1.F * 2^(E - bias) = (2^(p-1) + F) * 2^(E - bias - (p-1))
    const K_PRECISION: i32 = 53; // = p (includes the hidden bit)
    const K_BIAS: i32 = 1024 - 1 + (K_PRECISION - 1);
    const K_MIN_EXP: i32 = 1 - K_BIAS;
    const K_HIDDEN_BIT: u64 = 1u64 << (K_PRECISION - 1); // = 2^(p-1)

    let bits: u64 = value.to_bits();
    let e: u64 = bits >> (K_PRECISION - 1);
    let f: u64 = bits & (K_HIDDEN_BIT - 1);

    let is_denormal = e == 0;
    let v = if is_denormal { Diyfp { f, e: K_MIN_EXP } } else { Diyfp { f: f + K_HIDDEN_BIT, e: e as i32 - K_BIAS } };

    // Compute the boundaries m- and m+ of the floating-point value
    // v = f * 2^e.
    //
    // Determine v- and v+, the floating-point predecessor and successor if v,
    // respectively.
    //
    //      v- = v - 2^e        if f != 2^(p-1) or e == e_min                (A)
    //         = v - 2^(e-1)    if f == 2^(p-1) and e > e_min                (B)
    //
    //      v+ = v + 2^e
    //
    // Let m- = (v- + v) / 2 and m+ = (v + v+) / 2. All real numbers _strictly_
    // between m- and m+ round to v, regardless of how the input rounding
    // algorithm breaks ties.
    let lower_boundary_is_closer = f == 0 && e > 1;
    let m_plus = Diyfp { f: 2 * v.f + 1, e: v.e - 1 };
    let m_minus = if lower_boundary_is_closer {
        Diyfp { f: 4 * v.f - 1, e: v.e - 2 } // (B)
    } else {
        Diyfp { f: 2 * v.f - 1, e: v.e - 1 } // (A)
    };

    // Determine the normalized w+ = m+.
    let w_plus = Diyfp::normalize(m_plus);

    // Determine w- = m- such that e_(w-) = e_(w+).
    let w_minus = Diyfp::normalize_to(m_minus, w_plus.e);

    Boundaries { w: Diyfp::normalize(v), minus: w_minus, plus: w_plus }
}

// to_chars.hpp:306-307
const K_ALPHA: i32 = -60;
#[allow(dead_code)]
const K_GAMMA: i32 = -32;

/// to_chars.hpp:309-314 — c = f * 2^e ~= 10^k
#[derive(Clone, Copy)]
struct CachedPower {
    f: u64,
    e: i32,
    k: i32,
}

/// to_chars.hpp:375-458 — `kCachedPowers`
const K_CACHED_POWERS: [CachedPower; 79] = [
    CachedPower { f: 0xAB70FE17C79AC6CA, e: -1060, k: -300 },
    CachedPower { f: 0xFF77B1FCBEBCDC4F, e: -1034, k: -292 },
    CachedPower { f: 0xBE5691EF416BD60C, e: -1007, k: -284 },
    CachedPower { f: 0x8DD01FAD907FFC3C, e: -980, k: -276 },
    CachedPower { f: 0xD3515C2831559A83, e: -954, k: -268 },
    CachedPower { f: 0x9D71AC8FADA6C9B5, e: -927, k: -260 },
    CachedPower { f: 0xEA9C227723EE8BCB, e: -901, k: -252 },
    CachedPower { f: 0xAECC49914078536D, e: -874, k: -244 },
    CachedPower { f: 0x823C12795DB6CE57, e: -847, k: -236 },
    CachedPower { f: 0xC21094364DFB5637, e: -821, k: -228 },
    CachedPower { f: 0x9096EA6F3848984F, e: -794, k: -220 },
    CachedPower { f: 0xD77485CB25823AC7, e: -768, k: -212 },
    CachedPower { f: 0xA086CFCD97BF97F4, e: -741, k: -204 },
    CachedPower { f: 0xEF340A98172AACE5, e: -715, k: -196 },
    CachedPower { f: 0xB23867FB2A35B28E, e: -688, k: -188 },
    CachedPower { f: 0x84C8D4DFD2C63F3B, e: -661, k: -180 },
    CachedPower { f: 0xC5DD44271AD3CDBA, e: -635, k: -172 },
    CachedPower { f: 0x936B9FCEBB25C996, e: -608, k: -164 },
    CachedPower { f: 0xDBAC6C247D62A584, e: -582, k: -156 },
    CachedPower { f: 0xA3AB66580D5FDAF6, e: -555, k: -148 },
    CachedPower { f: 0xF3E2F893DEC3F126, e: -529, k: -140 },
    CachedPower { f: 0xB5B5ADA8AAFF80B8, e: -502, k: -132 },
    CachedPower { f: 0x87625F056C7C4A8B, e: -475, k: -124 },
    CachedPower { f: 0xC9BCFF6034C13053, e: -449, k: -116 },
    CachedPower { f: 0x964E858C91BA2655, e: -422, k: -108 },
    CachedPower { f: 0xDFF9772470297EBD, e: -396, k: -100 },
    CachedPower { f: 0xA6DFBD9FB8E5B88F, e: -369, k: -92 },
    CachedPower { f: 0xF8A95FCF88747D94, e: -343, k: -84 },
    CachedPower { f: 0xB94470938FA89BCF, e: -316, k: -76 },
    CachedPower { f: 0x8A08F0F8BF0F156B, e: -289, k: -68 },
    CachedPower { f: 0xCDB02555653131B6, e: -263, k: -60 },
    CachedPower { f: 0x993FE2C6D07B7FAC, e: -236, k: -52 },
    CachedPower { f: 0xE45C10C42A2B3B06, e: -210, k: -44 },
    CachedPower { f: 0xAA242499697392D3, e: -183, k: -36 },
    CachedPower { f: 0xFD87B5F28300CA0E, e: -157, k: -28 },
    CachedPower { f: 0xBCE5086492111AEB, e: -130, k: -20 },
    CachedPower { f: 0x8CBCCC096F5088CC, e: -103, k: -12 },
    CachedPower { f: 0xD1B71758E219652C, e: -77, k: -4 },
    CachedPower { f: 0x9C40000000000000, e: -50, k: 4 },
    CachedPower { f: 0xE8D4A51000000000, e: -24, k: 12 },
    CachedPower { f: 0xAD78EBC5AC620000, e: 3, k: 20 },
    CachedPower { f: 0x813F3978F8940984, e: 30, k: 28 },
    CachedPower { f: 0xC097CE7BC90715B3, e: 56, k: 36 },
    CachedPower { f: 0x8F7E32CE7BEA5C70, e: 83, k: 44 },
    CachedPower { f: 0xD5D238A4ABE98068, e: 109, k: 52 },
    CachedPower { f: 0x9F4F2726179A2245, e: 136, k: 60 },
    CachedPower { f: 0xED63A231D4C4FB27, e: 162, k: 68 },
    CachedPower { f: 0xB0DE65388CC8ADA8, e: 189, k: 76 },
    CachedPower { f: 0x83C7088E1AAB65DB, e: 216, k: 84 },
    CachedPower { f: 0xC45D1DF942711D9A, e: 242, k: 92 },
    CachedPower { f: 0x924D692CA61BE758, e: 269, k: 100 },
    CachedPower { f: 0xDA01EE641A708DEA, e: 295, k: 108 },
    CachedPower { f: 0xA26DA3999AEF774A, e: 322, k: 116 },
    CachedPower { f: 0xF209787BB47D6B85, e: 348, k: 124 },
    CachedPower { f: 0xB454E4A179DD1877, e: 375, k: 132 },
    CachedPower { f: 0x865B86925B9BC5C2, e: 402, k: 140 },
    CachedPower { f: 0xC83553C5C8965D3D, e: 428, k: 148 },
    CachedPower { f: 0x952AB45CFA97A0B3, e: 455, k: 156 },
    CachedPower { f: 0xDE469FBD99A05FE3, e: 481, k: 164 },
    CachedPower { f: 0xA59BC234DB398C25, e: 508, k: 172 },
    CachedPower { f: 0xF6C69A72A3989F5C, e: 534, k: 180 },
    CachedPower { f: 0xB7DCBF5354E9BECE, e: 561, k: 188 },
    CachedPower { f: 0x88FCF317F22241E2, e: 588, k: 196 },
    CachedPower { f: 0xCC20CE9BD35C78A5, e: 614, k: 204 },
    CachedPower { f: 0x98165AF37B2153DF, e: 641, k: 212 },
    CachedPower { f: 0xE2A0B5DC971F303A, e: 667, k: 220 },
    CachedPower { f: 0xA8D9D1535CE3B396, e: 694, k: 228 },
    CachedPower { f: 0xFB9B7CD9A4A7443C, e: 720, k: 236 },
    CachedPower { f: 0xBB764C4CA7A44410, e: 747, k: 244 },
    CachedPower { f: 0x8BAB8EEFB6409C1A, e: 774, k: 252 },
    CachedPower { f: 0xD01FEF10A657842C, e: 800, k: 260 },
    CachedPower { f: 0x9B10A4E5E9913129, e: 827, k: 268 },
    CachedPower { f: 0xE7109BFBA19C0C9D, e: 853, k: 276 },
    CachedPower { f: 0xAC2820D9623BF429, e: 880, k: 284 },
    CachedPower { f: 0x80444B5E7AA7CF85, e: 907, k: 292 },
    CachedPower { f: 0xBF21E44003ACDD2D, e: 933, k: 300 },
    CachedPower { f: 0x8E679C2F5E44FF8F, e: 960, k: 308 },
    CachedPower { f: 0xD433179D9C8CB841, e: 986, k: 316 },
    CachedPower { f: 0x9E19DB92B4E31BA9, e: 1013, k: 324 },
];

/// to_chars.hpp:323-487 — for a normalized diyfp w = f * 2^e, a (normalized) cached power-of-ten
/// c = f_c * 2^e_c, such that the exponent of the product w * c satisfies
/// alpha <= e_c + e + q <= gamma.
fn get_cached_power_for_binary_exponent(e: i32) -> CachedPower {
    const K_CACHED_POWERS_MIN_DEC_EXP: i32 = -300;
    const K_CACHED_POWERS_DEC_STEP: i32 = 8;

    // This computation gives exactly the same results for k as
    //      k = ceil((kAlpha - e - 1) * 0.30102999566398114)
    // for |e| <= 1500, but doesn't require floating-point operations.
    // NB: log_10(2) ~= 78913 / 2^18
    let f = K_ALPHA - e - 1;
    let k = (f * 78913) / (1 << 18) + i32::from(f > 0);

    let index = (-K_CACHED_POWERS_MIN_DEC_EXP + k + (K_CACHED_POWERS_DEC_STEP - 1)) / K_CACHED_POWERS_DEC_STEP;

    K_CACHED_POWERS[index as usize]
}

/// to_chars.hpp:493-550 — for n != 0, returns k such that pow10 := 10^(k-1) <= n < 10^k;
/// for n == 0, returns 1 and sets pow10 := 1.
fn find_largest_pow10(n: u32, pow10: &mut u32) -> i32 {
    // LCOV_EXCL_START
    if n >= 1000000000 {
        *pow10 = 1000000000;
        return 10;
    }
    // LCOV_EXCL_STOP
    if n >= 100000000 {
        *pow10 = 100000000;
        return 9;
    }
    if n >= 10000000 {
        *pow10 = 10000000;
        return 8;
    }
    if n >= 1000000 {
        *pow10 = 1000000;
        return 7;
    }
    if n >= 100000 {
        *pow10 = 100000;
        return 6;
    }
    if n >= 10000 {
        *pow10 = 10000;
        return 5;
    }
    if n >= 1000 {
        *pow10 = 1000;
        return 4;
    }
    if n >= 100 {
        *pow10 = 100;
        return 3;
    }
    if n >= 10 {
        *pow10 = 10;
        return 2;
    }

    *pow10 = 1;
    1
}

/// to_chars.hpp:552-591
fn grisu2_round(buf: &mut [u8], dist: u64, delta: u64, mut rest: u64, ten_k: u64) {
    //               <--------------------------- delta ---->
    //                                  <---- dist --------->
    // --------------[------------------+-------------------]--------------
    //               M-                 w                   M+
    //
    //                                  ten_k
    //                                <------>
    //                                       <---- rest ---->
    // --------------[------------------+----+--------------]--------------
    //                                  w    V
    //                                       = buf * 10^k
    //
    // ten_k represents a unit-in-the-last-place in the decimal representation
    // stored in buf.
    // Decrement buf by ten_k while this takes buf closer to w.

    // The tests are written in this order to avoid overflow in unsigned
    // integer arithmetic.
    let last = buf.len() - 1;
    while rest < dist && delta - rest >= ten_k && (rest + ten_k < dist || dist - rest > rest + ten_k - dist) {
        buf[last] -= 1;
        rest += ten_k;
    }
}

/// to_chars.hpp:593-831 — generates V = buffer * 10^decimal_exponent, such that M- <= V <= M+.
/// M- and M+ must be normalized and share the same exponent -60 <= e <= -32.
fn grisu2_digit_gen(buffer: &mut Vec<u8>, decimal_exponent: &mut i32, m_minus: Diyfp, w: Diyfp, m_plus: Diyfp) {
    let mut delta: u64 = Diyfp::sub(m_plus, m_minus).f; // (significand of (M+ - M-), implicit exponent is e)
    let mut dist: u64 = Diyfp::sub(m_plus, w).f; // (significand of (M+ - w ), implicit exponent is e)

    // Split M+ = f * 2^e into two parts p1 and p2 (note: e < 0):
    //
    //      M+ = f * 2^e
    //         = ((f div 2^-e) * 2^-e + (f mod 2^-e)) * 2^e
    //         = ((p1        ) * 2^-e + (p2        )) * 2^e
    //         = p1 + p2 * 2^e
    let one_e: i32 = m_plus.e;
    let shift: u32 = (-one_e) as u32;
    let one_f: u64 = 1u64 << shift;

    let mut p1: u32 = (m_plus.f >> shift) as u32; // p1 = f div 2^-e (Since -e >= 32, p1 fits into a 32-bit int.)
    let mut p2: u64 = m_plus.f & (one_f - 1); // p2 = f mod 2^-e

    // 1)
    //
    // Generate the digits of the integral part p1 = d[n-1]...d[1]d[0]
    let mut pow10: u32 = 0;
    let k = find_largest_pow10(p1, &mut pow10);

    let mut n = k;
    while n > 0 {
        // Invariants:
        //      M+ = buffer * 10^n + (p1 + p2 * 2^e)    (buffer = 0 for n = k)
        //      pow10 = 10^(n-1) <= p1 < 10^n
        let d: u32 = p1 / pow10; // d = p1 div 10^(n-1)
        let r: u32 = p1 % pow10; // r = p1 mod 10^(n-1)
        //
        //      M+ = buffer * 10^n + (d * 10^(n-1) + r) + p2 * 2^e
        //         = (buffer * 10 + d) * 10^(n-1) + (r + p2 * 2^e)
        buffer.push(b'0' + d as u8); // buffer := buffer * 10 + d
        //
        //      M+ = buffer * 10^(n-1) + (r + p2 * 2^e)
        p1 = r;
        n -= 1;
        //
        //      M+ = buffer * 10^n + (p1 + p2 * 2^e)
        //      pow10 = 10^n

        // Now check if enough digits have been generated.
        // Compute
        //
        //      p1 + p2 * 2^e = (p1 * 2^-e + p2) * 2^e = rest * 2^e
        //
        // Note:
        // Since rest and delta share the same exponent e, it suffices to
        // compare the significands.
        let rest: u64 = (u64::from(p1) << shift) + p2;
        if rest <= delta {
            // V = buffer * 10^n, with M- <= V <= M+.

            *decimal_exponent += n;

            // We may now just stop. But instead look if the buffer could be
            // decremented to bring V closer to w.
            //
            // pow10 = 10^n is now 1 ulp in the decimal representation V.
            // The rounding procedure works with diyfp's with an implicit
            // exponent of e.
            //
            //      10^n = (10^n * 2^-e) * 2^e = ulp * 2^e
            let ten_n: u64 = u64::from(pow10) << shift;
            grisu2_round(buffer, dist, delta, rest, ten_n);

            return;
        }

        pow10 /= 10;
        //
        //      pow10 = 10^(n-1) <= p1 < 10^n
        // Invariants restored.
    }

    // 2)
    //
    // The digits of the integral part have been generated:
    //
    //      M+ = d[k-1]...d[1]d[0] + p2 * 2^e
    //         = buffer            + p2 * 2^e
    //
    // Now generate the digits of the fractional part p2 * 2^e.
    let mut m: i32 = 0;
    loop {
        // Invariant:
        //      M+ = buffer * 10^-m + 10^-m * (d[-m-1] / 10 + d[-m-2] / 10^2 + ...) * 2^e
        //         = buffer * 10^-m + 10^-m * (p2                                 ) * 2^e
        p2 = p2.wrapping_mul(10);
        let d: u64 = p2 >> shift; // d = (10 * p2) div 2^-e
        let r: u64 = p2 & (one_f - 1); // r = (10 * p2) mod 2^-e
        //
        //      M+ = buffer * 10^-m + 10^-m * (1/10 * (d * 2^-e + r) * 2^e
        //         = (buffer * 10 + d) * 10^(-m-1) + 10^(-m-1) * r * 2^e
        buffer.push(b'0' + d as u8); // buffer := buffer * 10 + d
        //
        //      M+ = buffer * 10^(-m-1) + 10^(-m-1) * r * 2^e
        p2 = r;
        m += 1;
        //
        //      M+ = buffer * 10^-m + 10^-m * p2 * 2^e
        // Invariant restored.

        // Check if enough digits have been generated.
        //
        //      10^-m * p2 * 2^e <= delta * 2^e
        //              p2 * 2^e <= 10^m * delta * 2^e
        //                    p2 <= 10^m * delta
        delta = delta.wrapping_mul(10);
        dist = dist.wrapping_mul(10);
        if p2 <= delta {
            break;
        }
    }

    // V = buffer * 10^-m, with M- <= V <= M+.

    *decimal_exponent -= m;

    // 1 ulp in the decimal representation is now 10^-m.
    // Since delta and dist are now scaled by 10^m, we need to do the
    // same with ulp in order to keep the units in sync.
    //
    //      10^m * 10^-m = 1 = 2^-e * 2^e = ten_m * 2^e
    let ten_m: u64 = one_f;
    grisu2_round(buffer, dist, delta, p2, ten_m);

    // By construction this algorithm generates the shortest possible decimal
    // number (Loitsch, Theorem 6.2) which rounds back to w.
    // For an input number of precision p, at least
    //
    //      N = 1 + ceil(p * log_10(2))
    //
    // decimal digits are sufficient to identify all binary floating-point
    // numbers (Matula, "In-and-Out conversions").
    // This implies that the algorithm does not produce more than N decimal
    // digits.
    //
    //      N = 17 for p = 53 (IEEE double precision)
    //      N = 9  for p = 24 (IEEE single precision)
}

/// to_chars.hpp:838-914 — v = buf * 10^decimal_exponent; the digits go to `buf`.
fn grisu2(buf: &mut Vec<u8>, decimal_exponent: &mut i32, value: f64) {
    let w = compute_boundaries(value);
    let (m_minus, v, m_plus) = (w.minus, w.w, w.plus);

    //  --------(-----------------------+-----------------------)--------    (A)
    //          m-                      v                       m+
    //
    //  --------------------(-----------+-----------------------)--------    (B)
    //                      m-          v                       m+
    //
    // First scale v (and m- and m+) such that the exponent is in the range
    // [alpha, gamma].
    let cached = get_cached_power_for_binary_exponent(m_plus.e);

    let c_minus_k = Diyfp { f: cached.f, e: cached.e }; // = c ~= 10^-k

    // The exponent of the products is = v.e + c_minus_k.e + q and is in the range [alpha,gamma]
    let w = Diyfp::mul(v, c_minus_k);
    let w_minus = Diyfp::mul(m_minus, c_minus_k);
    let w_plus = Diyfp::mul(m_plus, c_minus_k);

    //  ----(---+---)---------------(---+---)---------------(---+---)----
    //          w-                      w                       w+
    //          = c*m-                  = c*v                   = c*m+
    //
    // diyfp::mul rounds its result and c_minus_k is approximated too. w, w- and
    // w+ are now off by a small amount.
    // In fact:
    //
    //      w - v * 10^k < 1 ulp
    //
    // To account for this inaccuracy, add resp. subtract 1 ulp.
    //
    //  --------+---[---------------(---+---)---------------]---+--------
    //          w-  M-                  w                   M+  w+
    //
    // Now any number in [M-, M+] (bounds included) will round to w when input,
    // regardless of how the input rounding algorithm breaks ties.
    //
    // And digit_gen generates the shortest possible such number in [M-, M+].
    // Note that this does not mean that Grisu2 always generates the shortest
    // possible number in the interval (m-, m+).
    let m_minus = Diyfp { f: w_minus.f + 1, e: w_minus.e };
    let m_plus = Diyfp { f: w_plus.f - 1, e: w_plus.e };

    *decimal_exponent = -cached.k; // = -(-k) = k

    grisu2_digit_gen(buf, decimal_exponent, m_minus, w, m_plus);
}

/// to_chars.hpp:916-960 — appends a decimal representation of e; -1000 < e < 1000
fn append_exponent(o: &mut String, mut e: i32) {
    if e < 0 {
        e = -e;
        o.push('-');
    } else {
        o.push('+');
    }

    let mut k = e as u32;
    if k < 10 {
        // Always print at least two digits in the exponent.
        // This is for compatibility with printf("%g").
        o.push('0');
        o.push((b'0' + k as u8) as char);
    } else if k < 100 {
        o.push((b'0' + (k / 10) as u8) as char);
        k %= 10;
        o.push((b'0' + k as u8) as char);
    } else {
        o.push((b'0' + (k / 100) as u8) as char);
        k %= 100;
        o.push((b'0' + (k / 10) as u8) as char);
        k %= 10;
        o.push((b'0' + k as u8) as char);
    }
}

/// to_chars.hpp:962-1047 — prettify v = buf * 10^decimal_exponent.
///
/// If v is in the range [10^min_exp, 10^max_exp) it will be printed in fixed-point
/// notation. Otherwise it will be printed in exponential notation.
fn format_buffer(buf: &[u8], decimal_exponent: i32, min_exp: i32, max_exp: i32, o: &mut String) {
    let digits = |range: std::ops::Range<usize>| -> &str { std::str::from_utf8(&buf[range]).unwrap_or("") };
    let len = buf.len() as i32;
    let k = len;
    let n = len + decimal_exponent;

    // v = buf * 10^(n-k)
    // k is the length of the buffer (number of decimal digits)
    // n is the position of the decimal point relative to the start of the buffer.

    if k <= n && n <= max_exp {
        // digits[000]
        // len <= max_exp + 2
        o.push_str(digits(0..k as usize));
        for _ in k..n {
            o.push('0');
        }
        // Make it look like a floating-point number (#362, #378)
        o.push_str(".0");
        return;
    }

    if 0 < n && n <= max_exp {
        // dig.its
        // len <= max_digits10 + 1
        o.push_str(digits(0..n as usize));
        o.push('.');
        o.push_str(digits(n as usize..k as usize));
        return;
    }

    if min_exp < n && n <= 0 {
        // 0.[000]digits
        // len <= 2 + (-min_exp - 1) + max_digits10
        o.push_str("0.");
        for _ in 0..-n {
            o.push('0');
        }
        o.push_str(digits(0..k as usize));
        return;
    }

    if k == 1 {
        // dE+123
        // len <= 1 + 5
        o.push_str(digits(0..1));
    } else {
        // d.igitsE+123
        // len <= max_digits10 + 1 + 5
        o.push_str(digits(0..1));
        o.push('.');
        o.push_str(digits(1..k as usize));
    }

    o.push('e');
    append_exponent(o, n - 1);
}
