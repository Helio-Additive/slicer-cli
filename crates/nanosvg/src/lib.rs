//! Safe Rust port of NanoSVG XML callbacks and cubic path geometry.
//! BambuStudio 56a57d0, src/nanosvg/nanosvg.h 221–226 and 241–355.
#![forbid(unsafe_code)]

pub mod path;

#[derive(Debug, PartialEq, Eq)]
pub enum XmlEvent<'a> {
    Start {
        name: &'a [u8],
        attributes: Vec<(&'a [u8], &'a [u8])>,
    },
    End(&'a [u8]),
    Content(&'a [u8]),
}

fn whitespace(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

fn trim_start(s: &[u8]) -> &[u8] {
    &s[s.iter().position(|&b| !whitespace(b)).unwrap_or(s.len())..]
}

fn parse_element<'a>(tag: &'a [u8], event: &mut impl FnMut(XmlEvent<'a>)) {
    let tag = trim_start(tag);
    let closing = tag.starts_with(b"/");
    let tag = if closing { &tag[1..] } else { tag };
    if matches!(tag.first(), None | Some(b'?' | b'!')) {
        return;
    }
    let name_end = tag.iter().position(|&b| whitespace(b)).unwrap_or(tag.len());
    let (name, mut tail) = tag.split_at(name_end);
    if closing {
        event(XmlEvent::End(name));
        return;
    }

    let mut attributes = Vec::new();
    let mut self_closing = false;
    // NSVG_XML_MAX_ATTRIBS=256 entries: 127 name/value pairs and two terminators.
    while attributes.len() < 127 {
        tail = trim_start(tail);
        if tail.is_empty() {
            break;
        }
        if tail[0] == b'/' {
            self_closing = true;
            break;
        }
        let Some(end) = tail.iter().position(|&b| whitespace(b) || b == b'=') else {
            break;
        };
        let key = &tail[..end];
        tail = &tail[end + 1..];
        let Some(quote_pos) = tail.iter().position(|&b| matches!(b, b'\'' | b'"')) else {
            break;
        };
        let quote = tail[quote_pos];
        tail = &tail[quote_pos + 1..];
        let value_end = tail.iter().position(|&b| b == quote).unwrap_or(tail.len());
        attributes.push((key, &tail[..value_end]));
        tail = tail.get(value_end + 1..).unwrap_or_default();
    }

    event(XmlEvent::Start { name, attributes });
    if self_closing {
        event(XmlEvent::End(name));
    }
}

/// Deliver the same start, end and text spans as nsvg__parseXML. NanoSVG
/// treats a zero byte as the end of the input and dispatches content at '<'.
pub fn parse_xml<'a>(input: &'a [u8], mut event: impl FnMut(XmlEvent<'a>)) {
    let mut mark = 0;
    let mut in_tag = false;
    for (i, &b) in input.iter().enumerate() {
        match (b, in_tag) {
            (0, _) => break,
            (b'<', false) => {
                let text = trim_start(&input[mark..i]);
                if !text.is_empty() {
                    event(XmlEvent::Content(text));
                }
                mark = i + 1;
                in_tag = true;
            }
            (b'>', true) => {
                parse_element(&input[mark..i], &mut event);
                mark = i + 1;
                in_tag = false;
            }
            _ => {}
        }
    }
}
