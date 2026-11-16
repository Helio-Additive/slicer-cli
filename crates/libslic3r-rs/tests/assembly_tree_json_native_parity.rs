//! `format::assembly_tree_json` against Format/AssemblyTreeJson.cpp at the pin.
//!
//! `tests/data/assembly_tree_json/vectors.txt` is the output of `gen.cpp` beside it, which
//! compiles AssemblyTreeJson.cpp itself with the engine's nlohmann::json (the header comment there
//! says how, and describes the lines). Both functions of the unit are compared with the built C++,
//! exactly: `to_json_string` by the bytes of the text, `from_json_string` by its result, the tree
//! it leaves and the text of the error. The one text that is not nlohmann's is that of a parse
//! error (the FIDELITY-NOTE of the unit). The inline members of the header are read off the C++.

use slicer::format::assembly_steps_json::PARSE_ERROR_WHAT;
use slicer::format::assembly_tree_json::*;

const VECTORS: &str = include_str!("data/assembly_tree_json/vectors.txt");

fn esc(s: &str) -> String {
    if s.is_empty() {
        return "\\e".to_string();
    }
    let mut out = String::new();
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            ' ' => out.push_str("\\s"),
            c if (c as u32) < 0x20 || c as u32 == 0x7f => out.push_str(&format!("\\x{:02X}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

fn unesc(s: &str) -> String {
    if s == "\\e" {
        return String::new();
    }
    let mut out = String::new();
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c != '\\' {
            out.push(c);
            continue;
        }
        match chars.next().expect("escape") {
            '\\' => out.push('\\'),
            'n' => out.push('\n'),
            'r' => out.push('\r'),
            's' => out.push(' '),
            'x' => {
                let hex: String = [chars.next().expect("hex"), chars.next().expect("hex")].iter().collect();
                out.push(u8::from_str_radix(&hex, 16).expect("hex") as char);
            }
            other => panic!("escape \\{other}"),
        }
    }
    out
}

fn ints(v: &[i32]) -> String {
    let mut s = format!(" {}", v.len());
    for i in v {
        s.push_str(&format!(" {i}"));
    }
    s
}

fn dump_tree(tree: &AssemblyTreeData) -> String {
    let mut s = format!("A {}", tree.nodes.len());
    for n in &tree.nodes {
        s.push_str(&format!(
            " {} {} {} {} {} {} {}{}",
            n.id,
            n.parent_id,
            n.object_idx,
            n.volume_idx,
            n.selectable as i32,
            esc(&n.uid),
            esc(&n.label),
            ints(&n.children)
        ));
    }
    s + &ints(&tree.roots) + &ints(&tree.checked_node_indices)
}

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn i(&mut self) -> i32 {
        self.s().parse().expect("integer")
    }
    fn ints(&mut self) -> Vec<i32> {
        let n = self.i();
        (0..n).map(|_| self.i()).collect()
    }
}

fn parse_tree(line: &str) -> AssemblyTreeData {
    let mut t = Tok(line.split_whitespace());
    assert_eq!(t.s(), "A");
    let mut tree = AssemblyTreeData::default();
    for _ in 0..t.i() {
        let mut node = AssemblyTreeNodeData::default();
        node.id = t.i();
        node.parent_id = t.i();
        node.object_idx = t.i();
        node.volume_idx = t.i();
        node.selectable = t.i() != 0;
        node.uid = unesc(t.s());
        node.label = unesc(t.s());
        node.children = t.ints();
        tree.nodes.push(node);
    }
    tree.roots = t.ints();
    tree.checked_node_indices = t.ints();
    assert!(t.0.next().is_none(), "tree line read to its end");
    tree
}

#[test]
fn both_functions_match_the_cpp() {
    let mut lines = VECTORS.lines();
    let (mut to, mut from, mut from_ok) = (0, 0, 0);
    while let Some(op) = lines.next() {
        match op {
            "TO" => {
                to += 1;
                let line = lines.next().expect("tree");
                let tree = parse_tree(line);
                assert_eq!(dump_tree(&tree), line, "the tree is read back as written");
                assert_eq!(esc(&tree.to_json_string()), lines.next().expect("out"), "TO {line}");
            }
            "FROM" => {
                from += 1;
                let text = unesc(lines.next().expect("text"));
                let mut tree = AssemblyTreeData::default();
                tree.nodes.push(AssemblyTreeNodeData::default());
                tree.roots.push(7);
                tree.checked_node_indices.push(8);
                let mut error = "unset".to_string();
                let ok = AssemblyTreeData::from_json_string(&text, &mut tree, Some(&mut error));
                assert_eq!((ok as i32).to_string(), lines.next().expect("ok"), "FROM {text:?}");
                from_ok += ok as usize;
                let want_error = unesc(lines.next().expect("error"));
                if want_error.starts_with("[json.exception.parse_error.") {
                    assert_eq!(error, PARSE_ERROR_WHAT, "FROM {text:?}");
                } else {
                    assert_eq!(error, want_error, "FROM {text:?}");
                }
                assert_eq!(dump_tree(&tree), lines.next().expect("tree"), "FROM {text:?}");
                assert_eq!(esc(&tree.to_json_string()), lines.next().expect("out"), "FROM {text:?}");
                // The error is optional.
                let mut again = AssemblyTreeData::default();
                assert_eq!(AssemblyTreeData::from_json_string(&text, &mut again, None), ok);
            }
            other => panic!("unknown case {other}"),
        }
    }
    assert_eq!((to, from, from_ok), (40, 194, 102));
}

/// AssemblyTreeJson.hpp:9-33 and AssemblyTreeJson.cpp:24.
#[test]
fn header_inline_members() {
    let node = AssemblyTreeNodeData::default();
    assert_eq!((node.id, node.parent_id, node.object_idx, node.volume_idx, node.selectable), (-1, -1, -1, -1, true));
    assert!(node.uid.is_empty() && node.label.is_empty() && node.children.is_empty());

    let mut tree = AssemblyTreeData::default();
    assert!(tree.empty());
    tree.nodes.push(node);
    tree.roots.push(0);
    tree.checked_node_indices.push(0);
    assert!(!tree.empty());
    tree.clear();
    assert!(tree.empty());
    assert_eq!(tree, AssemblyTreeData::default());

    assert!(!SHOW_ORIGIN_STEP_TREE.load(std::sync::atomic::Ordering::Relaxed));
}
