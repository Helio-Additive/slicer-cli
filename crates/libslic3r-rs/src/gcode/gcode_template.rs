//! Expression-capable G-code template evaluator — the working core of a
//! BambuStudio `PlaceholderParser` (src/libslic3r/PlaceholderParser.cpp) port.
//!
//! The legacy [`super::placeholder_parser`] is a fixed-string `.replace` stub.
//! The machine/filament/`change_filament_gcode` templates need much more: `[var]`
//! and `{expr}` substitution, `{if}/{elsif}/{else}/{endif}` conditionals
//! (nestable), arithmetic with parens, comparisons, `&&`/`||`, string equality,
//! and array indexing (`flush_temperatures[previous_extruder]`). This module
//! implements that with a small tokenizer + recursive-descent evaluator, kept
//! side-effect-free and unit-tested (see `tests/gcode_template.rs`) so it can be
//! validated before being wired into the tool-change emission.
//!
//! Scope note: this is the evaluator only; populating the ~29 change-filament
//! variables from the print/filament config and wiring it into the toolchange is
//! a separate step.

use std::collections::HashMap;

/// A template value. Numbers stay Int where possible so `[next_extruder]`
/// renders `3`, not `3.0`.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Int(i64),
    Float(f64),
    Bool(bool),
    Str(String),
}

impl Value {
    fn as_f64(&self) -> f64 {
        match self {
            Value::Int(i) => *i as f64,
            Value::Float(f) => *f,
            Value::Bool(b) => {
                if *b {
                    1.0
                } else {
                    0.0
                }
            }
            Value::Str(_) => 0.0,
        }
    }

    fn as_bool(&self) -> bool {
        match self {
            Value::Bool(b) => *b,
            Value::Int(i) => *i != 0,
            Value::Float(f) => *f != 0.0,
            Value::Str(s) => !s.is_empty(),
        }
    }

    fn is_number(&self) -> bool {
        matches!(self, Value::Int(_) | Value::Float(_) | Value::Bool(_))
    }

    /// Render as it appears in emitted G-code.
    pub fn render(&self) -> String {
        match self {
            Value::Int(i) => i.to_string(),
            Value::Bool(b) => (if *b { "1" } else { "0" }).to_string(),
            Value::Str(s) => s.clone(),
            Value::Float(f) => {
                // Trim trailing zeros for a stable, compact form.
                let s = format!("{:.6}", f);
                let s = s.trim_end_matches('0').trim_end_matches('.');
                if s.is_empty() || s == "-0" {
                    "0".to_string()
                } else {
                    s.to_string()
                }
            }
        }
    }
}

/// Variable bindings for template evaluation: scalars and arrays.
#[derive(Default, Clone)]
pub struct Context {
    scalars: HashMap<String, Value>,
    arrays: HashMap<String, Vec<Value>>,
}

impl Context {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn set(&mut self, key: &str, v: Value) -> &mut Self {
        self.scalars.insert(key.to_string(), v);
        self
    }
    pub fn set_int(&mut self, key: &str, v: i64) -> &mut Self {
        self.set(key, Value::Int(v))
    }
    pub fn set_float(&mut self, key: &str, v: f64) -> &mut Self {
        self.set(key, Value::Float(v))
    }
    pub fn set_array(&mut self, key: &str, v: Vec<Value>) -> &mut Self {
        self.arrays.insert(key.to_string(), v);
        self
    }
}

// ============================================================================
// Expression tokenizer
// ============================================================================

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Num(f64),
    IntNum(i64),
    Ident(String),
    Str(String),
    Op(String), // + - * / % < > <= >= == != && || !
    LParen,
    RParen,
    LBracket,
    RBracket,
}

fn tokenize(s: &str) -> Result<Vec<Tok>, String> {
    let mut toks = Vec::new();
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() {
        let c = b[i] as char;
        if c.is_whitespace() {
            i += 1;
            continue;
        }
        match c {
            '(' => {
                toks.push(Tok::LParen);
                i += 1;
            }
            ')' => {
                toks.push(Tok::RParen);
                i += 1;
            }
            '[' => {
                toks.push(Tok::LBracket);
                i += 1;
            }
            ']' => {
                toks.push(Tok::RBracket);
                i += 1;
            }
            '"' => {
                let mut j = i + 1;
                let mut val = String::new();
                while j < b.len() && b[j] as char != '"' {
                    val.push(b[j] as char);
                    j += 1;
                }
                if j >= b.len() {
                    return Err("unterminated string".into());
                }
                toks.push(Tok::Str(val));
                i = j + 1;
            }
            '0'..='9' | '.' => {
                let mut j = i;
                let mut has_dot = false;
                while j < b.len() {
                    let cj = b[j] as char;
                    if cj.is_ascii_digit() {
                        j += 1;
                    } else if cj == '.' && !has_dot {
                        has_dot = true;
                        j += 1;
                    } else {
                        break;
                    }
                }
                let numstr = &s[i..j];
                if has_dot {
                    toks.push(Tok::Num(
                        numstr.parse::<f64>().map_err(|e| e.to_string())?,
                    ));
                } else {
                    toks.push(Tok::IntNum(
                        numstr.parse::<i64>().map_err(|e| e.to_string())?,
                    ));
                }
                i = j;
            }
            'a'..='z' | 'A'..='Z' | '_' => {
                let mut j = i;
                while j < b.len() {
                    let cj = b[j] as char;
                    if cj.is_ascii_alphanumeric() || cj == '_' {
                        j += 1;
                    } else {
                        break;
                    }
                }
                toks.push(Tok::Ident(s[i..j].to_string()));
                i = j;
            }
            '<' | '>' | '=' | '!' | '&' | '|' | '+' | '-' | '*' | '/' | '%' => {
                // Two-char operators first.
                let two = if i + 1 < b.len() {
                    &s[i..i + 2]
                } else {
                    ""
                };
                if matches!(two, "<=" | ">=" | "==" | "!=" | "&&" | "||") {
                    toks.push(Tok::Op(two.to_string()));
                    i += 2;
                } else {
                    toks.push(Tok::Op(c.to_string()));
                    i += 1;
                }
            }
            _ => return Err(format!("unexpected char '{}'", c)),
        }
    }
    Ok(toks)
}

// ============================================================================
// Recursive-descent parser + evaluator
// Precedence (low→high): || , && , (== !=) , (< > <= >=) , (+ -) , (* / %) ,
// unary (- !) , primary.
// ============================================================================

struct Parser<'a> {
    toks: Vec<Tok>,
    pos: usize,
    ctx: &'a Context,
}

impl<'a> Parser<'a> {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.pos)
    }
    fn next(&mut self) -> Option<Tok> {
        let t = self.toks.get(self.pos).cloned();
        if t.is_some() {
            self.pos += 1;
        }
        t
    }
    fn eat_op(&mut self, ops: &[&str]) -> Option<String> {
        if let Some(Tok::Op(o)) = self.peek() {
            if ops.contains(&o.as_str()) {
                let o = o.clone();
                self.pos += 1;
                return Some(o);
            }
        }
        None
    }

    fn parse(&mut self) -> Result<Value, String> {
        let v = self.parse_or()?;
        if self.pos != self.toks.len() {
            return Err(format!("trailing tokens at {}", self.pos));
        }
        Ok(v)
    }

    fn parse_or(&mut self) -> Result<Value, String> {
        let mut left = self.parse_and()?;
        while self.eat_op(&["||"]).is_some() {
            let right = self.parse_and()?;
            left = Value::Bool(left.as_bool() || right.as_bool());
        }
        Ok(left)
    }
    fn parse_and(&mut self) -> Result<Value, String> {
        let mut left = self.parse_eq()?;
        while self.eat_op(&["&&"]).is_some() {
            let right = self.parse_eq()?;
            left = Value::Bool(left.as_bool() && right.as_bool());
        }
        Ok(left)
    }
    fn parse_eq(&mut self) -> Result<Value, String> {
        let mut left = self.parse_cmp()?;
        while let Some(op) = self.eat_op(&["==", "!="]) {
            let right = self.parse_cmp()?;
            let eq = match (&left, &right) {
                (Value::Str(a), Value::Str(b)) => a == b,
                _ => (left.as_f64() - right.as_f64()).abs() < 1e-9,
            };
            left = Value::Bool(if op == "==" { eq } else { !eq });
        }
        Ok(left)
    }
    fn parse_cmp(&mut self) -> Result<Value, String> {
        let mut left = self.parse_add()?;
        while let Some(op) = self.eat_op(&["<", ">", "<=", ">="]) {
            let right = self.parse_add()?;
            let (a, bb) = (left.as_f64(), right.as_f64());
            let r = match op.as_str() {
                "<" => a < bb,
                ">" => a > bb,
                "<=" => a <= bb,
                ">=" => a >= bb,
                _ => unreachable!(),
            };
            left = Value::Bool(r);
        }
        Ok(left)
    }
    fn parse_add(&mut self) -> Result<Value, String> {
        let mut left = self.parse_mul()?;
        while let Some(op) = self.eat_op(&["+", "-"]) {
            let right = self.parse_mul()?;
            left = num_binop(&left, &right, &op)?;
        }
        Ok(left)
    }
    fn parse_mul(&mut self) -> Result<Value, String> {
        let mut left = self.parse_unary()?;
        while let Some(op) = self.eat_op(&["*", "/", "%"]) {
            let right = self.parse_unary()?;
            left = num_binop(&left, &right, &op)?;
        }
        Ok(left)
    }
    fn parse_unary(&mut self) -> Result<Value, String> {
        if let Some(op) = self.eat_op(&["-", "!"]) {
            let v = self.parse_unary()?;
            return Ok(match op.as_str() {
                "-" => {
                    if let Value::Int(i) = v {
                        Value::Int(-i)
                    } else {
                        Value::Float(-v.as_f64())
                    }
                }
                "!" => Value::Bool(!v.as_bool()),
                _ => unreachable!(),
            });
        }
        self.parse_primary()
    }
    fn parse_primary(&mut self) -> Result<Value, String> {
        match self.next() {
            Some(Tok::IntNum(i)) => Ok(Value::Int(i)),
            Some(Tok::Num(f)) => Ok(Value::Float(f)),
            Some(Tok::Str(s)) => Ok(Value::Str(s)),
            Some(Tok::LParen) => {
                let v = self.parse_or()?;
                match self.next() {
                    Some(Tok::RParen) => Ok(v),
                    _ => Err("expected ')'".into()),
                }
            }
            Some(Tok::Ident(name)) => {
                if name == "true" {
                    return Ok(Value::Bool(true));
                }
                if name == "false" {
                    return Ok(Value::Bool(false));
                }
                // Array index?  ident '[' expr ']'
                if matches!(self.peek(), Some(Tok::LBracket)) {
                    self.pos += 1; // consume '['
                    let idx = self.parse_or()?;
                    match self.next() {
                        Some(Tok::RBracket) => {}
                        _ => return Err("expected ']'".into()),
                    }
                    let arr = self
                        .ctx
                        .arrays
                        .get(&name)
                        .ok_or_else(|| format!("unknown array '{}'", name))?;
                    let i = idx.as_f64() as i64;
                    if i < 0 || i as usize >= arr.len() {
                        return Err(format!("index {} out of range for '{}'", i, name));
                    }
                    Ok(arr[i as usize].clone())
                } else {
                    self.ctx
                        .scalars
                        .get(&name)
                        .cloned()
                        .ok_or_else(|| format!("unknown variable '{}'", name))
                }
            }
            other => Err(format!("unexpected token {:?}", other)),
        }
    }
}

fn num_binop(a: &Value, b: &Value, op: &str) -> Result<Value, String> {
    let both_int = matches!(a, Value::Int(_)) && matches!(b, Value::Int(_));
    if both_int && op != "/" {
        let (x, y) = (
            if let Value::Int(i) = a { *i } else { 0 },
            if let Value::Int(i) = b { *i } else { 0 },
        );
        return Ok(Value::Int(match op {
            "+" => x + y,
            "-" => x - y,
            "*" => x * y,
            "%" => {
                if y == 0 {
                    return Err("modulo by zero".into());
                }
                x % y
            }
            _ => unreachable!(),
        }));
    }
    let (x, y) = (a.as_f64(), b.as_f64());
    Ok(Value::Float(match op {
        "+" => x + y,
        "-" => x - y,
        "*" => x * y,
        "/" => {
            if y == 0.0 {
                return Err("division by zero".into());
            }
            x / y
        }
        "%" => x % y,
        _ => unreachable!(),
    }))
}

/// Evaluate a single expression string against the context.
pub fn eval_expr(expr: &str, ctx: &Context) -> Result<Value, String> {
    let toks = tokenize(expr)?;
    let mut p = Parser {
        toks,
        pos: 0,
        ctx,
    };
    p.parse()
}

// ============================================================================
// Template processing: `{if}/{elsif}/{else}/{endif}` + inline substitution.
// Directives occupy their own line (matches BambuStudio's machine/filament
// templates); substitution replaces `{expr}` then top-level `[expr]`.
// ============================================================================

struct Frame {
    active: bool,       // is this branch currently emitting
    matched: bool,      // has any branch in this if-chain matched
    parent_active: bool, // was the enclosing scope emitting
}

/// Process a full template into concrete G-code. Unknown variables leave the
/// enclosing line's placeholder untouched (so gaps are visible rather than
/// silently wrong) — callers should provide a complete context.
pub fn process(template: &str, ctx: &Context) -> String {
    let mut out = String::new();
    let mut stack: Vec<Frame> = Vec::new();
    let currently_active = |stack: &[Frame]| stack.last().map(|f| f.active).unwrap_or(true);

    // Emit the text that trails a directive on the same line, if the branch it
    // belongs to is active. Trailing text that is empty (the directive occupied
    // the whole line) emits nothing, preserving the original behaviour.
    let mut emit = |out: &mut String, rest: &str, active: bool| {
        if active && !rest.trim().is_empty() {
            out.push_str(&substitute_line(rest, ctx));
            out.push('\n');
        }
    };

    for raw_line in template.split('\n') {
        let trimmed = raw_line.trim();
        if let Some((cond, rest)) = directive(trimmed, "if") {
            let parent = currently_active(&stack);
            let val = parent && eval_expr(cond, ctx).map(|v| v.as_bool()).unwrap_or(false);
            stack.push(Frame {
                active: val,
                matched: val,
                parent_active: parent,
            });
            emit(&mut out, rest, val);
            continue;
        }
        if let Some((cond, rest)) = directive(trimmed, "elsif") {
            let mut take = false;
            if let Some(f) = stack.last_mut() {
                take = f.parent_active
                    && !f.matched
                    && eval_expr(cond, ctx).map(|v| v.as_bool()).unwrap_or(false);
                f.active = take;
                f.matched |= take;
            }
            emit(&mut out, rest, take);
            continue;
        }
        if trimmed.starts_with("{else}") {
            let mut take = false;
            if let Some(f) = stack.last_mut() {
                f.active = f.parent_active && !f.matched;
                f.matched = true;
                take = f.active;
            }
            emit(&mut out, &trimmed["{else}".len()..], take);
            continue;
        }
        if trimmed.starts_with("{endif}") {
            stack.pop();
            let rest = &trimmed["{endif}".len()..];
            let active = currently_active(&stack);
            emit(&mut out, rest, active);
            continue;
        }
        if !currently_active(&stack) {
            continue;
        }
        out.push_str(&substitute_line(raw_line, ctx));
        out.push('\n');
    }
    out
}

/// If `line` STARTS with `{kw <cond>}`, return the condition text and whatever
/// follows the closing brace on the same line.
///
/// BambuStudio's templates put the guarded text on the directive's own line —
/// the stock `filament_start_gcode` is
///
///     {if  (bed_temperature[current_extruder] >55)}M106 P3 S200
///     {elsif(bed_temperature[current_extruder] >50)}M106 P3 S150
///
/// so requiring the directive to be the WHOLE line (which is what this used to
/// do) left every such line unmatched, and `process` fell through and copied the
/// raw `{if ...}` text into the gcode. R495.
fn directive<'a>(line: &'a str, kw: &str) -> Option<(&'a str, &'a str)> {
    let prefix = format!("{{{}", kw); // "{if" / "{elsif"
    if !line.starts_with(&prefix) {
        return None;
    }
    // The condition ends at the first '}' — conditions use ()/[] but never {}.
    let close = line.find('}')?;
    let inner = &line[1..close];
    let rest = inner.strip_prefix(kw)?;
    // Require a separator so `{iffy}` doesn't match `{if`. `{elsif(a>1)}` has no
    // whitespace, so a leading '(' counts too.
    if !(rest.is_empty() || rest.starts_with(char::is_whitespace) || rest.starts_with('(')) {
        return None;
    }
    Some((rest.trim(), &line[close + 1..]))
}

/// Replace `{expr}` then top-level `[expr]` in one line.
fn substitute_line(line: &str, ctx: &Context) -> String {
    let after_braces = replace_delimited(line, '{', '}', ctx);
    replace_delimited(&after_braces, '[', ']', ctx)
}

fn replace_delimited(s: &str, open: char, close: char, ctx: &Context) -> String {
    let mut out = String::new();
    let chars: Vec<char> = s.chars().collect();
    let mut i = 0;
    while i < chars.len() {
        if chars[i] == open {
            // find matching close (no nesting of the same delimiter in templates)
            if let Some(j) = (i + 1..chars.len()).find(|&k| chars[k] == close) {
                let inner: String = chars[i + 1..j].iter().collect();
                match eval_expr(&inner, ctx) {
                    Ok(v) => out.push_str(&v.render()),
                    // Unknown/parse error: keep the original placeholder verbatim.
                    Err(_) => {
                        out.push(open);
                        out.push_str(&inner);
                        out.push(close);
                    }
                }
                i = j + 1;
                continue;
            }
        }
        out.push(chars[i]);
        i += 1;
    }
    out
}
