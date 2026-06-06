//! Faithful 1:1 port of BambuStudio `src/libslic3r/PlaceholderParser.{hpp,cpp}`.
//!
//! STATUS: partial. The PlaceholderParser is built on two C++ subsystems that
//! are NOT yet present in this Rust crate:
//!
//!   1. The dynamic config type system: `DynamicConfig` / `DynamicPrintConfig`
//!      (a key -> `ConfigOption` map with `option()`, `set_key_value()`,
//!      `keys()`, `operator+=`), the `ConfigOption` polymorphic hierarchy
//!      (`is_vector`/`is_scalar`/`type()`/`getInt`/`getFloat`/`getBool`/
//!      `serialize`/`vserialize`, plus the concrete `ConfigOptionInt`,
//!      `ConfigOptionFloat`, `ConfigOptionString`, `ConfigOptionFloats`,
//!      `ConfigOptionFloatsOrPercentsNullable`, etc.), and `print_config_def`
//!      / `ConfigOptionDef` with its `ratio_over` dependency graph.
//!      This crate uses a different, statically-typed config model
//!      (`PrintConfig`/`GCodeConfig`/`PrintObjectConfig` structs) and has no
//!      `DynamicConfig`/`ConfigOption` runtime-typed dictionary.
//!
//!   2. The Boost.Spirit Qi grammar (`macro_processor<Iterator>`) that drives
//!      `process()` / `evaluate_boolean_expression()`. There is no Rust
//!      equivalent; a faithful re-port requires a hand-written recursive
//!      descent parser, which in turn depends on subsystem (1).
//!
//! Therefore the genuinely config-independent core IS ported faithfully here:
//! the `expr` value type and ALL of its evaluation operations
//! (`PlaceholderParser.cpp:190`-`707`) -- unary minus / integer / round /
//! floor / ceil / not, the `+= -= *= /= %=` arithmetic, `compare_op` and the
//! six comparison helpers, `min`/`max`/`random`/`digits`, the logical and
//! ternary operators, and `to_string`. These are the heart of the expression
//! evaluator and are pure value semantics with no config dependency.
//!
//! Everything requiring subsystem (1) or (2) is left documented and blocked
//! below rather than faked: `PlaceholderParser` ctor, `apply_config`,
//! `config_diff`, `apply_only`, `apply_env_variables`, `set`, `process`,
//! `evaluate_boolean_expression`, `MyContext`, and the grammar.
//!
//! `coord_t -> i64`, `coordf_t -> f64`. The expression's integer type is the
//! C++ `int` (32-bit), preserved here as `i32` so that integer arithmetic,
//! division, modulo and `digits` formatting match the C++ exactly.

use crate::{Error, Result};
use rand::Rng;

// PlaceholderParser.cpp:70  #define L(s) (s)
// PlaceholderParser.cpp:71  #define _(s) Slic3r::I18N::translate(s)

// =====================================================================
// expr<Iterator>  (PlaceholderParser.cpp:190-707)
// ---------------------------------------------------------------------
// The C++ `expr` is a tagged union (`Type` enum + `Data` union) carrying a
// bool/int/double/string payload, together with the source `it_range` used
// for error reporting. We model the value payload faithfully; the iterator
// range is parser bookkeeping (used to throw `qi::expectation_failure` with a
// source location) and is replaced here by plain error strings, since the
// Boost.Spirit grammar that produces those iterators is not ported.
// =====================================================================

/// `expr<Iterator>::Type` (PlaceholderParser.cpp:305-311)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Type {
    // PlaceholderParser.cpp:306
    TypeEmpty = 0,
    // PlaceholderParser.cpp:307
    TypeBool,
    // PlaceholderParser.cpp:308
    TypeInt,
    // PlaceholderParser.cpp:309
    TypeDouble,
    // PlaceholderParser.cpp:310
    TypeString,
}

/// `expr<Iterator>::Data` (PlaceholderParser.cpp:291-303), modelled as a
/// Rust enum carrying the tag and payload together. The C++ union + separate
/// `type` member is collapsed into one safe enum; `Type` is recovered via
/// [`Expr::type`].
#[derive(Clone, Debug)]
enum Data {
    Empty,
    Bool(bool),
    Int(i32),
    Double(f64),
    Str(String),
}

/// `expr<Iterator>` (PlaceholderParser.cpp:191).
#[derive(Clone, Debug)]
pub struct Expr {
    data: Data,
}

impl Expr {
    // PlaceholderParser.cpp:193  expr() : type(TYPE_EMPTY) {}
    pub fn new() -> Self {
        Expr { data: Data::Empty }
    }

    // PlaceholderParser.cpp:194  explicit expr(bool b) : type(TYPE_BOOL) { data.b = b; }
    pub fn from_bool(b: bool) -> Self {
        Expr { data: Data::Bool(b) }
    }

    // PlaceholderParser.cpp:196  explicit expr(int i) : type(TYPE_INT) { data.i = i; }
    pub fn from_int(i: i32) -> Self {
        Expr { data: Data::Int(i) }
    }

    // PlaceholderParser.cpp:198  explicit expr(double d) : type(TYPE_DOUBLE) { data.d = d; }
    pub fn from_double(d: f64) -> Self {
        Expr { data: Data::Double(d) }
    }

    // PlaceholderParser.cpp:200-201  explicit expr(const char *s) / (const std::string &s) : type(TYPE_STRING)
    pub fn from_string(s: impl Into<String>) -> Self {
        Expr { data: Data::Str(s.into()) }
    }

    // PlaceholderParser.cpp:238-246  void reset()
    pub fn reset(&mut self) {
        // BBS: TYPE_STRING owned a heap std::string; in Rust dropping the
        // String reclaims it. Resetting to TYPE_EMPTY mirrors the C++.
        self.data = Data::Empty;
    }

    // PlaceholderParser.cpp:313  Type type;  -- recover the tag from the payload.
    pub fn type_(&self) -> Type {
        match self.data {
            Data::Empty => Type::TypeEmpty,
            Data::Bool(_) => Type::TypeBool,
            Data::Int(_) => Type::TypeInt,
            Data::Double(_) => Type::TypeDouble,
            Data::Str(_) => Type::TypeString,
        }
    }

    // PlaceholderParser.cpp:248-249  bool& b() / bool b() const
    pub fn b(&self) -> bool {
        match self.data {
            Data::Bool(b) => b,
            // The C++ reads the raw union member regardless of tag; callers
            // only invoke b() when type==TYPE_BOOL, so this is unreachable in
            // correct use.
            _ => false,
        }
    }

    // PlaceholderParser.cpp:250  void set_b(bool v) { reset(); data.b=v; type=TYPE_BOOL; }
    pub fn set_b(&mut self, v: bool) {
        self.reset();
        self.data = Data::Bool(v);
    }

    // PlaceholderParser.cpp:251-252  int& i() / int i() const
    pub fn i(&self) -> i32 {
        match self.data {
            Data::Int(i) => i,
            _ => 0,
        }
    }

    // PlaceholderParser.cpp:253  void set_i(int v) { reset(); data.i=v; type=TYPE_INT; }
    pub fn set_i(&mut self, v: i32) {
        self.reset();
        self.data = Data::Int(v);
    }

    // PlaceholderParser.cpp:254  int as_i() const { return (type==TYPE_INT) ? i() : int(d()); }
    pub fn as_i(&self) -> i32 {
        if self.type_() == Type::TypeInt {
            self.i()
        } else {
            self.d() as i32
        }
    }

    // PlaceholderParser.cpp:255  int as_i_rounded() const { return (type==TYPE_INT) ? i() : int(std::round(d())); }
    pub fn as_i_rounded(&self) -> i32 {
        if self.type_() == Type::TypeInt {
            self.i()
        } else {
            self.d().round() as i32
        }
    }

    // PlaceholderParser.cpp:256-257  double& d() / double d() const
    pub fn d(&self) -> f64 {
        match self.data {
            Data::Double(d) => d,
            _ => 0.0,
        }
    }

    // PlaceholderParser.cpp:258  void set_d(double v) { reset(); data.d=v; type=TYPE_DOUBLE; }
    pub fn set_d(&mut self, v: f64) {
        self.reset();
        self.data = Data::Double(v);
    }

    // PlaceholderParser.cpp:259  double as_d() const { return (type==TYPE_DOUBLE) ? d() : double(i()); }
    pub fn as_d(&self) -> f64 {
        if self.type_() == Type::TypeDouble {
            self.d()
        } else {
            self.i() as f64
        }
    }

    // PlaceholderParser.cpp:260-261  std::string& s() / const std::string& s() const
    pub fn s(&self) -> &str {
        match &self.data {
            Data::Str(s) => s,
            _ => "",
        }
    }

    // PlaceholderParser.cpp:262  void set_s(const std::string &s) { reset(); data.s=new std::string(s); type=TYPE_STRING; }
    // PlaceholderParser.cpp:263  void set_s(std::string &&s) { ... }
    pub fn set_s(&mut self, s: impl Into<String>) {
        self.reset();
        self.data = Data::Str(s.into());
    }

    // PlaceholderParser.cpp:265-289  std::string to_string() const
    pub fn to_string(&self) -> String {
        match &self.data {
            // PlaceholderParser.cpp:269  case TYPE_BOOL: out = data.b ? "true" : "false";
            Data::Bool(b) => {
                if *b {
                    "true".to_string()
                } else {
                    "false".to_string()
                }
            }
            // PlaceholderParser.cpp:270  case TYPE_INT: out = std::to_string(data.i);
            Data::Int(i) => i.to_string(),
            // PlaceholderParser.cpp:271-284  case TYPE_DOUBLE:
            //   ostringstream default converter produces no trailing zeros.
            Data::Double(d) => format_double_ostream(*d),
            // PlaceholderParser.cpp:285  case TYPE_STRING: out = *data.s;
            Data::Str(s) => s.clone(),
            // PlaceholderParser.cpp:286  default: break;  (TYPE_EMPTY -> empty string)
            Data::Empty => String::new(),
        }
    }

    // PlaceholderParser.cpp:319-332  expr unary_minus(const Iterator start_pos) const
    pub fn unary_minus(&self) -> Result<Expr> {
        match self.data {
            // PlaceholderParser.cpp:322-323  case TYPE_INT: return expr(-i());
            Data::Int(i) => Ok(Expr::from_int(-i)),
            // PlaceholderParser.cpp:324-325  case TYPE_DOUBLE: return expr(-d());
            Data::Double(d) => Ok(Expr::from_double(-d)),
            // PlaceholderParser.cpp:326-327  default: throw "Cannot apply unary minus operator."
            _ => self.throw_exception("Cannot apply unary minus operator."),
        }
    }

    // PlaceholderParser.cpp:334-347  expr unary_integer(const Iterator start_pos) const
    pub fn unary_integer(&self) -> Result<Expr> {
        match self.data {
            // PlaceholderParser.cpp:337-338  case TYPE_INT: return expr(i());
            Data::Int(i) => Ok(Expr::from_int(i)),
            // PlaceholderParser.cpp:339-340  case TYPE_DOUBLE: return expr(static_cast<int>(d()));
            Data::Double(d) => Ok(Expr::from_int(d as i32)),
            // PlaceholderParser.cpp:341-342  default: throw "Cannot convert to integer."
            _ => self.throw_exception("Cannot convert to integer."),
        }
    }

    // PlaceholderParser.cpp:349-362  expr round(const Iterator start_pos) const
    pub fn round(&self) -> Result<Expr> {
        match self.data {
            // PlaceholderParser.cpp:352-353  case TYPE_INT: return expr(i());
            Data::Int(i) => Ok(Expr::from_int(i)),
            // PlaceholderParser.cpp:354-355  case TYPE_DOUBLE: return expr(static_cast<int>(std::round(d())));
            Data::Double(d) => Ok(Expr::from_int(d.round() as i32)),
            // PlaceholderParser.cpp:356-357  default: throw "Cannot round a non-numeric value."
            _ => self.throw_exception("Cannot round a non-numeric value."),
        }
    }

    // PlaceholderParser.cpp:364-377  expr floor(const Iterator start_pos) const
    pub fn floor(&self) -> Result<Expr> {
        match self.data {
            // PlaceholderParser.cpp:367-368  case TYPE_INT: return expr(i());
            Data::Int(i) => Ok(Expr::from_int(i)),
            // PlaceholderParser.cpp:369-370  case TYPE_DOUBLE: return expr(static_cast<int>(std::floor(d())));
            Data::Double(d) => Ok(Expr::from_int(d.floor() as i32)),
            // PlaceholderParser.cpp:371-372  default: throw "Cannot floor a non-numeric value."
            _ => self.throw_exception("Cannot floor a non-numeric value."),
        }
    }

    // PlaceholderParser.cpp:379-392  expr ceil(const Iterator start_pos) const
    pub fn ceil(&self) -> Result<Expr> {
        match self.data {
            // PlaceholderParser.cpp:382-383  case TYPE_INT: return expr(i());
            Data::Int(i) => Ok(Expr::from_int(i)),
            // PlaceholderParser.cpp:384-385  case TYPE_DOUBLE: return expr(static_cast<int>(std::ceil(d())));
            Data::Double(d) => Ok(Expr::from_int(d.ceil() as i32)),
            // PlaceholderParser.cpp:386-387  default: throw "Cannot ceil a non-numeric value."
            _ => self.throw_exception("Cannot ceil a non-numeric value."),
        }
    }

    // PlaceholderParser.cpp:394-405  expr unary_not(const Iterator start_pos) const
    pub fn unary_not(&self) -> Result<Expr> {
        match self.data {
            // PlaceholderParser.cpp:397-398  case TYPE_BOOL: return expr(! b());
            Data::Bool(b) => Ok(Expr::from_bool(!b)),
            // PlaceholderParser.cpp:399-400  default: throw "Cannot apply a not operator."
            _ => self.throw_exception("Cannot apply a not operator."),
        }
    }

    // PlaceholderParser.cpp:407-429  expr &operator+=(const expr &rhs)
    pub fn add_assign(&mut self, rhs: &Expr) -> Result<()> {
        if self.type_() == Type::TypeString {
            // PlaceholderParser.cpp:409-411  Convert the right hand side to string and append.
            if let Data::Str(s) = &mut self.data {
                s.push_str(&rhs.to_string());
            }
        } else if rhs.type_() == Type::TypeString {
            // PlaceholderParser.cpp:412-415  Convert the left hand side to string, append rhs.
            let combined = self.to_string() + rhs.s();
            self.data = Data::Str(combined);
        } else {
            // PlaceholderParser.cpp:416-426
            let err_msg = "Cannot add non-numeric types.";
            self.throw_if_not_numeric(err_msg)?;
            rhs.throw_if_not_numeric(err_msg)?;
            if self.type_() == Type::TypeDouble || rhs.type_() == Type::TypeDouble {
                let d = self.as_d() + rhs.as_d();
                self.data = Data::Double(d);
            } else {
                // PlaceholderParser.cpp:425  this->data.i += rhs.i();
                self.data = Data::Int(self.i().wrapping_add(rhs.i()));
            }
        }
        Ok(())
    }

    // PlaceholderParser.cpp:431-444  expr &operator-=(const expr &rhs)
    pub fn sub_assign(&mut self, rhs: &Expr) -> Result<()> {
        let err_msg = "Cannot subtract non-numeric types.";
        self.throw_if_not_numeric(err_msg)?;
        rhs.throw_if_not_numeric(err_msg)?;
        if self.type_() == Type::TypeDouble || rhs.type_() == Type::TypeDouble {
            let d = self.as_d() - rhs.as_d();
            self.data = Data::Double(d);
        } else {
            // PlaceholderParser.cpp:441  this->data.i -= rhs.i();
            self.data = Data::Int(self.i().wrapping_sub(rhs.i()));
        }
        Ok(())
    }

    // PlaceholderParser.cpp:446-459  expr &operator*=(const expr &rhs)
    pub fn mul_assign(&mut self, rhs: &Expr) -> Result<()> {
        let err_msg = "Cannot multiply with non-numeric type.";
        self.throw_if_not_numeric(err_msg)?;
        rhs.throw_if_not_numeric(err_msg)?;
        if self.type_() == Type::TypeDouble || rhs.type_() == Type::TypeDouble {
            let d = self.as_d() * rhs.as_d();
            self.data = Data::Double(d);
        } else {
            // PlaceholderParser.cpp:456  this->data.i *= rhs.i();
            self.data = Data::Int(self.i().wrapping_mul(rhs.i()));
        }
        Ok(())
    }

    // PlaceholderParser.cpp:461-475  expr &operator/=(const expr &rhs)
    pub fn div_assign(&mut self, rhs: &Expr) -> Result<()> {
        self.throw_if_not_numeric("Cannot divide a non-numeric type.")?;
        rhs.throw_if_not_numeric("Cannot divide with a non-numeric type.")?;
        // PlaceholderParser.cpp:465-466
        let is_zero = if rhs.type_() == Type::TypeInt {
            rhs.i() == 0
        } else {
            rhs.d() == 0.0
        };
        if is_zero {
            return rhs.throw_exception_unit("Division by zero");
        }
        if self.type_() == Type::TypeDouble || rhs.type_() == Type::TypeDouble {
            let d = self.as_d() / rhs.as_d();
            self.data = Data::Double(d);
        } else {
            // PlaceholderParser.cpp:472  this->data.i /= rhs.i();
            self.data = Data::Int(self.i().wrapping_div(rhs.i()));
        }
        Ok(())
    }

    // PlaceholderParser.cpp:477-491  expr &operator%=(const expr &rhs)
    pub fn rem_assign(&mut self, rhs: &Expr) -> Result<()> {
        self.throw_if_not_numeric("Cannot divide a non-numeric type.")?;
        rhs.throw_if_not_numeric("Cannot divide with a non-numeric type.")?;
        // PlaceholderParser.cpp:481-482
        let is_zero = if rhs.type_() == Type::TypeInt {
            rhs.i() == 0
        } else {
            rhs.d() == 0.0
        };
        if is_zero {
            return rhs.throw_exception_unit("Division by zero");
        }
        if self.type_() == Type::TypeDouble || rhs.type_() == Type::TypeDouble {
            // PlaceholderParser.cpp:484  double d = std::fmod(this->as_d(), rhs.as_d());
            let d = self.as_d() % rhs.as_d();
            self.data = Data::Double(d);
        } else {
            // PlaceholderParser.cpp:488  this->data.i %= rhs.i();
            self.data = Data::Int(self.i().wrapping_rem(rhs.i()));
        }
        Ok(())
    }

    // PlaceholderParser.cpp:493-496  static void to_string2(expr &self, std::string &out)
    pub fn to_string2(&self) -> String {
        self.to_string()
    }

    // PlaceholderParser.cpp:498-503  static void evaluate_boolean(expr &self, bool &out)
    pub fn evaluate_boolean(&self) -> Result<bool> {
        if self.type_() != Type::TypeBool {
            return self.throw_exception_unit("Not a boolean expression").map(|_| false);
        }
        Ok(self.b())
    }

    // PlaceholderParser.cpp:505-510  static void evaluate_boolean_to_string(expr &self, std::string &out)
    pub fn evaluate_boolean_to_string(&self) -> Result<String> {
        if self.type_() != Type::TypeBool {
            return self
                .throw_exception_unit("Not a boolean expression")
                .map(|_| String::new());
        }
        Ok(if self.b() {
            "true".to_string()
        } else {
            "false".to_string()
        })
    }

    // PlaceholderParser.cpp:512-550  static void compare_op(expr &lhs, expr &rhs, char op, bool invert)
    // Is lhs<op>rhs? Store the result into lhs.
    pub fn compare_op(lhs: &mut Expr, rhs: &Expr, op: char, invert: bool) -> Result<()> {
        let mut value = false;
        let lhs_t = lhs.type_();
        let rhs_t = rhs.type_();
        if (lhs_t == Type::TypeInt || lhs_t == Type::TypeDouble)
            && (rhs_t == Type::TypeInt || rhs_t == Type::TypeDouble)
        {
            // PlaceholderParser.cpp:518  Both types are numeric.
            match op {
                // PlaceholderParser.cpp:520-523  case '=':
                '=' => {
                    value = if lhs_t == Type::TypeDouble || rhs_t == Type::TypeDouble {
                        (lhs.as_d() - rhs.as_d()).abs() < 1e-8
                    } else {
                        lhs.i() == rhs.i()
                    };
                }
                // PlaceholderParser.cpp:524-527  case '<':
                '<' => {
                    value = if lhs_t == Type::TypeDouble || rhs_t == Type::TypeDouble {
                        lhs.as_d() < rhs.as_d()
                    } else {
                        lhs.i() < rhs.i()
                    };
                }
                // PlaceholderParser.cpp:528-532  case '>': default:
                _ => {
                    value = if lhs_t == Type::TypeDouble || rhs_t == Type::TypeDouble {
                        lhs.as_d() > rhs.as_d()
                    } else {
                        lhs.i() > rhs.i()
                    };
                }
            }
        } else if lhs_t == Type::TypeBool && rhs_t == Type::TypeBool {
            // PlaceholderParser.cpp:534-539  Both type are bool.
            if op != '=' {
                // PlaceholderParser.cpp:536-538  throw "Cannot compare the types."
                return lhs.throw_exception_unit("Cannot compare the types.");
            }
            value = lhs.b() == rhs.b();
        } else if lhs_t == Type::TypeString || rhs_t == Type::TypeString {
            // PlaceholderParser.cpp:540-543  One type is string, the other could be converted to string.
            value = if op == '=' {
                lhs.to_string() == rhs.to_string()
            } else if op == '<' {
                lhs.to_string() < rhs.to_string()
            } else {
                lhs.to_string() > rhs.to_string()
            };
        } else {
            // PlaceholderParser.cpp:544-546  throw "Cannot compare the types."
            return lhs.throw_exception_unit("Cannot compare the types.");
        }
        // PlaceholderParser.cpp:548-549
        lhs.data = Data::Bool(if invert { !value } else { value });
        Ok(())
    }

    // PlaceholderParser.cpp:552-557  Compare operators, store the result into lhs.
    pub fn equal(lhs: &mut Expr, rhs: &Expr) -> Result<()> {
        Expr::compare_op(lhs, rhs, '=', false)
    }
    pub fn not_equal(lhs: &mut Expr, rhs: &Expr) -> Result<()> {
        Expr::compare_op(lhs, rhs, '=', true)
    }
    pub fn lower(lhs: &mut Expr, rhs: &Expr) -> Result<()> {
        Expr::compare_op(lhs, rhs, '<', false)
    }
    pub fn greater(lhs: &mut Expr, rhs: &Expr) -> Result<()> {
        Expr::compare_op(lhs, rhs, '>', false)
    }
    pub fn leq(lhs: &mut Expr, rhs: &Expr) -> Result<()> {
        Expr::compare_op(lhs, rhs, '>', true)
    }
    pub fn geq(lhs: &mut Expr, rhs: &Expr) -> Result<()> {
        Expr::compare_op(lhs, rhs, '<', true)
    }

    // PlaceholderParser.cpp:559-563  static void throw_if_not_numeric(const expr &param)
    pub fn throw_if_not_numeric_static(param: &Expr) -> Result<()> {
        let err_msg = "Not a numeric type.";
        param.throw_if_not_numeric(err_msg)
    }

    // PlaceholderParser.cpp:570-593  static void function_2params(expr &param1, expr &param2, Function2ParamsType fun)
    // Store the result into param1.
    fn function_2params(param1: &mut Expr, param2: &Expr, fun: Function2ParamsType) -> Result<()> {
        Expr::throw_if_not_numeric_static(param1)?;
        Expr::throw_if_not_numeric_static(param2)?;
        if param1.type_() == Type::TypeDouble || param2.type_() == Type::TypeDouble {
            let d = match fun {
                // PlaceholderParser.cpp:577  case FUNCTION_MIN
                Function2ParamsType::FunctionMin => param1.as_d().min(param2.as_d()),
                // PlaceholderParser.cpp:578  case FUNCTION_MAX
                Function2ParamsType::FunctionMax => param1.as_d().max(param2.as_d()),
            };
            param1.data = Data::Double(d);
        } else {
            let i = match fun {
                // PlaceholderParser.cpp:586  case FUNCTION_MIN
                Function2ParamsType::FunctionMin => param1.as_i().min(param2.as_i()),
                // PlaceholderParser.cpp:587  case FUNCTION_MAX
                Function2ParamsType::FunctionMax => param1.as_i().max(param2.as_i()),
            };
            param1.data = Data::Int(i);
        }
        Ok(())
    }

    // PlaceholderParser.cpp:595-596  Store the result into param1.
    pub fn min(param1: &mut Expr, param2: &Expr) -> Result<()> {
        Expr::function_2params(param1, param2, Function2ParamsType::FunctionMin)
    }
    pub fn max(param1: &mut Expr, param2: &Expr) -> Result<()> {
        Expr::function_2params(param1, param2, Function2ParamsType::FunctionMax)
    }

    // PlaceholderParser.cpp:598-610  static void random(expr &param1, expr &param2, std::mt19937 &rng)
    // Store the result into param1.
    pub fn random<R: Rng + ?Sized>(param1: &mut Expr, param2: &Expr, rng: &mut R) -> Result<()> {
        Expr::throw_if_not_numeric_static(param1)?;
        Expr::throw_if_not_numeric_static(param2)?;
        if param1.type_() == Type::TypeDouble || param2.type_() == Type::TypeDouble {
            // PlaceholderParser.cpp:604  std::uniform_real_distribution<>(as_d(), as_d())(rng)
            let v = rng.gen_range(param1.as_d()..param2.as_d());
            param1.data = Data::Double(v);
        } else {
            // PlaceholderParser.cpp:607  std::uniform_int_distribution<>(as_i(), as_i())(rng)
            // C++ uniform_int_distribution is inclusive on both ends.
            let v = rng.gen_range(param1.as_i()..=param2.as_i());
            param1.data = Data::Int(v);
        }
        Ok(())
    }

    // PlaceholderParser.cpp:612-634  template<bool leading_zeros> static void digits(param1, param2, param3)
    // Store the result into param1. param3 is optional.
    pub fn digits(
        param1: &mut Expr,
        param2: &Expr,
        param3: &Expr,
        leading_zeros: bool,
    ) -> Result<()> {
        Expr::throw_if_not_numeric_static(param1)?;
        // PlaceholderParser.cpp:618-619
        if param2.type_() != Type::TypeInt {
            return param2.throw_exception_unit("digits: second parameter must be integer");
        }
        // PlaceholderParser.cpp:620  bool has_decimals = param3.type != TYPE_EMPTY;
        let has_decimals = param3.type_() != Type::TypeEmpty;
        // PlaceholderParser.cpp:621-622
        if has_decimals && param3.type_() != Type::TypeInt {
            return param3.throw_exception_unit("digits: third parameter must be integer");
        }

        // PlaceholderParser.cpp:625  int ndigits = std::clamp(param2.as_i(), 0, 64);
        let ndigits = param2.as_i().clamp(0, 64);
        let buf = if has_decimals {
            // PlaceholderParser.cpp:626-629  Format as double.
            //   int decimals = std::clamp(param3.as_i(), 0, 64);
            //   sprintf(buf, leading_zeros ? "%0*.*lf" : "%*.*lf", ndigits, decimals, param1.as_d());
            let decimals = param3.as_i().clamp(0, 64);
            format_printf_f(param1.as_d(), ndigits, decimals, leading_zeros)
        } else {
            // PlaceholderParser.cpp:630-632  Format as int.
            //   sprintf(buf, leading_zeros ? "%0*d" : "%*d", ndigits, param1.as_i_rounded());
            format_printf_d(param1.as_i_rounded(), ndigits, leading_zeros)
        };
        // PlaceholderParser.cpp:633  param1.set_s(buf);
        param1.set_s(buf);
        Ok(())
    }

    // PlaceholderParser.cpp:636-661  regex_op / regex_matches / regex_doesnt_match
    //
    // BLOCKED: requires a regex engine (C++ uses boost::regex via
    // SLIC3R_REGEX_NAMESPACE). The `regex` crate is NOT present in this
    // crate's Cargo.toml and adding a backend is out of scope for this port
    // (must not add deps unilaterally). The control flow is preserved here as
    // a faithful skeleton that errors on the regex compile/match step instead
    // of producing a fake boolean result.
    pub fn regex_op(lhs: &mut Expr, rhs: &str, op: char) -> Result<()> {
        // PlaceholderParser.cpp:638-644
        if lhs.type_() != Type::TypeString {
            return lhs.throw_exception_unit("Left hand side of a regex match must be a string.");
        }
        // PlaceholderParser.cpp:646  std::string pattern(++rhs.begin(), --rhs.end());
        // (strip the enclosing '/' delimiters)
        let _pattern: &str = {
            let bytes = rhs.as_bytes();
            if bytes.len() >= 2 {
                &rhs[1..rhs.len() - 1]
            } else {
                rhs
            }
        };
        let _ = op;
        // No regex backend available: report as a runtime error rather than
        // faking a match result.
        Err(Error::ParseError(
            "Regular expression matching is not supported (no regex backend ported)".to_string(),
        ))
    }

    // PlaceholderParser.cpp:660-661
    pub fn regex_matches(lhs: &mut Expr, rhs: &str) -> Result<()> {
        Expr::regex_op(lhs, rhs, '=')
    }
    pub fn regex_doesnt_match(lhs: &mut Expr, rhs: &str) -> Result<()> {
        Expr::regex_op(lhs, rhs, '!')
    }

    // PlaceholderParser.cpp:663-674  static void logical_op(expr &lhs, expr &rhs, char op)
    pub fn logical_op(lhs: &mut Expr, rhs: &Expr, op: char) -> Result<()> {
        let value;
        // PlaceholderParser.cpp:666-671
        if lhs.type_() == Type::TypeBool && rhs.type_() == Type::TypeBool {
            value = if op == '|' {
                lhs.b() || rhs.b()
            } else {
                lhs.b() && rhs.b()
            };
        } else {
            return lhs
                .throw_exception_unit("Cannot apply logical operation to non-boolean operators.");
        }
        // PlaceholderParser.cpp:672-673
        lhs.data = Data::Bool(value);
        Ok(())
    }
    // PlaceholderParser.cpp:675-676
    pub fn logical_or(lhs: &mut Expr, rhs: &Expr) -> Result<()> {
        Expr::logical_op(lhs, rhs, '|')
    }
    pub fn logical_and(lhs: &mut Expr, rhs: &Expr) -> Result<()> {
        Expr::logical_op(lhs, rhs, '&')
    }

    // PlaceholderParser.cpp:678-686  static void ternary_op(expr &lhs, expr &rhs1, expr &rhs2)
    pub fn ternary_op(lhs: &mut Expr, rhs1: Expr, rhs2: Expr) -> Result<()> {
        // PlaceholderParser.cpp:680-681
        if lhs.type_() != Type::TypeBool {
            return lhs.throw_exception_unit("Not a boolean expression");
        }
        // PlaceholderParser.cpp:682-685
        if lhs.b() {
            *lhs = rhs1;
        } else {
            *lhs = rhs2;
        }
        Ok(())
    }

    // PlaceholderParser.cpp:688-694  static void set_if(bool &cond, bool &not_yet_consumed, str_in, str_out)
    pub fn set_if(cond: bool, not_yet_consumed: &mut bool, str_in: &str, str_out: &mut String) {
        if cond && *not_yet_consumed {
            *str_out = str_in.to_string();
            *not_yet_consumed = false;
        }
    }

    // PlaceholderParser.cpp:696-700  void throw_exception(const char *message) const
    // The C++ throws a qi::expectation_failure carrying the source it_range.
    // Without the parser iterators, we surface the message via ParseError.
    fn throw_exception<T>(&self, message: &str) -> Result<T> {
        Err(Error::ParseError(format!("*{}", message)))
    }

    fn throw_exception_unit(&self, message: &str) -> Result<()> {
        Err(Error::ParseError(format!("*{}", message)))
    }

    // PlaceholderParser.cpp:702-706  void throw_if_not_numeric(const char *message) const
    fn throw_if_not_numeric(&self, message: &str) -> Result<()> {
        if self.type_() != Type::TypeInt && self.type_() != Type::TypeDouble {
            self.throw_exception_unit(message)
        } else {
            Ok(())
        }
    }
}

impl Default for Expr {
    fn default() -> Self {
        Expr::new()
    }
}

// PlaceholderParser.cpp:565-568  enum Function2ParamsType { FUNCTION_MIN, FUNCTION_MAX };
#[derive(Clone, Copy)]
enum Function2ParamsType {
    FunctionMin,
    FunctionMax,
}

/// `expr::to_string` double path (PlaceholderParser.cpp:278-282): mimics the
/// C++ `std::ostringstream << double`, which uses `%g`-like default formatting
/// (6 significant digits, no trailing zeros, no forced decimal point).
fn format_double_ostream(d: f64) -> String {
    // std::ostringstream default float formatting == printf "%g" with default
    // precision of 6 significant digits.
    format_printf_g(d, 6)
}

/// printf `%g` with the given precision (number of significant digits).
/// Used to match C++ ostringstream default double formatting.
fn format_printf_g(value: f64, precision: usize) -> String {
    if value == 0.0 {
        return "0".to_string();
    }
    let precision = if precision == 0 { 1 } else { precision };
    let exp = value.abs().log10().floor() as i32;
    // %g uses %e if exp < -4 or exp >= precision, otherwise %f.
    if exp < -4 || exp >= precision as i32 {
        // Scientific notation with (precision-1) digits after the point,
        // trailing zeros removed.
        let mantissa_prec = precision - 1;
        let formatted = format!("{:.*e}", mantissa_prec, value);
        // Rust formats exponent as e.g. "1.5e2"; C++ uses "1.5e+02".
        normalize_exponent(&trim_g_mantissa(&formatted))
    } else {
        let decimals = (precision as i32 - 1 - exp).max(0) as usize;
        let formatted = format!("{:.*}", decimals, value);
        trim_trailing_zeros(&formatted)
    }
}

fn trim_g_mantissa(s: &str) -> String {
    if let Some(epos) = s.find(['e', 'E']) {
        let (mant, exp) = s.split_at(epos);
        let mant = trim_trailing_zeros(mant);
        format!("{}{}", mant, exp)
    } else {
        trim_trailing_zeros(s)
    }
}

fn trim_trailing_zeros(s: &str) -> String {
    if s.contains('.') {
        let t = s.trim_end_matches('0');
        let t = t.trim_end_matches('.');
        t.to_string()
    } else {
        s.to_string()
    }
}

fn normalize_exponent(s: &str) -> String {
    // Convert Rust "1.5e2" / "1.5e-2" to C++ "1.5e+02" / "1.5e-02".
    if let Some(epos) = s.find(['e', 'E']) {
        let (mant, exp) = s.split_at(epos);
        let exp = &exp[1..];
        let (sign, digits) = if let Some(rest) = exp.strip_prefix('-') {
            ('-', rest)
        } else if let Some(rest) = exp.strip_prefix('+') {
            ('+', rest)
        } else {
            ('+', exp)
        };
        let digits = if digits.len() < 2 {
            format!("{:0>2}", digits)
        } else {
            digits.to_string()
        };
        format!("{}e{}{}", mant, sign, digits)
    } else {
        s.to_string()
    }
}

/// printf `"%0*.*lf"` / `"%*.*lf"` (PlaceholderParser.cpp:629).
/// width = ndigits (total field width), prec = decimals, leading_zeros selects
/// the `0` flag (zero-pad) vs space-pad.
fn format_printf_f(value: f64, width: i32, prec: i32, leading_zeros: bool) -> String {
    let body = format!("{:.*}", prec.max(0) as usize, value);
    pad_to_width(body, width, leading_zeros, value < 0.0)
}

/// printf `"%0*d"` / `"%*d"` (PlaceholderParser.cpp:632).
fn format_printf_d(value: i32, width: i32, leading_zeros: bool) -> String {
    let body = value.to_string();
    pad_to_width(body, width, leading_zeros, value < 0)
}

/// Apply printf field-width padding. With the `0` flag the zero padding goes
/// between the sign and the digits; with space padding the whole field is
/// right-justified.
fn pad_to_width(body: String, width: i32, leading_zeros: bool, negative: bool) -> String {
    let width = width.max(0) as usize;
    if body.len() >= width {
        return body;
    }
    let pad = width - body.len();
    if leading_zeros {
        if negative {
            // body starts with '-': insert zeros after the sign.
            let (sign, rest) = body.split_at(1);
            format!("{}{}{}", sign, "0".repeat(pad), rest)
        } else {
            format!("{}{}", "0".repeat(pad), body)
        }
    } else {
        format!("{}{}", " ".repeat(pad), body)
    }
}

// =====================================================================
// update_timestamp time formatting (PlaceholderParser.cpp:80-103)
// ---------------------------------------------------------------------
// The C++ writes "timestamp"/"year"/"month"/"day"/"hour"/"minute"/"second"
// into a DynamicConfig. The DynamicConfig/ConfigOption system is NOT ported
// in this crate, so the config writes are blocked. The pure, faithful part --
// computing the formatted timestamp string and the integer fields from the
// local time -- is ported here and returned as a struct, so a future
// DynamicConfig can be populated identically.
// =====================================================================

/// Result of [`update_timestamp_values`], matching the six (+ timestamp)
/// values the C++ writes into the config (PlaceholderParser.cpp:95-102).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TimestampValues {
    pub timestamp: String, // PlaceholderParser.cpp:95  "timestamp"
    pub year: i32,         // PlaceholderParser.cpp:97  1900 + tm_year
    pub month: i32,        // PlaceholderParser.cpp:98  1 + tm_mon
    pub day: i32,          // PlaceholderParser.cpp:99  tm_mday
    pub hour: i32,         // PlaceholderParser.cpp:100 tm_hour
    pub minute: i32,       // PlaceholderParser.cpp:101 tm_min
    pub second: i32,       // PlaceholderParser.cpp:102 tm_sec
}

/// Faithful port of the timestamp computation in
/// `PlaceholderParser::update_timestamp` (PlaceholderParser.cpp:80-103).
/// The `localtime` call is provided via `chrono::Local::now()`.
///
/// NOTE: the C++ method's final step -- `config.set_key_value(...)` into a
/// `DynamicConfig` -- is BLOCKED (no DynamicConfig in this crate). This
/// function returns the computed values so the caller may store them.
pub fn update_timestamp_values() -> TimestampValues {
    use chrono::{Datelike, Local, Timelike};
    // PlaceholderParser.cpp:82-84  time(&rawtime); localtime(&rawtime);
    let timeinfo = Local::now();

    // PlaceholderParser.cpp:86-96  build the "timestamp" string:
    //   YYYYMMDD-HHMMSS, with the date part not zero-padded for the year and
    //   2-wide zero-padded month/day, then '-', then 2-wide hour/min/sec.
    let year = timeinfo.year(); // PlaceholderParser.cpp:88  1900 + tm_year
    let month = timeinfo.month() as i32; // PlaceholderParser.cpp:89  1 + tm_mon
    let day = timeinfo.day() as i32; // PlaceholderParser.cpp:90  tm_mday
    let hour = timeinfo.hour() as i32; // PlaceholderParser.cpp:92  tm_hour
    let minute = timeinfo.minute() as i32; // PlaceholderParser.cpp:93  tm_min
    let second = timeinfo.second() as i32; // PlaceholderParser.cpp:94  tm_sec

    // PlaceholderParser.cpp:88-94
    let timestamp = format!(
        "{}{:02}{:02}-{:02}{:02}{:02}",
        year, month, day, hour, minute, second
    );

    TimestampValues {
        timestamp,
        year,
        month,
        day,
        hour,
        minute,
        second,
    }
}

// =====================================================================
// PlaceholderParser  (PlaceholderParser.hpp:13-70, PlaceholderParser.cpp:73-162,1534-1555)
// =====================================================================
//
// BLOCKED -- the entire PlaceholderParser public class depends on subsystems
// that are not yet present in this crate (see module docs):
//
//   * PlaceholderParser ctor (PlaceholderParser.cpp:73-78): sets "version",
//     apply_env_variables(), update_timestamp() -- all write into a
//     DynamicConfig m_config.
//   * config_diff (PlaceholderParser.cpp:113-120) + opts_equal
//     (PlaceholderParser.cpp:105-111): compare ConfigOption values across
//     DynamicConfigs.
//   * apply_config (PlaceholderParser.cpp:128-138, 146-149) / apply_only
//     (140-144): clone ConfigOptions into m_config, m_config += rhs.
//   * apply_env_variables (PlaceholderParser.cpp:151-162): reads `environ`,
//     not wasm-safe and depends on m_config.
//   * set(...) overloads (PlaceholderParser.hpp:40-46): create ConfigOptionXxx.
//   * process (PlaceholderParser.cpp:1534-1543) /
//     evaluate_boolean_expression (1547-1554) / process_macro (1509-1532):
//     drive the Boost.Spirit `macro_processor` grammar (MyContext, the entire
//     grammar at PlaceholderParser.cpp:725-1506), which depends on
//     DynamicConfig/ConfigOption, print_config_def, and Flow::extrusion_width
//     overloads that do not exist here.
//
// These will be portable once a faithful DynamicConfig/ConfigOption type
// system (and a recursive-descent replacement for the Qi grammar) lands in the
// crate. The expression evaluator above (`Expr`) is the reusable core for that
// future grammar.

#[cfg(test)]
mod tests {
    use super::*;

    // Validate the faithful expr evaluation core.
    #[test]
    fn test_expr_arithmetic_int() {
        // PlaceholderParser.cpp:407-475  int + - * / %
        let mut a = Expr::from_int(7);
        a.add_assign(&Expr::from_int(3)).unwrap();
        assert_eq!(a.i(), 10);
        a.sub_assign(&Expr::from_int(4)).unwrap();
        assert_eq!(a.i(), 6);
        a.mul_assign(&Expr::from_int(3)).unwrap();
        assert_eq!(a.i(), 18);
        a.div_assign(&Expr::from_int(4)).unwrap();
        // integer division truncates toward zero like C++
        assert_eq!(a.i(), 4);
        a.rem_assign(&Expr::from_int(3)).unwrap();
        assert_eq!(a.i(), 1);
    }

    #[test]
    fn test_expr_arithmetic_promotes_double() {
        // PlaceholderParser.cpp:420-423  int op double -> double
        let mut a = Expr::from_int(7);
        a.add_assign(&Expr::from_double(0.5)).unwrap();
        assert_eq!(a.type_(), Type::TypeDouble);
        assert_eq!(a.d(), 7.5);
    }

    #[test]
    fn test_expr_string_concat() {
        // PlaceholderParser.cpp:409-415
        let mut a = Expr::from_string("a");
        a.add_assign(&Expr::from_int(3)).unwrap();
        assert_eq!(a.s(), "a3");
        let mut b = Expr::from_int(3);
        b.add_assign(&Expr::from_string("x")).unwrap();
        assert_eq!(b.s(), "3x");
    }

    #[test]
    fn test_expr_division_by_zero() {
        // PlaceholderParser.cpp:465-466
        let mut a = Expr::from_int(1);
        assert!(a.div_assign(&Expr::from_int(0)).is_err());
        let mut b = Expr::from_double(1.0);
        assert!(b.div_assign(&Expr::from_double(0.0)).is_err());
    }

    #[test]
    fn test_expr_compare() {
        // PlaceholderParser.cpp:512-557
        let mut a = Expr::from_int(3);
        Expr::lower(&mut a, &Expr::from_int(4)).unwrap();
        assert_eq!(a.type_(), Type::TypeBool);
        assert!(a.b());

        let mut e = Expr::from_double(1.0);
        Expr::equal(&mut e, &Expr::from_double(1.0 + 1e-12)).unwrap();
        assert!(e.b()); // within 1e-8 tolerance
    }

    #[test]
    fn test_expr_unary() {
        // PlaceholderParser.cpp:319-405
        assert_eq!(Expr::from_int(5).unary_minus().unwrap().i(), -5);
        assert_eq!(Expr::from_double(2.7).round().unwrap().i(), 3);
        assert_eq!(Expr::from_double(2.7).floor().unwrap().i(), 2);
        assert_eq!(Expr::from_double(2.1).ceil().unwrap().i(), 3);
        assert_eq!(Expr::from_double(2.9).unary_integer().unwrap().i(), 2);
        assert!(Expr::from_bool(true).unary_not().unwrap().b() == false);
    }

    #[test]
    fn test_expr_min_max() {
        // PlaceholderParser.cpp:595-596
        let mut a = Expr::from_int(3);
        Expr::min(&mut a, &Expr::from_int(5)).unwrap();
        assert_eq!(a.i(), 3);
        let mut b = Expr::from_int(3);
        Expr::max(&mut b, &Expr::from_int(5)).unwrap();
        assert_eq!(b.i(), 5);
    }

    #[test]
    fn test_expr_logical_ternary() {
        // PlaceholderParser.cpp:663-686
        let mut a = Expr::from_bool(true);
        Expr::logical_and(&mut a, &Expr::from_bool(false)).unwrap();
        assert!(!a.b());
        let mut c = Expr::from_bool(true);
        Expr::ternary_op(&mut c, Expr::from_int(1), Expr::from_int(2)).unwrap();
        assert_eq!(c.i(), 1);
    }

    #[test]
    fn test_expr_digits() {
        // PlaceholderParser.cpp:612-634
        // zdigits(5, 3) -> "00005", digits(5,3) -> "    5"
        let mut a = Expr::from_int(5);
        Expr::digits(&mut a, &Expr::from_int(3), &Expr::new(), true).unwrap();
        assert_eq!(a.s(), "005");
        let mut b = Expr::from_int(5);
        Expr::digits(&mut b, &Expr::from_int(3), &Expr::new(), false).unwrap();
        assert_eq!(b.s(), "  5");
        // digits with decimals: digits(3.14159, 6, 2) -> "  3.14"
        let mut c = Expr::from_double(3.14159);
        Expr::digits(&mut c, &Expr::from_int(6), &Expr::from_int(2), false).unwrap();
        assert_eq!(c.s(), "  3.14");
    }

    #[test]
    fn test_expr_to_string_double() {
        // PlaceholderParser.cpp:278-282  ostringstream default formatting
        assert_eq!(Expr::from_double(7.5).to_string(), "7.5");
        assert_eq!(Expr::from_double(7.0).to_string(), "7");
        assert_eq!(Expr::from_int(42).to_string(), "42");
        assert_eq!(Expr::from_bool(true).to_string(), "true");
    }
}
