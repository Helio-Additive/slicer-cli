//! Port of BambuStudio `src/libslic3r/PlaceholderParser.{hpp,cpp}` (pin 56a57d0).
//!
//! The C++ evaluates custom G-code templates with a Boost.Spirit Qi grammar
//! (`client::macro_processor<Iterator>`, PlaceholderParser.cpp:1183-1513) over the
//! `client::expr<Iterator>` value type (PlaceholderParser.cpp:190-707) and the
//! `client::MyContext` symbol resolver (PlaceholderParser.cpp:725-1073).
//!
//! FIDELITY-NOTE: the Spirit grammar is replaced by the hand-written parser
//! [`MacroProcessor`]. Each grammar rule is one method of the same name, and each
//! method reproduces what the Qi components of the rule do with the iterator:
//! primitives (`lit`, `eoi`, `eps`, `iter_pos`, `raw`, `lexeme`, `symbols`, the
//! number parsers) pre-skip white space (`spirit_encoding::space`, iso8859_1) on the
//! iterator they are handed, also when they then fail; sequences (`>>`, `>`) work on
//! a copy and commit it on success; an alternative hands the same iterator to each
//! branch; a kleene star commits its copy after the loop; `distinct` (`kw[...]`)
//! works on a copy. A failure after the first element of an expectation sequence
//! (`a > b`) throws `qi::expectation_failure` at the current iterator with the
//! `what()` tag of the failed component (rule name, "literal-char", "distinct",
//! "eoi", "alternative", "any-char"), which `start`'s `on_error` handler turns into
//! the "Parsing error at line ..." text of `MyContext::process_error_message`. So
//! the parser accepts and rejects the same inputs as the grammar, evaluates the
//! same semantic actions in the same order (every `{if}` branch and both arms of
//! `?:` are evaluated, `and`/`or` do not short-circuit), and reports the same
//! error texts at the same columns.
//!
//! `int` of the C++ `expr` stays `i32`; `size_t` is `usize`. Iterators are byte
//! offsets into the template.
//!
//! The grammar at the pin is the older Slic3r one: it has no local variables, `one_of`,
//! `interpolate_table` or `{switch}` (later PrusaSlicer additions), and neither has this port.
//! `tests/placeholder_parser_native_parity.rs` compares it with the C++ over the custom G-code of
//! every BBL profile and generated templates.

use crate::config::{
    ConfigOption, ConfigOptionBool, ConfigOptionDef, ConfigOptionFloat, ConfigOptionInt, ConfigOptionResolver,
    ConfigOptionString, ConfigOptionStrings, ConfigOptionType, ConfigurationError, DynamicConfig, FloatOrPercent,
};
use crate::flow::FlowRole;
use crate::geometry::Vec2d;
use crate::print_config::config_def::{print_config_def, DynamicPrintConfig};
use std::cell::{Cell, RefCell};

// PlaceholderParser.cpp:70  #define L(s) (s)
// PlaceholderParser.cpp:71  #define _(s) Slic3r::I18N::translate(s)

// =====================================================================
// Exceptions
// =====================================================================

/// What leaves `PlaceholderParser::process()` / `evaluate_boolean_expression()`
/// as a C++ exception: the class of the exception and its `what()`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlaceholderParserException {
    /// `Slic3r::PlaceholderParserError` (Exception.hpp:26), thrown by `process_macro`
    /// (PlaceholderParser.cpp:1533-1537) with the text of `process_error_message`.
    PlaceholderParserError(String),
    /// `Slic3r::FlowErrorMissingVariable` (Flow.hpp), thrown by `Flow::extrusion_width`
    /// (Flow.cpp:61-64) from inside a semantic action; not caught by the grammar.
    FlowErrorMissingVariable(String),
    /// `Slic3r::RuntimeError`, thrown by `opt_key_to_flow_role` (Flow.cpp:58).
    RuntimeError(String),
    /// `Slic3r::ConfigurationError` family, thrown by the ConfigOption accessors.
    ConfigurationError(String),
}

impl PlaceholderParserException {
    /// `std::exception::what()`
    pub fn what(&self) -> &str {
        match self {
            PlaceholderParserException::PlaceholderParserError(s)
            | PlaceholderParserException::FlowErrorMissingVariable(s)
            | PlaceholderParserException::RuntimeError(s)
            | PlaceholderParserException::ConfigurationError(s) => s,
        }
    }
}

impl std::fmt::Display for PlaceholderParserException {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.what())
    }
}

impl std::error::Error for PlaceholderParserException {}

impl From<PlaceholderParserException> for crate::Error {
    fn from(e: PlaceholderParserException) -> Self {
        crate::Error::ParseError(e.what().to_string())
    }
}

/// What the C++ throws while the grammar runs: a `qi::expectation_failure<Iterator>`
/// (caught by the `on_error<fail>` handler of `start`, PlaceholderParser.cpp:1227),
/// or any other exception, which propagates out of `process()`.
#[derive(Clone, Debug)]
pub enum Throw {
    /// `qi::expectation_failure<Iterator>(first, last, spirit::info(tag))`
    ExpectationFailure { first: usize, last: usize, tag: String },
    /// Any other exception.
    Exception(PlaceholderParserException),
}

type TResult<T> = std::result::Result<T, Throw>;

fn expectation_failure(first: usize, last: usize, tag: &str) -> Throw {
    Throw::ExpectationFailure { first, last, tag: tag.to_string() }
}

fn cfg_err(e: ConfigurationError) -> Throw {
    Throw::Exception(PlaceholderParserException::ConfigurationError(e.what().to_string()))
}

/// `std::ostringstream << double` with the default flags (config.rs `ostream_double`).
fn ostream_double(v: f64) -> String {
    crate::config::ostream_double(v)
}

// =====================================================================
// expr<Iterator>  (PlaceholderParser.cpp:190-707)
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

/// `expr<Iterator>::Data` (PlaceholderParser.cpp:291-303), modelled as a Rust enum
/// carrying the tag and payload together; `Type` is recovered via [`Expr::type_`].
#[derive(Clone, Debug)]
enum Data {
    Empty,
    Bool(bool),
    Int(i32),
    Double(f64),
    Str(String),
}

impl Data {
    /// PlaceholderParser.cpp:301-302 `void set(const Data &rhs)`: copy the raw image of the other
    /// data members (the copy and move constructors and assignments of `expr`).
    fn set(&mut self, rhs: &Data) {
        *self = rhs.clone();
    }
}

/// `expr<Iterator>` (PlaceholderParser.cpp:191).
#[derive(Clone, Debug)]
pub struct Expr {
    data: Data,
    /// PlaceholderParser.cpp:315-317 `boost::iterator_range<Iterator> it_range;`
    /// Range of input iterators covering this expression. Used for throwing parse exceptions.
    pub it_range: (usize, usize),
}

impl Expr {
    // PlaceholderParser.cpp:193  expr() : type(TYPE_EMPTY) {}
    pub fn new() -> Self {
        Expr { data: Data::Empty, it_range: (0, 0) }
    }

    // PlaceholderParser.cpp:194  explicit expr(bool b) : type(TYPE_BOOL) { data.b = b; }
    pub fn from_bool(b: bool) -> Self {
        Expr { data: Data::Bool(b), it_range: (0, 0) }
    }

    // PlaceholderParser.cpp:195  explicit expr(bool b, const Iterator &it_begin, const Iterator &it_end)
    pub fn from_bool_range(b: bool, it_begin: usize, it_end: usize) -> Self {
        Expr { data: Data::Bool(b), it_range: (it_begin, it_end) }
    }

    // PlaceholderParser.cpp:196  explicit expr(int i) : type(TYPE_INT) { data.i = i; }
    pub fn from_int(i: i32) -> Self {
        Expr { data: Data::Int(i), it_range: (0, 0) }
    }

    // PlaceholderParser.cpp:197  explicit expr(int i, const Iterator &it_begin, const Iterator &it_end)
    pub fn from_int_range(i: i32, it_begin: usize, it_end: usize) -> Self {
        Expr { data: Data::Int(i), it_range: (it_begin, it_end) }
    }

    // PlaceholderParser.cpp:198  explicit expr(double d) : type(TYPE_DOUBLE) { data.d = d; }
    pub fn from_double(d: f64) -> Self {
        Expr { data: Data::Double(d), it_range: (0, 0) }
    }

    // PlaceholderParser.cpp:199  explicit expr(double d, const Iterator &it_begin, const Iterator &it_end)
    pub fn from_double_range(d: f64, it_begin: usize, it_end: usize) -> Self {
        Expr { data: Data::Double(d), it_range: (it_begin, it_end) }
    }

    // PlaceholderParser.cpp:200-201  explicit expr(const char *s) / (const std::string &s) : type(TYPE_STRING)
    pub fn from_string(s: impl Into<String>) -> Self {
        Expr { data: Data::Str(s.into()), it_range: (0, 0) }
    }

    // PlaceholderParser.cpp:202-203  explicit expr(const std::string &s, const Iterator &it_begin, const Iterator &it_end)
    pub fn from_string_range(s: impl Into<String>, it_begin: usize, it_end: usize) -> Self {
        Expr { data: Data::Str(s.into()), it_range: (it_begin, it_end) }
    }

    // PlaceholderParser.cpp:208-209  explicit expr(expr &&rhs, const Iterator &it_begin, const Iterator &it_end)
    pub fn from_expr_range(rhs: Expr, it_begin: usize, it_end: usize) -> Self {
        // type(rhs.type), it_range(it_begin, it_end) { data.set(rhs.data); rhs.type = TYPE_EMPTY; }
        let mut out = Expr { data: Data::Empty, it_range: (it_begin, it_end) };
        out.data.set(&rhs.data);
        out
    }

    // PlaceholderParser.cpp:238-246  void reset()
    pub fn reset(&mut self) {
        // BBS: TYPE_STRING owned a heap std::string; dropping the String reclaims it.
        // The it_range is left as it is, as in the C++.
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
            // The C++ reads the raw union member regardless of the tag; the callers
            // only call b() when type == TYPE_BOOL.
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
    // `int(double)` is `as i32`: saturating, as the arm64 conversion (fcvtzs) is.
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
            //   ostringstream default converter produces no trailing zeros after the decimal point.
            Data::Double(d) => ostream_double(*d),
            // PlaceholderParser.cpp:285  case TYPE_STRING: out = *data.s;
            Data::Str(s) => s.clone(),
            // PlaceholderParser.cpp:286  default: break;
            Data::Empty => String::new(),
        }
    }

    // PlaceholderParser.cpp:319-332  expr unary_minus(const Iterator start_pos) const
    pub fn unary_minus(&self, start_pos: usize) -> TResult<Expr> {
        match self.data {
            // PlaceholderParser.cpp:322-323
            Data::Int(i) => Ok(Expr::from_int_range(i.wrapping_neg(), start_pos, self.it_range.1)),
            // PlaceholderParser.cpp:324-325
            Data::Double(d) => Ok(Expr::from_double_range(-d, start_pos, self.it_range.1)),
            // PlaceholderParser.cpp:326-327
            _ => Err(self.throw_exception("Cannot apply unary minus operator.")),
        }
    }

    // PlaceholderParser.cpp:334-347  expr unary_integer(const Iterator start_pos) const
    pub fn unary_integer(&self, start_pos: usize) -> TResult<Expr> {
        match self.data {
            // PlaceholderParser.cpp:337-338
            Data::Int(i) => Ok(Expr::from_int_range(i, start_pos, self.it_range.1)),
            // PlaceholderParser.cpp:339-340  static_cast<int>(this->d())
            Data::Double(d) => Ok(Expr::from_int_range(d as i32, start_pos, self.it_range.1)),
            // PlaceholderParser.cpp:341-342
            _ => Err(self.throw_exception("Cannot convert to integer.")),
        }
    }

    // PlaceholderParser.cpp:349-362  expr round(const Iterator start_pos) const
    pub fn round(&self, start_pos: usize) -> TResult<Expr> {
        match self.data {
            // PlaceholderParser.cpp:352-353
            Data::Int(i) => Ok(Expr::from_int_range(i, start_pos, self.it_range.1)),
            // PlaceholderParser.cpp:354-355  static_cast<int>(std::round(this->d()))
            Data::Double(d) => Ok(Expr::from_int_range(d.round() as i32, start_pos, self.it_range.1)),
            // PlaceholderParser.cpp:356-357
            _ => Err(self.throw_exception("Cannot round a non-numeric value.")),
        }
    }

    // PlaceholderParser.cpp:364-377  expr floor(const Iterator start_pos) const
    pub fn floor(&self, start_pos: usize) -> TResult<Expr> {
        match self.data {
            // PlaceholderParser.cpp:367-368
            Data::Int(i) => Ok(Expr::from_int_range(i, start_pos, self.it_range.1)),
            // PlaceholderParser.cpp:369-370  static_cast<int>(std::floor(this->d()))
            Data::Double(d) => Ok(Expr::from_int_range(d.floor() as i32, start_pos, self.it_range.1)),
            // PlaceholderParser.cpp:371-372
            _ => Err(self.throw_exception("Cannot floor a non-numeric value.")),
        }
    }

    // PlaceholderParser.cpp:379-392  expr ceil(const Iterator start_pos) const
    pub fn ceil(&self, start_pos: usize) -> TResult<Expr> {
        match self.data {
            // PlaceholderParser.cpp:382-383
            Data::Int(i) => Ok(Expr::from_int_range(i, start_pos, self.it_range.1)),
            // PlaceholderParser.cpp:384-385  static_cast<int>(std::ceil(this->d()))
            Data::Double(d) => Ok(Expr::from_int_range(d.ceil() as i32, start_pos, self.it_range.1)),
            // PlaceholderParser.cpp:386-387
            _ => Err(self.throw_exception("Cannot ceil a non-numeric value.")),
        }
    }

    // PlaceholderParser.cpp:394-405  expr unary_not(const Iterator start_pos) const
    pub fn unary_not(&self, start_pos: usize) -> TResult<Expr> {
        match self.data {
            // PlaceholderParser.cpp:397-398
            Data::Bool(b) => Ok(Expr::from_bool_range(!b, start_pos, self.it_range.1)),
            // PlaceholderParser.cpp:399-400
            _ => Err(self.throw_exception("Cannot apply a not operator.")),
        }
    }

    // PlaceholderParser.cpp:407-429  expr &operator+=(const expr &rhs)
    pub fn add_assign(&mut self, rhs: &Expr) -> TResult<()> {
        if self.type_() == Type::TypeString {
            // Convert the right hand side to string and append.
            let r = rhs.to_string();
            if let Data::Str(s) = &mut self.data {
                s.push_str(&r);
            }
        } else if rhs.type_() == Type::TypeString {
            // Conver the left hand side to string, append rhs.
            let combined = self.to_string() + rhs.s();
            self.data = Data::Str(combined);
        } else {
            let err_msg = "Cannot add non-numeric types.";
            self.throw_if_not_numeric(err_msg)?;
            rhs.throw_if_not_numeric(err_msg)?;
            if self.type_() == Type::TypeDouble || rhs.type_() == Type::TypeDouble {
                let d = self.as_d() + rhs.as_d();
                self.data = Data::Double(d);
            } else {
                // this->data.i += rhs.i();
                self.data = Data::Int(self.i().wrapping_add(rhs.i()));
            }
        }
        // PlaceholderParser.cpp:427
        self.it_range = (self.it_range.0, rhs.it_range.1);
        Ok(())
    }

    // PlaceholderParser.cpp:431-444  expr &operator-=(const expr &rhs)
    pub fn sub_assign(&mut self, rhs: &Expr) -> TResult<()> {
        let err_msg = "Cannot subtract non-numeric types.";
        self.throw_if_not_numeric(err_msg)?;
        rhs.throw_if_not_numeric(err_msg)?;
        if self.type_() == Type::TypeDouble || rhs.type_() == Type::TypeDouble {
            let d = self.as_d() - rhs.as_d();
            self.data = Data::Double(d);
        } else {
            self.data = Data::Int(self.i().wrapping_sub(rhs.i()));
        }
        self.it_range = (self.it_range.0, rhs.it_range.1);
        Ok(())
    }

    // PlaceholderParser.cpp:446-459  expr &operator*=(const expr &rhs)
    pub fn mul_assign(&mut self, rhs: &Expr) -> TResult<()> {
        let err_msg = "Cannot multiply with non-numeric type.";
        self.throw_if_not_numeric(err_msg)?;
        rhs.throw_if_not_numeric(err_msg)?;
        if self.type_() == Type::TypeDouble || rhs.type_() == Type::TypeDouble {
            let d = self.as_d() * rhs.as_d();
            self.data = Data::Double(d);
        } else {
            self.data = Data::Int(self.i().wrapping_mul(rhs.i()));
        }
        self.it_range = (self.it_range.0, rhs.it_range.1);
        Ok(())
    }

    // PlaceholderParser.cpp:461-475  expr &operator/=(const expr &rhs)
    pub fn div_assign(&mut self, rhs: &Expr) -> TResult<()> {
        self.throw_if_not_numeric("Cannot divide a non-numeric type.")?;
        rhs.throw_if_not_numeric("Cannot divide with a non-numeric type.")?;
        let is_zero = if rhs.type_() == Type::TypeInt { rhs.i() == 0 } else { rhs.d() == 0.0 };
        if is_zero {
            return Err(rhs.throw_exception("Division by zero"));
        }
        if self.type_() == Type::TypeDouble || rhs.type_() == Type::TypeDouble {
            let d = self.as_d() / rhs.as_d();
            self.data = Data::Double(d);
        } else {
            // INT_MIN / -1: the arm64 sdiv gives INT_MIN, as wrapping_div does.
            self.data = Data::Int(self.i().wrapping_div(rhs.i()));
        }
        self.it_range = (self.it_range.0, rhs.it_range.1);
        Ok(())
    }

    // PlaceholderParser.cpp:477-491  expr &operator%=(const expr &rhs)
    pub fn rem_assign(&mut self, rhs: &Expr) -> TResult<()> {
        self.throw_if_not_numeric("Cannot divide a non-numeric type.")?;
        rhs.throw_if_not_numeric("Cannot divide with a non-numeric type.")?;
        let is_zero = if rhs.type_() == Type::TypeInt { rhs.i() == 0 } else { rhs.d() == 0.0 };
        if is_zero {
            return Err(rhs.throw_exception("Division by zero"));
        }
        if self.type_() == Type::TypeDouble || rhs.type_() == Type::TypeDouble {
            // double d = std::fmod(this->as_d(), rhs.as_d());
            let d = self.as_d() % rhs.as_d();
            self.data = Data::Double(d);
        } else {
            self.data = Data::Int(self.i().wrapping_rem(rhs.i()));
        }
        self.it_range = (self.it_range.0, rhs.it_range.1);
        Ok(())
    }

    // PlaceholderParser.cpp:493-496  static void to_string2(expr &self, std::string &out)
    pub fn to_string2(&self) -> String {
        self.to_string()
    }

    // PlaceholderParser.cpp:498-503  static void evaluate_boolean(expr &self, bool &out)
    pub fn evaluate_boolean(&self) -> TResult<bool> {
        if self.type_() != Type::TypeBool {
            return Err(self.throw_exception("Not a boolean expression"));
        }
        Ok(self.b())
    }

    // PlaceholderParser.cpp:505-510  static void evaluate_boolean_to_string(expr &self, std::string &out)
    pub fn evaluate_boolean_to_string(&self) -> TResult<String> {
        if self.type_() != Type::TypeBool {
            return Err(self.throw_exception("Not a boolean expression"));
        }
        Ok(if self.b() { "true".to_string() } else { "false".to_string() })
    }

    // PlaceholderParser.cpp:512-550  static void compare_op(expr &lhs, expr &rhs, char op, bool invert)
    // Is lhs==rhs? Store the result into lhs.
    pub fn compare_op(lhs: &mut Expr, rhs: &Expr, op: char, invert: bool) -> TResult<()> {
        let value;
        let lhs_t = lhs.type_();
        let rhs_t = rhs.type_();
        if (lhs_t == Type::TypeInt || lhs_t == Type::TypeDouble) && (rhs_t == Type::TypeInt || rhs_t == Type::TypeDouble) {
            // Both types are numeric.
            match op {
                '=' => {
                    value = if lhs_t == Type::TypeDouble || rhs_t == Type::TypeDouble {
                        (lhs.as_d() - rhs.as_d()).abs() < 1e-8
                    } else {
                        lhs.i() == rhs.i()
                    };
                }
                '<' => {
                    value = if lhs_t == Type::TypeDouble || rhs_t == Type::TypeDouble {
                        lhs.as_d() < rhs.as_d()
                    } else {
                        lhs.i() < rhs.i()
                    };
                }
                _ => {
                    value = if lhs_t == Type::TypeDouble || rhs_t == Type::TypeDouble {
                        lhs.as_d() > rhs.as_d()
                    } else {
                        lhs.i() > rhs.i()
                    };
                }
            }
        } else if lhs_t == Type::TypeBool && rhs_t == Type::TypeBool {
            // Both type are bool.
            if op != '=' {
                return Err(expectation_failure(lhs.it_range.0, rhs.it_range.1, "*Cannot compare the types."));
            }
            value = lhs.b() == rhs.b();
        } else if lhs_t == Type::TypeString || rhs_t == Type::TypeString {
            // One type is string, the other could be converted to string.
            value = if op == '=' {
                lhs.to_string() == rhs.to_string()
            } else if op == '<' {
                lhs.to_string() < rhs.to_string()
            } else {
                lhs.to_string() > rhs.to_string()
            };
        } else {
            return Err(expectation_failure(lhs.it_range.0, rhs.it_range.1, "*Cannot compare the types."));
        }
        // PlaceholderParser.cpp:548-549
        lhs.data = Data::Bool(if invert { !value } else { value });
        Ok(())
    }

    // PlaceholderParser.cpp:552-557  Compare operators, store the result into lhs.
    pub fn equal(lhs: &mut Expr, rhs: &Expr) -> TResult<()> {
        Expr::compare_op(lhs, rhs, '=', false)
    }
    pub fn not_equal(lhs: &mut Expr, rhs: &Expr) -> TResult<()> {
        Expr::compare_op(lhs, rhs, '=', true)
    }
    pub fn lower(lhs: &mut Expr, rhs: &Expr) -> TResult<()> {
        Expr::compare_op(lhs, rhs, '<', false)
    }
    pub fn greater(lhs: &mut Expr, rhs: &Expr) -> TResult<()> {
        Expr::compare_op(lhs, rhs, '>', false)
    }
    pub fn leq(lhs: &mut Expr, rhs: &Expr) -> TResult<()> {
        Expr::compare_op(lhs, rhs, '>', true)
    }
    pub fn geq(lhs: &mut Expr, rhs: &Expr) -> TResult<()> {
        Expr::compare_op(lhs, rhs, '<', true)
    }

    // PlaceholderParser.cpp:559-563  static void throw_if_not_numeric(const expr &param)
    pub fn throw_if_not_numeric_static(param: &Expr) -> TResult<()> {
        let err_msg = "Not a numeric type.";
        param.throw_if_not_numeric(err_msg)
    }

    // PlaceholderParser.cpp:570-593  static void function_2params(expr &param1, expr &param2, Function2ParamsType fun)
    // Store the result into param1.
    fn function_2params(param1: &mut Expr, param2: &Expr, fun: Function2ParamsType) -> TResult<()> {
        Expr::throw_if_not_numeric_static(param1)?;
        Expr::throw_if_not_numeric_static(param2)?;
        if param1.type_() == Type::TypeDouble || param2.type_() == Type::TypeDouble {
            // std::min / std::max: `(b < a) ? b : a` / `(a < b) ? b : a`.
            let (a, b) = (param1.as_d(), param2.as_d());
            let d = match fun {
                Function2ParamsType::FunctionMin => {
                    if b < a {
                        b
                    } else {
                        a
                    }
                }
                Function2ParamsType::FunctionMax => {
                    if a < b {
                        b
                    } else {
                        a
                    }
                }
            };
            param1.data = Data::Double(d);
        } else {
            let i = match fun {
                Function2ParamsType::FunctionMin => param1.as_i().min(param2.as_i()),
                Function2ParamsType::FunctionMax => param1.as_i().max(param2.as_i()),
            };
            param1.data = Data::Int(i);
        }
        Ok(())
    }

    // PlaceholderParser.cpp:595-596  Store the result into param1.
    pub fn min(param1: &mut Expr, param2: &Expr) -> TResult<()> {
        Expr::function_2params(param1, param2, Function2ParamsType::FunctionMin)
    }
    pub fn max(param1: &mut Expr, param2: &Expr) -> TResult<()> {
        Expr::function_2params(param1, param2, Function2ParamsType::FunctionMax)
    }

    // PlaceholderParser.cpp:598-610  static void random(expr &param1, expr &param2, std::mt19937 &rng)
    // Store the result into param1.
    pub fn random(param1: &mut Expr, param2: &Expr, rng: &mut Mt19937) -> TResult<()> {
        Expr::throw_if_not_numeric_static(param1)?;
        Expr::throw_if_not_numeric_static(param2)?;
        if param1.type_() == Type::TypeDouble || param2.type_() == Type::TypeDouble {
            // std::uniform_real_distribution<>(param1.as_d(), param2.as_d())(rng)
            param1.data = Data::Double(uniform_real_distribution(param1.as_d(), param2.as_d(), rng));
        } else {
            // std::uniform_int_distribution<>(param1.as_i(), param2.as_i())(rng)
            param1.data = Data::Int(uniform_int_distribution(param1.as_i(), param2.as_i(), rng));
        }
        Ok(())
    }

    // PlaceholderParser.cpp:612-634  template<bool leading_zeros> static void digits(expr &param1, expr &param2, expr &param3)
    // Store the result into param1. param3 is optional.
    pub fn digits(param1: &mut Expr, param2: &Expr, param3: &Expr, leading_zeros: bool) -> TResult<()> {
        Expr::throw_if_not_numeric_static(param1)?;
        if param2.type_() != Type::TypeInt {
            return Err(param2.throw_exception("digits: second parameter must be integer"));
        }
        let has_decimals = param3.type_() != Type::TypeEmpty;
        if has_decimals && param3.type_() != Type::TypeInt {
            return Err(param3.throw_exception("digits: third parameter must be integer"));
        }

        let ndigits = param2.as_i().clamp(0, 64);
        let buf = if has_decimals {
            // Format as double.
            // sprintf(buf, leading_zeros ? "%0*.*lf" : "%*.*lf", ndigits, decimals, param1.as_d());
            let decimals = param3.as_i().clamp(0, 64);
            format_printf_f(param1.as_d(), ndigits, decimals, leading_zeros)
        } else {
            // Format as int.
            // sprintf(buf, leading_zeros ? "%0*d" : "%*d", ndigits, param1.as_i_rounded());
            format_printf_d(param1.as_i_rounded(), ndigits, leading_zeros)
        };
        param1.set_s(buf);
        Ok(())
    }

    // PlaceholderParser.cpp:636-658  static void regex_op(expr &lhs, boost::iterator_range<Iterator> &rhs, char op)
    // `rhs` is the source text of the regular expression literal including its two slashes.
    pub fn regex_op(lhs: &mut Expr, rhs: &[u8], rhs_range: (usize, usize), op: char) -> TResult<()> {
        if lhs.type_() != Type::TypeString {
            return Err(lhs.throw_exception("Left hand side of a regex match must be a string."));
        }
        // std::string pattern(++ rhs.begin(), -- rhs.end());
        let pattern = &rhs[1..rhs.len() - 1];
        match boost_regex::regex_match(lhs.s().as_bytes(), pattern) {
            Ok(mut result) => {
                if op == '!' {
                    result = !result;
                }
                lhs.reset();
                lhs.data = Data::Bool(result);
                Ok(())
            }
            // Syntax error in the regular expression
            Err(what) => Err(expectation_failure(
                rhs_range.0,
                rhs_range.1,
                &format!("*Regular expression compilation failed: {}", what),
            )),
        }
    }

    // PlaceholderParser.cpp:660-661
    pub fn regex_matches(lhs: &mut Expr, rhs: &[u8], rhs_range: (usize, usize)) -> TResult<()> {
        Expr::regex_op(lhs, rhs, rhs_range, '=')
    }
    pub fn regex_doesnt_match(lhs: &mut Expr, rhs: &[u8], rhs_range: (usize, usize)) -> TResult<()> {
        Expr::regex_op(lhs, rhs, rhs_range, '!')
    }

    // PlaceholderParser.cpp:663-674  static void logical_op(expr &lhs, expr &rhs, char op)
    pub fn logical_op(lhs: &mut Expr, rhs: &Expr, op: char) -> TResult<()> {
        let value;
        if lhs.type_() == Type::TypeBool && rhs.type_() == Type::TypeBool {
            value = if op == '|' { lhs.b() || rhs.b() } else { lhs.b() && rhs.b() };
        } else {
            return Err(expectation_failure(
                lhs.it_range.0,
                rhs.it_range.1,
                "*Cannot apply logical operation to non-boolean operators.",
            ));
        }
        lhs.data = Data::Bool(value);
        Ok(())
    }
    // PlaceholderParser.cpp:675-676
    pub fn logical_or(lhs: &mut Expr, rhs: &Expr) -> TResult<()> {
        Expr::logical_op(lhs, rhs, '|')
    }
    pub fn logical_and(lhs: &mut Expr, rhs: &Expr) -> TResult<()> {
        Expr::logical_op(lhs, rhs, '&')
    }

    // PlaceholderParser.cpp:678-686  static void ternary_op(expr &lhs, expr &rhs1, expr &rhs2)
    pub fn ternary_op(lhs: &mut Expr, rhs1: Expr, rhs2: Expr) -> TResult<()> {
        if lhs.type_() != Type::TypeBool {
            return Err(lhs.throw_exception("Not a boolean expression"));
        }
        // lhs = std::move(rhs1) / std::move(rhs2): the move assignment takes the it_range too.
        if lhs.b() {
            *lhs = rhs1;
        } else {
            *lhs = rhs2;
        }
        Ok(())
    }

    // PlaceholderParser.cpp:688-694  static void set_if(bool &cond, bool &not_yet_consumed, std::string &str_in, std::string &str_out)
    pub fn set_if(cond: bool, not_yet_consumed: &mut bool, str_in: &str, str_out: &mut String) {
        if cond && *not_yet_consumed {
            *str_out = str_in.to_string();
            *not_yet_consumed = false;
        }
    }

    // PlaceholderParser.cpp:696-700  void throw_exception(const char *message) const
    pub fn throw_exception(&self, message: &str) -> Throw {
        expectation_failure(self.it_range.0, self.it_range.1, &format!("*{}", message))
    }

    // PlaceholderParser.cpp:702-706  void throw_if_not_numeric(const char *message) const
    pub fn throw_if_not_numeric(&self, message: &str) -> TResult<()> {
        if self.type_() != Type::TypeInt && self.type_() != Type::TypeDouble {
            Err(self.throw_exception(message))
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

/// printf `"%0*.*lf"` / `"%*.*lf"` (PlaceholderParser.cpp:629).
fn format_printf_f(value: f64, width: i32, prec: i32, leading_zeros: bool) -> String {
    if !value.is_finite() {
        // printf prints "nan" / "inf" / "-inf" and ignores the 0 flag for them.
        // The BSD printf of Apple's libc prints "nan" whatever the sign bit.
        let body = if value.is_nan() {
            "nan".to_string()
        } else if value < 0.0 {
            "-inf".to_string()
        } else {
            "inf".to_string()
        };
        return pad_to_width(body, width, false, false);
    }
    let body = format!("{:.*}", prec.max(0) as usize, value);
    let negative = body.starts_with('-');
    pad_to_width(body, width, leading_zeros, negative)
}

/// printf `"%0*d"` / `"%*d"` (PlaceholderParser.cpp:632).
fn format_printf_d(value: i32, width: i32, leading_zeros: bool) -> String {
    let body = value.to_string();
    pad_to_width(body, width, leading_zeros, value < 0)
}

/// printf field-width padding: with the `0` flag the zeros go between the sign
/// and the digits; otherwise the field is right-justified with spaces.
fn pad_to_width(body: String, width: i32, leading_zeros: bool, negative: bool) -> String {
    let width = width.max(0) as usize;
    if body.len() >= width {
        return body;
    }
    let pad = width - body.len();
    if leading_zeros {
        if negative {
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
// std::mt19937 and the libc++ distributions used by expr::random
// =====================================================================

/// `std::mt19937` (`mersenne_twister_engine<uint_fast32_t, 32, 624, 397, 31, 0x9908b0df,
/// 11, 0xffffffff, 7, 0x9d2c5680, 15, 0xefc60000, 18, 1812433253>`), the engine of
/// `PlaceholderParser::ContextData` (PlaceholderParser.hpp:16-18).
#[derive(Clone, Debug)]
pub struct Mt19937 {
    x: [u32; 624],
    i: usize,
}

impl Mt19937 {
    /// `mersenne_twister_engine::default_seed`
    pub const DEFAULT_SEED: u32 = 5489;

    /// `explicit mersenne_twister_engine(result_type __sd = default_seed)`
    pub fn new(seed: u32) -> Self {
        let mut x = [0u32; 624];
        x[0] = seed;
        for i in 1..624 {
            x[i] = 1812433253u32.wrapping_mul(x[i - 1] ^ (x[i - 1] >> 30)).wrapping_add(i as u32);
        }
        Mt19937 { x, i: 0 }
    }

    /// `result_type operator()()`
    pub fn next_u32(&mut self) -> u32 {
        const N: usize = 624;
        const M: usize = 397;
        let j = (self.i + 1) % N;
        let mask: u32 = 0x8000_0000;
        let yp = (self.x[self.i] & mask) | (self.x[j] & !mask);
        let k = (self.i + M) % N;
        self.x[self.i] = self.x[k] ^ (yp >> 1) ^ (if yp & 1 != 0 { 0x9908_b0df } else { 0 });
        let mut z = self.x[self.i] ^ (self.x[self.i] >> 11);
        self.i = j;
        z ^= (z << 7) & 0x9d2c_5680;
        z ^= (z << 15) & 0xefc6_0000;
        z ^ (z >> 18)
    }
}

impl Default for Mt19937 {
    fn default() -> Self {
        Mt19937::new(Mt19937::DEFAULT_SEED)
    }
}

/// libc++ `uniform_int_distribution<int>::operator()(mt19937&)` with its
/// `__independent_bits_engine` (`_Rp == 0` for a 32-bit engine and a 32-bit working type:
/// one draw masked to `__w` bits per try).
///
/// FIDELITY-NOTE: the distributions are implementation-defined; this is libc++ (macOS).
/// libstdc++ and MSVC map the engine output differently.
fn uniform_int_distribution(a: i32, b: i32, g: &mut Mt19937) -> i32 {
    let rp: u32 = (b as u32).wrapping_sub(a as u32).wrapping_add(1);
    if rp == 1 {
        return a;
    }
    let dt: u32 = 32;
    if rp == 0 {
        return g.next_u32() as i32;
    }
    let mut w = dt - rp.leading_zeros() - 1;
    if (rp & (u32::MAX >> (dt - w))) != 0 {
        w += 1;
    }
    let mask0: u32 = if w > 0 { u32::MAX >> (dt - w) } else { 0 };
    loop {
        let u = g.next_u32() & mask0;
        if u < rp {
            return u.wrapping_add(a as u32) as i32;
        }
    }
}

/// libc++ `generate_canonical<double, 53>(mt19937&)`: two draws.
fn generate_canonical_53(g: &mut Mt19937) -> f64 {
    let rp: f64 = (u32::MAX as f64) + 1.0;
    let mut base = rp;
    let mut sp: f64 = g.next_u32() as f64;
    // for (size_t __i = 1; __i < __k; ++__i, __base *= __rp) __sp += (__g() - min) * __base;
    sp = (g.next_u32() as f64).mul_add(base, sp);
    base *= rp;
    sp / base
}

/// libc++ `uniform_real_distribution<double>::operator()`:
/// `(b - a) * generate_canonical(g) + a`, one contracted expression.
fn uniform_real_distribution(a: f64, b: f64, g: &mut Mt19937) -> f64 {
    (b - a).mul_add(generate_canonical_53(g), a)
}

// =====================================================================
// update_timestamp time formatting (PlaceholderParser.cpp:80-103)
// =====================================================================

/// The values `PlaceholderParser::update_timestamp` writes (PlaceholderParser.cpp:95-102).
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

/// The computation of `PlaceholderParser::update_timestamp` (PlaceholderParser.cpp:82-94):
/// `time()` + `localtime()` through `chrono::Local::now()`.
pub fn update_timestamp_values() -> TimestampValues {
    use chrono::{Datelike, Local, Timelike};
    let timeinfo = Local::now();
    let year = timeinfo.year();
    let month = timeinfo.month() as i32;
    let day = timeinfo.day() as i32;
    let hour = timeinfo.hour() as i32;
    let minute = timeinfo.minute() as i32;
    let second = timeinfo.second() as i32;
    // ss << (1900 + tm_year) << setw(2) << setfill('0') << (1 + tm_mon) << ... << "-" << ...
    let timestamp = format!("{}{:02}{:02}-{:02}{:02}{:02}", year, month, day, hour, minute, second);
    TimestampValues { timestamp, year, month, day, hour, minute, second }
}

// =====================================================================
// Flow::extrusion_width (Flow.cpp:21-91), the two overloads the grammar calls
// =====================================================================
//
// FIDELITY-NOTE: Flow.cpp's `extrusion_width(opt_key, ...)` overloads are only called by
// PlaceholderParser.cpp:906 and :935; flow.rs has no ConfigOptionResolver form of them, so
// they live here as private helpers over the resolver of the grammar.

/// Flow.cpp:21-36 `float Flow::auto_extrusion_width(FlowRole role, float nozzle_diameter)`
fn auto_extrusion_width(role: FlowRole, nozzle_diameter: f32) -> f32 {
    match role {
        FlowRole::SupportMaterial | FlowRole::SupportMaterialInterface | FlowRole::SupportTransition | FlowRole::TopSolidInfill => {
            nozzle_diameter
        }
        _ => 1.125f32 * nozzle_diameter,
    }
}

/// Flow.cpp:40-59 `static inline FlowRole opt_key_to_flow_role(const std::string &opt_key)`
fn opt_key_to_flow_role(opt_key: &str) -> TResult<FlowRole> {
    if opt_key == "inner_wall_line_width" ||
        // or all the defaults:
        opt_key == "line_width" || opt_key == "initial_layer_line_width"
    {
        Ok(FlowRole::Perimeter)
    } else if opt_key == "outer_wall_line_width" {
        Ok(FlowRole::ExternalPerimeter)
    } else if opt_key == "sparse_infill_line_width"
        || opt_key == "skeleton_infill_line_width"
        || opt_key == "initial_layer_infill_line_width"
    {
        Ok(FlowRole::Infill)
    } else if opt_key == "internal_solid_infill_line_width" || opt_key == "skin_infill_line_width" {
        Ok(FlowRole::SolidInfill)
    } else if opt_key == "top_surface_line_width" {
        Ok(FlowRole::TopSolidInfill)
    } else if opt_key == "support_line_width" {
        Ok(FlowRole::SupportMaterial)
    } else {
        Err(Throw::Exception(PlaceholderParserException::RuntimeError(
            "opt_key_to_flow_role: invalid argument".to_string(),
        )))
    }
}

/// Flow.cpp:61-64 `static inline void throw_on_missing_variable(const std::string &opt_key, const char *dependent_opt_key)`
fn throw_on_missing_variable(opt_key: &str, dependent_opt_key: &str) -> Throw {
    Throw::Exception(PlaceholderParserException::FlowErrorMissingVariable(format!(
        "Failed to calculate line width of {}. Can not get value of \"{}\" ",
        opt_key, dependent_opt_key
    )))
}

/// `config.option<ConfigOptionFloat>(key)`: the option if its type is coFloat.
fn option_float<R: ConfigOptionResolver + ?Sized>(config: &R, key: &str) -> Option<f64> {
    match config.option(key) {
        Some(o) if o.r#type() == ConfigOptionType::CoFloat => o.get_float().ok(),
        _ => None,
    }
}

/// `const std::vector<double>&` of a coFloats / coPercents option (nullable or not).
fn float_values(opt: &dyn ConfigOption) -> Option<&Vec<f64>> {
    opt.vector_values_any().and_then(|v| v.downcast_ref::<Vec<f64>>())
}

/// `ConfigOptionVector<T>::get_at(i)`: `values[(i < size) ? i : 0]`.
fn vector_get_at<T>(values: &[T], i: usize) -> Option<&T> {
    values.get(if i < values.len() { i } else { 0 })
}

/// Flow.cpp:67-86 `double Flow::extrusion_width(const std::string& opt_key, const ConfigOptionFloat* opt,
/// const ConfigOptionResolver& config, const unsigned int first_printing_extruder)`
fn flow_extrusion_width_opt<R: ConfigOptionResolver + ?Sized>(
    opt_key: &str,
    opt_value: f64,
    config: &R,
    first_printing_extruder: u32,
) -> TResult<f64> {
    let mut value = opt_value;
    if value == 0.0 {
        // The role specific extrusion width value was set to zero, try the role non-specific extrusion width.
        value = match option_float(config, "line_width") {
            Some(v) => v,
            None => return Err(throw_on_missing_variable(opt_key, "line_width")),
        };
    }

    if value == 0.0 {
        // If user left option to 0, calculate a sane default width.
        // config.option<ConfigOptionFloatsNullable>("nozzle_diameter")
        let opt_nozzle_diameters = match config.option("nozzle_diameter") {
            Some(o) if o.r#type() == ConfigOptionType::CoFloats => float_values(o),
            _ => None,
        };
        let nozzle_diameters = match opt_nozzle_diameters {
            Some(v) => v,
            None => return Err(throw_on_missing_variable(opt_key, "nozzle_diameter")),
        };
        let nozzle = vector_get_at(nozzle_diameters, first_printing_extruder as usize).copied().unwrap_or(0.0);
        return Ok(auto_extrusion_width(opt_key_to_flow_role(opt_key)?, nozzle as f32) as f64);
    }

    Ok(value)
}

/// Flow.cpp:88-91 `double Flow::extrusion_width(const std::string& opt_key, const ConfigOptionResolver &config,
/// const unsigned int first_printing_extruder)`
fn flow_extrusion_width<R: ConfigOptionResolver + ?Sized>(opt_key: &str, config: &R, first_printing_extruder: u32) -> TResult<f64> {
    // config.option<ConfigOptionFloat>(opt_key): the callers pass a coFloat key.
    let value = option_float(config, opt_key).unwrap_or(0.0);
    flow_extrusion_width_opt(opt_key, value, config, first_printing_extruder)
}

// =====================================================================
// iso8859_1 character classification (boost/spirit/home/support/char_encoding/iso8859_1.hpp)
// =====================================================================
//
// FIDELITY-NOTE: the grammar parses `char` (signed on the reference platforms), and Spirit's
// `char_class` test hands the char to `iso8859_1::isalpha(int)` / `isspace(int)` without a cast,
// so a byte >= 0x80 indexes `iso8859_1_char_types` with a negative value (undefined behaviour,
// the assertion is compiled out). In the pin build none of those bytes classifies as alpha, digit
// or space (the native vectors probe every UTF-8 lead byte); so here only ASCII does. The table's
// own entries for 0xA0 (space), 0xB2, 0xB3, 0xB9 (digits) and 0xC0-0xFF (alpha) are never reached.

/// `iso8859_1::isspace`: 0x09-0x0D, 0x20.
fn is_space(c: u8) -> bool {
    matches!(c, 0x09..=0x0d | 0x20)
}

/// `iso8859_1::isalpha`: the ASCII letters.
fn is_alpha(c: u8) -> bool {
    c.is_ascii_alphabetic()
}

/// `iso8859_1::isalnum`: the ASCII letters and digits.
fn is_alnum(c: u8) -> bool {
    c.is_ascii_alphanumeric()
}

// =====================================================================
// MyContext  (PlaceholderParser.cpp:176-188, 725-1099)
// =====================================================================

/// PlaceholderParser.cpp:175-181 `template<typename Iterator> struct OptWithPos`
#[derive(Clone, Copy)]
pub struct OptWithPos<'a> {
    pub opt: &'a dyn ConfigOption,
    pub it_range: (usize, usize),
}

/// PlaceholderParser.cpp:725-1073 `struct MyContext : public ConfigOptionResolver`
pub struct MyContext<'a> {
    pub external_config: Option<&'a DynamicConfig>,
    pub config: Option<&'a DynamicConfig>,
    pub config_override: Option<&'a DynamicConfig>,
    /// This is filament_id actually. A `Cell`: `filament_change` changes it through a `const_cast`.
    pub current_extruder_id: Cell<usize>,
    pub context_data: Option<RefCell<&'a mut ContextData>>,
    /// If false, the macro_processor will evaluate a full macro.
    /// If true, the macro processor will evaluate just a boolean condition using the full expressive power of the macro processor.
    pub just_boolean_expression: bool,
    pub error_message: RefCell<String>,
}

/// PlaceholderParser.cpp:1075-1099 `std::map<std::string, std::string> MyContext::tag_to_error_message`
fn tag_to_error_message(tag: &str) -> Option<&'static str> {
    Some(match tag {
        "eoi" => "Unknown syntax error",
        "start" => "Unknown syntax error",
        "text" => "Invalid text.",
        "text_block" => "Invalid text block.",
        "macro" => "Invalid macro.",
        "if_else_output" => "Not an {if}{else}{endif} macro.",
        "switch_output" => "Not a {switch} macro.",
        "legacy_variable_expansion" => "Expecting a legacy variable expansion format",
        "identifier" => "Expecting an identifier.",
        "conditional_expression" => "Expecting a conditional expression.",
        "logical_or_expression" => "Expecting a boolean expression.",
        "logical_and_expression" => "Expecting a boolean expression.",
        "equality_expression" => "Expecting an expression.",
        "bool_expr_eval" => "Expecting a boolean expression.",
        "relational_expression" => "Expecting an expression.",
        "additive_expression" => "Expecting an expression.",
        "multiplicative_expression" => "Expecting an expression.",
        "unary_expression" => "Expecting an expression.",
        "optional_parameter" => "Expecting a closing brace or an optional parameter.",
        "scalar_variable_reference" => "Expecting a scalar variable reference.",
        "variable_reference" => "Expecting a variable reference.",
        "regular_expression" => "Expecting a regular expression.",
        _ => return None,
    })
}

impl<'a> ConfigOptionResolver for MyContext<'a> {
    // PlaceholderParser.cpp:749-759  const ConfigOption* optptr(const t_config_option_key &opt_key) const override
    fn optptr(&self, opt_key: &str) -> Option<&dyn ConfigOption> {
        let mut opt: Option<&dyn ConfigOption> = None;
        if let Some(co) = self.config_override {
            opt = co.option(opt_key);
        }
        if opt.is_none() {
            opt = self.config.and_then(|c| c.option(opt_key));
        }
        if opt.is_none() {
            if let Some(ec) = self.external_config {
                opt = ec.option(opt_key);
            }
        }
        opt
    }
}

/// `std::string(it_range.begin(), it_range.end())`
fn range_string(src: &[u8], r: (usize, usize)) -> String {
    String::from_utf8_lossy(&src[r.0..r.1]).into_owned()
}

/// `strtol(str, &endptr, 10)` on a byte string: the value (saturated at LONG_MIN / LONG_MAX)
/// and the number of bytes consumed (0 when no conversion was performed).
fn strtol10(s: &[u8]) -> (i64, usize) {
    let mut i = 0usize;
    while i < s.len() && matches!(s[i], b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r') {
        i += 1;
    }
    let mut neg = false;
    if i < s.len() && (s[i] == b'+' || s[i] == b'-') {
        neg = s[i] == b'-';
        i += 1;
    }
    let digits_start = i;
    let mut acc: i128 = 0;
    while i < s.len() && s[i].is_ascii_digit() {
        if acc < (1i128 << 70) {
            acc = acc * 10 + (s[i] - b'0') as i128;
        }
        i += 1;
    }
    if i == digits_start {
        return (0, 0);
    }
    let v = if neg { -acc } else { acc };
    (v.clamp(i64::MIN as i128, i64::MAX as i128) as i64, i)
}

impl<'a> MyContext<'a> {
    fn new_empty() -> Self {
        MyContext {
            external_config: None,
            config: None,
            config_override: None,
            current_extruder_id: Cell::new(0),
            context_data: None,
            just_boolean_expression: false,
            error_message: RefCell::new(String::new()),
        }
    }

    // PlaceholderParser.cpp:739-745  size_t get_extruder_id() const
    pub fn get_extruder_id(&self) -> usize {
        // FIDELITY-NOTE: the C++ dereferences `external_config` unchecked (a null pointer in
        // evaluate_boolean_expression and in a parser constructed without one, which crashes);
        // without an external config this answers 0, as for a missing "filament_map".
        let filament_map_opt = self.external_config.and_then(|ec| match ec.option("filament_map") {
            Some(o) if o.r#type() == ConfigOptionType::CoInts => o.vector_values_any().and_then(|v| v.downcast_ref::<Vec<i32>>()),
            _ => None,
        });
        let current_extruder_id = self.current_extruder_id.get();
        if let Some(values) = filament_map_opt {
            if current_extruder_id < values.len() {
                return values[current_extruder_id] as usize;
            }
        }
        0
    }

    // PlaceholderParser.cpp:747  static void evaluate_full_macro(const MyContext *ctx, bool &result)
    pub fn evaluate_full_macro(ctx: &MyContext) -> bool {
        !ctx.just_boolean_expression
    }

    // PlaceholderParser.cpp:761  const ConfigOption* resolve_symbol(const std::string &opt_key) const
    pub fn resolve_symbol(&self, opt_key: &str) -> Option<&dyn ConfigOption> {
        self.optptr(opt_key)
    }

    // PlaceholderParser.cpp:763-797  static void legacy_variable_expansion(const MyContext *ctx, boost::iterator_range<Iterator> &opt_key, std::string &output)
    pub fn legacy_variable_expansion(ctx: &MyContext, src: &[u8], opt_key: (usize, usize)) -> TResult<String> {
        let opt_key_bytes = &src[opt_key.0..opt_key.1];
        let opt_key_str = range_string(src, opt_key);
        let mut opt = ctx.resolve_symbol(&opt_key_str);
        let mut idx: usize = ctx.current_extruder_id.get();
        if opt.is_none() {
            // Check whether this is a legacy vector indexing.
            if let Some(u) = opt_key_bytes.iter().rposition(|&c| c == b'_') {
                idx = u;
                opt = ctx.resolve_symbol(&String::from_utf8_lossy(&opt_key_bytes[..idx]));
                if let Some(o) = opt {
                    if !o.is_vector() {
                        return Err(MyContext::throw_exception("Trying to index a scalar variable", opt_key));
                    }
                    let rest = &opt_key_bytes[idx + 1..];
                    let (value, consumed) = strtol10(rest);
                    // idx = strtol(...): the parsed index replaces the position of '_'.
                    idx = value as usize;
                    // endptr == nptr when nothing was converted, *endptr != 0 when not all was.
                    if consumed != rest.len() {
                        // FIDELITY-NOTE: the C++ reports the error at opt_key.begin() + idx + 1 with the
                        // index just parsed, which may lie past the end of the template (undefined
                        // behaviour there); the position is clamped to the template here.
                        let begin = opt_key.0.saturating_add(idx).saturating_add(1).min(src.len());
                        return Err(MyContext::throw_exception("Invalid vector index", (begin, opt_key.1.max(begin))));
                    }
                }
            }
        }
        let opt = match opt {
            None => return Err(MyContext::throw_exception("Variable does not exist", opt_key)),
            Some(o) => o,
        };
        if opt.is_scalar() {
            opt.serialize().map_err(cfg_err)
        } else {
            let vec = opt.as_vector_base().expect("a vector option is a ConfigOptionVectorBase");
            if vec.empty() {
                return Err(MyContext::throw_exception("Indexing an empty vector variable", opt_key));
            }
            let vv = vec.vserialize().map_err(cfg_err)?;
            Ok(vv[if idx >= vec.size() { 0 } else { idx }].clone())
        }
    }

    // PlaceholderParser.cpp:799-828  static void legacy_variable_expansion2(const MyContext *ctx,
    //     boost::iterator_range<Iterator> &opt_key, boost::iterator_range<Iterator> &opt_vector_index, std::string &output)
    pub fn legacy_variable_expansion2(
        ctx: &MyContext,
        src: &[u8],
        opt_key: (usize, usize),
        opt_vector_index: (usize, usize),
    ) -> TResult<String> {
        let mut opt_key_str = range_string(src, opt_key);
        let mut opt = ctx.resolve_symbol(&opt_key_str);
        if opt.is_none() {
            // Check whether the opt_key ends with '_'.
            if opt_key_str.ends_with('_') {
                opt_key_str.pop();
            }
            opt = ctx.resolve_symbol(&opt_key_str);
        }
        // FIDELITY-NOTE: the C++ dereferences a null `opt` here (a crash); that case is reported
        // as a variable that does not exist.
        let opt = match opt {
            Some(o) => o,
            None => return Err(MyContext::throw_exception("Variable does not exist", opt_key)),
        };
        if !opt.is_vector() {
            return Err(MyContext::throw_exception("Trying to index a scalar variable", opt_key));
        }
        let vec = opt.as_vector_base().expect("a vector option is a ConfigOptionVectorBase");
        if vec.empty() {
            return Err(MyContext::throw_exception("Indexing an empty vector variable", opt_key));
        }
        let opt_index = match ctx.resolve_symbol(&range_string(src, opt_vector_index)) {
            None => return Err(MyContext::throw_exception("Variable does not exist", opt_key)),
            Some(o) => o,
        };
        if opt_index.r#type() != ConfigOptionType::CoInt {
            return Err(MyContext::throw_exception("Indexing variable has to be integer", opt_key));
        }
        let idx = opt_index.get_int().map_err(cfg_err)?;
        if idx < 0 {
            return Err(MyContext::throw_exception("Negative vector index", opt_key));
        }
        let vv = vec.vserialize().map_err(cfg_err)?;
        Ok(vv[if idx >= vec.size() as i32 { 0 } else { idx as usize }].clone())
    }

    // PlaceholderParser.cpp:830-841  static void resolve_variable(const MyContext *ctx,
    //     boost::iterator_range<Iterator> &opt_key, OptWithPos<Iterator> &output)
    pub fn resolve_variable<'c>(ctx: &'c MyContext, src: &[u8], opt_key: (usize, usize)) -> TResult<OptWithPos<'c>> {
        match ctx.resolve_symbol(&range_string(src, opt_key)) {
            None => Err(MyContext::throw_exception("Not a variable name", opt_key)),
            Some(opt) => Ok(OptWithPos { opt, it_range: opt_key }),
        }
    }

    // PlaceholderParser.cpp:843-961  static void scalar_variable_reference(const MyContext *ctx,
    //     OptWithPos<Iterator> &opt, expr<Iterator> &output)
    pub fn scalar_variable_reference(ctx: &MyContext, src: &[u8], opt: &OptWithPos, output: &mut Expr) -> TResult<()> {
        if opt.opt.is_vector() {
            match opt.opt.r#type() {
                ConfigOptionType::CoFloats => {
                    let values = float_values(opt.opt).expect("coFloats holds a vector of doubles");
                    let index = if values.len() == 1 {
                        // old version
                        0
                    } else {
                        ctx.get_extruder_id()
                    };
                    // FIDELITY-NOTE: get_at() of an empty vector reads past its end in the C++.
                    match vector_get_at(values, index) {
                        Some(v) => output.set_d(*v),
                        None => return Err(MyContext::throw_exception("Indexing an empty vector variable", opt.it_range)),
                    }
                }
                ConfigOptionType::CoFloatsOrPercents => {
                    let values = opt
                        .opt
                        .vector_values_any()
                        .and_then(|v| v.downcast_ref::<Vec<FloatOrPercent>>())
                        .expect("coFloatsOrPercents holds a vector of FloatOrPercent");
                    let mut index = 0usize; // old version only one value
                    if values.len() > 1 {
                        index = ctx.get_extruder_id();
                    }
                    let opt_key = range_string(src, opt.it_range);
                    let opt_value = match vector_get_at(values, index) {
                        Some(v) => *v,
                        None => return Err(MyContext::throw_exception("Indexing an empty vector variable", opt.it_range)),
                    };
                    if !opt_value.percent {
                        // Not a percent, just return the value.
                        output.set_d(opt_value.value);
                    } else {
                        let mut v = opt_value.value;
                        // Resolve dependencies using the "ratio_over" link to a parent value.
                        let opt_def: Option<&ConfigOptionDef> = print_config_def().get(&opt_key);
                        v *= 0.01;
                        loop {
                            let ratio_over = opt_def.map(|d| d.ratio_over.as_str()).unwrap_or("");
                            let opt_parent = if ratio_over.is_empty() { None } else { ctx.resolve_symbol(ratio_over) };
                            let opt_parent = match opt_parent {
                                None => {
                                    return Err(MyContext::throw_exception(
                                        "FloatOrPercentNullable variable failed to resolve the \"ratio_over\" dependencies",
                                        opt.it_range,
                                    ))
                                }
                                Some(p) => p,
                            };
                            let pt = opt_parent.r#type();
                            if pt == ConfigOptionType::CoFloat || pt == ConfigOptionType::CoFloatOrPercent {
                                v *= opt_parent.get_float().map_err(cfg_err)?;
                                if pt == ConfigOptionType::CoFloat || !float_or_percent_is_percent(opt_parent) {
                                    break;
                                }
                                // v *= 0.01; // percent to ratio
                            }
                            // FIDELITY-NOTE: the C++ loop does not step to the parent definition, so a
                            // parent that is a percentage or not a float makes it spin forever; that case
                            // is reported as an unresolved dependency here.
                            return Err(MyContext::throw_exception(
                                "FloatOrPercentNullable variable failed to resolve the \"ratio_over\" dependencies",
                                opt.it_range,
                            ));
                        }
                        output.set_d(v);
                    }
                }
                _ => return Err(MyContext::throw_exception("Unknown scalar variable type", opt.it_range)),
            }
        } else {
            match opt.opt.r#type() {
                ConfigOptionType::CoFloat => {
                    let opt_key = range_string(src, opt.it_range);
                    if opt_key.ends_with("line_width") {
                        output.set_d(flow_extrusion_width(&opt_key, ctx, ctx.current_extruder_id.get() as u32)?);
                    } else {
                        output.set_d(opt.opt.get_float().map_err(cfg_err)?);
                    }
                }
                ConfigOptionType::CoInt => output.set_i(opt.opt.get_int().map_err(cfg_err)?),
                ConfigOptionType::CoString => {
                    let s = opt.opt.as_any().downcast_ref::<ConfigOptionString>().expect("coString is a ConfigOptionString");
                    output.set_s(s.value.clone());
                }
                ConfigOptionType::CoPercent => output.set_d(opt.opt.get_float().map_err(cfg_err)?),
                ConfigOptionType::CoPoint => output.set_s(opt.opt.serialize().map_err(cfg_err)?),
                ConfigOptionType::CoBool => output.set_b(opt.opt.get_bool().map_err(cfg_err)?),
                ConfigOptionType::CoFloatOrPercent => {
                    let opt_key = range_string(src, opt.it_range);
                    if !float_or_percent_is_percent(opt.opt) {
                        // Not a percent, just return the value.
                        output.set_d(opt.opt.get_float().map_err(cfg_err)?);
                    } else {
                        // Resolve dependencies using the "ratio_over" link to a parent value.
                        let mut opt_def: Option<&ConfigOptionDef> = print_config_def().get(&opt_key);
                        let mut v = opt.opt.get_float().map_err(cfg_err)? * 0.01; // percent to ratio
                        loop {
                            // FIDELITY-NOTE: a definition missing from print_config_def is a null
                            // dereference in the C++; it is treated as one without "ratio_over".
                            let ratio_over = opt_def.map(|d| d.ratio_over.as_str()).unwrap_or("");
                            let opt_parent = if ratio_over.is_empty() { None } else { ctx.resolve_symbol(ratio_over) };
                            let opt_parent = match opt_parent {
                                None => {
                                    return Err(MyContext::throw_exception(
                                        "FloatOrPercent variable failed to resolve the \"ratio_over\" dependencies",
                                        opt.it_range,
                                    ))
                                }
                                Some(p) => p,
                            };
                            if ratio_over.ends_with("line_width") {
                                // Extrusion width supports defaults and a complex graph of dependencies.
                                if opt_parent.r#type() != ConfigOptionType::CoFloat {
                                    return Err(MyContext::throw_exception("Line width variable has invalid type", opt.it_range));
                                }
                                v *= flow_extrusion_width_opt(
                                    ratio_over,
                                    opt_parent.get_float().map_err(cfg_err)?,
                                    ctx,
                                    ctx.current_extruder_id.get() as u32,
                                )?;
                                break;
                            }
                            let pt = opt_parent.r#type();
                            if pt == ConfigOptionType::CoFloat || pt == ConfigOptionType::CoFloatOrPercent {
                                v *= opt_parent.get_float().map_err(cfg_err)?;
                                if pt == ConfigOptionType::CoFloat || !float_or_percent_is_percent(opt_parent) {
                                    break;
                                }
                                v *= 0.01; // percent to ratio
                            }
                            // Continue one level up in the "ratio_over" hierarchy.
                            opt_def = print_config_def().get(ratio_over);
                        }
                        output.set_d(v);
                    }
                }
                // BBS: Add enum. Otherwise enum can not be judged in placeholder
                ConfigOptionType::CoEnum => output.set_s(opt.opt.serialize().map_err(cfg_err)?),
                _ => return Err(MyContext::throw_exception("Unknown scalar variable type", opt.it_range)),
            }
        }
        output.it_range = opt.it_range;
        Ok(())
    }

    // PlaceholderParser.cpp:963-990  static void vector_variable_reference(const MyContext *ctx,
    //     OptWithPos<Iterator> &opt, int &index, Iterator it_end, expr<Iterator> &output)
    pub fn vector_variable_reference(
        _ctx: &MyContext,
        opt: &OptWithPos,
        index: i32,
        it_end: usize,
        output: &mut Expr,
    ) -> TResult<()> {
        if opt.opt.is_scalar() {
            return Err(MyContext::throw_exception("Referencing a scalar variable when vector is expected", opt.it_range));
        }
        let vec = opt.opt.as_vector_base().expect("a vector option is a ConfigOptionVectorBase");
        if vec.empty() {
            return Err(MyContext::throw_exception("Indexing an empty vector variable", opt.it_range));
        }
        let idx: usize = if index < 0 {
            0
        } else if index >= vec.size() as i32 {
            0
        } else {
            index as usize
        };
        let any = opt.opt.vector_values_any();
        match opt.opt.r#type() {
            ConfigOptionType::CoFloats | ConfigOptionType::CoPercents => {
                output.set_d(any.and_then(|v| v.downcast_ref::<Vec<f64>>()).expect("vector of doubles")[idx])
            }
            ConfigOptionType::CoInts => output.set_i(any.and_then(|v| v.downcast_ref::<Vec<i32>>()).expect("vector of ints")[idx]),
            ConfigOptionType::CoStrings => {
                output.set_s(any.and_then(|v| v.downcast_ref::<Vec<String>>()).expect("vector of strings")[idx].clone())
            }
            ConfigOptionType::CoPoints => {
                let p: &Vec2d = &any.and_then(|v| v.downcast_ref::<Vec<Vec2d>>()).expect("vector of points")[idx];
                // Point.hpp:139 to_string(const Vec2d &pt)
                output.set_s(format!(
                    "[{}, {}]",
                    crate::locales_utils::float_to_string_decimal_point(p.x, -1),
                    crate::locales_utils::float_to_string_decimal_point(p.y, -1)
                ))
            }
            ConfigOptionType::CoBools => output.set_b(any.and_then(|v| v.downcast_ref::<Vec<u8>>()).expect("vector of bools")[idx] != 0),
            // BBS
            ConfigOptionType::CoEnums => output.set_i(any.and_then(|v| v.downcast_ref::<Vec<i32>>()).expect("vector of enums")[idx]),
            _ => return Err(MyContext::throw_exception("Unknown vector variable type", opt.it_range)),
        }
        output.it_range = (opt.it_range.0, it_end);
        Ok(())
    }

    // PlaceholderParser.cpp:992-1000  static void evaluate_index(expr<Iterator> &expr_index, int &output)
    // Verify that the expression returns an integer, which may be used to address a vector.
    pub fn evaluate_index(expr_index: &Expr) -> TResult<i32> {
        if expr_index.type_() != Type::TypeInt {
            return Err(expr_index.throw_exception("Non-integer index is not allowed to address a vector variable."));
        }
        Ok(expr_index.i())
    }

    // PlaceholderParser.cpp:1002-1009  static void random(const MyContext *ctx, expr<Iterator> &param1, expr<Iterator> &param2)
    pub fn random(ctx: &MyContext, param1: &mut Expr, param2: &Expr) -> TResult<()> {
        match &ctx.context_data {
            None => Err(MyContext::throw_exception(
                "Random number generator not available in this context.",
                (param1.it_range.0, param2.it_range.1),
            )),
            Some(cd) => {
                let mut cd = cd.borrow_mut();
                Expr::random(param1, param2, &mut cd.rng)
            }
        }
    }

    // PlaceholderParser.cpp:1011-1016  static void filament_change(const MyContext* ctx, expr<Iterator>& param)
    pub fn filament_change(ctx: &MyContext, param: &Expr) {
        // MyContext *context = const_cast<MyContext *>(ctx); context->current_extruder_id = param.as_i();
        // FIDELITY-NOTE: as_i() of a non-numeric value reads the union's double in the C++
        // (garbage); here it is 0.
        ctx.current_extruder_id.set(param.as_i() as usize);
    }

    // PlaceholderParser.cpp:1018-1024  static void throw_exception(const std::string &msg, const boost::iterator_range<Iterator> &it_range)
    pub fn throw_exception(msg: &str, it_range: (usize, usize)) -> Throw {
        // An asterix is added to the start of the string to differentiate the boost::spirit::info::tag content
        // between the grammer terminal / non-terminal symbol name and a free-form error message.
        expectation_failure(it_range.0, it_range.1, &format!("*{}", msg))
    }

    // PlaceholderParser.cpp:1026-1072  static void process_error_message(const MyContext *context,
    //     const boost::spirit::info &info, const Iterator &it_begin, const Iterator &it_end, const Iterator &it_error)
    pub fn process_error_message(context: &MyContext, info_tag: &str, src: &[u8], it_begin: usize, it_end: usize, it_error: usize) {
        let mut msg = context.error_message.borrow_mut();
        let it_error = it_error.min(it_end);
        let first = &src[it_begin..it_error];
        let last = &src[it_error..it_end];
        let first_pos_opt = first.iter().rposition(|&c| c == b'\n');
        let last_pos = last.iter().position(|&c| c == b'\n').unwrap_or(last.len());
        let mut line_nr: i32 = 1;
        let first_pos = match first_pos_opt {
            None => 0,
            Some(p) => {
                // Calculate the current line number.
                for &c in &first[..=p] {
                    if c == b'\n' {
                        line_nr += 1;
                    }
                }
                p + 1
            }
        };
        let mut error_line: Vec<u8> = first[first_pos..].to_vec();
        error_line.extend_from_slice(&last[..last_pos]);
        // Position of the it_error from the start of its line.
        let error_pos = (it_error - it_begin) - first_pos;
        msg.push_str("Parsing error at line ");
        msg.push_str(&line_nr.to_string());
        if !info_tag.is_empty() && info_tag.starts_with('*') {
            // The gat contains an explanatory string.
            msg.push_str(": ");
            msg.push_str(&info_tag[1..]);
        } else {
            match tag_to_error_message(info_tag) {
                None => {
                    // A generic error report based on the nonterminal or terminal symbol name.
                    msg.push_str(". Expecting tag ");
                    msg.push_str(info_tag);
                }
                Some(m) => {
                    // Use the human readable error message.
                    msg.push_str(". ");
                    msg.push_str(m);
                }
            }
        }
        msg.push('\n');
        // This hack removes all non-UTF8 characters from the source line, so that the upstream wxWidgets conversions
        // from UTF8 to UTF16 don't bail out.
        msg.push_str(&nowide_narrow_widen(&error_line));
        msg.push('\n');
        for _ in 0..error_pos {
            msg.push(' ');
        }
        msg.push_str("^\n");
    }
}

/// `ConfigOptionFloatOrPercent::percent` of a coFloatOrPercent option.
fn float_or_percent_is_percent(opt: &dyn ConfigOption) -> bool {
    opt.single_value_any()
        .and_then(|v| v.downcast_ref::<FloatOrPercent>())
        .map(|v| v.percent)
        .or_else(|| opt.as_any().downcast_ref::<crate::config::ConfigOptionFloatOrPercent>().map(|o| o.percent))
        .unwrap_or(false)
}

/// `boost::nowide::narrow(boost::nowide::widen(s))`: `boost::locale::conv::utf_to_utf` with the
/// `skip` method both ways, i.e. the valid UTF-8 sequences of `s` with every illegal or
/// incomplete one dropped (boost/locale/utf.hpp `utf_traits<char>::decode`).
fn nowide_narrow_widen(s: &[u8]) -> String {
    fn trail_length(c: u8) -> i32 {
        if c < 128 {
            return 0;
        }
        if c < 194 {
            return -1;
        }
        if c < 224 {
            return 1;
        }
        if c < 240 {
            return 2;
        }
        if c <= 244 {
            return 3;
        }
        -1
    }
    fn width(v: u32) -> i32 {
        if v <= 0x7f {
            1
        } else if v <= 0x7ff {
            2
        } else if v <= 0xffff {
            3
        } else {
            4
        }
    }
    let mut out = String::with_capacity(s.len());
    let mut p = 0usize;
    let e = s.len();
    while p != e {
        let lead = s[p];
        p += 1;
        let trail_size = trail_length(lead);
        if trail_size < 0 {
            continue; // illegal
        }
        if trail_size == 0 {
            out.push(lead as char);
            continue;
        }
        let mut c: u32 = (lead as u32) & ((1u32 << (6 - trail_size)) - 1);
        let mut ok = true;
        for _ in 0..trail_size {
            if p == e {
                ok = false; // incomplete
                break;
            }
            let tmp = s[p];
            p += 1;
            if (tmp & 0xc0) != 0x80 {
                ok = false; // illegal
                break;
            }
            c = (c << 6) | (tmp as u32 & 0x3f);
        }
        if !ok {
            continue;
        }
        if c > 0x10ffff || (0xd800..=0xdfff).contains(&c) {
            continue;
        }
        if width(c) != trail_size + 1 {
            continue;
        }
        if let Some(ch) = char::from_u32(c) {
            out.push(ch);
        }
    }
    out
}

// =====================================================================
// The macro_processor grammar  (PlaceholderParser.cpp:1101-1513)
// =====================================================================

/// PlaceholderParser.cpp:1420-1440 `keywords`
const KEYWORDS: [&str; 19] = [
    "and",
    "digits",
    "zdigits",
    "if",
    "int",
    //"inf"
    "else",
    "elsif",
    "endif",
    "false",
    "min",
    "max",
    "random",
    "filament_change",
    "round",
    "floor",
    "ceil",
    "not",
    "or",
    "true",
];

/// PlaceholderParser.cpp:1116-1177 `struct utf8_char_skipper_parser`: to be used inside a raw[]
/// directive to accept a single valid UTF-8 character. If an invalid UTF-8 sequence is
/// encountered, a qi::expectation_failure is thrown.
pub struct Utf8CharSkipperParser;

impl Utf8CharSkipperParser {
    /// PlaceholderParser.cpp:1128-1169 `bool parse(Iterator& first, Iterator const& last, ...) const`
    pub fn parse(src: &[u8], first: &mut usize) -> TResult<bool> {
        let last = src.len();
        // The skipper shall always be empty, any white space will be accepted.
        // skip_over(first, last, skipper);
        if *first == last {
            return Ok(false);
        }
        // err: MyContext::throw_exception("Invalid utf8 sequence", boost::iterator_range<Iterator>(first, last));
        let err = |f: usize| Err(MyContext::throw_exception("Invalid utf8 sequence", (f, last)));
        // Iterator over the UTF-8 sequence.
        let mut it = *first;
        // Read the first byte of the UTF-8 sequence.
        let mut c = src[it];
        it += 1;
        // UTF-8 sequence must not start with a continuation character:
        if (c & 0xc0) == 0x80 {
            return err(*first);
        }
        // Skip high surrogate first if there is one.
        // If the most significant bit with a zero in it is in position
        // 8-N then there are N bytes in this UTF-8 sequence:
        let mut cnt: u32 = {
            let mut mask: u8 = 0x80;
            let mut result: u32 = 0;
            while c & mask != 0 {
                result += 1;
                mask >>= 1;
            }
            if result == 0 {
                1
            } else if result > 4 {
                4
            } else {
                result
            }
        };
        // Since we haven't read in a value, we need to validate the code points:
        cnt -= 1;
        while cnt > 0 {
            if it == last {
                return err(*first);
            }
            c = src[it];
            it += 1;
            // We must have a continuation byte:
            if cnt > 1 && (c & 0xc0) != 0x80 {
                return err(*first);
            }
            cnt -= 1;
        }
        *first = it;
        Ok(true)
    }

    /// PlaceholderParser.cpp:1171-1176 `spirit::info what(Context&) const`: this function is called
    /// during error handling to create a human readable string for the error context.
    pub fn what() -> &'static str {
        "unicode_char"
    }
}

/// PlaceholderParser.cpp:1109-1114 `struct strict_real_policies_without_nan_inf : public qi::strict_real_policies<double>`:
/// disable parsing int numbers (without decimals) and Inf/NaN symbols by the double parser.
pub struct StrictRealPoliciesWithoutNanInf;

impl StrictRealPoliciesWithoutNanInf {
    /// PlaceholderParser.cpp:1112 `template <typename It, typename Attr> static bool parse_nan(It&, It const&, Attr&) { return false; }`
    pub fn parse_nan(_first: &mut usize, _last: usize, _attr: &mut f64) -> bool {
        false
    }
    /// PlaceholderParser.cpp:1113 `template <typename It, typename Attr> static bool parse_inf(It&, It const&, Attr&) { return false; }`
    pub fn parse_inf(_first: &mut usize, _last: usize, _attr: &mut f64) -> bool {
        false
    }
}

/// `Ok(Some(v))`: the component matched; `Ok(None)`: it did not (the C++ `return false`);
/// `Err(..)`: an exception.
type PResult<T> = TResult<Option<T>>;

/// PlaceholderParser.cpp:1183-1513 `template <typename Iterator> struct macro_processor`.
///
/// FIDELITY-NOTE: hand-written in place of the Qi grammar, see the module documentation.
/// Each method is the rule of the same name; `first` is the iterator the rule is handed.
pub struct MacroProcessor<'s, 'c, 'a> {
    src: &'s [u8],
    ctx: &'c MyContext<'a>,
}

impl<'s, 'c, 'a> MacroProcessor<'s, 'c, 'a> {
    pub fn new(src: &'s [u8], ctx: &'c MyContext<'a>) -> Self {
        MacroProcessor { src, ctx }
    }

    fn len(&self) -> usize {
        self.src.len()
    }

    // ---- Qi primitives -------------------------------------------------------------------

    /// `qi::skip_over(first, last, space)`
    fn skip_over(&self, first: &mut usize) {
        while *first < self.len() && is_space(self.src[*first]) {
            *first += 1;
        }
    }

    /// `lit(ch)` / `'ch'` under the skipper: pre-skip, then test one character.
    fn lit_char(&self, first: &mut usize, ch: u8) -> bool {
        self.skip_over(first);
        if *first < self.len() && self.src[*first] == ch {
            *first += 1;
            true
        } else {
            false
        }
    }

    /// `lit("str")` under the skipper: pre-skip, then match the whole string or nothing.
    fn lit_str(&self, first: &mut usize, s: &str) -> bool {
        self.skip_over(first);
        let b = s.as_bytes();
        if self.src[*first..].starts_with(b) {
            *first += b.len();
            true
        } else {
            false
        }
    }

    /// The tail of `kw = distinct(qi::copy(alnum | '_'))`, parsed without a skipper.
    fn distinct_tail(&self, i: usize) -> bool {
        i < self.len() && (is_alnum(self.src[i]) || self.src[i] == b'_')
    }

    /// `kw["str"]`: `distinct_parser::parse` over `lit("str")` on a copy of the iterator.
    fn kw(&self, first: &mut usize, s: &str) -> bool {
        let mut iter = *first;
        if !self.lit_str(&mut iter, s) {
            return false;
        }
        if self.distinct_tail(iter) {
            return false;
        }
        *first = iter;
        true
    }

    /// `kw[keywords]`: the `qi::symbols<char>` table (longest match after a pre-skip) under `distinct`.
    fn kw_keywords(&self, first: &mut usize) -> bool {
        let mut iter = *first;
        self.skip_over(&mut iter);
        let mut best: Option<usize> = None;
        for k in KEYWORDS.iter() {
            if self.src[iter..].starts_with(k.as_bytes()) && best.map_or(true, |b| k.len() > b) {
                best = Some(k.len());
            }
        }
        let n = match best {
            None => return false,
            Some(n) => n,
        };
        iter += n;
        if self.distinct_tail(iter) {
            return false;
        }
        *first = iter;
        true
    }

    /// `eoi`: pre-skip, then test for the end of the input.
    fn eoi(&self, first: &mut usize) -> bool {
        self.skip_over(first);
        *first == self.len()
    }

    /// `iter_pos` (spirit repository): pre-skip, then expose the iterator.
    fn iter_pos(&self, first: &mut usize) -> usize {
        self.skip_over(first);
        *first
    }

    /// `utf8char`, the `utf8_char_skipper_parser` instance of the grammar (PlaceholderParser.cpp:1198).
    fn utf8char(&self, first: &mut usize) -> TResult<bool> {
        Utf8CharSkipperParser::parse(self.src, first)
    }

    /// `qi::real_parser<double, strict_real_policies_without_nan_inf>` (PlaceholderParser.cpp:1110-1114,
    /// boost/spirit/home/qi/numeric/detail/real_impl.hpp): pre-skip, then `real_impl::parse`.
    fn strict_double(&self, first: &mut usize) -> Option<f64> {
        self.skip_over(first);
        real_impl_parse(self.src, first)
    }

    /// `int_` (`qi::int_parser<int>`): pre-skip, optional sign, decimal digits; fails on overflow.
    fn int_(&self, first: &mut usize) -> Option<i32> {
        self.skip_over(first);
        extract_int_i32(self.src, first)
    }

    /// `kw[bool_]`: `bool_` (pre-skip, "true" or "false") under `distinct`.
    fn kw_bool(&self, first: &mut usize) -> Option<bool> {
        let mut iter = *first;
        self.skip_over(&mut iter);
        let value = if self.src[iter..].starts_with(b"true") {
            iter += 4;
            true
        } else if self.src[iter..].starts_with(b"false") {
            iter += 5;
            false
        } else {
            return None;
        };
        if self.distinct_tail(iter) {
            return None;
        }
        *first = iter;
        Some(value)
    }

    fn substr(&self, a: usize, b: usize) -> String {
        String::from_utf8_lossy(&self.src[a..b]).into_owned()
    }

    // ---- rules ---------------------------------------------------------------------------

    /// PlaceholderParser.cpp:1222-1227 `start`
    pub fn start(&self, first: &mut usize) -> TResult<String> {
        let mut iter = *first;
        // eps[px::bind(&MyContext::evaluate_full_macro, _r1, _a)]
        self.skip_over(&mut iter);
        let a = MyContext::evaluate_full_macro(self.ctx);
        // > ( (eps(_a==true) > text_block(_r1) [_val=_1]) | conditional_expression(_r1) [evaluate_boolean_to_string] )
        let val;
        {
            let mut it = iter;
            self.skip_over(&mut it);
            if a {
                val = self.text_block(&mut it)?;
                iter = it;
            } else {
                match self.conditional_expression(&mut iter)? {
                    Some(e) => val = e.evaluate_boolean_to_string()?,
                    None => return Err(expectation_failure(iter, iter, "alternative")),
                }
            }
        }
        // > eoi
        if !self.eoi(&mut iter) {
            return Err(expectation_failure(iter, iter, "eoi"));
        }
        *first = iter;
        Ok(val)
    }

    /// PlaceholderParser.cpp:1229-1237 `text_block`
    pub fn text_block(&self, first: &mut usize) -> TResult<String> {
        let mut iter = *first;
        let mut val = String::new();
        loop {
            // text [_val+=_1]
            if let Some(t) = self.text(&mut iter)? {
                val += &t;
                continue;
            }
            // (lit('{') >> macro(_r1) [_val+=_1] > '}')
            {
                let mut it = iter;
                let mut it2 = it;
                let m = if self.lit_char(&mut it2, b'{') { self.macro_(&mut it2)? } else { None };
                if let Some(m) = m {
                    it = it2;
                    val += &m;
                    if !self.lit_char(&mut it, b'}') {
                        return Err(expectation_failure(it, it, "literal-char"));
                    }
                    iter = it;
                    continue;
                }
            }
            // (lit('[') > legacy_variable_expansion(_r1) [_val+=_1] > ']')
            {
                let mut it = iter;
                if self.lit_char(&mut it, b'[') {
                    match self.legacy_variable_expansion(&mut it)? {
                        Some(s) => val += &s,
                        None => return Err(expectation_failure(it, it, "legacy_variable_expansion")),
                    }
                    if !self.lit_char(&mut it, b']') {
                        return Err(expectation_failure(it, it, "literal-char"));
                    }
                    iter = it;
                    continue;
                }
            }
            break;
        }
        *first = iter;
        Ok(val)
    }

    /// PlaceholderParser.cpp:1241 `text = no_skip[raw[+(utf8char - char_('[') - char_('{'))]]`
    pub fn text(&self, first: &mut usize) -> PResult<String> {
        let start = *first;
        let mut i = start;
        let mut count = 0usize;
        loop {
            if i < self.len() && (self.src[i] == b'{' || self.src[i] == b'[') {
                break;
            }
            if !self.utf8char(&mut i)? {
                break;
            }
            count += 1;
        }
        if count == 0 {
            return Ok(None);
        }
        *first = i;
        Ok(Some(self.substr(start, i)))
    }

    /// PlaceholderParser.cpp:1246-1250 `macro`
    pub fn macro_(&self, first: &mut usize) -> PResult<String> {
        // (kw["if"] > if_else_output(_r1) [_val = _1])
        {
            let mut it = *first;
            if self.kw(&mut it, "if") {
                match self.if_else_output(&mut it)? {
                    Some(s) => {
                        *first = it;
                        return Ok(Some(s));
                    }
                    None => return Err(expectation_failure(it, it, "if_else_output")),
                }
            }
        }
        // | additive_expression(_r1) [ px::bind(&expr<Iterator>::to_string2, _1, _val) ]
        match self.additive_expression(first)? {
            Some(e) => Ok(Some(e.to_string2())),
            None => Ok(None),
        }
    }

    /// PlaceholderParser.cpp:1253-1261 `if_else_output`
    pub fn if_else_output(&self, first: &mut usize) -> PResult<String> {
        let mut iter = *first;
        let mut val = String::new();
        let mut a: bool;
        // eps[_b=true]
        self.skip_over(&mut iter);
        let mut b = true;
        // > bool_expr_eval(_r1)[_a=_1] > '}' > text_block(_r1)[set_if(_a, _b, _1, _val)] > '{'
        match self.bool_expr_eval(&mut iter)? {
            Some(v) => a = v,
            None => return Err(expectation_failure(iter, iter, "bool_expr_eval")),
        }
        if !self.lit_char(&mut iter, b'}') {
            return Err(expectation_failure(iter, iter, "literal-char"));
        }
        let tb = self.text_block(&mut iter)?;
        Expr::set_if(a, &mut b, &tb, &mut val);
        if !self.lit_char(&mut iter, b'{') {
            return Err(expectation_failure(iter, iter, "literal-char"));
        }
        // > *(kw["elsif"] > bool_expr_eval(_r1)[_a=_1] > '}' > text_block(_r1)[set_if] > '{')
        {
            let mut k = iter;
            loop {
                let mut it = k;
                if !self.kw(&mut it, "elsif") {
                    break;
                }
                match self.bool_expr_eval(&mut it)? {
                    Some(v) => a = v,
                    None => return Err(expectation_failure(it, it, "bool_expr_eval")),
                }
                if !self.lit_char(&mut it, b'}') {
                    return Err(expectation_failure(it, it, "literal-char"));
                }
                let tb = self.text_block(&mut it)?;
                Expr::set_if(a, &mut b, &tb, &mut val);
                if !self.lit_char(&mut it, b'{') {
                    return Err(expectation_failure(it, it, "literal-char"));
                }
                k = it;
            }
            iter = k;
        }
        // > -(kw["else"] > lit('}') > text_block(_r1)[set_if(_b, _b, _1, _val)] > '{')
        {
            let mut it = iter;
            if self.kw(&mut it, "else") {
                if !self.lit_char(&mut it, b'}') {
                    return Err(expectation_failure(it, it, "literal-char"));
                }
                let tb = self.text_block(&mut it)?;
                let cond = b;
                Expr::set_if(cond, &mut b, &tb, &mut val);
                if !self.lit_char(&mut it, b'{') {
                    return Err(expectation_failure(it, it, "literal-char"));
                }
                iter = it;
            }
        }
        // > kw["endif"]
        if !self.kw(&mut iter, "endif") {
            return Err(expectation_failure(iter, iter, "distinct"));
        }
        *first = iter;
        Ok(Some(val))
    }

    /// PlaceholderParser.cpp:1274-1279 `legacy_variable_expansion`
    pub fn legacy_variable_expansion(&self, first: &mut usize) -> PResult<String> {
        // (identifier >> &lit(']')) [ legacy_variable_expansion(_r1, _1, _val) ]
        {
            let mut it = *first;
            if let Some(id) = self.identifier(&mut it)? {
                let mut probe = it;
                if self.lit_char(&mut probe, b']') {
                    let out = MyContext::legacy_variable_expansion(self.ctx, self.src, id)?;
                    *first = it;
                    return Ok(Some(out));
                }
            }
        }
        // | (identifier > lit('[') > identifier > ']') [ legacy_variable_expansion2(_r1, _1, _2, _val) ]
        {
            let mut it = *first;
            if let Some(id1) = self.identifier(&mut it)? {
                if !self.lit_char(&mut it, b'[') {
                    return Err(expectation_failure(it, it, "literal-char"));
                }
                let id2 = match self.identifier(&mut it)? {
                    Some(id) => id,
                    None => return Err(expectation_failure(it, it, "identifier")),
                };
                if !self.lit_char(&mut it, b']') {
                    return Err(expectation_failure(it, it, "literal-char"));
                }
                let out = MyContext::legacy_variable_expansion2(self.ctx, self.src, id1, id2)?;
                *first = it;
                return Ok(Some(out));
            }
        }
        Ok(None)
    }

    /// PlaceholderParser.cpp:1282-1284 `identifier = ! kw[keywords] >> raw[lexeme[(alpha | '_') >> *(alnum | '_')]]`
    pub fn identifier(&self, first: &mut usize) -> PResult<(usize, usize)> {
        let mut iter = *first;
        {
            let mut probe = iter;
            if self.kw_keywords(&mut probe) {
                return Ok(None);
            }
        }
        // raw[...] and lexeme[...] pre-skip.
        self.skip_over(&mut iter);
        let begin = iter;
        if iter < self.len() && (is_alpha(self.src[iter]) || self.src[iter] == b'_') {
            iter += 1;
        } else {
            return Ok(None);
        }
        while iter < self.len() && (is_alnum(self.src[iter]) || self.src[iter] == b'_') {
            iter += 1;
        }
        *first = iter;
        Ok(Some((begin, iter)))
    }

    /// PlaceholderParser.cpp:1287-1290 `conditional_expression`
    pub fn conditional_expression(&self, first: &mut usize) -> PResult<Expr> {
        let mut iter = *first;
        let mut val = match self.logical_or_expression(&mut iter)? {
            Some(e) => e,
            None => return Ok(None),
        };
        // >> -('?' > conditional_expression(_r1) > ':' > conditional_expression(_r1)) [ternary_op(_val, _1, _2)]
        {
            let mut it = iter;
            if self.lit_char(&mut it, b'?') {
                let e1 = match self.conditional_expression(&mut it)? {
                    Some(e) => e,
                    None => return Err(expectation_failure(it, it, "conditional_expression")),
                };
                if !self.lit_char(&mut it, b':') {
                    return Err(expectation_failure(it, it, "literal-char"));
                }
                let e2 = match self.conditional_expression(&mut it)? {
                    Some(e) => e,
                    None => return Err(expectation_failure(it, it, "conditional_expression")),
                };
                Expr::ternary_op(&mut val, e1, e2)?;
                iter = it;
            }
        }
        *first = iter;
        Ok(Some(val))
    }

    /// PlaceholderParser.cpp:1292-1295 `logical_or_expression`
    pub fn logical_or_expression(&self, first: &mut usize) -> PResult<Expr> {
        let mut iter = *first;
        let mut val = match self.logical_and_expression(&mut iter)? {
            Some(e) => e,
            None => return Ok(None),
        };
        // >> *( ((kw["or"] | "||") > logical_and_expression(_r1)) [logical_or(_val, _1)] )
        let mut k = iter;
        loop {
            let mut it = k;
            let op = self.kw(&mut it, "or") || self.lit_str(&mut it, "||");
            if !op {
                break;
            }
            let rhs = match self.logical_and_expression(&mut it)? {
                Some(e) => e,
                None => return Err(expectation_failure(it, it, "logical_and_expression")),
            };
            Expr::logical_or(&mut val, &rhs)?;
            k = it;
        }
        iter = k;
        *first = iter;
        Ok(Some(val))
    }

    /// PlaceholderParser.cpp:1297-1300 `logical_and_expression`
    pub fn logical_and_expression(&self, first: &mut usize) -> PResult<Expr> {
        let mut iter = *first;
        let mut val = match self.equality_expression(&mut iter)? {
            Some(e) => e,
            None => return Ok(None),
        };
        // >> *( ((kw["and"] | "&&") > equality_expression(_r1)) [logical_and(_val, _1)] )
        let mut k = iter;
        loop {
            let mut it = k;
            let op = self.kw(&mut it, "and") || self.lit_str(&mut it, "&&");
            if !op {
                break;
            }
            // equality_expression.name("bool expression")
            let rhs = match self.equality_expression(&mut it)? {
                Some(e) => e,
                None => return Err(expectation_failure(it, it, "bool expression")),
            };
            Expr::logical_and(&mut val, &rhs)?;
            k = it;
        }
        iter = k;
        *first = iter;
        Ok(Some(val))
    }

    /// PlaceholderParser.cpp:1302-1310 `equality_expression`
    pub fn equality_expression(&self, first: &mut usize) -> PResult<Expr> {
        let mut iter = *first;
        let mut val = match self.relational_expression(&mut iter)? {
            Some(e) => e,
            None => return Ok(None),
        };
        let mut k = iter;
        loop {
            // ("==" > relational_expression(_r1)) [equal(_val, _1)]
            // | ("!=" > relational_expression(_r1)) [not_equal(_val, _1)]
            // | ("<>" > relational_expression(_r1)) [not_equal(_val, _1)]
            let mut matched = false;
            for (op, invert) in [("==", false), ("!=", true), ("<>", true)] {
                let mut it = k;
                if self.lit_str(&mut it, op) {
                    let rhs = match self.relational_expression(&mut it)? {
                        Some(e) => e,
                        None => return Err(expectation_failure(it, it, "relational_expression")),
                    };
                    if invert {
                        Expr::not_equal(&mut val, &rhs)?;
                    } else {
                        Expr::equal(&mut val, &rhs)?;
                    }
                    k = it;
                    matched = true;
                    break;
                }
            }
            if matched {
                continue;
            }
            // | ("=~" > regular_expression) [regex_matches(_val, _1)]
            // | ("!~" > regular_expression) [regex_doesnt_match(_val, _1)]
            for (op, negate) in [("=~", false), ("!~", true)] {
                let mut it = k;
                if self.lit_str(&mut it, op) {
                    let r = match self.regular_expression(&mut it)? {
                        Some(r) => r,
                        None => return Err(expectation_failure(it, it, "regular_expression")),
                    };
                    if negate {
                        Expr::regex_doesnt_match(&mut val, &self.src[r.0..r.1], r)?;
                    } else {
                        Expr::regex_matches(&mut val, &self.src[r.0..r.1], r)?;
                    }
                    k = it;
                    matched = true;
                    break;
                }
            }
            if !matched {
                break;
            }
        }
        iter = k;
        *first = iter;
        Ok(Some(val))
    }

    /// PlaceholderParser.cpp:1314 `bool_expr_eval = conditional_expression(_r1) [evaluate_boolean(_1, _val)]`
    pub fn bool_expr_eval(&self, first: &mut usize) -> PResult<bool> {
        match self.conditional_expression(first)? {
            Some(e) => Ok(Some(e.evaluate_boolean()?)),
            None => Ok(None),
        }
    }

    /// PlaceholderParser.cpp:1317-1323 `relational_expression`
    pub fn relational_expression(&self, first: &mut usize) -> PResult<Expr> {
        let mut iter = *first;
        let mut val = match self.additive_expression(&mut iter)? {
            Some(e) => e,
            None => return Ok(None),
        };
        let mut k = iter;
        loop {
            let mut matched = false;
            // ("<=" > ...)[leq] | (">=" > ...)[geq] | (lit('<') > ...)[lower] | (lit('>') > ...)[greater]
            for op in ["<=", ">=", "<", ">"] {
                let mut it = k;
                if self.lit_str(&mut it, op) {
                    let rhs = match self.additive_expression(&mut it)? {
                        Some(e) => e,
                        None => return Err(expectation_failure(it, it, "additive_expression")),
                    };
                    match op {
                        "<=" => Expr::leq(&mut val, &rhs)?,
                        ">=" => Expr::geq(&mut val, &rhs)?,
                        "<" => Expr::lower(&mut val, &rhs)?,
                        _ => Expr::greater(&mut val, &rhs)?,
                    }
                    k = it;
                    matched = true;
                    break;
                }
            }
            if !matched {
                break;
            }
        }
        iter = k;
        *first = iter;
        Ok(Some(val))
    }

    /// PlaceholderParser.cpp:1326-1331 `additive_expression`
    pub fn additive_expression(&self, first: &mut usize) -> PResult<Expr> {
        let mut iter = *first;
        let mut val = match self.multiplicative_expression(&mut iter)? {
            Some(e) => e,
            None => return Ok(None),
        };
        let mut k = iter;
        loop {
            let mut matched = false;
            // (lit('+') > multiplicative_expression(_r1)) [_val += _1]
            // | (lit('-') > multiplicative_expression(_r1)) [_val -= _1]
            for op in [b'+', b'-'] {
                let mut it = k;
                if self.lit_char(&mut it, op) {
                    let rhs = match self.multiplicative_expression(&mut it)? {
                        Some(e) => e,
                        None => return Err(expectation_failure(it, it, "multiplicative_expression")),
                    };
                    if op == b'+' {
                        val.add_assign(&rhs)?;
                    } else {
                        val.sub_assign(&rhs)?;
                    }
                    k = it;
                    matched = true;
                    break;
                }
            }
            if !matched {
                break;
            }
        }
        iter = k;
        *first = iter;
        Ok(Some(val))
    }

    /// PlaceholderParser.cpp:1333-1339 `multiplicative_expression`
    pub fn multiplicative_expression(&self, first: &mut usize) -> PResult<Expr> {
        let mut iter = *first;
        let mut val = match self.unary_expression(&mut iter)? {
            Some(e) => e,
            None => return Ok(None),
        };
        let mut k = iter;
        loop {
            let mut matched = false;
            for op in [b'*', b'/', b'%'] {
                let mut it = k;
                if self.lit_char(&mut it, op) {
                    let rhs = match self.unary_expression(&mut it)? {
                        Some(e) => e,
                        None => return Err(expectation_failure(it, it, "unary_expression")),
                    };
                    match op {
                        b'*' => val.mul_assign(&rhs)?,
                        b'/' => val.div_assign(&rhs)?,
                        _ => val.rem_assign(&rhs)?,
                    }
                    k = it;
                    matched = true;
                    break;
                }
            }
            if !matched {
                break;
            }
        }
        iter = k;
        *first = iter;
        Ok(Some(val))
    }

    /// `kw["name"] > '(' > conditional_expression(_r1)`: the head shared by the function alternatives
    /// of `unary_expression`. `None`: the keyword did not match.
    fn function_head(&self, it: &mut usize, name: &str) -> PResult<Expr> {
        if !self.kw(it, name) {
            return Ok(None);
        }
        if !self.lit_char(it, b'(') {
            return Err(expectation_failure(*it, *it, "literal-char"));
        }
        match self.conditional_expression(it)? {
            Some(e) => Ok(Some(e)),
            None => Err(expectation_failure(*it, *it, "conditional_expression")),
        }
    }

    /// `> ',' > conditional_expression(_r1)`
    fn function_second(&self, it: &mut usize) -> TResult<Expr> {
        if !self.lit_char(it, b',') {
            return Err(expectation_failure(*it, *it, "literal-char"));
        }
        match self.conditional_expression(it)? {
            Some(e) => Ok(e),
            None => Err(expectation_failure(*it, *it, "conditional_expression")),
        }
    }

    /// `> ')'`
    fn function_close(&self, it: &mut usize) -> TResult<()> {
        if !self.lit_char(it, b')') {
            return Err(expectation_failure(*it, *it, "literal-char"));
        }
        Ok(())
    }

    /// PlaceholderParser.cpp:1341-1395 `unary_expression` with its `FactorActions`.
    pub fn unary_expression(&self, first: &mut usize) -> PResult<Expr> {
        let mut iter = *first;
        // iter_pos[px::bind(&FactorActions::set_start_pos, _1, _val)]
        let start_pos = self.iter_pos(&mut iter);
        let mut val = Expr::new();
        val.it_range = (start_pos, start_pos);
        if !self.unary_alternatives(&mut iter, &mut val)? {
            return Ok(None);
        }
        *first = iter;
        Ok(Some(val))
    }

    /// The alternatives of `unary_expression`, handed the iterator by reference.
    fn unary_alternatives(&self, iter: &mut usize, val: &mut Expr) -> TResult<bool> {
        let begin = val.it_range.0;
        // scalar_variable_reference(_r1) [ _val = _1 ]
        if let Some(e) = self.scalar_variable_reference(iter)? {
            *val = e;
            return Ok(true);
        }
        // (lit('(') > conditional_expression(_r1) > ')' > iter_pos) [ FactorActions::expr_(_1, _2, _val) ]
        {
            let mut it = *iter;
            if self.lit_char(&mut it, b'(') {
                let e = match self.conditional_expression(&mut it)? {
                    Some(e) => e,
                    None => return Err(expectation_failure(it, it, "conditional_expression")),
                };
                self.function_close(&mut it)?;
                let end_pos = self.iter_pos(&mut it);
                *val = Expr::from_expr_range(e, begin, end_pos);
                *iter = it;
                return Ok(true);
            }
        }
        // (lit('-') > unary_expression(_r1)) [ FactorActions::minus_(_1, _val) ]
        {
            let mut it = *iter;
            if self.lit_char(&mut it, b'-') {
                let e = match self.unary_expression(&mut it)? {
                    Some(e) => e,
                    None => return Err(expectation_failure(it, it, "unary_expression")),
                };
                *val = e.unary_minus(begin)?;
                *iter = it;
                return Ok(true);
            }
        }
        // (lit('+') > unary_expression(_r1) > iter_pos) [ FactorActions::expr_(_1, _2, _val) ]
        {
            let mut it = *iter;
            if self.lit_char(&mut it, b'+') {
                let e = match self.unary_expression(&mut it)? {
                    Some(e) => e,
                    None => return Err(expectation_failure(it, it, "unary_expression")),
                };
                let end_pos = self.iter_pos(&mut it);
                *val = Expr::from_expr_range(e, begin, end_pos);
                *iter = it;
                return Ok(true);
            }
        }
        // ((kw["not"] | '!') > unary_expression(_r1) > iter_pos) [ FactorActions::not_(_1, _val) ]
        {
            let mut it = *iter;
            if self.kw(&mut it, "not") || self.lit_char(&mut it, b'!') {
                let e = match self.unary_expression(&mut it)? {
                    Some(e) => e,
                    None => return Err(expectation_failure(it, it, "unary_expression")),
                };
                self.iter_pos(&mut it);
                *val = e.unary_not(begin)?;
                *iter = it;
                return Ok(true);
            }
        }
        // (kw["min"] > '(' > conditional_expression(_r1) [_val = _1] > ',' > conditional_expression(_r1) > ')') [ min(_val, _2) ]
        // (kw["max"] ...) [ max(_val, _2) ]
        // (kw["random"] ...) [ MyContext::random(_r1, _val, _2) ]
        for name in ["min", "max", "random"] {
            let mut it = *iter;
            if let Some(e1) = self.function_head(&mut it, name)? {
                *val = e1;
                let e2 = self.function_second(&mut it)?;
                self.function_close(&mut it)?;
                match name {
                    "min" => Expr::min(val, &e2)?,
                    "max" => Expr::max(val, &e2)?,
                    _ => MyContext::random(self.ctx, val, &e2)?,
                }
                *iter = it;
                return Ok(true);
            }
        }
        // (kw["filament_change"] > '(' > conditional_expression(_r1) > ')') [ MyContext::filament_change(_r1, _1) ]
        {
            let mut it = *iter;
            if let Some(e) = self.function_head(&mut it, "filament_change")? {
                self.function_close(&mut it)?;
                MyContext::filament_change(self.ctx, &e);
                *iter = it;
                return Ok(true);
            }
        }
        // (kw["digits"] > '(' > conditional_expression(_r1) [_val = _1] > ',' > conditional_expression(_r1) > optional_parameter(_r1))
        //      [ digits<false>(_val, _2, _3) ]
        // (kw["zdigits"] ...) [ digits<true>(_val, _2, _3) ]
        for (name, leading_zeros) in [("digits", false), ("zdigits", true)] {
            let mut it = *iter;
            if let Some(e1) = self.function_head(&mut it, name)? {
                *val = e1;
                let e2 = self.function_second(&mut it)?;
                let e3 = match self.optional_parameter(&mut it)? {
                    Some(e) => e,
                    None => return Err(expectation_failure(it, it, "optional_parameter")),
                };
                Expr::digits(val, &e2, &e3, leading_zeros)?;
                *iter = it;
                return Ok(true);
            }
        }
        // (kw["int"]   > '(' > conditional_expression(_r1) > ')') [ FactorActions::to_int(_1, _val) ]
        // (kw["round"] ...) [ round ]  (kw["ceil"] ...) [ ceil ]  (kw["floor"] ...) [ floor ]
        for name in ["int", "round", "ceil", "floor"] {
            let mut it = *iter;
            if let Some(e) = self.function_head(&mut it, name)? {
                self.function_close(&mut it)?;
                *val = match name {
                    "int" => e.unary_integer(begin)?,
                    "round" => e.round(begin)?,
                    "ceil" => e.ceil(begin)?,
                    _ => e.floor(begin)?,
                };
                *iter = it;
                return Ok(true);
            }
        }
        // (strict_double > iter_pos) [ FactorActions::double_(_1, _2, _val) ]
        {
            let mut it = *iter;
            if let Some(d) = self.strict_double(&mut it) {
                let end_pos = self.iter_pos(&mut it);
                *val = Expr::from_double_range(d, begin, end_pos);
                *iter = it;
                return Ok(true);
            }
        }
        // (int_ > iter_pos) [ FactorActions::int_(_1, _2, _val) ]
        {
            let mut it = *iter;
            if let Some(i) = self.int_(&mut it) {
                let end_pos = self.iter_pos(&mut it);
                *val = Expr::from_int_range(i, begin, end_pos);
                *iter = it;
                return Ok(true);
            }
        }
        // (kw[bool_] > iter_pos) [ FactorActions::bool_(_1, _2, _val) ]
        {
            let mut it = *iter;
            if let Some(b) = self.kw_bool(&mut it) {
                let end_pos = self.iter_pos(&mut it);
                *val = Expr::from_bool_range(b, begin, end_pos);
                *iter = it;
                return Ok(true);
            }
        }
        // raw[lexeme['"' > *((utf8char - char_('\\') - char_('"')) | ('\\' > char_)) > '"']] [ FactorActions::string_(_1, _val) ]
        {
            // raw and lexeme pre-skip the iterator they are handed.
            self.skip_over(iter);
            let mut it = *iter;
            if let Some((b, e)) = self.quoted(&mut it, b'"')? {
                *val = Expr::from_string_range(self.substr(b + 1, e - 1), b, e);
                *iter = it;
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// `lexeme[q > *((utf8char - char_('\\') - char_(q)) | ('\\' > char_)) > q]` without a skipper,
    /// the body of the string literal (q = '"') and of `regular_expression` (q = '/').
    /// Returns the range from the opening to past the closing delimiter.
    fn quoted(&self, first: &mut usize, q: u8) -> PResult<(usize, usize)> {
        let begin = *first;
        let mut it = begin;
        if it < self.len() && self.src[it] == q {
            it += 1;
        } else {
            return Ok(None);
        }
        loop {
            // (utf8char - char_('\\') - char_(q))
            if it < self.len() && self.src[it] != q && self.src[it] != b'\\' {
                if self.utf8char(&mut it)? {
                    continue;
                }
                break;
            }
            // ('\\' > char_)
            if it < self.len() && self.src[it] == b'\\' {
                let mut i2 = it + 1;
                if i2 >= self.len() {
                    return Err(expectation_failure(i2, i2, "char"));
                }
                i2 += 1;
                it = i2;
                continue;
            }
            break;
        }
        if !(it < self.len() && self.src[it] == q) {
            return Err(expectation_failure(it, it, "literal-char"));
        }
        it += 1;
        *first = it;
        Ok(Some((begin, it)))
    }

    /// PlaceholderParser.cpp:1398-1402 `optional_parameter`
    pub fn optional_parameter(&self, first: &mut usize) -> PResult<Expr> {
        let mut iter = *first;
        // iter_pos[px::bind(&FactorActions::set_start_pos, _1, _val)]
        let start_pos = self.iter_pos(&mut iter);
        let mut val = Expr::new();
        val.it_range = (start_pos, start_pos);
        // lit(')') [ FactorActions::noexpr(_val) ]
        if self.lit_char(&mut iter, b')') {
            val.reset();
            *first = iter;
            return Ok(Some(val));
        }
        // | (lit(',') > conditional_expression(_r1) > ')') [ _val = _1 ]
        {
            let mut it = iter;
            if self.lit_char(&mut it, b',') {
                let e = match self.conditional_expression(&mut it)? {
                    Some(e) => e,
                    None => return Err(expectation_failure(it, it, "conditional_expression")),
                };
                self.function_close(&mut it)?;
                *first = it;
                return Ok(Some(e));
            }
        }
        Ok(None)
    }

    /// PlaceholderParser.cpp:1404-1411 `scalar_variable_reference`
    pub fn scalar_variable_reference(&self, first: &mut usize) -> PResult<Expr> {
        let mut iter = *first;
        // variable_reference(_r1)[_a=_1]
        let opt = match self.variable_reference(&mut iter)? {
            Some(o) => o,
            None => return Ok(None),
        };
        let mut out = Expr::new();
        // ('[' > additive_expression(_r1)[evaluate_index(_1, _b)] > ']' > iter_pos[vector_variable_reference(_r1, _a, _b, _1, _val)])
        {
            let mut it = iter;
            if self.lit_char(&mut it, b'[') {
                let index_expr = match self.additive_expression(&mut it)? {
                    Some(e) => e,
                    None => return Err(expectation_failure(it, it, "additive_expression")),
                };
                let index = MyContext::evaluate_index(&index_expr)?;
                if !self.lit_char(&mut it, b']') {
                    return Err(expectation_failure(it, it, "literal-char"));
                }
                let it_end = self.iter_pos(&mut it);
                MyContext::vector_variable_reference(self.ctx, &opt, index, it_end, &mut out)?;
                *first = it;
                return Ok(Some(out));
            }
        }
        // | eps[scalar_variable_reference(_r1, _a, _val)]
        self.skip_over(&mut iter);
        MyContext::scalar_variable_reference(self.ctx, self.src, &opt, &mut out)?;
        *first = iter;
        Ok(Some(out))
    }

    /// PlaceholderParser.cpp:1413-1414 `variable_reference = identifier[resolve_variable(_r1, _1, _val)]`
    pub fn variable_reference(&self, first: &mut usize) -> PResult<OptWithPos<'c>> {
        match self.identifier(first)? {
            Some(id) => Ok(Some(MyContext::resolve_variable(self.ctx, self.src, id)?)),
            None => Ok(None),
        }
    }

    /// PlaceholderParser.cpp:1417 `regular_expression = raw[lexeme['/' > *(...) > '/']]`
    pub fn regular_expression(&self, first: &mut usize) -> PResult<(usize, usize)> {
        // raw and lexeme pre-skip the iterator the rule is handed.
        self.skip_over(first);
        let mut it = *first;
        match self.quoted(&mut it, b'/')? {
            Some(r) => {
                *first = it;
                Ok(Some(r))
            }
            None => Ok(None),
        }
    }
}

// =====================================================================
// Spirit numeric parsers (boost/spirit/home/qi/numeric)
// =====================================================================

/// `traits::pow10<double>(dim)`: the table of the correctly rounded literals `1e0` ... `1e308`.
fn pow10(dim: u32) -> f64 {
    format!("1e{}", dim).parse::<f64>().expect("1eN")
}

/// `extract_uint<u64, 10, 1, max_digits10<double> = 17, false, true>` (`ureal_policies::parse_n`).
fn parse_n(s: &[u8], first: &mut usize, acc: &mut u64) -> bool {
    if *first == s.len() {
        return false;
    }
    let mut it = *first;
    let mut leading_zeros = 0usize;
    while it < s.len() && s[it] == b'0' && leading_zeros < 17 {
        it += 1;
        leading_zeros += 1;
    }
    let mut val: u64 = 0;
    let mut count = 0usize;
    loop {
        if !(count + leading_zeros < 17) || it == s.len() {
            break;
        }
        let ch = s[it];
        if !ch.is_ascii_digit() {
            break;
        }
        // MaxDigits 17 <= digits10 of u64: the unchecked add.
        val = val.wrapping_mul(10).wrapping_add((ch - b'0') as u64);
        it += 1;
        count += 1;
    }
    if count + leading_zeros >= 1 {
        *acc = val;
        *first = it;
        true
    } else {
        false
    }
}

/// `ureal_policies::ignore_excess_digits`: `extract_uint<unused_type, 10, 1, -1>`, the number of
/// digits skipped.
fn ignore_excess_digits(s: &[u8], first: &mut usize) -> usize {
    let save = *first;
    while *first < s.len() && s[*first].is_ascii_digit() {
        *first += 1;
    }
    *first - save
}

/// `ureal_policies::parse_frac_n`: `extract_uint<u64, 10, 1, -1, true, true>` accumulating into
/// `acc` with overflow checks, then the extra digits skipped.
fn parse_frac_n(s: &[u8], first: &mut usize, acc: &mut u64, frac_digits: &mut i32) -> bool {
    if *first == s.len() {
        return false;
    }
    let savef = *first;
    let mut it = *first;
    let mut val = *acc;
    let mut count = 0usize;
    let mut overflow = false;
    loop {
        if it == s.len() {
            break;
        }
        let ch = s[it];
        if !ch.is_ascii_digit() {
            break;
        }
        let digit = (ch - b'0') as u64;
        // positive_accumulator::add(n, ch, mpl::true_): the checked add.
        if val > u64::MAX / 10 || val * 10 > u64::MAX - digit {
            overflow = true;
            break;
        }
        val = val * 10 + digit;
        it += 1;
        count += 1;
    }
    if !overflow && count == 0 {
        return false;
    }
    *acc = val;
    *first = it;
    *frac_digits = (*first - savef) as i32;
    // ignore extra (non-significant digits)
    ignore_excess_digits(s, first);
    true
}

/// `extract_int<int, 10, 1, -1>::call`: optional sign, digits, `false` on overflow (the iterator restored).
fn extract_int_i32(s: &[u8], first: &mut usize) -> Option<i32> {
    if *first == s.len() {
        return None;
    }
    let save = *first;
    let mut it = *first;
    let mut neg = false;
    if s[it] == b'-' || s[it] == b'+' {
        neg = s[it] == b'-';
        it += 1;
    }
    let digits_start = it;
    let mut val: i64 = 0;
    while it < s.len() && s[it].is_ascii_digit() {
        val = val * 10 + (s[it] - b'0') as i64;
        if (!neg && val > i32::MAX as i64) || (neg && -val < i32::MIN as i64) {
            *first = save;
            return None;
        }
        it += 1;
    }
    if it == digits_start {
        *first = save;
        return None;
    }
    *first = it;
    Some(if neg { (-val) as i32 } else { val as i32 })
}

/// `traits::scale(int exp, T& n, AccT acc_n)` (real_impl.hpp)
fn scale(exp: i32, n: &mut f64, acc_n: u64) -> bool {
    if exp >= 0 {
        let max_exp = 308; // std::numeric_limits<double>::max_exponent10
        if exp > max_exp {
            return false;
        }
        *n = (acc_n as f64) * pow10(exp as u32);
    } else if exp < -307 {
        // std::numeric_limits<double>::min_exponent10
        let min_exp = -307;
        // detail::compensate_roundoff(n, acc_n)
        let comp: u64 = 10;
        *n = ((acc_n / comp) * comp) as f64;
        *n += (acc_n % comp) as f64;
        *n /= pow10((-min_exp) as u32);
        let exp = exp + (-min_exp);
        if exp < min_exp {
            return false;
        }
        *n /= pow10((-exp) as u32);
    } else {
        *n = (acc_n as f64) / pow10((-exp) as u32);
    }
    true
}

/// `real_impl<double, strict_real_policies_without_nan_inf>::parse` (real_impl.hpp).
fn real_impl_parse(s: &[u8], first: &mut usize) -> Option<f64> {
    if *first == s.len() {
        return None;
    }
    let save = *first;
    // Start by parsing the sign. neg will be true if we got a "-" sign, false otherwise.
    let mut neg = false;
    if s[*first] == b'-' || s[*first] == b'+' {
        neg = s[*first] == b'-';
        *first += 1;
    }
    let mut n: f64 = 0.0;
    let mut acc_n: u64 = 0;
    let got_a_number = parse_n(s, first, &mut acc_n);
    let mut excess_n: i32 = 0;
    if !got_a_number {
        // Check whether the number to parse is a NaN or Inf
        if StrictRealPoliciesWithoutNanInf::parse_nan(first, s.len(), &mut n)
            || StrictRealPoliciesWithoutNanInf::parse_inf(first, s.len(), &mut n)
        {
            // If we got a negative sign, negate the number
            return Some(if neg { -n } else { n });
        }
        // allow_leading_dot is true: go on.
    } else {
        excess_n = ignore_excess_digits(s, first) as i32;
    }
    let e_hit;
    let e_pos;
    let mut frac_digits: i32 = 0;
    // Try to parse the dot ('.' decimal point)
    if *first < s.len() && s[*first] == b'.' {
        *first += 1;
        if excess_n != 0 {
            // We skip the fractions if we already exceeded our digits capacity
            ignore_excess_digits(s, first);
        } else if parse_frac_n(s, first, &mut acc_n, &mut frac_digits) {
        } else if !got_a_number {
            // allow_trailing_dot is true.
            *first = save;
            return None;
        }
        e_pos = *first;
        e_hit = *first < s.len() && (s[*first] == b'e' || s[*first] == b'E');
        if e_hit {
            *first += 1;
        }
    } else {
        // No dot and no number! Return no-match.
        if !got_a_number {
            *first = save;
            return None;
        }
        // If we must expect a dot and we didn't see an exponent prefix, return no-match.
        e_pos = *first;
        e_hit = *first < s.len() && (s[*first] == b'e' || s[*first] == b'E');
        if e_hit {
            *first += 1;
        }
        if !e_hit {
            // expect_dot is true.
            *first = save;
            return None;
        }
    }
    if e_hit {
        // We got the exponent prefix. Now we will try to parse the actual exponent.
        if let Some(exp) = extract_int_i32(s, first) {
            // Got the exponent value. Scale the number by exp + excess_n - frac_digits.
            if !scale(exp.wrapping_add(excess_n).wrapping_sub(frac_digits), &mut n, acc_n) {
                return None;
            }
        } else {
            // If there is no number, disregard the exponent altogether.
            *first = e_pos;
            scale(-frac_digits, &mut n, acc_n);
        }
    } else if frac_digits != 0 {
        scale(-frac_digits, &mut n, acc_n);
    } else if excess_n != 0 {
        if !scale(excess_n, &mut n, acc_n) {
            return None;
        }
    } else {
        n = acc_n as f64;
    }
    Some(if neg { -n } else { n })
}

// =====================================================================
// process_macro, PlaceholderParser  (PlaceholderParser.hpp:13-70, PlaceholderParser.cpp:73-162, 1516-1562)
// =====================================================================

/// PlaceholderParser.cpp:1516-1539 `static std::string process_macro(const std::string &templ, client::MyContext &context)`
fn process_macro(templ: &str, context: &MyContext) -> Result<String, PlaceholderParserException> {
    let src = templ.as_bytes();
    let macro_processor_instance = MacroProcessor::new(src, context);
    // Iterators over the source template.
    let mut iter = 0usize;
    let end = src.len();
    // phrase_parse(iter, end, macro_processor_instance(&context), space, output);
    let output = match macro_processor_instance.start(&mut iter) {
        Ok(s) => s,
        Err(Throw::ExpectationFailure { first, tag, .. }) => {
            // qi::on_error<qi::fail>(start, px::bind(&MyContext::process_error_message<Iterator>, _r1, _4, _1, _2, _3));
            MyContext::process_error_message(context, &tag, src, 0, end, first);
            String::new()
        }
        Err(Throw::Exception(e)) => return Err(e),
    };
    let msg = context.error_message.borrow();
    if !msg.is_empty() {
        let mut m = msg.clone();
        if !m.ends_with('\n') && !m.ends_with('\r') {
            m.push('\n');
        }
        return Err(PlaceholderParserException::PlaceholderParserError(m));
    }
    Ok(output)
}

/// PlaceholderParser.cpp:105-111 `static inline bool opts_equal(const DynamicConfig &config_old, const DynamicConfig &config_new, const std::string &opt_key)`
fn opts_equal(config_old: &DynamicConfig, config_new: &DynamicConfig, opt_key: &str) -> Result<bool, ConfigurationError> {
    let opt_old = config_old.option(opt_key);
    let opt_new = config_new.option(opt_key).expect("opts_equal: opt_new != nullptr");
    match opt_old {
        None => Ok(false),
        Some(o) => opt_new.operator_eq(o),
    }
}

/// PlaceholderParser.hpp:16-18 `struct ContextData { std::mt19937 rng; };`
#[derive(Clone, Debug, Default)]
pub struct ContextData {
    pub rng: Mt19937,
}

/// The value overloads of `PlaceholderParser::set` (PlaceholderParser.hpp:40-46).
pub trait PlaceholderValue {
    fn into_config_option(self) -> Box<dyn ConfigOption>;
}

/// PlaceholderParser.hpp:40 `void set(const std::string &key, const std::string &value)`
impl PlaceholderValue for &str {
    fn into_config_option(self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionString::with_value(self))
    }
}
impl PlaceholderValue for String {
    fn into_config_option(self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionString::with_value(self))
    }
}
/// PlaceholderParser.hpp:41 `void set(const std::string &key, int value)`
impl PlaceholderValue for i32 {
    fn into_config_option(self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionInt::with_value(self))
    }
}
/// PlaceholderParser.hpp:42 `void set(const std::string &key, unsigned int value) { this->set(key, int(value)); }`
impl PlaceholderValue for u32 {
    fn into_config_option(self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionInt::with_value(self as i32))
    }
}
/// PlaceholderParser.hpp:43 `void set(const std::string &key, bool value)`
impl PlaceholderValue for bool {
    fn into_config_option(self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionBool::with_value(self))
    }
}
/// PlaceholderParser.hpp:44 `void set(const std::string &key, double value)`
impl PlaceholderValue for f64 {
    fn into_config_option(self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionFloat::with_value(self))
    }
}
/// PlaceholderParser.hpp:45 `void set(const std::string &key, const std::vector<std::string> &values)`
impl PlaceholderValue for Vec<String> {
    fn into_config_option(self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionStrings::from_values(self))
    }
}
impl PlaceholderValue for &[String] {
    fn into_config_option(self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionStrings::from_values(self.to_vec()))
    }
}
/// PlaceholderParser.hpp:46 `void set(const std::string &key, ConfigOption *opt)`
impl PlaceholderValue for Box<dyn ConfigOption> {
    fn into_config_option(self) -> Box<dyn ConfigOption> {
        self
    }
}

/// PlaceholderParser.hpp:13-70 `class PlaceholderParser`
pub struct PlaceholderParser<'a> {
    m_config: DynamicConfig,
    m_external_config: Option<&'a DynamicConfig>,
}

impl<'a> PlaceholderParser<'a> {
    /// PlaceholderParser.cpp:73-78 `PlaceholderParser::PlaceholderParser(const DynamicConfig *external_config)`
    pub fn new(external_config: Option<&'a DynamicConfig>) -> Self {
        let mut pp = PlaceholderParser { m_config: DynamicConfig::new(), m_external_config: external_config };
        pp.set("version", crate::semver::SLIC3R_VERSION.to_string());
        pp.apply_env_variables();
        pp.update_timestamp();
        pp
    }

    /// PlaceholderParser.hpp:27 `void clear_config() { m_config.clear(); }`
    pub fn clear_config(&mut self) {
        self.m_config.clear();
    }

    /// PlaceholderParser.cpp:113-120 `std::vector<std::string> PlaceholderParser::config_diff(const DynamicPrintConfig &rhs)`
    pub fn config_diff(&self, rhs: &DynamicPrintConfig) -> Result<Vec<String>, ConfigurationError> {
        let mut diff_keys = Vec::new();
        for opt_key in crate::config::ConfigBase::keys(rhs) {
            if !opts_equal(&self.m_config, &rhs.base, &opt_key)? {
                diff_keys.push(opt_key);
            }
        }
        Ok(diff_keys)
    }

    /// PlaceholderParser.cpp:122-138 `bool PlaceholderParser::apply_config(const DynamicPrintConfig &rhs)`
    ///
    /// Scalar configuration values are stored into m_single, vector configuration values are
    /// stored into m_multiple. All vector configuration values stored into the PlaceholderParser
    /// are expected to be addressed by the extruder ID, therefore if a vector configuration value
    /// is addressed without an index, a current extruder ID is used.
    pub fn apply_config(&mut self, rhs: &DynamicPrintConfig) -> Result<bool, ConfigurationError> {
        let mut modified = false;
        for opt_key in crate::config::ConfigBase::keys(rhs) {
            if !opts_equal(&self.m_config, &rhs.base, &opt_key)? {
                let opt = ConfigOption::clone(rhs.option(&opt_key).expect("rhs.option(opt_key)"));
                self.set(&opt_key, opt);
                modified = true;
            }
        }
        Ok(modified)
    }

    /// PlaceholderParser.cpp:140-144 `void PlaceholderParser::apply_only(const DynamicPrintConfig &rhs, const std::vector<std::string> &keys)`
    pub fn apply_only(&mut self, rhs: &DynamicPrintConfig, keys: &[String]) {
        for opt_key in keys {
            let opt = ConfigOption::clone(rhs.option(opt_key).expect("rhs.option(opt_key)"));
            self.set(opt_key, opt);
        }
    }

    /// PlaceholderParser.cpp:146-149 `void PlaceholderParser::apply_config(DynamicPrintConfig &&rhs)`
    pub fn apply_config_move(&mut self, mut rhs: DynamicPrintConfig) {
        self.m_config.operator_add_assign_move(&mut rhs.base);
    }

    /// PlaceholderParser.cpp:151-162 `void PlaceholderParser::apply_env_variables()`
    pub fn apply_env_variables(&mut self) {
        for (k, v) in std::env::vars_os() {
            let key = k.to_string_lossy().into_owned();
            if key.as_bytes().starts_with(b"SLIC3R_") {
                // std::getline(ss, key, '='); ss >> value;
                let v = v.to_string_lossy();
                let value: String = v
                    .trim_start_matches(|c: char| matches!(c, ' ' | '\t' | '\n' | '\x0b' | '\x0c' | '\r'))
                    .chars()
                    .take_while(|c| !matches!(c, ' ' | '\t' | '\n' | '\x0b' | '\x0c' | '\r'))
                    .collect();
                self.set(&key, value);
            }
        }
    }

    /// PlaceholderParser.hpp:40-46 `void set(const std::string &key, ...)`: every overload.
    pub fn set<V: PlaceholderValue>(&mut self, key: &str, value: V) {
        self.m_config.set_key_value(key, value.into_config_option());
    }

    /// PlaceholderParser.hpp:47 `DynamicConfig& config_writable()`
    pub fn config_writable(&mut self) -> &mut DynamicConfig {
        &mut self.m_config
    }

    /// PlaceholderParser.hpp:48 `const DynamicConfig& config() const`
    pub fn config(&self) -> &DynamicConfig {
        &self.m_config
    }

    /// PlaceholderParser.hpp:49 `const ConfigOption* option(const std::string &key) const`
    pub fn option(&self, key: &str) -> Option<&dyn ConfigOption> {
        self.m_config.option(key)
    }

    /// PlaceholderParser.hpp:50 `const DynamicConfig* external_config() const`
    pub fn external_config(&self) -> Option<&'a DynamicConfig> {
        self.m_external_config
    }

    /// PlaceholderParser.cpp:1541-1550 `std::string PlaceholderParser::process(const std::string &templ,
    /// unsigned int current_extruder_id, const DynamicConfig *config_override, ContextData *context_data) const`
    pub fn process(
        &self,
        templ: &str,
        current_extruder_id: u32,
        config_override: Option<&DynamicConfig>,
        context_data: Option<&mut ContextData>,
    ) -> Result<String, PlaceholderParserException> {
        let mut context = MyContext::new_empty();
        context.external_config = self.external_config();
        context.config = Some(self.config());
        context.config_override = config_override;
        context.current_extruder_id = Cell::new(current_extruder_id as usize);
        context.context_data = context_data.map(RefCell::new);
        process_macro(templ, &context)
    }

    /// PlaceholderParser.cpp:1552-1562 `bool PlaceholderParser::evaluate_boolean_expression(const std::string &templ,
    /// const DynamicConfig &config, const DynamicConfig *config_override)`
    ///
    /// Evaluate a boolean expression using the full expressive power of the PlaceholderParser
    /// boolean expression syntax. Throws Slic3r::RuntimeError on syntax or runtime error.
    pub fn evaluate_boolean_expression(
        templ: &str,
        config: &DynamicConfig,
        config_override: Option<&DynamicConfig>,
    ) -> Result<bool, PlaceholderParserException> {
        let mut context = MyContext::new_empty();
        context.config = Some(config);
        context.config_override = config_override;
        // Let the macro processor parse just a boolean expression, not the full macro language.
        context.just_boolean_expression = true;
        Ok(process_macro(templ, &context)? == "true")
    }

    /// PlaceholderParser.cpp:80-103 `void PlaceholderParser::update_timestamp(DynamicConfig &config)`
    pub fn update_timestamp_of(config: &mut DynamicConfig) {
        let t = update_timestamp_values();
        config.set_key_value("timestamp", Box::new(ConfigOptionString::with_value(t.timestamp)));
        config.set_key_value("year", Box::new(ConfigOptionInt::with_value(t.year)));
        config.set_key_value("month", Box::new(ConfigOptionInt::with_value(t.month)));
        config.set_key_value("day", Box::new(ConfigOptionInt::with_value(t.day)));
        config.set_key_value("hour", Box::new(ConfigOptionInt::with_value(t.hour)));
        config.set_key_value("minute", Box::new(ConfigOptionInt::with_value(t.minute)));
        config.set_key_value("second", Box::new(ConfigOptionInt::with_value(t.second)));
    }

    /// PlaceholderParser.hpp:64 `void update_timestamp() { update_timestamp(m_config); }`
    pub fn update_timestamp(&mut self) {
        PlaceholderParser::update_timestamp_of(&mut self.m_config);
    }
}

// =====================================================================
// boost::regex_match for the `=~` / `!~` operators (PlaceholderParser.cpp:62-64, 647)
// =====================================================================

/// `boost::regex_match(subject, boost::regex(pattern))` with the default (perl) syntax.
///
/// FIDELITY-NOTE: a backtracking matcher over the common Perl subset of Boost.Regex, not a
/// port of boost/regex: literals, `.` (any byte, newlines included, as Boost's default),
/// `^` / `$` (line anchors, Boost's default), `[...]` sets with ranges, negation, escapes
/// and `[:class:]` names, groups `(...)` `(?:...)` `(?=...)` `(?!...)`, alternation, the
/// quantifiers `* + ? {n} {n,} {n,m}` with their lazy and possessive forms, the escapes
/// `\d \D \w \W \s \S \b \B \A \z \Z \t \n \r \f \v \a \e \xHH \0`, back references
/// `\1`-`\9`, and an escaped other character as itself. The error texts follow
/// basic_regex_parser.hpp for the errors it detects (unmatched parentheses and brackets,
/// a leading or misplaced repeat, an incomplete escape, a bad back reference); other
/// Boost.Regex syntax is reported as an invalid regular expression.
mod boost_regex {
    #[derive(Clone, Debug)]
    enum Node {
        Literal(u8),
        Any,
        Set(Box<[bool; 256]>),
        StartLine,
        EndLine,
        BufferStart,
        BufferEnd,
        SoftBufferEnd,
        WordBoundary(bool),
        Group(Box<Node>, Option<usize>),
        Look(Box<Node>, bool),
        Repeat { node: Box<Node>, min: usize, max: usize, greedy: bool, possessive: bool },
        Alt(Vec<Node>),
        Concat(Vec<Node>),
        Backref(usize),
    }

    const DEFAULT_ERRORS: [&str; 22] = [
        "Success",
        "No match",
        "Invalid regular expression.",
        "Invalid collation character.",
        "Invalid character class name, collating name, or character range.",
        "Invalid or unterminated escape sequence.",
        "Invalid back reference: specified capturing group does not exist.",
        "Unmatched [ or [^ in character class declaration.",
        "Unmatched marking parenthesis ( or \\(.",
        "Unmatched quantified repeat operator { or \\{.",
        "Invalid content of repeat range.",
        "Invalid range end in character class",
        "Out of memory.",
        "Invalid preceding regular expression prior to repetition operator.",
        "Premature end of regular expression",
        "Regular expression is too large.",
        "Unmatched ) or \\)",
        "Empty regular expression.",
        "The complexity of matching the regular expression exceeded predefined bounds.",
        "Ran out of stack space trying to match the regular expression.",
        "Invalid or unterminated Perl (?...) sequence.",
        "Unknown error.",
    ];
    const ERROR_BAD_PATTERN: usize = 2;
    const ERROR_CTYPE: usize = 4;
    const ERROR_ESCAPE: usize = 5;
    const ERROR_BACKREF: usize = 6;
    const ERROR_BRACK: usize = 7;
    const ERROR_PAREN: usize = 8;
    const ERROR_BRACE: usize = 9;
    const ERROR_BADBRACE: usize = 10;
    const ERROR_RANGE: usize = 11;
    const ERROR_BADREPEAT: usize = 13;
    const ERROR_PERL: usize = 20;

    /// basic_regex_parser.hpp:215-245 `fail(error_code, position, message, start_pos)`: the message
    /// augmented with the regular expression text around the position.
    fn fail(pattern: &[u8], position: usize, message: &str) -> String {
        let position = position.min(pattern.len());
        let start_pos = position.saturating_sub(10);
        let end_pos = (position + 10).min(pattern.len());
        let mut m = message.to_string();
        if start_pos != 0 || end_pos != pattern.len() {
            m.push_str("  The error occurred while parsing the regular expression fragment: '");
        } else {
            m.push_str("  The error occurred while parsing the regular expression: '");
        }
        if start_pos != end_pos {
            m.push_str(&String::from_utf8_lossy(&pattern[start_pos..position]));
            m.push_str(">>>HERE>>>");
            m.push_str(&String::from_utf8_lossy(&pattern[position..end_pos]));
        }
        m.push_str("'.");
        m
    }

    fn is_word(c: u8) -> bool {
        c.is_ascii_alphanumeric() || c == b'_'
    }
    fn is_separator(c: u8) -> bool {
        c == b'\n' || c == b'\r' || c == b'\x0c'
    }
    fn class_set(name: &str) -> Option<Box<[bool; 256]>> {
        let mut s = Box::new([false; 256]);
        for c in 0..256usize {
            let b = c as u8;
            s[c] = match name {
                "alnum" => b.is_ascii_alphanumeric(),
                "alpha" => b.is_ascii_alphabetic(),
                "blank" => b == b' ' || b == b'\t',
                "cntrl" => b.is_ascii_control(),
                "digit" | "d" => b.is_ascii_digit(),
                "graph" => b.is_ascii_graphic(),
                "lower" | "l" => b.is_ascii_lowercase(),
                "print" => b.is_ascii_graphic() || b == b' ',
                "punct" => b.is_ascii_punctuation(),
                "space" | "s" => matches!(b, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r'),
                "upper" | "u" => b.is_ascii_uppercase(),
                "xdigit" => b.is_ascii_hexdigit(),
                "word" | "w" => is_word(b),
                _ => return None,
            };
        }
        Some(s)
    }

    struct Parser<'p> {
        p: &'p [u8],
        pos: usize,
        mark_count: usize,
        max_backref: usize,
    }

    impl<'p> Parser<'p> {
        fn err(&self, code: usize, position: usize) -> String {
            fail(self.p, position, DEFAULT_ERRORS[code])
        }
        fn err_msg(&self, position: usize, message: &str) -> String {
            fail(self.p, position, message)
        }

        fn parse_alt(&mut self, depth: usize) -> Result<Node, String> {
            let mut alts = vec![self.parse_concat(depth)?];
            while self.pos < self.p.len() && self.p[self.pos] == b'|' {
                self.pos += 1;
                alts.push(self.parse_concat(depth)?);
            }
            Ok(if alts.len() == 1 { alts.pop().unwrap() } else { Node::Alt(alts) })
        }

        fn parse_concat(&mut self, depth: usize) -> Result<Node, String> {
            let mut items: Vec<Node> = Vec::new();
            // Whether the last state can be repeated (basic_regex_parser.hpp:1012-1056); at the start of
            // the expression, of a group or of an alternative the last state is a startmark or a jump.
            let mut last_repeatable = false;
            while self.pos < self.p.len() {
                let c = self.p[self.pos];
                match c {
                    b'|' => break,
                    b')' => {
                        if depth == 0 {
                            return Err(self.err_msg(self.pos, "Found a closing ) with no corresponding opening parenthesis."));
                        }
                        break;
                    }
                    b'*' | b'+' | b'?' | b'{' => {
                        // basic_regex_parser.hpp:358-386 (parse_extended), 1096-1287 (parse_repeat_range)
                        if self.pos == 0 && c != b'{' {
                            return Err(self.err_msg(
                                0,
                                &format!("The repeat operator \"{}\" cannot start a regular expression.", c as char),
                            ));
                        }
                        let open = self.pos;
                        self.pos += 1;
                        let (min, max) = match c {
                            b'*' => (0, usize::MAX),
                            b'+' => (1, usize::MAX),
                            b'?' => (0, 1),
                            _ => match self.parse_repeat_range()? {
                                Some(r) => r,
                                None => {
                                    // Treat the opening '{' as a literal character.
                                    self.pos = open + 1;
                                    items.push(Node::Literal(b'{'));
                                    last_repeatable = true;
                                    continue;
                                }
                            },
                        };
                        let mut greedy = true;
                        let mut possessive = false;
                        if self.pos < self.p.len() && self.p[self.pos] == b'?' {
                            greedy = false;
                            self.pos += 1;
                        }
                        if self.pos < self.p.len() && self.p[self.pos] == b'+' {
                            possessive = true;
                            self.pos += 1;
                        }
                        if !last_repeatable {
                            return Err(self.err(ERROR_BADREPEAT, self.pos));
                        }
                        let node = items.pop().expect("an item to repeat");
                        items.push(Node::Repeat { node: Box::new(node), min, max, greedy, possessive });
                        last_repeatable = false;
                        continue;
                    }
                    _ => {}
                }
                let (node, repeatable) = self.parse_atom(depth)?;
                items.push(node);
                last_repeatable = repeatable;
            }
            Ok(Node::Concat(items))
        }

        /// basic_regex_parser.hpp:1150-1287 `parse_repeat_range(false)` after the '{' of a Perl
        /// expression: `None` when the braces do not form a repeat (the '{' is then a literal).
        fn parse_repeat_range(&mut self) -> Result<Option<(usize, usize)>, String> {
            let is_space = |c: u8| matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r');
            // regex_traits::toi(m_position, m_end, 10): -1 without a digit.
            let toi = |s: &mut Self| -> i64 {
                let st = s.pos;
                let mut v: i64 = 0;
                while s.pos < s.p.len() && s.p[s.pos].is_ascii_digit() {
                    v = v.saturating_mul(10).saturating_add((s.p[s.pos] - b'0') as i64);
                    s.pos += 1;
                }
                if st == s.pos {
                    -1
                } else {
                    v
                }
            };
            while self.pos < self.p.len() && is_space(self.p[self.pos]) {
                self.pos += 1;
            }
            if self.pos == self.p.len() {
                return Ok(None);
            }
            let v = toi(self);
            if v < 0 {
                return Ok(None);
            }
            while self.pos < self.p.len() && is_space(self.p[self.pos]) {
                self.pos += 1;
            }
            if self.pos == self.p.len() {
                return Ok(None);
            }
            let min = v as usize;
            let max;
            if self.p[self.pos] == b',' {
                self.pos += 1;
                while self.pos < self.p.len() && is_space(self.p[self.pos]) {
                    self.pos += 1;
                }
                if self.pos == self.p.len() {
                    return Ok(None);
                }
                let v = toi(self);
                max = if v >= 0 { v as usize } else { usize::MAX };
            } else {
                max = min;
            }
            while self.pos < self.p.len() && is_space(self.p[self.pos]) {
                self.pos += 1;
            }
            if self.pos == self.p.len() || self.p[self.pos] != b'}' {
                return Ok(None);
            }
            self.pos += 1;
            if min > max {
                // Backtrack to error location:
                let mut pos = self.pos - 2;
                while pos > 0 && is_word(self.p[pos]) {
                    pos -= 1;
                }
                pos += 1;
                let _ = ERROR_BRACE;
                return Err(self.err(ERROR_BADBRACE, pos));
            }
            Ok(Some((min, max)))
        }

        fn parse_atom(&mut self, depth: usize) -> Result<(Node, bool), String> {
            let c = self.p[self.pos];
            match c {
                b'(' => {
                    let open = self.pos;
                    self.pos += 1;
                    if self.pos == self.p.len() {
                        return Err(self.err(ERROR_PAREN, self.pos));
                    }
                    let mut kind = 0u8; // 0 capture, 1 non-capture, 2 lookahead, 3 negative lookahead
                    if self.p[self.pos] == b'*' {
                        // A Perl verb (*...): none is supported here; the error of an invalid one.
                        return Err(self.err(ERROR_PERL, open));
                    }
                    if self.p[self.pos] == b'?' {
                        self.pos += 1;
                        match self.p.get(self.pos) {
                            Some(b':') => kind = 1,
                            Some(b'=') => kind = 2,
                            Some(b'!') => kind = 3,
                            _ => return Err(self.err(ERROR_PERL, open)),
                        }
                        self.pos += 1;
                    }
                    let index = if kind == 0 {
                        self.mark_count += 1;
                        Some(self.mark_count)
                    } else {
                        None
                    };
                    let inner = self.parse_alt(depth + 1)?;
                    if self.pos >= self.p.len() || self.p[self.pos] != b')' {
                        return Err(self.err(ERROR_PAREN, self.p.len()));
                    }
                    self.pos += 1;
                    Ok(match kind {
                        2 => (Node::Look(Box::new(inner), false), true),
                        3 => (Node::Look(Box::new(inner), true), true),
                        _ => (Node::Group(Box::new(inner), index), true),
                    })
                }
                b'[' => Ok((self.parse_set()?, true)),
                b'.' => {
                    self.pos += 1;
                    Ok((Node::Any, true))
                }
                b'^' => {
                    self.pos += 1;
                    Ok((Node::StartLine, false))
                }
                b'$' => {
                    self.pos += 1;
                    Ok((Node::EndLine, false))
                }
                b'\\' => self.parse_escape(),
                _ => {
                    self.pos += 1;
                    Ok((Node::Literal(c), true))
                }
            }
        }

        fn parse_escape(&mut self) -> Result<(Node, bool), String> {
            let esc_start = self.pos;
            self.pos += 1;
            if self.pos >= self.p.len() {
                return Err(self.err_msg(self.pos, "Incomplete escape sequence found."));
            }
            let c = self.p[self.pos];
            self.pos += 1;
            let negated_class = |name: &str| -> Node {
                let mut s = class_set(name).expect("class");
                for v in s.iter_mut() {
                    *v = !*v;
                }
                Node::Set(s)
            };
            Ok(match c {
                b'd' => (Node::Set(class_set("d").unwrap()), true),
                b'w' => (Node::Set(class_set("w").unwrap()), true),
                b's' => (Node::Set(class_set("s").unwrap()), true),
                b'l' => (Node::Set(class_set("l").unwrap()), true),
                b'u' => (Node::Set(class_set("u").unwrap()), true),
                b'D' => (negated_class("d"), true),
                b'W' => (negated_class("w"), true),
                b'S' => (negated_class("s"), true),
                b'L' => (negated_class("l"), true),
                b'U' => (negated_class("u"), true),
                b'b' => (Node::WordBoundary(true), false),
                b'B' => (Node::WordBoundary(false), false),
                b'A' | b'`' => (Node::BufferStart, false),
                b'z' | b'\'' => (Node::BufferEnd, false),
                b'Z' => (Node::SoftBufferEnd, false),
                b'1'..=b'9' => {
                    let n = (c - b'0') as usize;
                    if n > self.max_backref {
                        self.max_backref = n;
                    }
                    (Node::Backref(n), true)
                }
                _ => {
                    let lit = self.escape_literal(c, esc_start)?;
                    (Node::Literal(lit), true)
                }
            })
        }

        /// The character an escape that is not a class or an assertion stands for.
        fn escape_literal(&mut self, c: u8, esc_start: usize) -> Result<u8, String> {
            Ok(match c {
                b'a' => 0x07,
                b'e' => 0x1b,
                b'f' => 0x0c,
                b'n' => b'\n',
                b'r' => b'\r',
                b't' => b'\t',
                b'v' => 0x0b,
                b'0' => {
                    let mut v: u32 = 0;
                    let mut k = 0;
                    while k < 3 && self.pos < self.p.len() && (b'0'..=b'7').contains(&self.p[self.pos]) {
                        v = v * 8 + (self.p[self.pos] - b'0') as u32;
                        self.pos += 1;
                        k += 1;
                    }
                    v as u8
                }
                b'x' => {
                    let mut v: u32 = 0;
                    let mut k = 0;
                    while k < 2 && self.pos < self.p.len() && self.p[self.pos].is_ascii_hexdigit() {
                        v = v * 16 + (self.p[self.pos] as char).to_digit(16).unwrap();
                        self.pos += 1;
                        k += 1;
                    }
                    if k == 0 {
                        let _ = ERROR_ESCAPE;
                        return Err(self.err_msg(esc_start, "Escape sequence did not encode a valid character."));
                    }
                    v as u8
                }
                c if c.is_ascii_alphanumeric() => return Err(self.err(ERROR_BAD_PATTERN, self.pos)),
                c => c,
            })
        }

        /// basic_regex_parser.hpp:1358-1441 `parse_set`
        fn parse_set(&mut self) -> Result<Node, String> {
            let incomplete = "Character set declaration starting with [ terminated prematurely - either no ] was found or the set had no content.";
            self.pos += 1;
            if self.pos == self.p.len() {
                return Err(self.err_msg(self.pos, incomplete));
            }
            let mut set = Box::new([false; 256]);
            let mut empty = true;
            let mut invalid = false;
            let mut negate = false;
            let base = self.pos;
            let mut item_base = self.pos;
            while self.pos != self.p.len() {
                let c = self.p[self.pos];
                match c {
                    b'^' if self.pos == base => {
                        negate = true;
                        self.pos += 1;
                        item_base = self.pos;
                    }
                    b']' if self.pos != item_base => {
                        self.pos += 1;
                        if invalid {
                            // basic_regex_parser.hpp:1390-1395: append_set() fails on an invalid range.
                            return Err(self.err(ERROR_CTYPE, self.pos));
                        }
                        if negate {
                            for v in set.iter_mut() {
                                *v = !*v;
                            }
                        }
                        let _ = ERROR_BRACK;
                        return Ok(Node::Set(set));
                    }
                    b'[' if self.p.get(self.pos + 1) == Some(&b':') => {
                        // parse_inner_set: a character class [:name:]
                        let st = self.pos + 2;
                        let mut e = st;
                        while e + 1 < self.p.len() && !(self.p[e] == b':' && self.p[e + 1] == b']') {
                            e += 1;
                        }
                        if e + 1 >= self.p.len() {
                            return Err(self.err_msg(
                                self.p.len(),
                                "Character class declaration starting with [ terminated prematurely - either no ] was found or the set had no content.",
                            ));
                        }
                        let name = String::from_utf8_lossy(&self.p[st..e]).into_owned();
                        let cls = match class_set(&name) {
                            Some(s) => s,
                            None => return Err(self.err(ERROR_CTYPE, st)),
                        };
                        for k in 0..256 {
                            set[k] |= cls[k];
                        }
                        empty = false;
                        self.pos = e + 2;
                    }
                    b'\\' if self
                        .p
                        .get(self.pos + 1)
                        .map_or(false, |e| matches!(e, b'd' | b'w' | b's' | b'l' | b'u' | b'D' | b'W' | b'S' | b'L' | b'U')) =>
                    {
                        // \d \w \s etc. inside a set
                        let e = self.p[self.pos + 1];
                        let cls = class_set(&(e.to_ascii_lowercase() as char).to_string()).unwrap();
                        let neg = e.is_ascii_uppercase();
                        for k in 0..256 {
                            set[k] |= cls[k] != neg;
                        }
                        empty = false;
                        self.pos += 2;
                    }
                    _ => {
                        self.parse_set_literal(&mut set, &mut empty, &mut invalid)?;
                    }
                }
            }
            // parse_set returned false: parse_all fails with the closing parenthesis message.
            Err(self.err_msg(self.pos, "Found a closing ) with no corresponding opening parenthesis."))
        }

        /// basic_regex_parser.hpp:1655-1700 `get_next_set_literal`
        fn get_next_set_literal(&mut self, empty: bool) -> Result<u8, String> {
            let c = self.p[self.pos];
            if c == b'-' {
                if !empty {
                    // see if we are at the end of the set:
                    self.pos += 1;
                    if self.pos == self.p.len() || self.p[self.pos] != b']' {
                        return Err(self.err(ERROR_RANGE, self.pos));
                    }
                    self.pos -= 1;
                }
                self.pos += 1;
                return Ok(b'-');
            }
            if c == b'\\' {
                let esc_start = self.pos;
                self.pos += 1;
                if self.pos == self.p.len() {
                    return Err(self.err(ERROR_BRACK, self.pos));
                }
                let e = self.p[self.pos];
                self.pos += 1;
                return self.escape_literal(e, esc_start);
            }
            self.pos += 1;
            Ok(c)
        }

        /// basic_regex_parser.hpp:1611-1652 `parse_set_literal`
        fn parse_set_literal(&mut self, set: &mut [bool; 256], empty: &mut bool, invalid: &mut bool) -> Result<(), String> {
            let start_range = self.get_next_set_literal(*empty)?;
            if self.pos == self.p.len() {
                return Err(self.err(ERROR_BRACK, self.pos));
            }
            if self.p[self.pos] == b'-' {
                // we have a range:
                self.pos += 1;
                if self.pos == self.p.len() {
                    return Err(self.err(ERROR_BRACK, self.pos));
                }
                if self.p[self.pos] != b']' {
                    let end_range = self.get_next_set_literal(*empty)?;
                    if end_range < start_range {
                        // basic_char_set::add_range keeps it; append_set rejects the set at its ']'.
                        *invalid = true;
                    } else {
                        for k in start_range..=end_range {
                            set[k as usize] = true;
                        }
                    }
                    *empty = false;
                    if self.pos < self.p.len() && self.p[self.pos] == b'-' {
                        self.pos += 1;
                        if self.pos == self.p.len() {
                            return Err(self.err(ERROR_BRACK, self.pos));
                        }
                        if self.p[self.pos] == b']' {
                            // trailing - :
                            self.pos -= 1;
                            return Ok(());
                        }
                        return Err(self.err(ERROR_RANGE, self.pos));
                    }
                    return Ok(());
                }
                self.pos -= 1;
            }
            set[start_range as usize] = true;
            *empty = false;
            Ok(())
        }
    }

    struct Matcher<'m> {
        s: &'m [u8],
        caps: Vec<Option<(usize, usize)>>,
        steps: usize,
    }

    const MAX_STEPS: usize = 10_000_000;

    impl<'m> Matcher<'m> {
        fn word_at(&self, i: usize) -> bool {
            i < self.s.len() && is_word(self.s[i])
        }

        /// Match `nodes[idx..]` of a concatenation at `pos`, then the continuation `k`.
        fn m_seq(&mut self, nodes: &[Node], pos: usize, k: &mut dyn FnMut(&mut Self, usize) -> bool) -> bool {
            match nodes.split_first() {
                None => k(self, pos),
                Some((head, rest)) => self.m(head, pos, &mut |me: &mut Self, p: usize| me.m_seq(rest, p, k)),
            }
        }

        fn m(&mut self, node: &Node, pos: usize, k: &mut dyn FnMut(&mut Self, usize) -> bool) -> bool {
            self.steps += 1;
            if self.steps > MAX_STEPS {
                return false;
            }
            let s = self.s;
            match node {
                Node::Literal(c) => pos < s.len() && s[pos] == *c && k(self, pos + 1),
                Node::Any => pos < s.len() && k(self, pos + 1),
                Node::Set(set) => pos < s.len() && set[s[pos] as usize] && k(self, pos + 1),
                Node::StartLine => {
                    let ok = if pos == 0 {
                        true
                    } else {
                        let t = s[pos - 1];
                        if pos != s.len() {
                            is_separator(t) && !(t == b'\r' && s[pos] == b'\n')
                        } else {
                            is_separator(t)
                        }
                    };
                    ok && k(self, pos)
                }
                Node::EndLine => {
                    let ok = if pos != s.len() {
                        is_separator(s[pos]) && !(pos > 0 && s[pos - 1] == b'\r' && s[pos] == b'\n')
                    } else {
                        true
                    };
                    ok && k(self, pos)
                }
                Node::BufferStart => pos == 0 && k(self, pos),
                Node::BufferEnd => pos == s.len() && k(self, pos),
                Node::SoftBufferEnd => {
                    let ok = pos == s.len() || s[pos..].iter().all(|&c| is_separator(c));
                    ok && k(self, pos)
                }
                Node::WordBoundary(want) => {
                    let before = pos > 0 && self.word_at(pos - 1);
                    let after = self.word_at(pos);
                    ((before != after) == *want) && k(self, pos)
                }
                Node::Group(inner, index) => match index {
                    None => self.m(inner, pos, k),
                    Some(i) => {
                        let i = *i;
                        let saved = self.caps[i];
                        let r = self.m(inner, pos, &mut |me: &mut Self, p: usize| {
                            let inner_saved = me.caps[i];
                            me.caps[i] = Some((pos, p));
                            if k(me, p) {
                                return true;
                            }
                            me.caps[i] = inner_saved;
                            false
                        });
                        if !r {
                            self.caps[i] = saved;
                        }
                        r
                    }
                },
                Node::Look(inner, negative) => {
                    let saved = self.caps.clone();
                    let found = self.m(inner, pos, &mut |_me: &mut Self, _p: usize| true);
                    if *negative {
                        self.caps = saved;
                        !found && k(self, pos)
                    } else if found {
                        k(self, pos) || {
                            self.caps = saved;
                            false
                        }
                    } else {
                        self.caps = saved;
                        false
                    }
                }
                Node::Repeat { node, min, max, greedy, possessive } => {
                    if *possessive {
                        // Atomic: take the iterations a greedy match takes, never give any back.
                        let mut p = pos;
                        let mut count = 0usize;
                        while count < *max {
                            let mut np: Option<usize> = None;
                            if !self.m(node, p, &mut |_me: &mut Self, q: usize| {
                                np = Some(q);
                                true
                            }) {
                                break;
                            }
                            let q = np.unwrap();
                            count += 1;
                            if q == p {
                                break;
                            }
                            p = q;
                        }
                        count >= *min && k(self, p)
                    } else {
                        self.m_repeat(node, *min, *max, *greedy, pos, 0, usize::MAX, k)
                    }
                }
                Node::Alt(alts) => {
                    for a in alts {
                        if self.m(a, pos, k) {
                            return true;
                        }
                    }
                    false
                }
                Node::Concat(items) => self.m_seq(items, pos, k),
                Node::Backref(n) => match self.caps.get(*n).copied().flatten() {
                    None => false,
                    Some((a, b)) => {
                        let len = b - a;
                        pos + len <= s.len() && s[a..b] == s[pos..pos + len] && k(self, pos + len)
                    }
                },
            }
        }

        #[allow(clippy::too_many_arguments)]
        fn m_repeat(
            &mut self,
            node: &Node,
            min: usize,
            max: usize,
            greedy: bool,
            pos: usize,
            count: usize,
            last_pos: usize,
            k: &mut dyn FnMut(&mut Self, usize) -> bool,
        ) -> bool {
            // An iteration that matched nothing ends the repeat.
            if count > min && last_pos == pos {
                return k(self, pos);
            }
            let can_more = count < max;
            let try_more = |me: &mut Self, k: &mut dyn FnMut(&mut Self, usize) -> bool| -> bool {
                me.m(node, pos, &mut |me2: &mut Self, p: usize| me2.m_repeat(node, min, max, greedy, p, count + 1, pos, k))
            };
            if count < min {
                return try_more(self, k);
            }
            if greedy {
                (can_more && try_more(self, k)) || k(self, pos)
            } else {
                k(self, pos) || (can_more && try_more(self, k))
            }
        }
    }

    /// `regex_match(subject, regex(pattern))`: `Err` carries `regex_error::what()`.
    pub fn regex_match(subject: &[u8], pattern: &[u8]) -> Result<bool, String> {
        let mut parser = Parser { p: pattern, pos: 0, mark_count: 0, max_backref: 0 };
        let node = parser.parse_alt(0)?;
        if parser.pos != pattern.len() {
            return Err(parser.err_msg(parser.pos, "Found a closing ) with no corresponding opening parenthesis."));
        }
        if parser.max_backref > parser.mark_count {
            return Err(parser.err_msg(parser.pos, "Found a backreference to a non-existant sub-expression."));
        }
        let _ = ERROR_BACKREF;
        let mut m = Matcher { s: subject, caps: vec![None; parser.mark_count + 1], steps: 0 };
        let len = subject.len();
        Ok(m.m(&node, 0, &mut |_me: &mut Matcher, p: usize| p == len))
    }
}

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
        assert_eq!(Expr::from_int(5).unary_minus(0).unwrap().i(), -5);
        assert_eq!(Expr::from_double(2.7).round(0).unwrap().i(), 3);
        assert_eq!(Expr::from_double(2.7).floor(0).unwrap().i(), 2);
        assert_eq!(Expr::from_double(2.1).ceil(0).unwrap().i(), 3);
        assert_eq!(Expr::from_double(2.9).unary_integer(0).unwrap().i(), 2);
        assert!(!Expr::from_bool(true).unary_not(0).unwrap().b());
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

    #[test]
    fn test_process_basic() {
        let mut pp = PlaceholderParser::new(None);
        pp.set("a", 3i32);
        pp.set("b", 1.5f64);
        pp.set("s", "abc");
        assert_eq!(pp.process("x{a+1}y[a]{b*2}", 0, None, None).unwrap(), "x4y33");
        assert_eq!(pp.process("{if a==3}yes{else}no{endif}", 0, None, None).unwrap(), "yes");
        assert_eq!(pp.process("  lead", 0, None, None).unwrap(), "lead");
        assert!(pp.process("{nonexistent}", 0, None, None).is_err());
        let e = pp.process("{a+}", 0, None, None).unwrap_err();
        assert_eq!(e.what(), "Parsing error at line 1. Expecting an expression.\n{a+}\n   ^\n");
    }

    #[test]
    fn test_evaluate_boolean_expression() {
        let mut c = DynamicConfig::new();
        c.set_key_value("n", Box::new(ConfigOptionInt::with_value(2)));
        assert!(PlaceholderParser::evaluate_boolean_expression("n == 2", &c, None).unwrap());
        assert!(!PlaceholderParser::evaluate_boolean_expression("n > 2 or false", &c, None).unwrap());
        assert!(PlaceholderParser::evaluate_boolean_expression("n", &c, None).is_err());
    }

    #[test]
    fn test_regex_subset() {
        assert_eq!(boost_regex::regex_match(b"abc", b"a.c"), Ok(true));
        assert_eq!(boost_regex::regex_match(b"abc", b"a.*"), Ok(true));
        assert_eq!(boost_regex::regex_match(b"abc", b"b"), Ok(false));
        assert_eq!(boost_regex::regex_match(b"PRINTER_VENDOR_BBL x", b".*PRINTER_VENDOR_BBL.*"), Ok(true));
        assert!(boost_regex::regex_match(b"a", b"(a").is_err());
    }
}
