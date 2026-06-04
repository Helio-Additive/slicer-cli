//! Faithful 1:1 port of `ObjectID.hpp` / `ObjectID.cpp` from BambuStudio's libslic3r.
//!
//! Unique identifier of a mutable object across the application.
//! Used to synchronize the front end (UI) with the back end (BackgroundSlicingProcess /
//! Print / PrintObject) (for Model, ModelObject, ModelVolume, ModelInstance or ModelMaterial
//! classes) and to serialize / deserialize an object onto the Undo / Redo stack.
//!
//! NOTE on the cereal serialization machinery: the C++ header threads cereal `serialize` /
//! `load_and_construct` template members through these classes. cereal is a C++ Undo/Redo
//! serialization backend that is not part of the Rust slicing pipeline; those members are
//! intentionally not ported. Everything else is a faithful translation.

use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::OnceLock;

// ObjectID.hpp:20
// Unique identifier of a mutable object accross the application.
// Used to synchronize the front end (UI) with the back end (BackgroundSlicingProcess / Print / PrintObject)
// (for Model, ModelObject, ModelVolume, ModelInstance or ModelMaterial classes)
// and to serialize / deserialize an object onto the Undo / Redo stack.
// Valid IDs are strictly positive (non zero).
// It is declared as an object, as some compilers (notably msvcc) consider a typedef size_t equivalent to size_t
// for parameter overload.
//
// ObjectID.hpp:27-32 : the six comparison operators map directly onto the derived
// `PartialOrd`/`Ord`/`PartialEq`/`Eq` implementations, which compare `id` exactly as C++ does.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct ObjectID {
    // ObjectID.hpp:37
    pub id: usize,
}

impl ObjectID {
    // ObjectID.hpp:23 : ObjectID(size_t id) : id(id) {}
    pub fn new(id: usize) -> Self {
        ObjectID { id }
    }

    // ObjectID.hpp:24-25
    // Default constructor constructs an invalid ObjectID.
    // ObjectID() : id(0) {}
    pub fn default_invalid() -> Self {
        ObjectID { id: 0 }
    }

    // ObjectID.hpp:34 : bool valid() const { return id != 0; }
    pub fn valid(&self) -> bool {
        self.id != 0
    }

    // ObjectID.hpp:35 : bool invalid() const { return id == 0; }
    pub fn invalid(&self) -> bool {
        self.id == 0
    }
}

// ObjectID.cpp:5 : size_t ObjectBase::s_last_id = 0;
//
// Achtung! The s_last_id counter is not thread safe in C++, so it is expected that the
// ObjectBase derived instances are only instantiated from the main thread (ObjectID.hpp:48-49).
// Modelled here as an atomic so the global state remains sound under Rust's aliasing rules
// while preserving the monotonically-increasing `++ s_last_id` semantics.
static S_LAST_ID: AtomicUsize = AtomicUsize::new(0);

// ObjectID.hpp:86 : static inline ObjectID generate_new_id() { return ObjectID(++ s_last_id); }
fn generate_new_id() -> ObjectID {
    // Pre-increment: increment first, then use the new value. fetch_add returns the previous
    // value, so add 1 to it to reproduce the `++ s_last_id` result.
    let new_id = S_LAST_ID.fetch_add(1, Ordering::SeqCst) + 1;
    ObjectID::new(new_id)
}

// ObjectID.hpp:50
// Base for Model, ModelObject, ModelVolume, ModelInstance or ModelMaterial to provide a unique ID
// to synchronize the front end (UI) with the back end (BackgroundSlicingProcess / Print / PrintObject).
// Also base for Print, PrintObject, SLAPrint, SLAPrintObject to provide a unique ID for matching Model / ModelObject
// with their corresponding Print / PrintObject objects by the notification center at the UI when processing back-end warnings.
// Achtung! The s_last_id counter is not thread safe, so it is expected, that the ObjectBase derived instances
// are only instantiated from the main thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ObjectBase {
    // ObjectID.hpp:84
    m_id: ObjectID,
}

impl ObjectBase {
    // ObjectID.hpp:53 : using Timestamp = uint64_t;
    // (See the `Timestamp` type alias below.)

    // ObjectID.hpp:65
    // Constructors to be only called by derived classes.
    // Default constructor to assign a unique ID.
    // ObjectBase() : m_id(generate_new_id()) {}
    pub fn new() -> Self {
        ObjectBase {
            m_id: generate_new_id(),
        }
    }

    // ObjectID.hpp:66-68
    // Constructor with ignored int parameter to assign an invalid ID, to be replaced
    // by an existing ID copied from elsewhere.
    // ObjectBase(int) : m_id(ObjectID(0)) {}
    pub fn new_invalid() -> Self {
        ObjectBase {
            m_id: ObjectID::new(0),
        }
    }

    // ObjectID.hpp:70 : ObjectBase(const ObjectID id) : m_id(id) {}
    pub fn from_id(id: ObjectID) -> Self {
        ObjectBase { m_id: id }
    }

    // ObjectID.hpp:55 : ObjectID id() const { return m_id; }
    pub fn id(&self) -> ObjectID {
        self.m_id
    }

    // ObjectID.hpp:56-60
    // Return an optional timestamp of this object.
    // If the timestamp returned is non-zero, then the serialization framework will
    // only save this object on the Undo/Redo stack if the timestamp is different
    // from the timestmap of the object at the top of the Undo / Redo stack.
    // virtual Timestamp timestamp() const { return 0; }
    pub fn timestamp(&self) -> Timestamp {
        0
    }

    // ObjectID.hpp:74-75
    // Use with caution!
    // void set_new_unique_id() { m_id = generate_new_id(); }
    pub fn set_new_unique_id(&mut self) {
        self.m_id = generate_new_id();
    }

    // ObjectID.hpp:76 : void set_invalid_id() { m_id = 0; }
    pub fn set_invalid_id(&mut self) {
        self.m_id = ObjectID::new(0);
    }

    // ObjectID.hpp:77-78
    // Use with caution!
    // void copy_id(const ObjectBase &rhs) { m_id = rhs.id(); }
    pub fn copy_id(&mut self, rhs: &ObjectBase) {
        self.m_id = rhs.id();
    }

    // ObjectID.hpp:80-81
    // Override this method if a ObjectBase derived class owns other ObjectBase derived instances.
    // virtual void assign_new_unique_ids_recursive() { this->set_new_unique_id(); }
    pub fn assign_new_unique_ids_recursive(&mut self) {
        self.set_new_unique_id();
    }
}

impl Default for ObjectBase {
    fn default() -> Self {
        // ObjectID.hpp:65 : default constructor assigns a unique ID.
        ObjectBase::new()
    }
}

// ObjectID.hpp:53 : using Timestamp = uint64_t;
pub type Timestamp = u64;

// ObjectID.cpp:7-12
// Unique object / instance ID for the wipe tower.
// ObjectID wipe_tower_object_id()
// {
//     static ObjectBase mine;
//     return mine.id();
// }
//
// The C++ function-local `static ObjectBase mine` is constructed exactly once (lazily on first
// call), consuming one id from the global counter. `OnceLock` reproduces that exact-once,
// lazily-initialized semantics.
pub fn wipe_tower_object_id() -> ObjectID {
    static MINE: OnceLock<ObjectBase> = OnceLock::new();
    MINE.get_or_init(ObjectBase::new).id()
}

// ObjectID.cpp:14-18
// ObjectID wipe_tower_instance_id()
// {
//     static ObjectBase mine;
//     return mine.id();
// }
pub fn wipe_tower_instance_id() -> ObjectID {
    static MINE: OnceLock<ObjectBase> = OnceLock::new();
    MINE.get_or_init(ObjectBase::new).id()
}

// ObjectID.cpp:20 : ObjectWithTimestamp::Timestamp ObjectWithTimestamp::s_last_timestamp = 1;
static S_LAST_TIMESTAMP: AtomicU64 = AtomicU64::new(1);

// ObjectID.hpp:98
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ObjectWithTimestamp {
    // ObjectID.hpp:50 : public ObjectBase
    base: ObjectBase,
    // ObjectID.hpp:124-125
    // The first timestamp is non-zero, as zero timestamp means the timestamp is not reliable.
    // Timestamp m_timestamp { 1 };
    m_timestamp: Timestamp,
}

impl ObjectWithTimestamp {
    // ObjectID.hpp:101-103
    // Constructors to be only called by derived classes.
    // Default constructor to assign a new timestamp unique to this object's history.
    // ObjectWithTimestamp() = default;
    //
    // The defaulted constructor default-constructs the ObjectBase base (assigning a new unique
    // id) and the in-class member initializer `m_timestamp { 1 }`.
    pub fn new() -> Self {
        ObjectWithTimestamp {
            base: ObjectBase::new(),
            m_timestamp: 1,
        }
    }

    // ObjectID.hpp:104-106
    // Constructor with ignored int parameter to assign an invalid ID, to be replaced
    // by an existing ID copied from elsewhere.
    // ObjectWithTimestamp(int) : ObjectBase(-1) {}
    pub fn new_invalid() -> Self {
        ObjectWithTimestamp {
            base: ObjectBase::new_invalid(),
            m_timestamp: 1,
        }
    }

    // ObjectID.hpp:110-111
    // The timestamp uniquely identifies content of the derived class' data, therefore it makes sense to copy the timestamp if the content data was copied.
    // void copy_timestamp(const ObjectWithTimestamp& rhs) { m_timestamp = rhs.m_timestamp; }
    pub fn copy_timestamp(&mut self, rhs: &ObjectWithTimestamp) {
        self.m_timestamp = rhs.m_timestamp;
    }

    // ObjectID.hpp:114-118
    // Return an optional timestamp of this object.
    // If the timestamp returned is non-zero, then the serialization framework will
    // only save this object on the Undo/Redo stack if the timestamp is different
    // from the timestmap of the object at the top of the Undo / Redo stack.
    // Timestamp timestamp() const throw() override { return m_timestamp; }
    pub fn timestamp(&self) -> Timestamp {
        self.m_timestamp
    }

    // ObjectID.hpp:119 : bool timestamp_matches(const ObjectWithTimestamp &rhs) const throw() { return m_timestamp == rhs.m_timestamp; }
    pub fn timestamp_matches(&self, rhs: &ObjectWithTimestamp) -> bool {
        self.m_timestamp == rhs.m_timestamp
    }

    // ObjectID.hpp:120 : bool object_id_and_timestamp_match(const ObjectWithTimestamp &rhs) const throw() { return this->id() == rhs.id() && m_timestamp == rhs.m_timestamp; }
    pub fn object_id_and_timestamp_match(&self, rhs: &ObjectWithTimestamp) -> bool {
        self.id() == rhs.id() && self.m_timestamp == rhs.m_timestamp
    }

    // ObjectID.hpp:121 : void touch() { m_timestamp = ++ s_last_timestamp; }
    pub fn touch(&mut self) {
        // Pre-increment of the global counter, then assign.
        self.m_timestamp = S_LAST_TIMESTAMP.fetch_add(1, Ordering::SeqCst) + 1;
    }

    // ObjectID.hpp:55 : inherited ObjectID id() const { return m_id; }
    pub fn id(&self) -> ObjectID {
        self.base.id()
    }

    // Inherited mutators from ObjectBase, surfaced for derived-class parity.
    // ObjectID.hpp:74-75
    pub fn set_new_unique_id(&mut self) {
        self.base.set_new_unique_id();
    }

    // ObjectID.hpp:76
    pub fn set_invalid_id(&mut self) {
        self.base.set_invalid_id();
    }

    // ObjectID.hpp:77-78
    pub fn copy_id(&mut self, rhs: &ObjectBase) {
        self.base.copy_id(rhs);
    }
}

impl Default for ObjectWithTimestamp {
    fn default() -> Self {
        // ObjectID.hpp:103 : ObjectWithTimestamp() = default;
        ObjectWithTimestamp::new()
    }
}

// ObjectID.hpp:133
#[derive(Debug, Clone, Copy)]
pub struct CutObjectBase {
    // ObjectID.hpp:50 : public ObjectBase
    base: ObjectBase,
    // ObjectID.hpp:135-136
    // check sum of CutParts in initial Object
    // size_t m_check_sum{1};
    m_check_sum: usize,
    // ObjectID.hpp:137-138
    // connectors count
    // size_t m_connectors_cnt{0};
    m_connectors_cnt: usize,
}

impl CutObjectBase {
    // ObjectID.hpp:141-142
    // Default Constructor to assign an invalid ID
    // CutObjectBase() : ObjectBase(-1) {}
    pub fn new() -> Self {
        CutObjectBase {
            base: ObjectBase::new_invalid(),
            m_check_sum: 1,
            m_connectors_cnt: 0,
        }
    }

    // ObjectID.hpp:143-145
    // Constructor with ignored int parameter to assign an invalid ID, to be replaced
    // by an existing ID copied from elsewhere.
    // CutObjectBase(int) : ObjectBase(-1) {}
    pub fn new_invalid() -> Self {
        CutObjectBase {
            base: ObjectBase::new_invalid(),
            m_check_sum: 1,
            m_connectors_cnt: 0,
        }
    }

    // ObjectID.hpp:146-147
    // Constructor to initialize full information from 3mf
    // CutObjectBase(ObjectID id, size_t check_sum, size_t connectors_cnt) : ObjectBase(id), m_check_sum(check_sum), m_connectors_cnt(connectors_cnt) {}
    pub fn from_full(id: ObjectID, check_sum: usize, connectors_cnt: usize) -> Self {
        CutObjectBase {
            base: ObjectBase::from_id(id),
            m_check_sum: check_sum,
            m_connectors_cnt: connectors_cnt,
        }
    }

    // ObjectID.hpp:55 : inherited ObjectID id() const { return m_id; }
    pub fn id(&self) -> ObjectID {
        self.base.id()
    }

    // ObjectID.hpp:154-159
    // void copy(const CutObjectBase &rhs)
    // {
    //     this->copy_id(rhs);
    //     this->m_check_sum      = rhs.check_sum();
    //     this->m_connectors_cnt = rhs.connectors_cnt();
    // }
    pub fn copy(&mut self, rhs: &CutObjectBase) {
        self.base.copy_id(&rhs.base);
        self.m_check_sum = rhs.check_sum();
        self.m_connectors_cnt = rhs.connectors_cnt();
    }

    // ObjectID.hpp:166-171
    // void invalidate()
    // {
    //     set_invalid_id();
    //     m_check_sum      = 1;
    //     m_connectors_cnt = 0;
    // }
    pub fn invalidate(&mut self) {
        self.base.set_invalid_id();
        self.m_check_sum = 1;
        self.m_connectors_cnt = 0;
    }

    // ObjectID.hpp:173 : void init() { this->set_new_unique_id(); }
    pub fn init(&mut self) {
        self.base.set_new_unique_id();
    }

    // ObjectID.hpp:174 : bool has_same_id(const CutObjectBase &rhs) { return this->id() == rhs.id(); }
    pub fn has_same_id(&self, rhs: &CutObjectBase) -> bool {
        self.id() == rhs.id()
    }

    // ObjectID.hpp:175 : bool is_equal(const CutObjectBase &rhs) { return this->id() == rhs.id() && this->check_sum() == rhs.check_sum() && this->connectors_cnt() == rhs.connectors_cnt(); }
    pub fn is_equal(&self, rhs: &CutObjectBase) -> bool {
        self.id() == rhs.id()
            && self.check_sum() == rhs.check_sum()
            && self.connectors_cnt() == rhs.connectors_cnt()
    }

    // ObjectID.hpp:177 : size_t check_sum() const { return m_check_sum; }
    pub fn check_sum(&self) -> usize {
        self.m_check_sum
    }

    // ObjectID.hpp:178 : void set_check_sum(size_t cs) { m_check_sum = cs; }
    pub fn set_check_sum(&mut self, cs: usize) {
        self.m_check_sum = cs;
    }

    // ObjectID.hpp:179 : void increase_check_sum(size_t cnt) { m_check_sum += cnt; }
    pub fn increase_check_sum(&mut self, cnt: usize) {
        self.m_check_sum += cnt;
    }

    // ObjectID.hpp:181 : size_t connectors_cnt() const { return m_connectors_cnt; }
    pub fn connectors_cnt(&self) -> usize {
        self.m_connectors_cnt
    }

    // ObjectID.hpp:182 : void increase_connectors_cnt(size_t connectors_cnt) { m_connectors_cnt += connectors_cnt; }
    pub fn increase_connectors_cnt(&mut self, connectors_cnt: usize) {
        self.m_connectors_cnt += connectors_cnt;
    }
}

// ObjectID.hpp:151 : bool operator<(const CutObjectBase &other) const { return other.id() > this->id(); }
// ObjectID.hpp:152 : bool operator==(const CutObjectBase &other) const { return other.id() == this->id(); }
//
// NB: equality and ordering compare *only* the id (not the check sum / connector count), matching
// C++ exactly. `PartialEq`/`PartialOrd` are implemented by hand for that reason; deriving them
// would also compare the other fields.
impl PartialEq for CutObjectBase {
    fn eq(&self, other: &Self) -> bool {
        // ObjectID.hpp:152
        other.id() == self.id()
    }
}

impl PartialOrd for CutObjectBase {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        // ObjectID.hpp:151 : `*this < other` iff `other.id() > this->id()`.
        if other.id() > self.id() {
            Some(std::cmp::Ordering::Less)
        } else if self.id() > other.id() {
            Some(std::cmp::Ordering::Greater)
        } else {
            Some(std::cmp::Ordering::Equal)
        }
    }
}

// ObjectID.hpp:160-164
// CutObjectBase &operator=(const CutObjectBase &other)
// {
//     this->copy(other);
//     return *this;
// }
//
// C++ models copy-assignment as a `copy()`; the explicit `Clone` derive provides the
// member-wise copy, and `assign` mirrors `operator=` semantics exactly (delegating to `copy`).
impl CutObjectBase {
    pub fn assign(&mut self, other: &CutObjectBase) {
        self.copy(other);
    }
}

impl Default for CutObjectBase {
    fn default() -> Self {
        // ObjectID.hpp:142 : CutObjectBase() : ObjectBase(-1) {}
        CutObjectBase::new()
    }
}
