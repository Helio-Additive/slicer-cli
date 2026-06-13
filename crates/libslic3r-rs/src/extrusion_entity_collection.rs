//! Faithful 1:1 port of `src/libslic3r/ExtrusionEntityCollection.cpp` (BambuStudio).
//!
//! C++ Reference:
//! - ExtrusionEntityCollection.hpp
//! - ExtrusionEntityCollection.cpp
//!
//! The `ExtrusionEntityCollection` struct itself (and its enum `ExtrusionEntityType`)
//! lives in [`crate::extrusion_entity`] — it was placed there together with the rest
//! of the `ExtrusionEntity` hierarchy. This module ports the free functions and the
//! methods that the C++ defines in `ExtrusionEntityCollection.cpp`, operating on that
//! same struct. The struct is re-exported here for convenience.
//!
//! C++ uses raw owning pointers (`ExtrusionEntitiesPtr = std::vector<ExtrusionEntity*>`)
//! with manual `clone()`/`delete`. The Rust port uses an owned `Vec<ExtrusionEntityType>`
//! enum, so the deep-copy semantics of `clone()`/`operator=` are provided by `#[derive(Clone)]`
//! and the destructor (`~ExtrusionEntityCollection`/`clear`) is provided by `Drop`/`Vec`.

// ExtrusionEntityCollection.cpp:1
// #include "ExtrusionEntityCollection.hpp"
// ExtrusionEntityCollection.cpp:2
// #include "ShortestPath.hpp"
pub use crate::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionPath, ExtrusionRole,
};
use crate::geometry::{Point, Polygons};
use crate::CoordF;

// ExtrusionEntityCollection.cpp:7
// namespace Slic3r {

// In C++ `ExtrusionEntitiesPtr` is `std::vector<ExtrusionEntity*>`. The Rust analog of
// a heterogeneous owning vector of extrusion entities is `Vec<ExtrusionEntityType>`.
type ExtrusionEntitiesPtr = Vec<ExtrusionEntityType>;

/// Return the role of a single entity, mirroring `ExtrusionEntity::role()`.
///
/// `ExtrusionLoop::role()` / `ExtrusionEntityCollection::role()` collapse multiple
/// distinct child roles to `erMixed` (see ExtrusionEntityCollection.hpp:54-61).
fn entity_role(entity: &ExtrusionEntityType) -> ExtrusionRole {
    match entity {
        ExtrusionEntityType::Path(p) => p.role,
        ExtrusionEntityType::Loop(l) => {
            // ExtrusionLoop::role(): all paths share a role -> that role, else erMixed.
            if l.paths.is_empty() {
                ExtrusionRole::None
            } else {
                let first_role = l.paths[0].role;
                if l.paths.iter().all(|p| p.role == first_role) {
                    first_role
                } else {
                    ExtrusionRole::Mixed
                }
            }
        }
        ExtrusionEntityType::Collection(c) => c.role(),
    }
}

// ExtrusionEntityCollection.cpp:9
// void filter_by_extrusion_role_in_place(ExtrusionEntitiesPtr &extrusion_entities, ExtrusionRole role)
//
// Remove those items from extrusion_entities, that do not match role.
// Do nothing if role is mixed. (ExtrusionEntityCollection.hpp:10-12)
// Removed elements are NOT being deleted.
pub fn filter_by_extrusion_role_in_place(
    extrusion_entities: &mut ExtrusionEntitiesPtr,
    role: ExtrusionRole,
) {
    // ExtrusionEntityCollection.cpp:11
    // if (role != erMixed) {
    if role != ExtrusionRole::Mixed {
        // ExtrusionEntityCollection.cpp:12-19
        // auto first  = extrusion_entities.begin();
        // auto last   = extrusion_entities.end();
        // extrusion_entities.erase(
        //     std::remove_if(first, last, [&role](const ExtrusionEntity* ee) {
        //         if((ee->role() == erSupportTransition && role ==erSupportMaterial))
        //             return false;
        //         return ee->role() != role; }),
        //     last);
        extrusion_entities.retain(|ee| {
            let ee_role = entity_role(ee);
            if ee_role == ExtrusionRole::SupportTransition && role == ExtrusionRole::SupportMaterial
            {
                // remove_if predicate returned false -> keep the element
                return true;
            }
            // remove_if predicate returned `ee->role() != role` -> keep iff role == role
            ee_role == role
        });
    }
    // ExtrusionEntityCollection.cpp:20
    // }
}

// ExtrusionEntityCollection.hpp:15-23
// Return new vector of ExtrusionEntities* with only those items from input extrusion_entities,
// that match role. Return all extrusion entities if role is mixed.
//
// NOTE: in C++ the returned pointers are *shared* with the source vector (not cloned).
// In the Rust port we own the entities by value, so producing a filtered list necessarily
// clones the kept entities. Callers that need the shared-pointer semantics should filter
// in place via `filter_by_extrusion_role_in_place`.
//
// inline ExtrusionEntitiesPtr filter_by_extrusion_role(const ExtrusionEntitiesPtr &extrusion_entities, ExtrusionRole role)
pub fn filter_by_extrusion_role(
    extrusion_entities: &ExtrusionEntitiesPtr,
    role: ExtrusionRole,
) -> ExtrusionEntitiesPtr {
    // ExtrusionEntityCollection.hpp:20
    // ExtrusionEntitiesPtr out { extrusion_entities };
    let mut out: ExtrusionEntitiesPtr = extrusion_entities.clone();
    // ExtrusionEntityCollection.hpp:21
    // filter_by_extrusion_role_in_place(out, role);
    filter_by_extrusion_role_in_place(&mut out, role);
    // ExtrusionEntityCollection.hpp:22
    // return out;
    out
}

impl ExtrusionEntityCollection {
    // ExtrusionEntityCollection.hpp:54-61
    // ExtrusionRole role() const override {
    //     ExtrusionRole out = erNone;
    //     for (const ExtrusionEntity *ee : entities) {
    //         ExtrusionRole er = ee->role();
    //         out = (out == erNone || out == er) ? er : erMixed;
    //     }
    //     return out;
    // }
    pub fn role(&self) -> ExtrusionRole {
        let mut out = ExtrusionRole::None;
        for ee in &self.entities {
            let er = entity_role(ee);
            out = if out == ExtrusionRole::None || out == er {
                er
            } else {
                ExtrusionRole::Mixed
            };
        }
        out
    }

    // ExtrusionEntityCollection.cpp:23
    // ExtrusionEntityCollection::ExtrusionEntityCollection(const ExtrusionPaths &paths)
    //     : no_sort(false)
    // {
    //     this->append(paths);
    // }
    pub fn from_paths(paths: &[ExtrusionPath]) -> Self {
        let mut out = ExtrusionEntityCollection {
            entities: Vec::new(),
            no_sort: false,
            orig_indices: Vec::new(),
            // ExtrusionEntityCollection.hpp:148 `bool is_reverse{true};`
            is_reverse: true,
        };
        out.append_paths(paths);
        out
    }

    // ExtrusionEntityCollection.hpp:89-93
    // void append(const ExtrusionPaths &paths) {
    //     this->entities.reserve(this->entities.size() + paths.size());
    //     for (const ExtrusionPath &path : paths)
    //         this->entities.emplace_back(path.clone());
    // }
    pub fn append_paths(&mut self, paths: &[ExtrusionPath]) {
        self.entities.reserve(self.entities.len() + paths.len());
        for path in paths {
            self.entities
                .push(ExtrusionEntityType::Path(path.clone()));
        }
    }

    // ExtrusionEntityCollection.cpp:29
    // ExtrusionEntityCollection& ExtrusionEntityCollection::operator=(const ExtrusionEntityCollection &other)
    // {
    //     clear();
    //     this->entities      = other.entities;
    //     for (size_t i = 0; i < this->entities.size(); ++i)
    //         this->entities[i] = this->entities[i]->clone();
    //     this->no_sort       = other.no_sort;
    //     return *this;
    // }
    //
    // The copy-assignment operator (ExtrusionEntityCollection.cpp:29) deep-clones `entities`
    // and copies `no_sort` only; it intentionally leaves `is_reverse`/`loop_node_range`
    // unchanged (unlike the move-assignment at ExtrusionEntityCollection.hpp:42-49, which
    // copies them). We mirror that: `is_reverse` is left as-is. `loop_node_range` is not
    // modeled in the Rust struct.
    pub fn assign(&mut self, other: &ExtrusionEntityCollection) {
        // clear();
        self.clear();
        // this->entities = other.entities; then deep clone every element.
        self.entities = other.entities.clone();
        // this->no_sort = other.no_sort;
        self.no_sort = other.no_sort;
    }

    // ExtrusionEntityCollection.cpp:39
    // void ExtrusionEntityCollection::swap(ExtrusionEntityCollection &c)
    // {
    //     std::swap(this->entities, c.entities);
    //     std::swap(this->no_sort, c.no_sort);
    // }
    pub fn swap(&mut self, c: &mut ExtrusionEntityCollection) {
        std::mem::swap(&mut self.entities, &mut c.entities);
        std::mem::swap(&mut self.no_sort, &mut c.no_sort);
    }

    // ExtrusionEntityCollection.cpp:45
    // void ExtrusionEntityCollection::clear()
    // {
    //     for (size_t i = 0; i < this->entities.size(); ++i)
    //         delete this->entities[i];
    //     this->entities.clear();
    // }
    //
    // Already implemented in `crate::extrusion_entity` (Vec drop handles the C++ `delete`).
    // Not re-declared here to avoid a duplicate-method definition.

    // ExtrusionEntityCollection.cpp:52
    // ExtrusionEntityCollection::operator ExtrusionPaths() const
    // {
    //     ExtrusionPaths paths;
    //     for (const ExtrusionEntity *ptr : this->entities) {
    //         if (const ExtrusionPath *path = dynamic_cast<const ExtrusionPath*>(ptr))
    //             paths.push_back(*path);
    //     }
    //     return paths;
    // }
    pub fn to_extrusion_paths(&self) -> Vec<ExtrusionPath> {
        // ExtrusionEntityCollection.cpp:54
        // ExtrusionPaths paths;
        let mut paths: Vec<ExtrusionPath> = Vec::new();
        // ExtrusionEntityCollection.cpp:55-58
        // for (const ExtrusionEntity *ptr : this->entities) {
        //     if (const ExtrusionPath *path = dynamic_cast<const ExtrusionPath*>(ptr))
        //         paths.push_back(*path);
        // }
        for ptr in &self.entities {
            if let ExtrusionEntityType::Path(path) = ptr {
                paths.push(path.clone());
            }
        }
        // ExtrusionEntityCollection.cpp:59
        // return paths;
        paths
    }

    // ExtrusionEntityCollection.cpp:62
    // ExtrusionEntity* ExtrusionEntityCollection::clone() const
    // {
    //     return new ExtrusionEntityCollection(*this);
    // }
    //
    // Provided by `#[derive(Clone)]` on `ExtrusionEntityCollection`.

    // ExtrusionEntityCollection.cpp:67
    // void ExtrusionEntityCollection::reverse()
    //
    // Ported in `crate::extrusion_entity` (it must coexist with the rest of the struct's
    // inherent methods); the loop-skipping behaviour matches the C++ exactly.

    // ExtrusionEntityCollection.cpp:77
    // void ExtrusionEntityCollection::replace(size_t i, const ExtrusionEntity &entity)
    // {
    //     delete this->entities[i];
    //     this->entities[i] = entity.clone();
    // }
    pub fn replace(&mut self, i: usize, entity: ExtrusionEntityType) {
        // delete this->entities[i]; this->entities[i] = entity.clone();
        self.entities[i] = entity;
    }

    // ExtrusionEntityCollection.cpp:83
    // void ExtrusionEntityCollection::remove(size_t i)
    // {
    //     delete this->entities[i];
    //     this->entities.erase(this->entities.begin() + i);
    // }
    pub fn remove(&mut self, i: usize) {
        // delete this->entities[i]; this->entities.erase(begin + i);
        self.entities.remove(i);
    }

    // ExtrusionEntityCollection.cpp:101
    // void ExtrusionEntityCollection::polygons_covered_by_width(Polygons &out, const float scaled_epsilon) const
    // {
    //     for (const ExtrusionEntity *entity : this->entities)
    //         entity->polygons_covered_by_width(out, scaled_epsilon);
    // }
    pub fn polygons_covered_by_width(&self, out: &mut Polygons, scaled_epsilon: f32) {
        // for (const ExtrusionEntity *entity : this->entities)
        //     entity->polygons_covered_by_width(out, scaled_epsilon);
        for entity in &self.entities {
            match entity {
                ExtrusionEntityType::Path(p) => p.polygons_covered_by_width(out, scaled_epsilon),
                ExtrusionEntityType::Loop(l) => l.polygons_covered_by_width(out, scaled_epsilon),
                ExtrusionEntityType::Collection(c) => {
                    c.polygons_covered_by_width(out, scaled_epsilon)
                }
            }
        }
    }

    // ExtrusionEntityCollection.cpp:107
    // void ExtrusionEntityCollection::polygons_covered_by_spacing(Polygons &out, const float scaled_epsilon) const
    // {
    //     for (const ExtrusionEntity *entity : this->entities)
    //         entity->polygons_covered_by_spacing(out, scaled_epsilon);
    // }
    pub fn polygons_covered_by_spacing(&self, out: &mut Polygons, scaled_epsilon: f32) {
        // for (const ExtrusionEntity *entity : this->entities)
        //     entity->polygons_covered_by_spacing(out, scaled_epsilon);
        for entity in &self.entities {
            match entity {
                ExtrusionEntityType::Path(p) => p.polygons_covered_by_spacing(out, scaled_epsilon),
                ExtrusionEntityType::Loop(l) => l.polygons_covered_by_spacing(out, scaled_epsilon),
                ExtrusionEntityType::Collection(c) => {
                    c.polygons_covered_by_spacing(out, scaled_epsilon)
                }
            }
        }
    }

    // ExtrusionEntityCollection.cpp:113-114
    // Recursively count paths and loops contained in this collection.
    // size_t ExtrusionEntityCollection::items_count() const
    pub fn items_count(&self) -> usize {
        // ExtrusionEntityCollection.cpp:116
        // size_t count = 0;
        let mut count: usize = 0;
        // ExtrusionEntityCollection.cpp:117-121
        // for (const ExtrusionEntity *entity : this->entities)
        //     if (entity->is_collection())
        //         count += static_cast<const ExtrusionEntityCollection*>(entity)->items_count();
        //     else
        //         ++ count;
        for entity in &self.entities {
            if let ExtrusionEntityType::Collection(c) = entity {
                count += c.items_count();
            } else {
                count += 1;
            }
        }
        // ExtrusionEntityCollection.cpp:122
        // return count;
        count
    }

    // ExtrusionEntityCollection.cpp:125-148
    // Returns a single vector of pointers to all non-collection items contained in this one.
    // ExtrusionEntityCollection ExtrusionEntityCollection::flatten(bool preserve_ordering) const
    pub fn flatten(&self, preserve_ordering: bool) -> ExtrusionEntityCollection {
        // ExtrusionEntityCollection.cpp:128-144
        // struct Flatten {
        //     Flatten(bool preserve_ordering) : preserve_ordering(preserve_ordering) {}
        //     ExtrusionEntityCollection out;
        //     bool                      preserve_ordering;
        //     void recursive_do(const ExtrusionEntityCollection &collection) { ... }
        // } flatten(preserve_ordering);
        fn recursive_do(
            out: &mut ExtrusionEntityCollection,
            preserve_ordering: bool,
            collection: &ExtrusionEntityCollection,
        ) {
            // ExtrusionEntityCollection.cpp:133-135
            // if (collection.no_sort && preserve_ordering) {
            //     // Don't flatten whatever happens below this level.
            //     out.append(collection);
            if collection.no_sort && preserve_ordering {
                // out.append(const ExtrusionEntity&) clones the whole collection as one entity.
                out.append(ExtrusionEntityType::Collection(Box::new(collection.clone())));
            } else {
                // ExtrusionEntityCollection.cpp:137-141
                // for (const ExtrusionEntity *entity : collection.entities)
                //     if (entity->is_collection())
                //         this->recursive_do(*static_cast<const ExtrusionEntityCollection*>(entity));
                //     else
                //         out.append(*entity);
                for entity in &collection.entities {
                    if let ExtrusionEntityType::Collection(c) = entity {
                        recursive_do(out, preserve_ordering, c);
                    } else {
                        out.append(entity.clone());
                    }
                }
            }
        }

        // ExtrusionEntityCollection.cpp:130
        // ExtrusionEntityCollection out;
        let mut out = ExtrusionEntityCollection {
            entities: Vec::new(),
            no_sort: false,
            orig_indices: Vec::new(),
            // ExtrusionEntityCollection.hpp:148 `bool is_reverse{true};`
            is_reverse: true,
        };
        // ExtrusionEntityCollection.cpp:146
        // flatten.recursive_do(*this);
        recursive_do(&mut out, preserve_ordering, self);
        // ExtrusionEntityCollection.cpp:147
        // return flatten.out;
        out
    }

    // ExtrusionEntityCollection.cpp:150
    // double ExtrusionEntityCollection::min_mm3_per_mm() const
    pub fn min_mm3_per_mm(&self) -> CoordF {
        // ExtrusionEntityCollection.cpp:152
        // double min_mm3_per_mm = std::numeric_limits<double>::max();
        let mut min_mm3_per_mm = f64::MAX;
        // ExtrusionEntityCollection.cpp:153-154
        // for (const ExtrusionEntity *entity : this->entities)
        //     min_mm3_per_mm = std::min(min_mm3_per_mm, entity->min_mm3_per_mm());
        for entity in &self.entities {
            let entity_min = match entity {
                // ExtrusionPath::min_mm3_per_mm() == this->mm3_per_mm
                ExtrusionEntityType::Path(p) => p.mm3_per_mm,
                // ExtrusionLoop::min_mm3_per_mm() == min over paths' mm3_per_mm
                ExtrusionEntityType::Loop(l) => l
                    .paths
                    .iter()
                    .map(|p| p.mm3_per_mm)
                    .fold(f64::MAX, f64::min),
                // ExtrusionEntityCollection::min_mm3_per_mm() recurses
                ExtrusionEntityType::Collection(c) => c.min_mm3_per_mm(),
            };
            min_mm3_per_mm = min_mm3_per_mm.min(entity_min);
        }
        // ExtrusionEntityCollection.cpp:155
        // return min_mm3_per_mm;
        min_mm3_per_mm
    }

    // ExtrusionEntityCollection.hpp:102-103
    // ExtrusionEntityCollection chained_path_from(const Point &start_near, ExtrusionRole role = erMixed) const
    //     { return this->no_sort ? *this : chained_path_from(this->entities, start_near, role); }
    pub fn chained_path_from_self(
        &self,
        start_near: &Point,
        role: ExtrusionRole,
    ) -> ExtrusionEntityCollection {
        if self.no_sort {
            self.clone()
        } else {
            chained_path_from(&self.entities, start_near, role)
        }
    }
}

// ExtrusionEntityCollection.cpp:89
// ExtrusionEntityCollection ExtrusionEntityCollection::chained_path_from(const ExtrusionEntitiesPtr& extrusion_entities, const Point &start_near, ExtrusionRole role)
// {
//     // Return a filtered copy of the collection.
//     ExtrusionEntityCollection out;
//     out.entities = filter_by_extrusion_role(extrusion_entities, role);
//     // Clone the extrusion entities.
//     for (auto &ptr : out.entities)
//         ptr = ptr->clone();
//     chain_and_reorder_extrusion_entities(out.entities, &start_near);
//     return out;
// }
//
pub fn chained_path_from(
    extrusion_entities: &ExtrusionEntitiesPtr,
    start_near: &Point,
    role: ExtrusionRole,
) -> ExtrusionEntityCollection {
    // Return a filtered copy of the collection.
    let mut out = ExtrusionEntityCollection {
        // ExtrusionEntityCollection.cpp:93 + 95-96
        // out.entities = filter_by_extrusion_role(extrusion_entities, role);
        // // Clone the extrusion entities.
        // for (auto &ptr : out.entities) ptr = ptr->clone();
        // (filter already returns cloned entities, fusing C++ lines 93 + 95-96.)
        entities: filter_by_extrusion_role(extrusion_entities, role),
        no_sort: false,
        orig_indices: Vec::new(),
        // ExtrusionEntityCollection.hpp:148 `bool is_reverse{true};`
        is_reverse: true,
    };
    // ExtrusionEntityCollection.cpp:97
    // chain_and_reorder_extrusion_entities(out.entities, &start_near);
    crate::shortest_path::chain_and_reorder_extrusion_entities(&mut out.entities, Some(start_near));
    // ExtrusionEntityCollection.cpp:98
    // return out;
    out
}

// ExtrusionEntityCollection.cpp:158
// } // namespace Slic3r
