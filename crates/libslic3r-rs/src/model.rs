//! Model representation for 3D printing.
//!
//! This module provides Model and related types for representing 3D models
//! with multiple objects, instances, and metadata. Mirrors BambuStudio's Model.cpp.

use crate::geometry::{BoundingBox3F, Point3F, Transform3D};
use crate::triangle_mesh::TriangleMesh;
use std::path::PathBuf;

#[derive(Debug, Clone, Default)]
/// Container for all objects in a 3D print job with associated metadata.
/// Model.hpp:1581
pub struct Model {
    /// Objects in this model - corresponds to ModelObjectPtrs collection.
    /// Model.hpp:1588
    pub objects: Vec<ModelObject>,
    /// Model metadata - contains title, author, and descriptive information.
    /// Model.hpp:1581
    pub metadata: ModelMetadata,
    /// File path if loaded from file - for reference to source file.
    /// Model.hpp:350
    pub file_path: Option<PathBuf>,
}

/// Implementation of Model methods.
/// Model.hpp:1581-1680
impl Model {
    // Initialize a new empty Model with default constructor.
    // Model.hpp:1637
    pub fn new() -> Self {
        // Return default-constructed Model
        // Model.hpp:1637
        Self::default()
    }

    /// Insert a ModelObject into the objects collection and return its index.
    /// Model.cpp:461-465
    pub fn add_object(&mut self, object: ModelObject) -> usize {
        // Calculate index before pushing
        // Model.cpp:462
        let id = self.objects.len();
        // Append object to collection
        // Model.cpp:463
        self.objects.push(object);
        // Return new object index
        // Model.cpp:464
        id
    }

    /// Retrieve a reference to an object at the given index.
    /// Model.hpp:1592-1594
    pub fn get_object(&self, id: usize) -> Option<&ModelObject> {
        // Bounds-checked access returning None if out of range
        // Model.hpp:1593
        self.objects.get(id)
    }

    /// Retrieve a mutable reference to an object at the given index.
    /// Model.hpp:1592-1594
    pub fn get_object_mut(&mut self, id: usize) -> Option<&mut ModelObject> {
        // Bounds-checked mutable access returning None if out of range
        // Model.hpp:1593
        self.objects.get_mut(id)
    }

    /// Calculate bounding box encompassing all objects and instances.
    /// Model.hpp:437
    pub fn bounding_box(&self) -> BoundingBox3F {
        // Initialize empty bounding box
        // Model.hpp:437
        let mut bbox = BoundingBox3F::new();
        // Merge each object's bounding box into the total
        // Model.hpp:437
        for obj in &self.objects {
            // Merge individual object bounding box
            // Model.hpp:437
            bbox.merge(&obj.bounding_box());
        }
        // Return merged bounding box
        // Model.hpp:437
        bbox
    }

    /// Count total number of instances across all ModelObjects.
    /// Model.hpp:1588
    pub fn total_instances(&self) -> usize {
        // Sum instance counts across all objects
        // Model.hpp:1588
        self.objects.iter().map(|o| o.instances.len()).sum()
    }

    /// Check whether the model contains no objects.
    /// Model.hpp:1588
    pub fn is_empty(&self) -> bool {
        // Check if objects vector is empty
        // Model.hpp:1588
        self.objects.is_empty()
    }

    /// Return the number of ModelObjects in this Model.
    /// Model.hpp:1588
    pub fn object_count(&self) -> usize {
        // Return objects vector length
        // Model.hpp:1588
        self.objects.len()
    }
}

#[derive(Debug, Clone)]
/// ModelObject with mesh geometry and instance placements.
/// Model.hpp:344
pub struct ModelObject {
    /// Object name - user-visible identifier for the object.
    /// Model.hpp:347
    pub name: String,
    /// The mesh geometry - C++ stores this as volumes (ModelVolumePtrs).
    /// Model.hpp:356
    pub mesh: TriangleMesh,
    /// Instances of this object - each is a placement on the print bed.
    /// Model.hpp:353
    pub instances: Vec<Instance>,
    /// Per-object configuration - layer height, infill, extruder overrides.
    /// Model.hpp:358
    pub config: ObjectConfig,
    /// Whether this object is included in the print.
    /// Model.hpp:365
    pub printable: bool,
}

/// ModelObject methods for geometry and instance management.
/// Model.hpp:344-460
impl ModelObject {
    // Create a new ModelObject from a name and triangle mesh.
    // Model.cpp:467-483
    pub fn new(name: impl Into<String>, mesh: TriangleMesh) -> Self {
        // Construct ModelObject with name, mesh, and default instance
        // Model.cpp:468-474
        Self {
            // Set object name
            // Model.cpp:469
            name: name.into(),
            // Set mesh geometry
            // Model.cpp:470
            mesh,
            // Initialize with one default instance at the origin
            // Model.cpp:474
            instances: vec![Instance::default()],
            // Default configuration
            // Model.hpp:358
            config: ObjectConfig::default(),
            // Printable by default
            // Model.hpp:365
            printable: true,
        }
    }

    /// Calculate axis-aligned bounding box of this object's mesh.
    /// Model.hpp:437
    pub fn bounding_box(&self) -> BoundingBox3F {
        // Compute bounding box from mesh geometry
        // Model.hpp:437
        self.mesh.compute_bounding_box()
    }

    /// Add a new instance at the specified position.
    /// Model.hpp:427-429
    pub fn add_instance(&mut self, position: Point3F) -> usize {
        // Calculate instance index
        // Model.hpp:428
        let id = self.instances.len();
        // Create and push new instance at position
        // Model.hpp:428
        self.instances.push(Instance::at(position));
        // Return new instance index
        // Model.hpp:429
        id
    }

    /// Return the total number of instances for this object.
    /// Model.hpp:353
    pub fn instance_count(&self) -> usize {
        // Return instances vector length
        // Model.hpp:353
        self.instances.len()
    }

    /// Set the printable flag for this object.
    /// Model.hpp:365
    pub fn set_printable(&mut self, printable: bool) {
        // Set printable field directly
        // Model.hpp:365
        self.printable = printable;
    }
}

#[derive(Debug, Clone, Copy)]
/// An instance of an object placement on the print bed.
/// Model.hpp:1336
pub struct Instance {
    /// Instance position on the print bed.
    /// Model.hpp:1382
    pub position: Point3F,
    /// Rotation around Z axis (in degrees).
    /// Model.hpp:1388
    pub rotation_z: f64,
    /// Scaling factors for X, Y, Z axes.
    /// Model.hpp:1396
    pub scale: [f64; 3],
    /// Whether this instance is printable.
    /// Model.hpp:1349
    pub printable: bool,
}

/// Instance methods for positioning and transformation.
/// Model.hpp:1336-1460
impl Instance {
    // Create an instance at the origin with default values.
    // Model.hpp:1454
    pub fn new() -> Self {
        // Default instance at origin with identity transform
        // Model.hpp:1454
        Self {
            // Position at origin
            // Model.hpp:1382
            position: Point3F::new(0.0, 0.0, 0.0),
            // No rotation
            // Model.hpp:1388
            rotation_z: 0.0,
            // Unit scale
            // Model.hpp:1396
            scale: [1.0, 1.0, 1.0],
            // Printable by default
            // Model.hpp:1349
            printable: true,
        }
    }

    /// Create an instance at a specific position.
    /// Model.hpp:1454
    pub fn at(position: Point3F) -> Self {
        // Instance with specified position and default transform
        // Model.hpp:1454
        Self {
            // Set position
            // Model.hpp:1382
            position,
            // No rotation
            // Model.hpp:1388
            rotation_z: 0.0,
            // Unit scale
            // Model.hpp:1396
            scale: [1.0, 1.0, 1.0],
            // Printable by default
            // Model.hpp:1349
            printable: true,
        }
    }

    /// Set the Z rotation and return self for chaining.
    /// Model.hpp:1391
    pub fn with_rotation(mut self, degrees: f64) -> Self {
        // Set rotation angle in degrees
        // Model.hpp:1391
        self.rotation_z = degrees;
        // Return self for builder pattern
        // Model.hpp:1391
        self
    }

    /// Set uniform scale and return self for chaining.
    /// Model.hpp:1399
    pub fn with_scale(mut self, scale: f64) -> Self {
        // Set uniform scale on all three axes
        // Model.hpp:1399
        self.scale = [scale, scale, scale];
        // Return self for builder pattern
        // Model.hpp:1399
        self
    }

    /// Get the transformation matrix for this instance.
    /// Model.hpp:1421
    pub fn transform(&self) -> Transform3D {
        // Build transformation matrix from position, rotation, and scale
        // Model.hpp:1421
        Transform3D::identity()
            .translate(self.position.x, self.position.y, self.position.z)
            .rotate_z(self.rotation_z.to_radians())
            .scale(self.scale[0], self.scale[1], self.scale[2])
    }
}

/// Default implementation for Instance - creates instance at origin.
/// Model.hpp:1454
impl Default for Instance {
    // Return default instance at origin.
    // Model.hpp:1454
    fn default() -> Self {
        // Delegate to Instance::new()
        // Model.hpp:1454
        Self::new()
    }
}

#[derive(Debug, Clone, Default)]
/// Model metadata for descriptive information.
/// Model.hpp:1544
pub struct ModelMetadata {
    /// Model name/title.
    /// Model.hpp:1544
    pub title: Option<String>,
    /// Author/creator.
    /// Model.hpp:1544
    pub author: Option<String>,
    /// Description text.
    /// Model.hpp:1544
    pub description: Option<String>,
    /// Creation timestamp.
    /// Model.hpp:1544
    pub created_at: Option<String>,
    /// Modification timestamp.
    /// Model.hpp:1544
    pub modified_at: Option<String>,
    /// Application that created the model.
    /// Model.hpp:1544
    pub application: Option<String>,
}

#[derive(Debug, Clone, Default)]
/// Per-object configuration overrides.
/// Model.hpp:70
pub struct ObjectConfig {
    /// Layer height override.
    /// Model.hpp:70
    pub layer_height: Option<f64>,
    /// Infill density override.
    /// Model.hpp:70
    pub infill_density: Option<f64>,
    /// Number of perimeters override.
    /// Model.hpp:70
    pub perimeters: Option<u32>,
    /// Extruder to use for this object.
    /// Model.hpp:70
    pub extruder: Option<u32>,
}

/// Load a model from a file (detects format by extension).
/// Model.cpp:244-459
pub fn load_model(path: &PathBuf) -> crate::Result<Model> {
    // Extract file extension and normalize to lowercase
    // Model.cpp:245-246
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.to_lowercase());

    // Dispatch to format-specific loader based on extension
    // Model.cpp:247-459
    match ext.as_deref() {
        Some("stl") => {
            // Load STL file and wrap in Model
            // Model.cpp:247-300
            let mesh = crate::stl::load_stl(path)?;
            // Create empty model
            // Model.cpp:248
            let mut model = Model::new();
            // Create ModelObject from loaded mesh
            // Model.cpp:249
            let object = ModelObject::new("object", mesh);
            // Add object to model
            // Model.cpp:250
            model.add_object(object);
            // Store source file path
            // Model.cpp:251
            model.file_path = Some(path.clone());
            // Return loaded model
            // Model.cpp:252
            Ok(model)
        }
        Some("obj") => {
            // Load OBJ file with multiple objects
            // Model.cpp:301-350
            let loaded = crate::obj::load_obj(path)?;
            // Create empty model
            // Model.cpp:302
            let mut model = Model::new();
            // Add each object from loaded file
            // Model.cpp:303-305
            for obj in loaded.objects.into_iter() {
                // Add individual object
                // Model.cpp:304
                model.add_object(obj);
            }
            // Store source file path
            // Model.cpp:306
            model.file_path = Some(path.clone());
            // Return loaded model
            // Model.cpp:307
            Ok(model)
        }
        Some("3mf") => {
            // Load 3MF file with multiple objects
            // Model.cpp:351-459
            let loaded = crate::threemf::load_3mf(path)?;
            // Create empty model
            // Model.cpp:352
            let mut model = Model::new();
            // Add each object from loaded file
            // Model.cpp:353-355
            for obj in loaded.objects.into_iter() {
                // Add individual object
                // Model.cpp:354
                model.add_object(obj);
            }
            // Store source file path
            // Model.cpp:356
            model.file_path = Some(path.clone());
            // Return loaded model
            // Model.cpp:357
            Ok(model)
        }
        _ => {
            // Unsupported file format error
            // Model.cpp:458-459
            Err(crate::Error::Mesh(format!(
                "Unsupported file format: {:?}",
                ext
            )))
        }
    }
}
