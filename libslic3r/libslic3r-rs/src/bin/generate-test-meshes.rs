//! Generates binary STL files for BambuStudio primitive shapes used in parity testing.
//!
//! Shapes match the exact vertex ordering and face winding from BambuStudio's
//! `TriangleMesh.cpp` so that slicing output is topology-identical.

use std::f32::consts::PI;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

// ---------------------------------------------------------------------------
// Binary STL writer
// ---------------------------------------------------------------------------

struct Triangle {
    normal: [f32; 3],
    v1: [f32; 3],
    v2: [f32; 3],
    v3: [f32; 3],
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn normalize(v: [f32; 3]) -> [f32; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len == 0.0 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

fn compute_normal(v1: [f32; 3], v2: [f32; 3], v3: [f32; 3]) -> [f32; 3] {
    normalize(cross(sub(v2, v1), sub(v3, v1)))
}

fn write_stl(path: &Path, triangles: &[Triangle]) -> io::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let mut f = fs::File::create(path)?;

    // 80-byte header (zeros)
    f.write_all(&[0u8; 80])?;

    // Triangle count
    let count = triangles.len() as u32;
    f.write_all(&count.to_le_bytes())?;

    for tri in triangles {
        // Normal
        for v in &tri.normal {
            f.write_all(&v.to_le_bytes())?;
        }
        // Vertices
        for v in &tri.v1 {
            f.write_all(&v.to_le_bytes())?;
        }
        for v in &tri.v2 {
            f.write_all(&v.to_le_bytes())?;
        }
        for v in &tri.v3 {
            f.write_all(&v.to_le_bytes())?;
        }
        // Attribute byte count
        f.write_all(&0u16.to_le_bytes())?;
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Cube  —  its_make_cube(25.6, 25.6, 25.6)
// ---------------------------------------------------------------------------

fn make_cube(xd: f64, yd: f64, zd: f64) -> Vec<Triangle> {
    let x = xd as f32;
    let y = yd as f32;
    let z = zd as f32;

    // Vertex ordering exactly matches BambuStudio
    let verts: [[f32; 3]; 8] = [
        [x, y, 0.0],     // 0
        [x, 0.0, 0.0],   // 1
        [0.0, 0.0, 0.0], // 2
        [0.0, y, 0.0],   // 3
        [x, y, z],       // 4
        [0.0, y, z],     // 5
        [0.0, 0.0, z],   // 6
        [x, 0.0, z],     // 7
    ];

    // Face indices exactly match BambuStudio
    let faces: [[usize; 3]; 12] = [
        [0, 1, 2],
        [0, 2, 3],
        [4, 5, 6],
        [4, 6, 7],
        [0, 4, 7],
        [0, 7, 1],
        [1, 7, 6],
        [1, 6, 2],
        [2, 6, 5],
        [2, 5, 3],
        [4, 0, 3],
        [4, 3, 5],
    ];

    faces
        .iter()
        .map(|f| {
            let v1 = verts[f[0]];
            let v2 = verts[f[1]];
            let v3 = verts[f[2]];
            Triangle {
                normal: compute_normal(v1, v2, v3),
                v1,
                v2,
                v3,
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Cylinder  —  its_make_cylinder(12.8, 25.6)
// ---------------------------------------------------------------------------

fn make_cylinder(r: f64, h: f64) -> Vec<Triangle> {
    let r = r as f32;
    let h = h as f32;

    // BambuStudio: fa = 2*PI/360 (1 degree), n_steps = ceil(2*PI / fa) = 360
    let n_steps: usize = 360;

    // Vertex layout matches BambuStudio:
    //   0 = bottom center (0,0,0)
    //   1 = top center (0,0,h)
    //   then pairs: bottom_i, top_i for each step around the circle
    let mut verts: Vec<[f32; 3]> = Vec::with_capacity(2 + 2 * n_steps);
    verts.push([0.0, 0.0, 0.0]); // index 0: bottom center
    verts.push([0.0, 0.0, h]); // index 1: top center

    for i in 0..n_steps {
        let angle = 2.0 * PI * (i as f32) / (n_steps as f32);
        let (s, c) = angle.sin_cos();
        verts.push([r * c, r * s, 0.0]); // bottom vertex, index 2 + 2*i
        verts.push([r * c, r * s, h]); // top vertex,    index 2 + 2*i + 1
    }

    // BambuStudio face construction:
    //   For each step i (0..n_steps):
    //     i1 = current step, i2 = next step (wrapping)
    //     idx_bottom_1 = 2 + 2*i1
    //     idx_top_1    = 2 + 2*i1 + 1
    //     idx_bottom_2 = 2 + 2*i2
    //     idx_top_2    = 2 + 2*i2 + 1
    //
    //   Bottom fan:  (0, idx_bottom_2, idx_bottom_1)
    //   Top fan:     (1, idx_top_1, idx_top_2)
    //   Side quad:   (idx_bottom_1, idx_bottom_2, idx_top_2)
    //                (idx_top_2, idx_top_1, idx_bottom_1)

    let mut tris: Vec<Triangle> = Vec::with_capacity(4 * n_steps);

    for i1 in 0..n_steps {
        let i2 = (i1 + 1) % n_steps;
        let ib1 = 2 + 2 * i1;
        let it1 = 2 + 2 * i1 + 1;
        let ib2 = 2 + 2 * i2;
        let it2 = 2 + 2 * i2 + 1;

        // Bottom fan
        let (v1, v2, v3) = (verts[0], verts[ib2], verts[ib1]);
        tris.push(Triangle {
            normal: compute_normal(v1, v2, v3),
            v1,
            v2,
            v3,
        });

        // Top fan
        let (v1, v2, v3) = (verts[1], verts[it1], verts[it2]);
        tris.push(Triangle {
            normal: compute_normal(v1, v2, v3),
            v1,
            v2,
            v3,
        });

        // Side quad - triangle 1
        let (v1, v2, v3) = (verts[ib1], verts[ib2], verts[it2]);
        tris.push(Triangle {
            normal: compute_normal(v1, v2, v3),
            v1,
            v2,
            v3,
        });

        // Side quad - triangle 2
        let (v1, v2, v3) = (verts[it2], verts[it1], verts[ib1]);
        tris.push(Triangle {
            normal: compute_normal(v1, v2, v3),
            v1,
            v2,
            v3,
        });
    }

    tris
}

// ---------------------------------------------------------------------------
// Sphere  —  its_make_sphere(12.8, PI/18)
// ---------------------------------------------------------------------------

fn make_sphere(r: f64, fa: f64) -> Vec<Triangle> {
    let r = r as f32;

    // BambuStudio: sectorCount = ceil(2*PI/fa), stackCount = ceil(PI/fa)
    let sector_count = (2.0 * std::f64::consts::PI / fa).ceil() as usize; // 36
    let stack_count = (std::f64::consts::PI / fa).ceil() as usize; // 18

    // Build vertices: UV sphere layout matching BambuStudio
    // For each stack (0..=stack_count), for each sector (0..=sector_count):
    //   stack_angle = PI/2 - stack * (PI / stack_count)
    //   sector_angle = sector * (2*PI / sector_count)
    let mut verts: Vec<[f32; 3]> = Vec::new();

    for i in 0..=stack_count {
        let stack_angle = PI / 2.0 - (i as f32) * PI / (stack_count as f32);
        let xy = r * stack_angle.cos();
        let z = r * stack_angle.sin();

        for j in 0..=sector_count {
            let sector_angle = (j as f32) * 2.0 * PI / (sector_count as f32);
            let x = xy * sector_angle.cos();
            let y = xy * sector_angle.sin();
            verts.push([x, y, z]);
        }
    }

    // Build triangles matching BambuStudio's winding
    // Each stack ring has (sector_count + 1) vertices.
    // For stack i, sector j:
    //   k1 = i * (sector_count + 1) + j
    //   k2 = k1 + (sector_count + 1)   [= next stack]
    let mut tris: Vec<Triangle> = Vec::new();
    let row = sector_count + 1;

    for i in 0..stack_count {
        for j in 0..sector_count {
            let k1 = i * row + j;
            let k2 = k1 + row;

            // First triangle (skip degenerate at top pole, stack 0)
            if i != 0 {
                let (v1, v2, v3) = (verts[k1], verts[k2], verts[k1 + 1]);
                tris.push(Triangle {
                    normal: compute_normal(v1, v2, v3),
                    v1,
                    v2,
                    v3,
                });
            }

            // Second triangle (skip degenerate at bottom pole, last stack)
            if i != stack_count - 1 {
                let (v1, v2, v3) = (verts[k1 + 1], verts[k2], verts[k2 + 1]);
                tris.push(Triangle {
                    normal: compute_normal(v1, v2, v3),
                    v1,
                    v2,
                    v3,
                });
            }
        }
    }

    tris
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let out_dir = std::env::var_os("HELIO_FIXTURE_STL_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("fixtures/smoke/stl"));
    fs::create_dir_all(&out_dir).expect("Failed to create output directory");

    // Cube 25.6mm
    let cube = make_cube(25.6, 25.6, 25.6);
    let cube_path = out_dir.join("Cube_25.6.stl");
    write_stl(&cube_path, &cube).expect("Failed to write Cube STL");
    println!("Wrote {} ({} triangles)", cube_path.display(), cube.len());

    // Cylinder r=12.8, h=25.6
    let cylinder = make_cylinder(12.8, 25.6);
    let cyl_path = out_dir.join("Cylinder_25.6.stl");
    write_stl(&cyl_path, &cylinder).expect("Failed to write Cylinder STL");
    println!(
        "Wrote {} ({} triangles)",
        cyl_path.display(),
        cylinder.len()
    );

    // Sphere r=12.8, fa=PI/18
    let sphere = make_sphere(12.8, std::f64::consts::PI / 18.0);
    let sph_path = out_dir.join("Sphere_12.8.stl");
    write_stl(&sph_path, &sphere).expect("Failed to write Sphere STL");
    println!("Wrote {} ({} triangles)", sph_path.display(), sphere.len());
}
