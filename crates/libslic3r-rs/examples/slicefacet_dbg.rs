// SLICEFACET_DBG — temporary M1 diagnostic. Strip before final commit.
// Loads the Benchy STL exactly as app_slice does (bed-drop, no XY center),
// slices at z=0.3 and z=0.5, and reports loop/open-polyline/expolygon stats.
use slicer::geometry::Point3F;
use slicer::stl::read_stl_file;
use slicer::triangle_mesh_slicer::slicefacet_dbg_single;

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| "_downloads/3DBenchy.stl".into());
    let mut mesh = read_stl_file(std::path::Path::new(&path)).expect("load stl");
    // Bed drop (app_slice.rs:66-73): translate Z only.
    let bbox = mesh.bounding_box();
    let dz = -bbox.min.z;
    if dz != 0.0 {
        mesh.translate(Point3F { x: 0.0, y: 0.0, z: dz });
    }
    eprintln!("tris={} dz={:.6}", mesh.triangle_count(), dz);
    // Fine sweep to find the cavity-floor transition.
    if std::env::var("ZS").is_ok() {
        // Report mesh vertex z's near 0.30 and the exact f32(0.3) value.
        let z03: f32 = 0.3;
        eprintln!("f32(0.3)={:.10} bits={:#010x}", z03, z03.to_bits());
        let z03d: f32 = 0.3f64 as f32;
        eprintln!("(0.3f64 as f32)={:.10} bits={:#010x}", z03d, z03d.to_bits());
        // distinct vertex z values in [0.28, 0.34]
        let mut zs: Vec<f32> = mesh.vertices().iter()
            .map(|p| p.z as f32)
            .filter(|&z| z >= 0.28 && z <= 0.34)
            .collect();
        zs.sort_by(|a,b| a.partial_cmp(b).unwrap());
        zs.dedup();
        eprintln!("distinct vertex z in [0.28,0.34]: {} values", zs.len());
        for z in &zs {
            eprintln!("  z={:.10} bits={:#010x} {}", z, z.to_bits(),
                if (*z - z03).abs() < 1e-6 { "<-- ~0.3" } else { "" });
        }
        return;
    }
    if std::env::var("FINE").is_ok() {
        // Find cabin-floor facet z's near 0.3: report min/max vertex z of facets
        // whose z-extent straddles 0.30..0.31, plus exact-transition probe.
        for i in 0..1000 {
            let z = 0.300 + (i as f32) * 0.0001; // 0.300 .. 0.400
            if z > 0.320 { break; }
            let d = slicefacet_dbg_single(&mesh, z);
            eprintln!("z={:.5} holes={} biggest={:.2}", z, d.hole_count,
                d.hole_areas.iter().cloned().fold(0.0f64, f64::max));
        }
        return;
    }
    if std::env::var("SWEEP").is_ok() {
        let mut z = 0.20f32;
        while z <= 0.62 {
            let d = slicefacet_dbg_single(&mesh, z);
            eprintln!("z={:.4} holes={} expolys={} biggest_hole={:.2}",
                z, d.hole_count, d.expolygon_count,
                d.hole_areas.iter().cloned().fold(0.0f64, f64::max));
            z += 0.01;
        }
        return;
    }
    for &z in &[0.1f32, 0.3, 0.5] {
        let d = slicefacet_dbg_single(&mesh, z);
        eprintln!("=== z={:.4} ===", d.plane_z);
        eprintln!("  raw_lines={}", d.raw_line_count);
        eprintln!("  after_conn: loops={} open={}", d.after_conn_loops, d.after_conn_open);
        eprintln!("  after_exact: loops={} open={}", d.after_exact_loops, d.after_exact_open);
        eprintln!("  after_gap: loops={}", d.after_gap_loops);
        eprintln!("  remaining_open(mm)={:?}", d.remaining_open.iter().map(|v| (v*1000.0).round()/1000.0).collect::<Vec<_>>());
        eprintln!("  expolys={} holes={}", d.expolygon_count, d.hole_count);
        let mut ha: Vec<f64> = d.hole_areas.iter().map(|v| (v*100.0).round()/100.0).collect();
        ha.sort_by(|a,b| b.partial_cmp(a).unwrap());
        eprintln!("  hole_areas_mm2={:?}", ha);
        let mut la: Vec<f64> = d.loop_areas.iter().map(|v| (v*100.0).round()/100.0).collect();
        la.sort_by(|a,b| b.abs().partial_cmp(&a.abs()).unwrap());
        eprintln!("  loop_areas_mm2(signed,top)={:?}", &la.iter().take(20).collect::<Vec<_>>());
    }
}
