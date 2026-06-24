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
