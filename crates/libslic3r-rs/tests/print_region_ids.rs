//! The ids of the live PrintRegions (R950): `Print::apply` gives every region of an object's `all_regions` its
//! index in that list as `print_object_region_id` (`get_create_region`, PrintApply.cpp:1025-1041) and, when it
//! rebuilds `Print::m_print_regions`, its index there as `print_region_id` (PrintApply.cpp:2273-2291; `region_set`
//! is never inserted into, so every region gets an index of its own). The live region setup of one object
//! (`set_default_region_config`, `install_painted_regions`) makes both equal to the region's index.
//! With PRINT_REGION_PRINT_ID_PIN=0 the `print_region_id`s stay -1 and this test fails.

use slicer::print::Print;
use slicer::region_config::PrintRegionConfig;

#[test]
fn live_regions_carry_the_pins_ids() {
    let mut print = Print::new();
    print.set_default_region_config(PrintRegionConfig::default());
    // Filament 1 is the default region's own (deduplicated, PrintApply.cpp:1004-1016); 2 and 3 add regions.
    print.install_painted_regions(&[1, 2, 3]);
    assert_eq!(print.num_print_regions(), 3);
    for idx in 0..print.num_print_regions() {
        let region = print.get_print_region(idx);
        assert_eq!(region.print_object_region_id(), idx as i32, "print_object_region_id of region {idx}");
        assert_eq!(region.print_region_id(), idx as i32, "print_region_id of region {idx}");
    }
}

/// Two objects (R951): each object has regions of its own, numbered 0.. in its `all_regions`
/// (`print_object_region_id`, PrintApply.cpp:1036-1038), and `Print::m_print_regions` numbers the second
/// object's regions after the first's (`print_region_id`, PrintApply.cpp:2273-2291). With
/// PRINT_REGION_IDS_PER_OBJECT=0 both objects share the first object's regions and this test fails.
#[test]
fn a_second_object_numbers_its_regions_after_the_first() {
    use slicer::print::PrintObject;
    let mut print = Print::new();
    print.set_default_region_config(PrintRegionConfig::default());
    print.install_painted_regions(&[1, 2, 3]);
    let n = print.num_print_regions();
    assert_eq!(n, 3);
    for _ in 0..2 {
        print.add_object(PrintObject::new());
    }
    assert_eq!(print.num_print_regions(), 2 * n, "m_print_regions holds both objects' regions");
    for (o, object) in print.objects().iter().enumerate() {
        assert_eq!(object.num_printing_regions(), n);
        for i in 0..n {
            let region = object.printing_region(i).expect("region");
            assert_eq!(region.print_object_region_id(), i as i32, "object {o} region {i}");
            assert_eq!(region.print_region_id(), (o * n + i) as i32, "object {o} region {i}");
            assert_eq!(print.get_print_region(o * n + i).print_region_id(), (o * n + i) as i32);
        }
    }
}
