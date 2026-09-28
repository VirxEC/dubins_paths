#[test]
/// Ensure src/f32.rs and src/f64.rs are the same, barring some substitutions
fn f32_f64_are_similar() {
    let base_dir = env!("CARGO_MANIFEST_DIR");
    let f32_path = format!("{base_dir}/src/f32.rs");
    let f64_path = format!("{base_dir}/src/f64.rs");

    let mut f32_content = std::fs::read_to_string(f32_path).unwrap();
    let f64_content = std::fs::read_to_string(f64_path).unwrap();

    let substitutions = [("f32", "f64"), ("Vec2", "DVec2")];

    f32_content = substitutions
        .iter()
        .fold(f32_content, |content, (from, to)| content.replace(from, to));

    let f32_lines = f32_content.lines();
    let f64_lines = f64_content.lines();

    for (i, (line32, line64)) in f32_lines.zip(f64_lines).enumerate() {
        assert_eq!(line32, line64, "Line {} differs", i + 1);
    }
}
