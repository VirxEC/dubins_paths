use std::{
    env, fs, io,
    path::{Path, PathBuf},
};

fn copy_template(template: &Path, output: &TemplateOutput) -> io::Result<()> {
    let mut template = fs::read_to_string(template)?;

    for (replace_from, replace_to) in &output.replace {
        template = template.replace(replace_from, replace_to);
    }

    fs::write(&output.path, template)?;

    Ok(())
}

struct TemplateOutput {
    path: PathBuf,
    replace: Vec<(&'static str, &'static str)>,
}

fn main() -> io::Result<()> {
    // copy path.rs to f32.rs and f64.rs
    let base_dir_str = env::var("CARGO_MANIFEST_DIR").unwrap();
    let base_dir = PathBuf::from(base_dir_str);

    let src_dir = base_dir.join("src");
    let template_file = src_dir.join("f32.rs");

    let outputs = [TemplateOutput {
        path: src_dir.join("f64.rs"),
        replace: vec![("f32", "f64"), ("Vec2", "DVec2")],
    }];

    for output in outputs {
        copy_template(&template_file, &output)?;
    }

    let tests_dir = base_dir.join("tests");
    let template_file = tests_dir.join("f32.rs");

    let outputs = [TemplateOutput {
        path: tests_dir.join("f64.rs"),
        replace: vec![
            ("f32", "f64"),
            ("Vec3", "DVec3"),
            (
                "const POSROT_EPSILON: f32 = 0.00001;",
                "const POSROT_EPSILON: f64 = f64::EPSILON;",
            ),
        ],
    }];

    for output in outputs {
        copy_template(&template_file, &output)?;
    }

    Ok(())
}
