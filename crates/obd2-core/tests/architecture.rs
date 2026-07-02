//! Architecture invariants for obd2-core source layout.

use std::fs;
use std::path::{Path, PathBuf};

fn collect_rs_files(dir: &Path, out: &mut Vec<PathBuf>) {
    let entries =
        fs::read_dir(dir).unwrap_or_else(|err| panic!("failed to read {}: {err}", dir.display()));

    for entry in entries {
        let path = entry
            .unwrap_or_else(|err| panic!("failed to read entry in {}: {err}", dir.display()))
            .path();

        if path.is_dir() {
            collect_rs_files(&path, out);
        } else if path.extension().and_then(|ext| ext.to_str()) == Some("rs") {
            out.push(path);
        }
    }
}

/// INV-5: protocol/ must stay wire-dialect-pure. ELM/AT text handling belongs
/// outside this layer, including in future nested modules.
#[test]
fn protocol_module_is_elm_free_recursively() {
    let protocol_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("src/protocol");
    let mut files = Vec::new();
    collect_rs_files(&protocol_dir, &mut files);
    files.sort();

    assert!(
        !files.is_empty(),
        "protocol/ dir not found or contains no Rust files"
    );

    let needles = [
        "ELM327",
        "decode_elm_response_payload",
        "SEARCHING...",
        "AT SH",
    ];

    let mut offenders = Vec::new();
    for path in files {
        let src = fs::read_to_string(&path)
            .unwrap_or_else(|err| panic!("failed to read {}: {err}", path.display()));

        for needle in needles {
            if src.contains(needle) {
                offenders.push(format!("{}: contains {needle:?}", path.display()));
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "protocol/ must be ELM-free (INV-5):\n{}",
        offenders.join("\n")
    );
}
