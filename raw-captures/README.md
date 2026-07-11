# Raw captures (scrubbed / synthetic fixtures only)

- See `docs/2026-07-11-vin-scrub-and-corpus-graduation.md` before committing partner data.
- **Never** commit unscrubbed partner captures or real VINs.
- `fixtures/synthetic/` — identity-free golden payloads wired to CI (`synthetic_corpus` test).
- Partner A1 graduation still requires scrubbing private `obd2-dash` captures first.

Live multi-MB captures currently live in private `obd2-dash` workspaces.
